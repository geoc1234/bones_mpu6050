// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your projec

// ***** NOTE:  I edited the "MPU6050_6Axis_MotionApps_V6_12.h" header to reduce the gyro range to 250deg/sec; doesn't work otherwise

#include <ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>  // use this as a 4-vector for the 4 sonar ranges and IMU output
#include <std_msgs/Time.h>
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

 #define USE_SONARS

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


ros::NodeHandle  nh;
geometry_msgs::QuaternionStamped sonar_ranges_msgs;
geometry_msgs::TwistWithCovarianceStamped mpu6050_ypr_msgs;
sensor_msgs::Imu mpu6050_msgs;

ros::Publisher pub_sonar_ranges( "sonars", &sonar_ranges_msgs);
ros::Publisher pub_mpu6050( "mpu6050", &mpu6050_msgs);

#define INTERRUPT_PIN 18  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gv;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t trigRR = 2;
uint8_t echoRR = 3;
uint8_t trigRC = 4;
uint8_t echoRC = 5;
uint8_t trigLL = 6;
uint8_t echoLL = 7;

uint8_t trig[3] = {trigRC, trigLL, trigRR};
uint8_t echo[3] = {echoRC, echoLL, echoRR};
uint32_t distance_log[3][3];  //number of sensors , sample index

uint32_t last_pub = 0;
int aperture = 12000;     //microseconds to keep the aperature open on each sonar
int echo_quench_time = 0; //milliseconds pause between sonar triggers, allowing echos time to clear;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool lin_cov_flag = true;
bool orient_cov_flag = true;

class LDatalogger {
    public:
        VectorInt16 data[100];
        int16_t nstack;

        LDatalogger() {}

        void add(VectorInt16 inputdata){
            if(nstack == 99){
                for(int i=0; i < 99; i++){
                    data[i] = data[i+1];
                }
                nstack = 98;
            }
                data[nstack] = inputdata;
                nstack++;
        }
                     
        void mean(VectorInt16 * v){
            for (int i = 0; i < nstack; i++){
                v->x += data[i].x;
                v->y += data[i].y;
                v->z += data[i].z;
            }
            v->x = v->x / nstack;
            v->y = v->y / nstack;
            v->z = v->z / nstack;
        }

        void correlation(float * corr){
            VectorInt16 vmean;
            this->mean(&vmean);
            for(int i=0; i<nstack; i++)
            {
                corr[0] += (data[i].x - vmean.x)  * (data[i].x - vmean.x);
                corr[1] += (data[i].x - vmean.x)  * (data[i].y - vmean.y);
                corr[2] += (data[i].x - vmean.x)  * (data[i].z - vmean.z);
                corr[3]  = corr[1];
                corr[4] += (data[i].y - vmean.y)  * (data[i].y - vmean.y);
                corr[5] += (data[i].y - vmean.y)  * (data[i].z - vmean.z);
                corr[6]  = corr[2];
                corr[7]  = corr[5];
                corr[8] += (data[i].z - vmean.z)  * (data[i].z - vmean.z);
            }
            for(int i=0; i<9; i++) {corr[i] = corr[i]/nstack;}

        }
};

class FDatalogger {
    public:
        VectorFloat data[100];
        int16_t nstack;

        FDatalogger() {}

        void add(VectorFloat inputdata){
            if(nstack == 99){
                for(int i=0; i < 99; i++){
                    data[i] = data[i+1];
                }
                nstack = 98;
            }
                data[nstack] = inputdata;
                nstack++;
        }
                     
        void mean(VectorFloat * v){
            for (int i = 0; i < nstack; i++){
                v->x += data[i].x;
                v->y += data[i].y;
                v->z += data[i].z;
            }
            v->x = v->x / nstack;
            v->y = v->y / nstack;
            v->z = v->z / nstack;
        }

        void correlation(float * corr){
            VectorFloat vmean;
            this->mean(&vmean);
            for(int i=0; i<nstack; i++)
            {
                corr[0] += (data[i].x - vmean.x)  * (data[i].x - vmean.x);
                corr[1] += (data[i].x - vmean.x)  * (data[i].y - vmean.y);
                corr[2] += (data[i].x - vmean.x)  * (data[i].z - vmean.z);
                corr[3]  = corr[1];
                corr[4] += (data[i].y - vmean.y)  * (data[i].y - vmean.y);
                corr[5] += (data[i].y - vmean.y)  * (data[i].z - vmean.z);
                corr[6]  = corr[2];
                corr[7]  = corr[5];
                corr[8] += (data[i].z - vmean.z)  * (data[i].z - vmean.z);
            }
            for(int i=0; i<9; i++) {corr[i] = corr[i]/nstack;}

        }
};

FDatalogger  mpu6050_orientation;
LDatalogger  mpu6050_linear;
LDatalogger  mpu6050_gyro;
ros::Time    lastSample;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    nh.initNode();
    nh.advertise(pub_sonar_ranges);
    nh.advertise(pub_mpu6050);
    lastSample = nh.now();
        
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(2,OUTPUT);
    pinMode(3,INPUT);
    pinMode(4,OUTPUT);
    pinMode(5,INPUT);
    pinMode(6,OUTPUT);
    pinMode(7,INPUT);

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // Serial.begin(115200);
    // while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // captured 11/20/2021 
    mpu.setXGyroOffset(29);
    mpu.setYGyroOffset(-28);
    mpu.setZGyroOffset(26);
    mpu.setZAccelOffset(1329);
    mpu.setXAccelOffset(228);
    mpu.setYAccelOffset(-3503);

    nh.spinOnce();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        // Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(devStatus);
        // Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
           
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {// Get the Latest packet 

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            q.normalize();
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGyro(&gv, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("areal\t");
            // Serial.print(aaReal.x * 9.806 / 8192);
            // Serial.print("\t");
            // Serial.print(aaReal.y * 9.806 / 8192);
            // Serial.print("\t");
            // Serial.println(aaReal.z * 9.806 / 8192);
            mpu6050_msgs.linear_acceleration.x = aaReal.x * 9.806 / 8192;
            mpu6050_msgs.linear_acceleration.y = aaReal.y * 9.806 / 8192;
            mpu6050_msgs.linear_acceleration.z = aaReal.z * 9.806 / 8192;
            mpu6050_msgs.angular_velocity.x = gv.x * 3.14159 / 180.0 / 131.0;
            mpu6050_msgs.angular_velocity.y = gv.y * 3.14159 / 180.0 / 131.0;
            mpu6050_msgs.angular_velocity.z = gv.z * 3.14159 / 180.0 / 131.0;
            mpu6050_msgs.orientation.x = 0;
            mpu6050_msgs.orientation.y = 0;
            if(ypr[0] > 3.14) {ypr[0] -= 6.28; } else if (ypr[0] < -3.14) { ypr[0] += 6.28;}
            mpu6050_msgs.orientation.z = - sin(ypr[0]/2);
            mpu6050_msgs.orientation.w = cos(ypr[0]/2);
            mpu6050_msgs.header.stamp = nh.now();
            mpu6050_msgs.header.frame_id = "mpu6050";
            VectorFloat Vf;
            Vf.x = 2.0*asin(q.x);
            Vf.y = 2.0*asin(q.y);
            Vf.z = 2.0*asin(q.z);
            mpu6050_orientation.add(Vf);
            mpu6050_linear.add(aaReal);
            mpu6050_gyro.add(gv);

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

    uint32_t distance;

#ifdef USE_SONARS
        digitalWrite(trig[0],LOW);   delayMicroseconds(4);
        digitalWrite(trig[0],HIGH);  delayMicroseconds(10);
        digitalWrite(trig[0],LOW);
        distance = pulseIn(echo[0],HIGH,(uint32_t) aperture);
          distance_log[0][2] = distance_log[0][1];
          distance_log[0][1] = distance_log[0][0];
          distance_log[0][0] = distance;
          distance = max(distance_log[0][2],max(distance_log[0][1],distance_log[0][0]));
          sonar_ranges_msgs.quaternion.x = 0.00017*(float)distance; //convert to meters
        delay((uint32_t) echo_quench_time); //pause to reduce echo
        
        digitalWrite(trig[1],LOW);   delayMicroseconds(4);
        digitalWrite(trig[1],HIGH);  delayMicroseconds(10);
        digitalWrite(trig[1],LOW);
        distance = pulseIn(echo[1],HIGH,(uint32_t) aperture);
          distance_log[1][2] = distance_log[1][1];
          distance_log[1][1] = distance_log[1][0];
          distance_log[1][0] = distance;
          distance = max(distance_log[1][2],max(distance_log[1][1],distance_log[1][0]));
        sonar_ranges_msgs.quaternion.y = 0.00017*(float)distance; //convert to meters
        delay((uint32_t) echo_quench_time); //pause to reduce echo

        digitalWrite(trig[2],LOW);   delayMicroseconds(4);
        digitalWrite(trig[2],HIGH);  delayMicroseconds(10);
        digitalWrite(trig[2],LOW);
        distance = pulseIn(echo[2],HIGH,(uint32_t) aperture);
          distance_log[2][2] = distance_log[2][1];
          distance_log[2][1] = distance_log[2][0];
          distance_log[2][0] = distance;
          distance = max(distance_log[2][2],max(distance_log[2][1],distance_log[2][0]));
        sonar_ranges_msgs.quaternion.z = 0.00017*(float)distance; //convert to meters
        delay((uint32_t) echo_quench_time); //pause to reduce echo
#endif
        sonar_ranges_msgs.header.stamp = nh.now();

        if(mpu6050_linear.nstack > 98 && lin_cov_flag)
        {
            lin_cov_flag = false;
            float corr[9] = {0.0};
            float corrg[9]= {0.0};
            mpu6050_linear.correlation(corr);
            mpu6050_gyro.correlation(corrg);
            for(int i=0; i<9; i++){ 
                mpu6050_msgs.linear_acceleration_covariance[i] = corr[i] * pow(9.806/8192.0,2) * 10;
                mpu6050_msgs.angular_velocity_covariance[i] = corrg[i] * pow(3.14159 / 180.0 / 131.0 ,2) * 10;
            }
        }

        if(mpu6050_orientation.nstack > 98 && orient_cov_flag)
        {
            orient_cov_flag = false;
            float corr[9] = {0.0};
            mpu6050_orientation.correlation(corr);
            for(int i=0; i<9; i++){ 
                mpu6050_msgs.orientation_covariance[i] = corr[i] * 10;
            }
        }

    if(millis() - last_pub > 80)
    {
        last_pub = millis();
        pub_sonar_ranges.publish(&sonar_ranges_msgs);
        pub_mpu6050.publish(&mpu6050_msgs);
        nh.spinOnce();
    }
}
