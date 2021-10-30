/*
Hey there! This code was originally written for our YouTube channel, https://www.youtube.com/RKadeGaming
We're glad that you've taken an interest in our project, and we hope that you have a good time building it!
We've made this code public, and you're free to use it however you like. If you end up sharing the project with others though, 
we'd appreciate some attribution so that people know where to find more stuff like this.
Thanks, and have fun! :)
*/

// this file got way more convoluted than I like. Someday I'll write a really nice C++ class for this stuff...

#include "Wire.h"
#include <Keyboard.h>
#include "ListLib.h"

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69

float curX, curY, curZ; // variables for accelerometer raw data
float roll, pitch, curRoll, curPitch = 0;        // variables for the processed accerlerometer data

// the number of keys that I'll code the accelerometer to output. In this instance, map accelerometer values to steer left/right
const int numKeys = 2;
const int keyCodes[numKeys] = {KEY_LEFT_ARROW, KEY_RIGHT_ARROW};

const float rollThresholds[numKeys] =  {40, -40};

struct AccelInput {
    int keycode;
    float roll;
    boolean wasActive = false;
};

boolean shouldActivateAccel = false;     // this keeps track of whether a given key should be pressed or not given the current accelerometer readings

// this is an array of AccelInput structs called "Inputs" that contains numKeys items in it (left/right)
AccelInput AccelInputs[numKeys];


void setup(){
    Serial.begin(115200);
    initAccelerometer();        // initiate communication with the accelerometer
    initAccelInputs();      // initialize the accelerometer inputs (the left/right keys)
    
}

void loop(){
    // this function reads from the accelerometer and calculates the roll and pitch, which are stored in curRoll and curPitch
    getRollPitch();

    // now determine if the arrow keys to be pressed or released
    for(int i = 0; i < numKeys; i++){
        shouldActivateAccel = AccelInputs[i].wasActive;       // assume that the state will not change
        // determine whether the key is within the range to register a press
        if(isWithinRange(curRoll, AccelInputs[i].roll, AccelInputs[i].keycode)){
            // if the values are within range, the current key should be pressed
            shouldActivateAccel = true;
        }else{
            shouldActivateAccel = false;
        }
        
        // if the activity state on this iteration is different from the previous iteration, press or release the key
        if(shouldActivateAccel != AccelInputs[i].wasActive){
            if(shouldActivateAccel){
                Keyboard.press(Inputs[i].keycode);
            }else{
                Keyboard.release(Inputs[i].keycode);
            }
            Inputs[i].wasActive = shouldActivateAccel;
        }
    }
}

// is the current roll measurement within range to press the associated key?
bool isWithinRange(float curRoll, float targetRoll, int keycode){
    // check if the accelerometer reading is within range to press the left or right arrow keys repsectively
    if(keycode == KEY_LEFT_ARROW && curRoll > targetRoll){
        return true;
    }else if(keycode == KEY_RIGHT_ARROW && curRoll < targetRoll){
        return true;
    }
    return false;
}

// reads from the accelerometer and calculates the roll and pitch of the board
void getRollPitch(){
    // Read from the accelerometer to get the position of the balance board
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 3*2, true); // used to be 7 * 2 but since I only need 3 pieces of data, I don't need to process the rest
    
    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    curX = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    curY = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    curZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40  (ACCEL_ZOUT_L)
    curX = curX / 256;
    curY = curY / 256;
    curZ = curZ / 256;


    // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
    roll = atan(curY / sqrt(pow(curX, 2) + pow(curZ, 2))) * 180 / PI;
    pitch = atan(-1 * curX / sqrt(pow(curY, 2) + pow(curZ, 2))) * 180 / PI;
  
    // Low-pass filter (to reduce noise). This is really important because the sensors are pretty noisy
    curRoll = 0.94 * curRoll + 0.06 * roll;
    curPitch = 0.94 * curPitch + 0.06 * pitch;
}


void initAccelerometer(){
    // set up the accelerometer
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void initAccelInputs(){
    for(int i = 0; i < numKeys; i++){
        AccelInputs[i].keycode = keyCodes[i];
        AccelInputs[i].roll = rollThresholds[i];
    }
}

