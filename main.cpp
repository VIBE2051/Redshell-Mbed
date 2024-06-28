/*
 * Group Project by: 
 * Francisco Jorquera,
 * Timothy Mead and
 * Ganesh Srinivasa
 * 
 */
 
#include "ADXL345.h"
#include "mbed.h"
#include <cmath>
#include <cstdio>
/////Pin allocations made specifically for NUCLEO-L476RG///////

int QEM[16] = {0, -1, 1, 2,
               1, 0, 2, -1,
               -1, 2, 0, 1,
               2, 1, -1, 0};

volatile int outVal, oldReading = 0, newReading; // Initialize oldReading
volatile long encoderCount = 0; 
volatile long newCount;
volatile long oldCount = 0;
// Number of ticks per cycle
const int ticksPerCycle = 48;

InterruptIn inputA1(PA_9);
InterruptIn inputB1(PA_8);
InterruptIn inputA2(PB_5);
InterruptIn inputB2(PA_10);

PwmOut enableA(PB_10);
PwmOut enableB(PB_4);
DigitalOut IN1(PC_8);
DigitalOut IN2(PC_6);
DigitalOut IN3(PC_5);
DigitalOut IN4(PA_12);

ADXL345 accelerometer(PA_7, PA_6, PA_5, PB_6); // mosi, miso, sck, cs
BufferedSerial pc(USBTX, USBRX);
Ticker encoderTicker;

// Variables for encoder state
volatile long position = 0;
volatile int lastEncoded = 0;
volatile double speed = 0.0;
volatile long lastPosition = 0;
volatile long lastTime = 0;

// Quadrature Encoder State Table
const int8_t ENCODER_STATE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// Function to update the encoder position
/*
void updateEncoder() {
    //int MSB = inputA1.read();
    //int LSB = inputB1.read();
    //int encoded = (MSB << 1) | LSB;
    //int sum = (lastEncoded << 2) | encoded;
    //outVal = ENCODER_STATE_TABLE[sum & 0x0f];
    newReading = oldReading;
    newReading = (int)inputA1.read() * 2 + (int)inputB1.read();
    outVal = QEM[(oldReading * 4 + newReading)];

    switch (outVal) {
        case -1: {
            position--;
            break;
        }
        case 1: {
            position++;
        }
    }
    //lastEncoded = encoded;
}
*/
void updateMotorPosition() {
    if (inputA1.read() != inputB1.read()) {
        position++;
    } else {
        position--;
    }
}
// Function to calculate speed
void calculateSpeed() {
    long currentTime = us_ticker_read();
    long timeDelta = currentTime - lastTime;
    int positionDelta = position - lastPosition;

    // Calculate speed in cycles per second
    speed = ((int)positionDelta / ticksPerCycle) / (timeDelta / 1e6);
    //printf("Speed: %d\n", (int32_t)speed);

    // Update last position and time
    lastPosition = position;
    lastTime = currentTime;
}



void motorA(float duty, int dir);
void motorB(float duty, int dir);


int main() {
    // Initialising Button pull direction
    inputA1.mode(PullDown);
    inputB1.mode(PullDown);
    inputA2.mode(PullDown);
    inputB2.mode(PullDown);

    // Attach the updateEncoder function to be called on the rising edge of inputA
    inputA1.rise(&updateMotorPosition);
    inputA1.fall(&updateMotorPosition);
    inputB1.rise(&updateMotorPosition);
    inputB1.fall(&updateMotorPosition);

    // Attach the calculateSpeed function to be called every 100ms
    //encoderTicker.attach(&calculateSpeed, 100ms);
    // Initialising pwm pins
    enableA.period(0.001f);
    enableB.period(0.001f);
    // Initialising variables
    double speed; 
    int readings[3] = {0, 0, 0};
    
    printf("Starting ADXL345 test...\n");
    printf("Device ID is: 0x%02x\n", accelerometer.getDevId());

    // Go into standby mode to configure the device.
    accelerometer.setPowerControl(0x00);

    // Full resolution, +/-16g, 4mg/LSB.
    accelerometer.setDataFormatControl(0x0B);
    
    // 3.2kHz data rate.
    accelerometer.setDataRate(ADXL345_3200HZ);

    // Measurement mode.
    accelerometer.setPowerControl(0x08);

    while (1) {
        ThisThread::sleep_for(100ms);
        calculateSpeed();
        // 13-bit, sign extended values.
        //printf("%i, %i, %i\n", (int16_t)readings[0], (int16_t)readings[1], (int16_t)readings[2];
        printf("Position: %d, Last Position: %d, Speed: %d counts/s\n", position, (int16_t)lastPosition, (int16_t)speed);
    }
}

void motorA(float duty, int dir) {
    if (duty <= 1.0) {
        switch (dir) {
            case 1: {
                IN1 = 1;
                IN2 = 0;
                enableA.write(duty);
                break;
            }
            case -1: {
                IN1 = 0;
                IN2 = 1;
                enableA.write(duty);
                break;
            }
        }
    }
}

void motorB(float duty, int dir) {
    if (duty <= 1.0) {
        switch (dir) {
            case 1: {
                IN3 = 1;
                IN4 = 0;
                enableB.write(duty);
                break;
            }
            case -1: {
                IN3 = 0;
                IN4 = 1;
                enableB.write(duty);
                break;
            }
        }
    }
}