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
// Variables for encoder state
volatile long position1 = 0;
volatile long position2 = 0;
volatile int lastEncoded = 0;
volatile double speed = 0.0;
volatile long lastPosition = 0;
volatile long lastTime = 0;

void updateMotorPosition1() {
    if (inputA1.read() != inputB1.read()) {
        position1++;
    } else {
        position1--;
    }
}

void updateMotorPosition2() {
    if (inputA2.read() != inputB2.read()) {
        position2++;
    } else {
        position2--;
    }
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
    inputA1.rise(&updateMotorPosition1);
    inputA2.rise(&updateMotorPosition2);
    //inputA1.fall(&updateMotorPosition);
    //inputB1.rise(&updateMotorPosition);
    //inputB1.fall(&updateMotorPosition);

    // Attach the calculateSpeed function to be called every 100ms
    
    // Initialising pwm pins
    enableA.period(0.001f);
    enableB.period(0.001f);
    // Initialising variables
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
        // 13-bit, sign extended values.
        accelerometer.getOutput(readings);

        printf("X Axis: %i, Y Axis: %i, Z Axis %i, Motor Right: %i, Motor Left %i\n", (int16_t)readings[0], (int16_t)readings[1], (int16_t)readings[2], (int16_t)position1, (int16_t)position2);
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