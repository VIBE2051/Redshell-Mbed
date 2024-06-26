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
/////Pin allocations made specifically for NUCLEO-L476RG///////

int QEM[16] = {0, -1, 1, 2,
               1, 0, 2, -1,
               -1, 2, 0, 1,
               2, 1, -1, 0};

volatile int outVal, oldReading, newReading;
volatile long count = 0; 
volatile long newCount;
volatile long oldCount = 0;

DigitalIn inputA1(PA_9);
DigitalIn inputB1(PA_8);
DigitalIn inputA2(PB_5);
DigitalIn inputB2(PA_10);

PwmOut enableA(PB_10);
PwmOut enableB(PB_4);
DigitalOut IN1(PC_8);
DigitalOut IN2(PC_6);
DigitalOut IN3(PC_5);
DigitalOut IN4(PA_12);

ADXL345 accelerometer(PA_7, PA_6, PA_5, PB_6); // mosi, miso, sck, cs
BufferedSerial pc(USBTX, USBRX);

long QEMReading(long count);
long deltaCount(long newCount, long oldCount);
void motorA(float duty, int dir);
void motorB(float duty, int dir);
double speedRead(long oldCount);
long returnOldCount(long newCount, long oldCount);

int main() {
    enableA.period(0.001f);
    enableB.period(0.001f);

    double speed; 
    int readings[3] = {0, 0, 0};
    
    printf("Starting ADXL345 test...\n");
    printf("Device ID is: 0x%02x\n", accelerometer.getDevId());

    //Go into standby mode to configure the device.
    accelerometer.setPowerControl(0x00);

    //Full resolution, +/-16g, 4mg/LSB.
    accelerometer.setDataFormatControl(0x0B);
    
    //3.2kHz data rate.
    accelerometer.setDataRate(ADXL345_3200HZ);

    //Measurement mode.
    accelerometer.setPowerControl(0x08);

    while (1) {
        ThisThread::sleep_for(100ms);
        
        speed = speedRead(oldCount);

        //13-bit, sign extended values.
        printf("%i, %i, %i, %f\n", (int16_t)readings[0], (int16_t)readings[1], (int16_t)readings[2], speed);

    }
}

double speedRead(long oldCount) {
    double speed;
    newCount = QEMReading(oldCount);
    long delta = deltaCount(newCount, oldCount);
    speed = (double)delta * 48 / 100*pow(10, 3);

    return speed;
}

long returnOldCount(long newCount, long oldCount) {
    return oldCount = newCount;
}

long QEMReading(long count) {
    newReading = oldReading;
    newReading = inputA1 * 2 + inputB1;
    outVal = QEM[oldReading * 4 + newReading];

    switch (outVal) {
        case -1: {
            count--;
            break;
        }
        case 1: {
            count++;
        }
    }
    return count;
}

long deltaCount(long newCount, long oldCount) {
    long delta = newCount - oldCount;
    return delta;
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