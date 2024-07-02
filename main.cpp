/*
 * Group Project by: 
 * Francisco Jorquera,
 * Timothy Mead and
 * Ganesh Srinivasa
 * 
 */
 
#include "ADXL345.h"
#include "encoder.h"
#include "redshell_messages/imu.h"
#include "redshell_messages/power.h"
#include "redshell_messages/command.h"
#include "mbed.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

/////Pin allocations made specifically for NUCLEO-L476RG///////
InterruptIn inputA1(PA_9);
InterruptIn inputB1(PA_8);
InterruptIn inputA2(PB_5);
InterruptIn inputB2(PA_10);

PwmOut enableA(PB_10);
PwmOut enableB(PB_4);
DigitalOut IN1(PA_11);
DigitalOut IN2(PC_6);
DigitalOut IN3(PC_5);
DigitalOut IN4(PA_12);

ADXL345 accelerometer(PA_7, PA_6, PA_5, PC_7); // mosi, miso, sck, cs
BufferedSerial pc(USBTX, USBRX);



// Variables for encoder state
volatile uint32_t position1_ticks = 0;
volatile uint32_t position2_ticks = 0;

const int BUFFER_SIZE = 256;
char buffer[BUFFER_SIZE];
int buffer_index = 0;

void updateMotorPosition1();
void updateMotorPosition2();
void updateSpeed(double &speed1_rpm, double &speed2_rpm);
void setSpeedM1M2(char input[]);
void readInput();
int stringToInt(char data[]);
void motorA(float duty, int dir);
void motorB(float duty, int dir);

int main() {
    pc.set_blocking(true); // Set non-blocking mode
    // Initialising Button pull direction
    inputA1.mode(PullDown);
    inputB1.mode(PullDown);
    inputA2.mode(PullDown);
    inputB2.mode(PullDown);

    // Attach the updateEncoder function to be called on the rising edge of inputA
    inputA1.rise(&updateMotorPosition1);
    inputA2.rise(&updateMotorPosition2);
    
    // Initialising pwm pins
    enableA.period(0.001f); //Setting a period of 100ms
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

        motorA(1.0, 1);
        motorB(1.0, 1);

        ThisThread::sleep_for(500ms);
        //printf("here\n");

        double speed1_rpm, speed2_rpm;
        updateSpeed(speed1_rpm, speed2_rpm);
        //printf("Enter a string (press Enter to send):\n");
        // imuData = transform_imu_data(readings[0], readings[1], readings[2]);
        // encoderSpeed = transform_encoder_data(speed1_rpm, speed2_rpm);
        // imuPack = info_to_packet(imuData);
        // encoderPack = info_to_packet(encoderSpeed);
        //readInput();

        PacketInfo imuPacket = msg_imu_encode(readings[0], readings[1], readings[2]);
        uint8_t imuData[12];
        serialize(imuPacket, imuData);

        for (int i = 0; i < sizeof(imuData); i++) {
            std::printf("%02X", imuData[i]);
        }

        //std::printf("%i, %i, %i\n", (int16_t)readings[0], (int16_t)readings[1], (int16_t)readings[2]);
        //std::printf("Speed of Motor 1: %f, Speed of Motor 2: %f\n", speed1_rpm, speed2_rpm);
    }
}

void updateMotorPosition1() {
    if (inputA1.read() != inputB1.read()) {
        position1_ticks++;
    } else {
        position1_ticks--;
    }
}

void updateMotorPosition2() {
    if (inputA2.read() != inputB2.read()) {
        position2_ticks++;
    } else {
        position2_ticks--;
    }
}

void updateSpeed(double &speed1_rpm, double &speed2_rpm) {
    static int32_t lastPosition1_ticks = 0;
    static int32_t lastPosition2_ticks = 0;
    static int32_t lastTime_us = 0;

    static constexpr double us_to_s = 1e-6;
    const int32_t currentTime_us = us_ticker_read();
    const double timeDelta_s = static_cast<double>(currentTime_us - lastTime_us) * us_to_s;

    static constexpr int16_t ticksPerCycle = 48;
    const double positionDelta1_cycles = static_cast<double>(position1_ticks - lastPosition1_ticks) / ticksPerCycle;
    const double positionDelta2_cycles = static_cast<double>(position2_ticks - lastPosition2_ticks) / ticksPerCycle;

    static constexpr double rps_to_rpm = 1.0 / 60.0;
    speed1_rpm = (positionDelta1_cycles / timeDelta_s) * rps_to_rpm;
    speed2_rpm = (positionDelta2_cycles / timeDelta_s) * rps_to_rpm;

    lastPosition1_ticks = position1_ticks;
    lastPosition2_ticks = position2_ticks;
    lastTime_us = currentTime_us;
}

void readInput() {
    while (!pc.readable()) {
        char c;
        if (pc.read(&c, 1)) {
            // Check for end of line
            if (c == '\n' || c == '\r') {
                buffer[buffer_index] = '\0'; // Null-terminate the string
                printf("Received string: %s\n", buffer);
                setSpeedM1M2(buffer);
                buffer_index = 0; // Reset buffer index
            } else {
                if (buffer_index < BUFFER_SIZE - 1) {
                    buffer[buffer_index++] = c; // Add character to buffer
                } else {
                    // Buffer overflow handling
                    printf("Buffer overflow, clearing buffer\n");
                    buffer_index = 0;
                }
            }
        }
    }
}

void setSpeedM1M2(char input[]) {
    float duty = input[3];
    if (input[0] == '=') {
        if (input[1] == 'L') {
            if (input[2] == 'F') {
                motorA(stringToInt(input), 1);
            } else if (input[2] == 'B') {
                motorA(stringToInt(input), -1);
            }
        } else if (input[1] == 'R') {
            if (input[2] == 'F') {
                motorB(stringToInt(input), 1);
            } else if (input[2] == 'B') {
                motorB(stringToInt(input), -1);
            }
        }
    }
}

int stringToInt(char data[]) {
    char *ptr;
    char num[3];
    num[0] = data[2];
    num[1] = data[3];
    num[2] = '0';
    long int value = strtol(num, &ptr, 10);
    return value; 
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