/*
 * Group Project by: 
 * Francisco Jorquera,
 * Timothy Mead and
 * Ganesh Srinivasa
 * 
 */
 
#include "ADXL345.h"
#include "redshell-messages/include/redshell/encoder.h"
#include "redshell-messages/include/redshell/imu.h"
#include "redshell-messages/include/redshell/power.h"
#include "redshell-messages/include/redshell/command.h"

#include "mbed.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>

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

Thread IMU, ENCODER, COMMAND;

Mutex write_mutex;

// Variables for encoder state
volatile uint32_t position1_ticks = 0;
volatile uint32_t position2_ticks = 0;

const int BUFFER_SIZE = 256;
char buffer[BUFFER_SIZE];
int buffer_index = 0;

void thread_imuSend();
void thread_encoderSend();
void thread_commandReceive();
void updateMotorPosition1();
void updateMotorPosition2();
void updateSpeed(double &speed1_rpm, double &speed2_rpm);
void setSpeedM1M2(char input[]);
void readInput();
int stringToInt(char data[]);
void motorA(float duty, int dir);
void motorB(float duty, int dir);

int main() {
    pc.set_blocking(false); // Set non-blocking mode
    pc.set_baud(19200);
    pc.set_format(
        8,                      // Number of bits (5, 6, 7, 8)
        BufferedSerial::None,   // Parity (None, Odd, Even)
        1                       // Number of stop bits (1 or 2)
    );

    // Initialising Button pull direction
    inputA1.mode(PullDown);
    inputB1.mode(PullDown);
    inputA2.mode(PullDown);
    inputB2.mode(PullDown);

    // Attach the updateEncoder function to be called on the rising edge of inputA
    inputA1.rise(&updateMotorPosition1);
    inputA2.rise(&updateMotorPosition2);
    
    // Initialising pwm pins
    enableA.period(0.001f); //Setting a period of 1ms
    enableB.period(0.001f);
    
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

    printf("Starting Threads...");
    IMU.start(thread_imuSend);
    ENCODER.start(thread_encoderSend);
    COMMAND.start(thread_commandReceive);

    while (1) {
        ThisThread::sleep_for(100ms);
    }
}

void thread_imuSend() {
    int readings[3] = {0, 0, 0};
    while (true) {
        accelerometer.getOutput(readings);

        PacketInfo imuPacket = msg_imu_encode(readings[0], readings[1], readings[2]);
        uint8_t imuData[REDSHELL_MESSAGE_SIZE];
        serialize(imuPacket, imuData);
        // write_mutex.lock();
        pc.write(imuData, sizeof(imuData));
        // write_mutex.unlock();
        ThisThread::sleep_for(100ms);
    }
}

void thread_encoderSend() {
    double speed1_rpm, speed2_rpm;
    while(true) {
        updateSpeed(speed1_rpm, speed2_rpm);
        PacketInfo encoderPacket = msg_encoder_encode(speed2_rpm, speed1_rpm);
        uint8_t encodeData[REDSHELL_MESSAGE_SIZE];
        serialize(encoderPacket, encodeData);
        // write_mutex.lock();
        pc.write(encodeData, sizeof(encodeData));
        // write_mutex.unlock();
        ThisThread::sleep_for(100ms);
    }
}

static constexpr int32_t sign(const float x) {
    return (0 < x) - (x < 0);
}

static constexpr float clamp(const float value, const  float low, const  float high) {
    return std::max(std::min(value, high), low);
}

void thread_commandReceive() {
    int32_t speed1, speed2;
    char readBuff[1];
    char cmdData[12];
    uint8_t cmdIndex = 0;
    bool is_reading = false;

    int32_t last_received_time_us = us_ticker_read();

    while (true) {
        pc.read(readBuff, 1);

        if (readBuff[0] == REDSHELL_START_BYTE)
        {
            cmdIndex = 0;
            is_reading = true;
        }

        if (is_reading)
        {
            cmdData[cmdIndex] = readBuff[0];
            cmdIndex++;

            if (cmdIndex >= REDSHELL_MESSAGE_SIZE)
            {
                is_reading = false;

                PacketInfo commandPacket;
                deserialize(&commandPacket, (uint8_t*)cmdData);
                msg_command_decode(commandPacket, &speed1, &speed2);

                const float a_speed_magnitude = clamp(std::abs(static_cast<float>(speed1) * 0.01), 0.0, 1.0);
                const float a_speed_dir = sign(speed1);
                motorA(a_speed_magnitude, a_speed_dir);

                const float b_speed_magnitude = clamp(std::abs(static_cast<float>(speed2) * 0.01), 0.0, 1.0);
                const float b_speed_dir = sign(speed2);
                motorB(b_speed_magnitude, b_speed_dir);

                last_received_time_us = us_ticker_read();
            }
        }

        static constexpr double timeout_us = 5e5;
        if ((us_ticker_read() - last_received_time_us) > timeout_us)
        {
            motorA(0.0, 0);
            motorB(0.0, 0);
        }
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

// void readInput() {
//     while (!pc.readable()) {
//         char c;
//         if (pc.read(&c, 1)) {
//             // Check for end of line
//             if (c == '\n' || c == '\r') {
//                 buffer[buffer_index] = '\0'; // Null-terminate the string
//                 printf("Received string: %s\n", buffer);
//                 setSpeedM1M2(buffer);
//                 buffer_index = 0; // Reset buffer index
//             } else {
//                 if (buffer_index < BUFFER_SIZE - 1) {
//                     buffer[buffer_index++] = c; // Add character to buffer
//                 } else {
//                     // Buffer overflow handling
//                     printf("Buffer overflow, clearing buffer\n");
//                     buffer_index = 0;
//                 }
//             }
//         }
//     }
// }

// void setSpeedM1M2(char input[]) {
//     float duty = input[3];
//     if (input[0] == '=') {
//         if (input[1] == 'L') {
//             if (input[2] == 'F') {
//                 motorA(stringToInt(input), 1);
//             } else if (input[2] == 'B') {
//                 motorA(stringToInt(input), -1);
//             }
//         } else if (input[1] == 'R') {
//             if (input[2] == 'F') {
//                 motorB(stringToInt(input), 1);
//             } else if (input[2] == 'B') {
//                 motorB(stringToInt(input), -1);
//             }
//         }
//     }
// }

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
            default: {
                IN1 = 0;
                IN2 = 0;
                enableA.write(0.0);
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
            default: {
                IN3 = 0;
                IN4 = 0;
                enableB.write(0.0);
                break;
            }
        }
    }
}