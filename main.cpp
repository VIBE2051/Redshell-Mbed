/*
 * Group Project by: 
 * Francisco Jorquera,
 * Timothy Mead and
 * Ganesh Srinivasa
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

Thread IMU, ENCODER, COMMAND, TIMEOUT;
Mutex write_mutex;

// Variables for encoder state
volatile uint32_t position1_ticks = 0;
volatile uint32_t position2_ticks = 0;

static constexpr int8_t sign(const float x) {
    return (0 < x) - (x < 0);
}

void set_motor_val(PwmOut& pin, DigitalOut& in_front, DigitalOut& in_back, const float duty, const int8_t dir)
{
    switch (dir) {
        case 1: {
            in_front = 1;
            in_back = 0;
            pin.write(duty);
            break;
        }
        case -1: {
            in_front = 0;
            in_back = 1;
            pin.write(duty);
            break;
        }
        default: {
            in_front = 0;
            in_back = 0;
            pin.write(0.0);
            break;
        }
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

void thread_imuSend() {
    int readings[3] = {0, 0, 0};
    while (true) {
        accelerometer.getOutput(readings);

        PacketInfo imuPacket = msg_imu_encode(readings[0], readings[1], readings[2]);
        uint8_t imuData[REDSHELL_MESSAGE_SIZE];
        serialize(imuPacket, imuData);
        write_mutex.lock();
        pc.write(imuData, sizeof(imuData));
        write_mutex.unlock();
        ThisThread::sleep_for(100ms);
    }
}

void thread_encoderSend() {
    double speed1_rpm, speed2_rpm;
    while(true) {
        updateSpeed(speed1_rpm, speed2_rpm);
        PacketInfo encoderPacket = msg_encoder_encode(static_cast<int32_t>(speed1_rpm), static_cast<int32_t>(speed2_rpm));
        uint8_t encodeData[REDSHELL_MESSAGE_SIZE];
        serialize(encoderPacket, encodeData);
        write_mutex.lock();
        pc.write(encodeData, sizeof(encodeData));
        write_mutex.unlock();
        ThisThread::sleep_for(100ms);
    }
}

void thread_commandReceive() {
    uint8_t readBuff[1];
    uint8_t cmdData[12];
    uint8_t cmdIndex = 0;
    bool is_reading = false;
    int32_t speed1 = 0;
    int32_t speed2 = 0;
    int32_t last_received_time_us = us_ticker_read();

    while (true) {
        static constexpr double timeout_us = 2e5;
        if ((us_ticker_read() - last_received_time_us) > timeout_us)
        {
            set_motor_val(enableA, IN1, IN2, 0.0, 0);
            set_motor_val(enableB, IN3, IN4, 0.0, 0);
        }
        
        const size_t err = pc.read(readBuff, 1);
        if (err != 1)
        {
            continue;
        }

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
                deserialize(&commandPacket, cmdData);
                msg_command_decode(commandPacket, &speed1, &speed2);

                const float a_speed_magnitude = std::fabs(static_cast<float>(speed1) * 0.01);
                const int8_t a_speed_dir = -sign(speed1);
                set_motor_val(enableA, IN1, IN2, a_speed_magnitude, a_speed_dir);

                const float b_speed_magnitude = std::fabs(static_cast<float>(speed2) * 0.01);
                const int8_t b_speed_dir = -sign(speed2);
                set_motor_val(enableB, IN3, IN4, b_speed_magnitude, b_speed_dir);

                last_received_time_us = us_ticker_read();
            }
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
    // TIMEOUT.start(thread_commandTimeout);

    while (true) {
        ThisThread::sleep_for(100ms);
    }
}
