#ifndef RECEIVE_H
#define RECEIVE_H

#include "usb_protocol.h"

#define MSG_COMMAND_ID 0x2

PacketInfo msg_command_encode(uint32_t speed_motor_left_rpm, uint32_t speed_motor_right_rpm) {
    PacketInfo result;
    result.start = START;
    result.id = MSG_COMMAND_ID;
    
    const int speed_size_bytes = 4;
    const int byte_size = 8;
    for (int i = 0; i < speed_size_bytes; i++) {
        result.data[i] = (speed_motor_left_rpm >> (byte_size * i)) & 0xFF;
        result.data[i + speed_size_bytes] = (speed_motor_right_rpm >> (byte_size * i)) & 0xFF;
    }

    result.crc = gen_crc16(result.data, MAX_DATA_SIZE);

    return result;
}

void msg_command_decode(PacketInfo packet, uint32_t* speed_motor_left_rpm, uint32_t* speed_motor_right_rpm) {
    *speed_motor_left_rpm = packet.data[0] | (packet.data[1] << 8) | (packet.data[2] << 16) | (packet.data[3] << 24);
    *speed_motor_right_rpm = packet.data[4] | (packet.data[5] << 8) | (packet.data[6] << 16) | (packet.data[7] << 24);
}

#endif