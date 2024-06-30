#ifndef RECEIVE_H
#define RECEIVE_H
#include "usb_protocol.h"
#define RECEIVE_ID 0x2
#define RECEIVE_ZERO_PADDING 0

#define START_OFFSET 24
#define GRADIENT 8

PacketInfo transform_receive_data(uint32_t speed_motor_left_rpm, uint32_t speed_motor_right_rpm) {
    PacketInfo result;
    result.start = START;  // Preset start marker
    result.id = RECEIVE_ID;  // Set packet ID for receive data
    for (int i = 0; i < MAX_DATA_SIZE/2; i++) {
        result.data[i] = (speed_motor_left_rpm >> (START_OFFSET - GRADIENT * i)) & 0xFF;
        result.data[i + MAX_DATA_SIZE/2] = (speed_motor_right_rpm >> (START_OFFSET - GRADIENT * i)) & 0xFF;
    }
    result.crc = gen_crc16(result.data, MAX_DATA_SIZE);  // Compute CRC16 checksum for data integrity
    return result;
}
#endif