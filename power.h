#ifndef POWER_H
#define POWER_H
#include "usb_protocol.h"
#define POWER_ID 0x3
#define POWER_ZERO_PADDING 32

#define START_OFFSET 24
#define GRADIENT 8

PacketInfo transform_power_data(uint32_t voltage) {
    PacketInfo result;
    result.start = START;  // Preset start marker
    result.id = POWER_ID;  // Set packet ID for power data
    for (int i = 0; i < MAX_DATA_SIZE/2; i++) {
        result.data[i] = (voltage >> (START_OFFSET - GRADIENT * i)) & 0xFF;
        result.data[i + MAX_DATA_SIZE/2] = 0;
    }
    result.crc = gen_crc16(result.data, MAX_DATA_SIZE);  // Compute CRC16 checksum for data integrity
    return result;
}
#endif