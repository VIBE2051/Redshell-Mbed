#ifndef IMU_H
#define IMU_H

#include "usb_protocol.h"

#define MSG_IMU_ID 0x0

PacketInfo msg_imu_encode(uint16_t x, uint16_t y, uint16_t z) {
    PacketInfo result;
    result.start = START;  // Preset start marker
    result.id = MSG_IMU_ID;  // Set packet ID for imu data
    result.data[0] = (x >> MAX_DATA_SIZE) & 0xFF;
    result.data[1] = (x & 0xFF);
    result.data[2] = (y >> MAX_DATA_SIZE) & 0xFF;
    result.data[3] = (y & 0xFF);
    result.data[4] = (z >> MAX_DATA_SIZE) & 0xFF;
    result.data[5] = (z & 0xFF);
    result.data[6] = 0;
    result.data[7] = 0;
    result.crc = gen_crc16(result.data, MAX_DATA_SIZE);  // Compute CRC16 checksum for data integrity
    return result;
}

void msg_imu_decode(PacketInfo packet, uint16_t* x, uint16_t* y, uint16_t* z) {
    *x = packet.data[1] | (packet.data[0] << MAX_DATA_SIZE);
    *y = packet.data[3] | (packet.data[2] << MAX_DATA_SIZE);
    *z = packet.data[5] | (packet.data[4] << MAX_DATA_SIZE);
}

#endif
