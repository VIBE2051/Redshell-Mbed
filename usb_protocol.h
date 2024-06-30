#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H
#include <stdlib.h>
#include <stdint.h>

/* PACKET INTEGER FORMAT
    |START_BIT|ID|PACKET|CRC|
      where: 
        START_BIT is 1 byte
        ID is 1 byte
        PACKET is 8 bytes
        CRC is 2 bytes
*/

#define START 0x66 // Execute Order 66 :D
#define CRC16 0x8005
#define MAX_DATA_SIZE 8

#define FIRST_BYTE_SHIFT (3 * 8)
#define SECOND_BYTE_SHIFT (2 * 8)
#define THIRD_BYTE_SHIFT (1 * 8)

typedef struct {
    uint32_t ff_bytes; // First Four Bytes 
    uint32_t sf_bytes; // Second Four Bytes 
    uint32_t lf_bytes; // Last Four Bytes 
}Packet;


typedef struct {
    uint8_t start; 
    uint8_t id;
    uint8_t data[MAX_DATA_SIZE];
    uint16_t crc;
}PacketInfo;

uint16_t gen_crc16(const uint8_t *data, uint16_t size){
    uint16_t out = 0;
    int bits_read = 0, bit_flag;
    /* Sanity check: */
    if(data == NULL){
        return 0;
    }
    while(size > 0){
        bit_flag = out >> 15;
        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits
        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7){
            bits_read = 0;
            data++;
            size--;
        }
        /* Cycle check: */
        if(bit_flag){
            out ^= CRC16;
        }
    }
    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }
    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }
    return crc;
}

Packet info_to_packet(PacketInfo packet_info) {
    Packet packet;
    packet.ff_bytes = (packet_info.start << FIRST_BYTE_SHIFT) \
    | (packet_info.id << SECOND_BYTE_SHIFT) | (packet_info.data[0] << THIRD_BYTE_SHIFT) | (packet_info.data[1]);
   
    packet.sf_bytes = (packet_info.data[2] << FIRST_BYTE_SHIFT) \
    | (packet_info.data[3] << SECOND_BYTE_SHIFT) | (packet_info.data[4] << THIRD_BYTE_SHIFT) | (packet_info.data[5]);
   
    packet.lf_bytes = (packet_info.data[6] << FIRST_BYTE_SHIFT) \
    | (packet_info.data[7] << SECOND_BYTE_SHIFT) | packet_info.crc;

    return packet;
}
#endif 