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

void serialize(PacketInfo packet, uint8_t* serial_packet) {
    serial_packet[0] = packet.start;
    serial_packet[1] = packet.id;

    for (int i = 0; i < MAX_DATA_SIZE; i++)
    {
        serial_packet[2 + i] = packet.data[i];
    }

    serial_packet[2 + MAX_DATA_SIZE] = (packet.crc & 0xFF);
    serial_packet[2 + MAX_DATA_SIZE + 1] = ((packet.crc >> 8) & 0xFF);
}

void deserialize(PacketInfo packet, uint8_t* serial_packet) {
    packet.start = serial_packet[0];
    packet.id = serial_packet[1];

    for (int i = 0; i < MAX_DATA_SIZE; i++)
    {
        packet.data[i] = serial_packet[2 + i];
    }

    packet.crc = serial_packet[2 + MAX_DATA_SIZE] | (packet.crc << 8);
}
#endif 