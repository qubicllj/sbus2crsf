#include "crsf.h" // NOLINT

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

void crsfPreparePacket(uint8_t packet[], int channels[])
{

    // static int output[CRSF_MAX_CHANNEL] = {0};
    // const uint8_t crc = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3);
    // /*
    //  * Map 1000-2000 with middle at 1500 chanel values to
    //  * 173-1811 with middle at 992 S.BUS protocol requires
    //  */
    // for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
    //     output[i] = channels[i];
    // }

    // packet[0] = UART_SYNC; //Header
    packet[0] = ADDR_MODULE; // Header
    packet[1] = 24;          // length of type (24) + payload + crc
    packet[2] = TYPE_CHANNELS;
    packet[3] = (uint8_t)(channels[0] & 0x07FF);
    packet[4] = (uint8_t)((channels[0] & 0x07FF) >> 8 | (channels[1] & 0x07FF) << 3);
    packet[5] = (uint8_t)((channels[1] & 0x07FF) >> 5 | (channels[2] & 0x07FF) << 6);
    packet[6] = (uint8_t)((channels[2] & 0x07FF) >> 2);
    packet[7] = (uint8_t)((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
    packet[8] = (uint8_t)((channels[3] & 0x07FF) >> 7 | (channels[4] & 0x07FF) << 4);
    packet[9] = (uint8_t)((channels[4] & 0x07FF) >> 4 | (channels[5] & 0x07FF) << 7);
    packet[10] = (uint8_t)((channels[5] & 0x07FF) >> 1);
    packet[11] = (uint8_t)((channels[5] & 0x07FF) >> 9 | (channels[6] & 0x07FF) << 2);
    packet[12] = (uint8_t)((channels[6] & 0x07FF) >> 6 | (channels[7] & 0x07FF) << 5);
    packet[13] = (uint8_t)((channels[7] & 0x07FF) >> 3);
    packet[14] = (uint8_t)((channels[8] & 0x07FF));
    packet[15] = (uint8_t)((channels[8] & 0x07FF) >> 8 | (channels[9] & 0x07FF) << 3);
    packet[16] = (uint8_t)((channels[9] & 0x07FF) >> 5 | (channels[10] & 0x07FF) << 6);
    packet[17] = (uint8_t)((channels[10] & 0x07FF) >> 2);
    packet[18] = (uint8_t)((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
    packet[19] = (uint8_t)((channels[11] & 0x07FF) >> 7 | (channels[12] & 0x07FF) << 4);
    packet[20] = (uint8_t)((channels[12] & 0x07FF) >> 4 | (channels[13] & 0x07FF) << 7);
    packet[21] = (uint8_t)((channels[13] & 0x07FF) >> 1);
    packet[22] = (uint8_t)((channels[13] & 0x07FF) >> 9 | (channels[14] & 0x07FF) << 2);
    packet[23] = (uint8_t)((channels[14] & 0x07FF) >> 6 | (channels[15] & 0x07FF) << 5);
    packet[24] = (uint8_t)((channels[15] & 0x07FF) >> 3);

    packet[25] = crsf_crc8(&packet[2], CRSF_PACKET_SIZE - 3); // CRC
}