#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection" // disable "unused variable" code insight

#include <Arduino.h>

#ifndef DATAGRAM_SPEC_H
#define DATAGRAM_SPEC_H

static_assert(sizeof(float) == 4,
              "In order for the decoder to be able to cast floats to uint32 safely, sizeof(float) should return 4");

typedef union {
    float fl;
    uint32_t uint32;
} float_cast;

enum class DatagramPayloadType {
    TELEMETRY = 0x00, EVENT = 0x01, ROCKET_PAYLOAD = 0x02
};


// XBee API mode

static constexpr uint8_t XBEE_START = 0x7e;
static constexpr uint8_t XBEE_ESCAPE = 0x7d;
static constexpr uint8_t XBEE_TX_FRAME = 0x10;
static constexpr uint8_t XBEE_FRAME_OPTIONS[] = {
        XBEE_TX_FRAME,  // Frame type
        0x00,           // Frame ID
        0x00,           // dest address MSB
        0x13,
        0xa2,
        0x00,
        0x41,
        0x4f,
        0x9e,
        0x54,           // dest address LSB
        0xff,           // reserved
        0xfe,           // reserved
        0x00,           // Broadcast radius (0 = max)
        0x43};          // Transmit options (disable ACK and Route discovery)

static constexpr uint8_t XBEE_FRAME_OPTIONS_CRC = 0x87; // Needs to be adjusted if the options are changed!

static constexpr uint8_t HEADER_PREAMBLE_FLAG = 0x55;
static constexpr uint8_t CONTROL_FLAG = 0xFF;

// Field sizes in bytes
static constexpr size_t SEQUENCE_NUMBER_SIZE = sizeof(uint32_t);
static constexpr size_t PAYLOAD_TYPE_SIZE = sizeof(uint8_t);

// Sections sizes in bytes
static constexpr size_t PREAMBLE_SIZE = 4;
static constexpr size_t HEADER_SIZE = SEQUENCE_NUMBER_SIZE + PAYLOAD_TYPE_SIZE;
static constexpr size_t CONTROL_FLAG_SIZE = 1;
static constexpr size_t CHECKSUM_SIZE = 2;

#endif //DATAGRAM_SPEC_H

#pragma clang diagnostic pop