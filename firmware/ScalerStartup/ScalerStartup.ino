#include <Arduino.h>

#define IR_RECEIVE_PIN 3
#define IR_SEND_PIN 2

#define MAX_FILES 7
#define REMOTE_ADDR 0
#define KEY_POWER 0x0
#define KEY_ENTER 0x10
#define KEY_RIGHT 0x12

#include "TinyIRReceiver.hpp"
#include "TinyIRSender.hpp"

void sendMark_raw(uint8_t pin, uint16_t us);
void sendNEC_raw(uint8_t pin, uint16_t addr, uint16_t cmd);

void sendRemote(uint8_t cmd, uint16_t ms) {
    sendNEC_raw(IR_SEND_PIN, REMOTE_ADDR, cmd);
    delay(ms);
}

void setup() {
    Serial.begin(115200);

    pinModeFast(IR_SEND_PIN, OUTPUT);
    digitalWriteFast(IR_SEND_PIN, HIGH);

    initPCIInterruptForTinyReceiver();

    delay(3000);

    sendRemote(KEY_POWER, 6000);
    sendRemote(KEY_RIGHT, 300);
    sendRemote(KEY_RIGHT, 300);
    sendRemote(KEY_ENTER, 1000);
    sendRemote(KEY_ENTER, 1000);

    randomSeed(analogRead(0));
    uint16_t rnd = random(1, MAX_FILES);

    for (uint8_t i = 0; i < rnd; i++) {
        sendRemote(KEY_RIGHT, 500);
    }

    sendRemote(KEY_ENTER, 0);
}

void loop() {
}

// func
void handleReceivedTinyIRData(uint8_t addr, uint8_t cmd, uint8_t flags) {
    Serial.print(addr, HEX);
    Serial.print(',');
    Serial.println(cmd, HEX);
}

void sendMark_raw(uint8_t pin, uint16_t us) {
    digitalWriteFast(pin, LOW);
    delayMicroseconds(us);
    digitalWriteFast(pin, HIGH);
}

void sendNEC_raw(uint8_t pin, uint16_t addr, uint16_t cmd) {
    noInterrupts();
    sendMark_raw(pin, NEC_HEADER_MARK);
    delayMicroseconds(NEC_HEADER_SPACE);

    LongUnion tData;
    if (addr > 0xFF) {
        tData.UWord.LowWord = addr;
    } else {
        tData.UByte.LowByte = addr;  // LSB first
        tData.UByte.MidLowByte = ~addr;

        tData.UByte.MidLowByte &= ~0b01000000;  // kostyl
        
    }
    if (cmd > 0xFF) {
        tData.UWord.HighWord = cmd;
    } else {
        tData.UByte.MidHighByte = cmd;
        tData.UByte.HighByte = ~cmd;  // LSB first
    }

    // Send data
    for (uint_fast8_t i = 0; i < NEC_BITS; ++i) {
        sendMark_raw(pin, NEC_BIT_MARK);  // constant mark length
        if (tData.ULong & 1) {
            delayMicroseconds(NEC_ONE_SPACE);
        } else {
            delayMicroseconds(NEC_ZERO_SPACE);
        }
        tData.ULong >>= 1;  // shift command for next bit
    }

    sendMark_raw(pin, NEC_BIT_MARK);
    interrupts();
}