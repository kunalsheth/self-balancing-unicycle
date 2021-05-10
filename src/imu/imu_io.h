//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_IMU_IO_H
#define UNICYCLE_IMU_IO_H

#include <SPI.h>
#include <Arduino.h>
#include "smart_delay.h"

extern "C" {
#include "imu.h"
};

#define IMU_CALIBRATION_MS 5000
#define IMU_DELAY_US  255000

#define IMU_CS 0
#define IMU_MOSI 2
#define IMU_MISO 3
#define IMU_SCK 1
#define IMU_INT 24
#define IMU_INT_VECT 6
#define IMU_RST 4

bool imuIoSetup(uint32_t periodUs);

void imuIoService();

bool imuIoGetReading(ImuState *r);

uint16_t parseCommandReport();

uint16_t parseInputReport();

uint16_t getQ1(uint16_t recordID);

uint16_t getQ2(uint16_t recordID);

uint16_t getQ3(uint16_t recordID);

float getResolution(uint16_t recordID);

float getRange(uint16_t recordID);

uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);

void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);

bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

uint8_t resetReason();

float qToFloat(int16_t fixedPointValue, uint8_t qPoint);

void setFeatureCommand(uint8_t reportID, uint32_t periodUs);

void setFeatureCommand(uint8_t reportID, uint32_t periodUs, uint32_t specificConfig);

void sendCommand(uint8_t command);

void sendCalibrateCommand();

bool calibrationComplete();

bool calibrationSuccessful();

void saveCalibration();

bool waitForSPI();

bool receivePacket();

bool sendPacket(uint8_t channelNumber, uint8_t dataLength);

void printPacket();

void printHeader();


#endif //UNICYCLE_IMU_IO_H
