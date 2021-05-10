#include "imu_io.h"

//Registers
#define CHANNEL_COMMAND 0
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO 5

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

volatile bool isConfigured = false;

volatile bool isCalibrating = false;
volatile bool isAccurate = false;

volatile uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
volatile uint8_t shtpData[MAX_PACKET_SIZE];
volatile uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
volatile uint8_t commandSequenceNumber = 0;                //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
volatile uint32_t metaData[MAX_METADATA_SIZE];            //There is more than 10 words in a metadata record but we'll stop at Q point 3

int16_t rotVec_Q1 = 14;
int16_t rotVecAccuracy_Q1 = 12;
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;

void imuInterrupt() {
    if (isConfigured) imuIoService();
}

const SPISettings spiSettings = SPISettings(3000000, MSBFIRST, SPI_MODE3);

bool imuIoSetup(uint32_t periodUs) {
    pinMode(IMU_CS, OUTPUT);
    pinMode(IMU_INT, INPUT_PULLUP);
    pinMode(IMU_RST, OUTPUT);

    attachInterrupt(IMU_INT_VECT, imuInterrupt, FALLING);
    SPI.usingInterrupt(IMU_INT_VECT);

    digitalWrite(IMU_CS, HIGH); //Deselect BNO080

    //Configure the BNO080 for SPI communication
    digitalWrite(IMU_RST, LOW);   //Reset BNO080
    smartDelayUs(10000);       //Min length not specified in datasheet?
    digitalWrite(IMU_RST, HIGH);  //Bring out of reset

    //Wait for first assertion of INT before using WAK pin. Can take ~104ms
    waitForSPI();

    //if(wakeBNO080() == false) //Bring IC out of sleep after reset
    //  Serial.println("BNO080 did not wake up");

    SPI.begin(); //Turn on SPI hardware

    //At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
    //host. It must not send any other data until this step is complete.
    //When BNO080 first boots it broadcasts big startup packet
    //Read it and dump it
    waitForSPI(); //Wait for assertion of INT before reading advert message.
    receivePacket();

    //The BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)
    //Read it and dump it
    waitForSPI(); //Wait for assertion of INT before reading Init response
    receivePacket();

    //Check communication with device
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0;                              //Reserved

    //Transmit packet on channel 2, 2 bytes
    sendPacket(CHANNEL_CONTROL, 2);

    //Now we wait for response
    waitForSPI();
    if (!receivePacket()) return false; //Something went wrong

    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
        Serial.print(F("SW Version Major: 0x"));
        Serial.print(shtpData[2], HEX);
        Serial.print(F(" SW Version Minor: 0x"));
        Serial.print(shtpData[3], HEX);
        uint32_t SW_Part_Number = ((uint32_t) shtpData[7] << 24) | ((uint32_t) shtpData[6] << 16) |
                                  ((uint32_t) shtpData[5] << 8) | ((uint32_t) shtpData[4]);
        Serial.print(F(" SW Part Number: 0x"));
        Serial.print(SW_Part_Number, HEX);
        uint32_t SW_Build_Number = ((uint32_t) shtpData[11] << 24) | ((uint32_t) shtpData[10] << 16) |
                                   ((uint32_t) shtpData[9] << 8) | ((uint32_t) shtpData[8]);
        Serial.print(F(" SW Build Number: 0x"));
        Serial.print(SW_Build_Number, HEX);
        uint16_t SW_Version_Patch = ((uint16_t) shtpData[13] << 8) | ((uint16_t) shtpData[12]);
        Serial.print(F(" SW Version Patch: 0x"));
        Serial.println(SW_Version_Patch, HEX);
    }

//    if ((int16_t) getQ1(FRS_RECORDID_ACCELEROMETER) != accelerometer_Q1) {
//        Serial.printf(
//                F("[WARNING] (getQ1(FRS_RECORDID_ACCELEROMETER) == %d) != (accelerometer_Q1 == %d)\n"),
//                getQ1(FRS_RECORDID_ACCELEROMETER), accelerometer_Q1
//        );
//    }
//
//    if ((int16_t) getQ1(FRS_RECORDID_GYROSCOPE_CALIBRATED) != accelerometer_Q1) {
//        Serial.printf(
//                F("[WARNING] (getQ1(FRS_RECORDID_GYROSCOPE_CALIBRATED) == %d) != (gyro_Q1 == %d)\n"),
//                getQ1(FRS_RECORDID_GYROSCOPE_CALIBRATED), gyro_Q1
//        );
//    }
//
//    if ((int16_t) getQ1(FRS_RECORDID_ROTATION_VECTOR) != rotVec_Q1) {
//        Serial.printf(
//                F("[WARNING] (getQ1(FRS_RECORDID_ROTATION_VECTOR) == %d) != (rotVec_Q1 == %d)\n"),
//                getQ1(FRS_RECORDID_ROTATION_VECTOR), rotVec_Q1
//        );
//    }

    setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, periodUs);
//    setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, periodUs);
//    setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, periodUs);
//    setFeatureCommand(SENSOR_REPORTID_GRAVITY, periodUs);

    isConfigured = true;
    return true;
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
uint16_t parseCommandReport() {
    if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE) {
        //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        uint8_t command = shtpData[2]; //This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE) {
            isCalibrating = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
        }
        return shtpData[0];
    } else {
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
    }

    //TODO additional feature reports may be strung together. Parse them all.
    return 0;
}

volatile uint16_t rawRotVecI, rawRotVecJ, rawRotVecK, rawRotVecReal, rawRotVecAccuracy;
volatile uint32_t rotVecTimestamp;
volatile bool readRotVec = false;

volatile uint16_t rawGyroX, rawGyroY, rawGyroZ;
volatile uint32_t gyroTimestamp;
volatile bool readGyro = false;

volatile uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ;
volatile uint32_t linAccelTimestamp;
volatile bool readLinAccel = false;

volatile uint16_t rawGravityX, rawGravityY, rawGravityZ;
volatile uint32_t gravityTimestamp;
volatile bool readGravity = false;

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
uint16_t parseInputReport() {
    //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t) shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
    //Ignore it for now. TODO catch this as an error and exit

    dataLength -= 4; //Remove the header bytes from the data count

    uint32_t timestamp = ((uint32_t) shtpData[4] << (8 * 3)) | ((uint32_t) shtpData[3] << (8 * 2)) |
                         ((uint32_t) shtpData[2] << (8 * 1)) | ((uint32_t) shtpData[1] << (8 * 0));

    uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
    uint16_t data1 = (uint16_t) shtpData[5 + 5] << 8 | shtpData[5 + 4];
    uint16_t data2 = (uint16_t) shtpData[5 + 7] << 8 | shtpData[5 + 6];
    uint16_t data3 = (uint16_t) shtpData[5 + 9] << 8 | shtpData[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

    if (dataLength - 5 > 9) {
        data4 = (uint16_t) shtpData[5 + 11] << 8 | shtpData[5 + 10];
    }
    if (dataLength - 5 > 11) {
        data5 = (uint16_t) shtpData[5 + 13] << 8 | shtpData[5 + 12];
    }

    //Store these generic values to their proper global variable
    if (shtpData[5] == SENSOR_REPORTID_GRAVITY) {
        isAccurate = status == 3;
        rawGravityX = data1;
        rawGravityY = data2;
        rawGravityZ = data3;
        readGravity = true;
    } else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION) {
        isAccurate = status == 3;
        rawLinAccelX = data1;
        rawLinAccelY = data2;
        rawLinAccelZ = data3;
        gravityTimestamp = timestamp;
        readLinAccel = true;
    } else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE) {
        isAccurate = status == 3;
        rawGyroX = data1;
        rawGyroY = data2;
        rawGyroZ = data3;
        gyroTimestamp = timestamp;
        readGyro = true;
    } else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR) {
        isAccurate = status == 3;
        rawRotVecI = data1;
        rawRotVecJ = data2;
        rawRotVecK = data3;
        rawRotVecReal = data4;
        rawRotVecAccuracy = data5;
        rotVecTimestamp = timestamp;
        readRotVec = true;
    } else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE) {
        uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response
        if (command == COMMAND_ME_CALIBRATE)
            isCalibrating = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
    }

    //TODO additional feature reports may be strung together. Parse them all.
    return shtpData[5];
}

void imuIoService() {
//        If we have an interrupt pin connection available, check if data is available.
//        If int pin is not set, then we'll rely on receivePacket() to timeout
//        See issue 13: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/13
    if (receivePacket()) {
//            Check to see if this packet is a sensor reporting its data to us
        if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP) {
            parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
        } else if (shtpHeader[2] == CHANNEL_CONTROL) {
            parseCommandReport(); //This will update responses to commands, isCalibrating, etc.
        } else if (shtpHeader[2] == CHANNEL_GYRO) {
            parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
        }
    }
}

bool imuIoGetReading(ImuState *r) {
    if (readRotVec) {
        float i = qToFloat(rawRotVecI, rotVec_Q1);
        float j = qToFloat(rawRotVecJ, rotVec_Q1);
        float k = qToFloat(rawRotVecK, rotVec_Q1);
        float real = qToFloat(rawRotVecReal, rotVec_Q1);
        quaternionToEuler(i, j, k, real, &r->angle.x, &r->angle.y, &r->angle.z);
        r->angle.timestampUs = rotVecTimestamp;
        r->angle.accuracy = qToFloat(rawRotVecAccuracy, rotVecAccuracy_Q1);
    }

    if (readGyro) {
        r->angular_velocity.x = qToFloat(rawGyroX, gyro_Q1);
        r->angular_velocity.y = qToFloat(rawGyroY, gyro_Q1);
        r->angular_velocity.z = qToFloat(rawGyroZ, gyro_Q1);
        r->angular_velocity.timestampUs = gyroTimestamp;
        r->angular_velocity.accuracy = -1;
    }

    if (readLinAccel) {
        r->linear_acceleration.x = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
        r->linear_acceleration.y = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
        r->linear_acceleration.z = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
        r->linear_acceleration.timestampUs = linAccelTimestamp;
        r->linear_acceleration.accuracy = -1;
    }

    if (readGravity) {
        r->gravity.x = qToFloat(rawGravityX, accelerometer_Q1);
        r->gravity.y = qToFloat(rawGravityY, accelerometer_Q1);
        r->gravity.z = qToFloat(rawGravityZ, accelerometer_Q1);
        r->gravity.timestampUs = gravityTimestamp;
        r->gravity.accuracy = -1;
    }

    bool changed = readRotVec || readGyro || readLinAccel || readGravity;
    readRotVec = readGyro = readLinAccel = readGravity = false;

    return changed;
}

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
uint16_t getQ1(uint16_t recordID) {
    //Q1 is always the lower 16 bits of word 7
    return readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
uint16_t getQ2(uint16_t recordID) {
    //Q2 is always the upper 16 bits of word 7
    return readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
uint16_t getQ3(uint16_t recordID) {
    //Q3 is always the upper 16 bits of word 8
    return readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float getResolution(uint16_t recordID) {
    //The resolution Q value are 'the same as those used in the sensor's input report'
    //This should be Q1.
    int16_t Q = getQ1(recordID);

    //Resolution is always word 2
    uint32_t value = readFRSword(recordID, 2); //Get word 2

    float resolution = qToFloat(value, Q);

    return resolution;
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float getRange(uint16_t recordID) {
    //The resolution Q value are 'the same as those used in the sensor's input report'
    //This should be Q1.
    int16_t Q = getQ1(recordID);

    //Range is always word 1
    uint32_t value = readFRSword(recordID, 1); //Get word 1

    float range = qToFloat(value, Q);

    return range;
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber) {
    if (readFRSdata(recordID, wordNumber, 1) == true) //Get word number, just one word in length from FRS
        return metaData[0];                          //Return this one word

    return 0; //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize) {
    shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
    shtpData[1] = 0;                            //Reserved
    shtpData[2] = (readOffset >> 0) & 0xFF;        //Read Offset LSB
    shtpData[3] = (readOffset >> 8) & 0xFF;        //Read Offset MSB
    shtpData[4] = (recordID >> 0) & 0xFF;        //FRS Type LSB
    shtpData[5] = (recordID >> 8) & 0xFF;        //FRS Type MSB
    shtpData[6] = (blockSize >> 0) & 0xFF;        //Block size LSB
    shtpData[7] = (blockSize >> 8) & 0xFF;        //Block size MSB

    //Transmit packet on channel 2, 8 bytes
    sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead) {
    uint8_t spot = 0;

    //First we send a Flash Record System (FRS) request
    frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

    //Read bytes until FRS reports that the read is complete
    while (true) {
        //Now we wait for response
        while (true) {
            uint8_t counter = 0;
            while (!receivePacket()) {
                if (counter++ > 100) return false; //Give up
                smartDelayUs(1000);
            }

            //We have the packet, inspect it for the right contents
            //See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
            if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
                if (((((uint16_t) shtpData[13]) << 8) | shtpData[12]) == recordID)
                    break; //This packet is one we are looking for
        }

        uint8_t dataLength = shtpData[1] >> 4;
        uint8_t frsStatus = shtpData[1] & 0x0F;

        uint32_t data0 = (uint32_t) shtpData[7] << 24 | (uint32_t) shtpData[6] << 16 | (uint32_t) shtpData[5] << 8 |
                         (uint32_t) shtpData[4];
        uint32_t data1 = (uint32_t) shtpData[11] << 24 | (uint32_t) shtpData[10] << 16 | (uint32_t) shtpData[9] << 8 |
                         (uint32_t) shtpData[8];

        //Record these words to the metaData array
        if (dataLength > 0) {
            metaData[spot++] = data0;
        }
        if (dataLength > 1) {
            metaData[spot++] = data1;
        }

        if (spot >= MAX_METADATA_SIZE) {
            Serial.println(F("metaData array over run. Returning."));
            return true; //We have run out of space in our array. Bail.
        }

        if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7) {
            return true; //FRS status is read completed! We're done!
        }
    }
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t resetReason() {
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0;                              //Reserved

    //Transmit packet on channel 2, 2 bytes
    sendPacket(CHANNEL_CONTROL, 2);

    //Now we wait for response
    if (receivePacket()) {
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            return shtpData[1];
        }
    }

    return 0;
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return qFloat;
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in isCalibrating
bool calibrationComplete() {
    return !isCalibrating;
}

bool calibrationSuccessful() {
    return isAccurate;
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void setFeatureCommand(uint8_t reportID, uint32_t periodUs) {
    setFeatureCommand(reportID, periodUs, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void setFeatureCommand(uint8_t reportID, uint32_t periodUs, uint32_t specificConfig) {
    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;     //Set feature command. Reference page 55
    shtpData[1] = reportID;                               //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0;                                   //Feature flags
    shtpData[3] = 0;                                   //Change sensitivity (LSB)
    shtpData[4] = 0;                                   //Change sensitivity (MSB)
    shtpData[5] = (periodUs >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (periodUs >> 8) & 0xFF;  //Report interval
    shtpData[7] = (periodUs >> 16) & 0xFF; //Report interval
    shtpData[8] = (periodUs >> 24) & 0xFF; //Report interval (MSB)
    shtpData[9] = 0;                                   //Batch Interval (LSB)
    shtpData[10] = 0;                                   //Batch Interval
    shtpData[11] = 0;                                   //Batch Interval
    shtpData[12] = 0;                                   //Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF;       //Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF;       //Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF;      //Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF;      //Sensor-specific config (MSB)

    //Transmit packet on channel 2, 17 bytes
    sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void sendCommand(uint8_t command) {
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
    shtpData[1] = commandSequenceNumber++;     //Increments automatically each function call
    shtpData[2] = command;                       //Command

    //Caller must set these
    /*shtpData[3] = 0; //P0
    shtpData[4] = 0; //P1
    shtpData[5] = 0; //P2
    shtpData[6] = 0;
    shtpData[7] = 0;
    shtpData[8] = 0;
    shtpData[9] = 0;
    shtpData[10] = 0;
    shtpData[11] = 0;*/

    //Transmit packet on channel 2, 12 bytes
    sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void sendCalibrateCommand() {
    /*shtpData[3] = 0; //P0 - Accel Cal Enable
    shtpData[4] = 0; //P1 - Gyro Cal Enable
    shtpData[5] = 0; //P2 - Mag Cal Enable
    shtpData[6] = 0; //P3 - Subcommand 0x00
    shtpData[7] = 0; //P4 - Planar Accel Cal Enable
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    shtpData[3] = 1;
    shtpData[4] = 1;
    shtpData[5] = 1;
    shtpData[7] = 1;

    //Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
    isCalibrating = 1;

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void sendStopCalibrateCommand() {
    /*shtpData[3] = 0; //P0 - Accel Cal Enable
    shtpData[4] = 0; //P1 - Gyro Cal Enable
    shtpData[5] = 0; //P2 - Mag Cal Enable
    shtpData[6] = 0; //P3 - Subcommand 0x00
    shtpData[7] = 0; //P4 - Planar Accel Cal Enable
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    //Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
    isCalibrating = 1;

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from BNO080
//See page 51 of reference manual
void requestCalibrationStatus() {
    /*shtpData[3] = 0; //P0 - Reserved
    shtpData[4] = 0; //P1 - Reserved
    shtpData[5] = 0; //P2 - Reserved
    shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
    shtpData[7] = 0; //P4 - Reserved
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void saveCalibration() {
    /*shtpData[3] = 0; //P0 - Reserved
    shtpData[4] = 0; //P1 - Reserved
    shtpData[5] = 0; //P2 - Reserved
    shtpData[6] = 0; //P3 - Reserved
    shtpData[7] = 0; //P4 - Reserved
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_DCD); //Save DCD command
}

//Blocking wait for BNO080 to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
elapsedMicros wait;

bool waitForSPI() {
    wait = 0;
    while (wait < IMU_DELAY_US) {
        if (digitalRead(IMU_INT) == LOW) return true;
        smartDelayUs(1000);
    }

    Serial.println(F("[ERROR] waitForSPI() timed out."));
    return false;
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool receivePacket() {
    if (digitalRead(IMU_INT) == HIGH)
        return false; //Data is not available

    //Old way: if (waitForSPI() == false) return false; //Something went wrong

    //Get first four bytes to find out how much data we need to read

    SPI.beginTransaction(spiSettings);
    digitalWrite(IMU_CS, LOW);

    //Get the first four bytes, aka the packet header
    uint8_t packetLSB = SPI.transfer(0);
    uint8_t packetMSB = SPI.transfer(0);
    uint8_t channelNumber = SPI.transfer(0);
    uint8_t sequenceNumber = SPI.transfer(0); //Not sure if we need to store this or not

    //Store the header info
    shtpHeader[0] = packetLSB;
    shtpHeader[1] = packetMSB;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNumber;

    //Calculate the number of data bytes in this packet
    uint16_t dataLength = (((uint16_t) packetMSB) << 8) | ((uint16_t) packetLSB);
    dataLength &= ~(1 << 15); //Clear the MSbit.
    //This bit indicates if this package is a continuation of the last. Ignore it for now.
    //TODO catch this as an error and exit
    if (dataLength == 0) {
        //Packet is empty
//        printHeader();
        return false; //All done
    }
    dataLength -= 4; //Remove the header bytes from the data count

    //Read incoming data into the shtpData array
    for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++) {
        uint8_t incoming = SPI.transfer(0xFF);
        if (dataSpot < MAX_PACKET_SIZE)    //BNO080 can respond with upto 270 bytes, avoid overflow
            shtpData[dataSpot] = incoming; //Store data into the shtpData array
    }

    digitalWrite(IMU_CS, HIGH); //Release BNO080

    SPI.endTransaction();
//    printPacket();

    return true; //We're done!
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool sendPacket(uint8_t channelNumber, uint8_t dataLength) {
    uint8_t packetLength = dataLength + 4; //Add four bytes for the header

    //Wait for BNO080 to indicate it is available for communication
    if (!waitForSPI()) return false; //Something went wrong

    //BNO080 has max CLK of 3MHz, MSB first,
    //The BNO080 uses CPOL = 1 and CPHA = 1. This is mode3
    SPI.beginTransaction(spiSettings);
    digitalWrite(IMU_CS, LOW);

    //Send the 4 byte packet header
    SPI.transfer(packetLength & 0xFF);             //Packet length LSB
    SPI.transfer(packetLength >> 8);                 //Packet length MSB
    SPI.transfer(channelNumber);                     //Channel number
    SPI.transfer(
            sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

    //Send the user's data packet
    for (uint8_t i = 0; i < dataLength; i++) {
        SPI.transfer(shtpData[i]);
    }

    digitalWrite(IMU_CS, HIGH);
    SPI.endTransaction();

    return true;
}

//Pretty prints the contents of the current shtp header and data packets
void printPacket() {
    uint16_t packetLength = (uint16_t) shtpHeader[1] << 8 | shtpHeader[0];

    //Print the four byte header
    Serial.print(F("Header:"));
    for (unsigned char x : shtpHeader) {
        Serial.print(F(" "));
        if (x < 0x10)
            Serial.print(F("0"));
        Serial.print(x, HEX);
    }

    uint8_t printLength = packetLength - 4;
    if (printLength > 40)
        printLength = 40; //Artificial limit. We don't want the phone book.

    Serial.print(F(" Body:"));
    for (uint8_t x = 0; x < printLength; x++) {
        Serial.print(F(" "));
        if (shtpData[x] < 0x10)
            Serial.print(F("0"));
        Serial.print(shtpData[x], HEX);
    }

    if (packetLength & 1 << 15) {
        Serial.println(F(" [Continued packet] "));
        packetLength &= ~(1 << 15);
    }

    Serial.print(F(" Length:"));
    Serial.print(packetLength);

    Serial.print(F(" Channel:"));
    if (shtpHeader[2] == 0)
        Serial.print(F("Command"));
    else if (shtpHeader[2] == 1)
        Serial.print(F("Executable"));
    else if (shtpHeader[2] == 2)
        Serial.print(F("Control"));
    else if (shtpHeader[2] == 3)
        Serial.print(F("Sensor-report"));
    else if (shtpHeader[2] == 4)
        Serial.print(F("Wake-report"));
    else if (shtpHeader[2] == 5)
        Serial.print(F("Gyro-vector"));
    else
        Serial.print(shtpHeader[2]);

    Serial.println();
}

//Pretty prints the contents of the current shtp header (only)
void printHeader() {
    //Print the four byte header
    Serial.print(F("Header:"));
    for (unsigned char x : shtpHeader) {
        Serial.print(F(" "));
        if (x < 0x10)
            Serial.print(F("0"));
        Serial.print(x, HEX);
    }
    Serial.println();
}
