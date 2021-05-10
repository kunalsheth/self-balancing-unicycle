#include <Arduino.h>

#include "display/display_io.h"
#include "imu/imu_io.h"
#include "encoder/encoder_io.h"
#include "motor/motor_io.h"
#include "current/current_io.h"

extern "C" {
#include "robot.h"
#include "display/display.h"
#include "imu/imu.h"
#include "current/current.h"
}

RobotState robot;

void dispOutBackground() { displayIoOut(&robot.display); }

void usbFlushBackground() { Serial.flush(); }

void setup() {
    robot.display.size = 0;
    Serial.begin(115200);

    Serial.println("HELLO WORLD!");

    delay(1000);

    displayIoSetup();
    displayText(&robot.display, "CALIBRATING...");
    registerBackgroundTask("dispOut", 10, dispOutBackground);
    registerBackgroundTask("usbflush", 10, usbFlushBackground);

    while (!imuIoSetup(20000)) {
        Serial.println("[ERROR] !imuIoSetup(20000)");
    }
    registerBackgroundTask("imuServ", 400, imuIoService);
    sendCalibrateCommand();

    currentIoSetup();
    motorIoSetup();

    while (!calibrationComplete()) smartDelayUs(5000);

    displayClear(&robot.display);
    displayText(&robot.display, "CALIBRATED!");
}

#define yGain (-5E1)
#define yTarget asin(sin(0 * DEG_TO_RAD))
#define xGain (-1E3)
#define xTarget asin(sin(175 * DEG_TO_RAD))

char text[64];

void loop() {
    if (imuIoGetReading(&robot.imu)) {
        robot.imu.angle.x = asin(sin(robot.imu.angle.x));
        robot.imu.angle.y = asin(sin(robot.imu.angle.y));
        robot.imu.angle.z = asin(sin(robot.imu.angle.z));

        displayClear(&robot.display);
        snprintf(text, 50, "x %d\r\ny %d\r\nz %d\r\n",
                 (int) (robot.imu.angle.x * RAD_TO_DEG),
                 (int) (robot.imu.angle.y * RAD_TO_DEG),
                 (int) (robot.imu.angle.z * RAD_TO_DEG));

        displayText(&robot.display, text);

        if (!calibrationSuccessful()) displayText(&robot.display, "!cs\r\n");

        displayDrawLine(&robot.display,
                        DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2,
                        DISPLAY_WIDTH / 2 + (uint8_t) (24 * cos(-robot.imu.angle.y - PI / 2)),
                        DISPLAY_HEIGHT / 2 + (uint8_t) (24 * sin(-robot.imu.angle.y - PI / 2)),
                        true
        );
    }
    encoderIoGetReading(&robot.encoder);
    currentIoGetReading(&robot.current);

    Serial.print("Drive Wheel: ");
    Serial.print(robot.encoder.driveRotations, 3);
    Serial.print(" rots at ");
    Serial.print(robot.encoder.driveRps, 3);
    Serial.print(" rps and ");
    Serial.print(robot.current.drive, 3);
    Serial.print(" amps.\n");

    static double reaction = 0;
    reaction += (yTarget - robot.imu.angle.y) * yGain;
    double drive = (xTarget - robot.imu.angle.x) * xGain;

    if (reaction > 100) reaction = 100;
    if (reaction < -100) reaction = -100;
    if (drive > 100) drive = 100;
    if (drive < -100) drive = -100;

    snprintf(text, 50, "r %d\r\nd %d\r\n",
             (int) (reaction),
             (int) (drive));
    displayText(&robot.display, text);

    motorSet(&robot.motor, (int8_t) reaction, (int8_t) drive);
    motorIoOut(&robot.motor);

    smartDelayUs(5000);
}
