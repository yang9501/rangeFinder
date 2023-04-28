//
// Created by Justin Yang on 4/24/23.
//
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#include <math.h>
#include "SSD1306-OLED-display-driver-for-BeagleBone-master/SSD1306_OLED_Library/SSD1306_OLED.h"
#include "SSD1306-OLED-display-driver-for-BeagleBone-master/I2C_Library/I2C.h"
#include "pi-bno055-master/getbno055.h"
#include "unitTests.h"
#ifndef RANGEFINDER_RANGEFINDER_H
#define RANGEFINDER_RANGEFINDER_H//Writes specified value to specified GPIO directory
static void writeGPIO(char *filename, char *port, char *value);
//Reads input to GPIO pin
static uint32_t readGPIO(char *filename, char *port);
//Primary button press detection
void getButtonPress(void *buttonPort);
void printCalibrationDisplay();
//GPS functions
double degreesToDecimal(double degreeCoord);                                                                                 ///TESTABLE
void newCoords(double initLat, double initLong, double dx, double dy, double* targetLat, double* targetLong);                ///TESTABLE
void parseGPSMessage(char* message, double* latResult, double* longResult);                                                                                         ///TESTABLE
void readGPS();
//Rangefinder function
void rangeFinder();
//Compass functions
void getCalStatus();
void polarToCartesianCoords(double r, double theta, double* x, double* y);                                                   ///TESTABLE
void bno055();
void tiltCompensatedCompass();
#endif //RANGEFINDER_RANGEFINDER_H
