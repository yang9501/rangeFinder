//
// Created by Justin Yang on 4/24/23.
//

#include "unitTests.h"
#include "rangeFinder.h"
#include <assert.h>


void runRegressionTests() {
    testDegreesToDecimal();
    testNewCoords();
    testParseGPSMessage();
    testPolarToCartesianCoords();
}

void testDegreesToDecimal() {
    double val = degreesToDecimal(3852.76334);
    assert(val == 38.879389);

    val = degreesToDecimal(07713.69836);
    assert(val == 77.228306);
}

void testNewCoords() {
    double targetLat = 0.0;
    double targetLong = 0.0;
    newCoords(38.879389, -77.228306, 0, -500, &targetLat, &targetLong);
    assert(targetLat == 38.874897);
    assert(targetLong == -77.228306);
}

void testParseGPSMessage() {
    double latResult = 0.0;
    double longResult = 0.0;
    char* testNMEASentence = "$GNGGA,202530.00,3852.76334,N,07713.69836,W,0,40,0.5,1097.36,M,-17.00,M,18,TSTR*61";
    parseGPSMessage(testNMEASentence, &latResult, &longResult);

    assert(latResult == 38.879389);
    assert(longResult == -77.228306);
}

void testPolarToCartesianCoords() {
    double xOffset = 0.0;
    double yOffset = 0.0;
    polarToCartesianCoords(500, 0, &xOffset, &yOffset);

    assert(xOffset == 0.0);
    assert(yOffset == 500);
}