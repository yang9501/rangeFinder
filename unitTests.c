//
// Created by Justin Yang on 4/24/23.
//

#include "unitTests.h"
#include "rangeFinder.h"
#include <assert.h>

void resetGlobalVariables() {
    gpsReadyFlag = 0;
    rangeFinderReadyFlag = 0;
    compassReadyFlag = 0;
//GPS variables
    latitude = 0.0;
    longitude = 0.0;
//Rangefinder variables
    range = 0.0;
//Compass variables
    heading = 0.0;
//Output variables
    targetLatitude = 0.0;
    targetLongitude = 0.0;
}

int compare_float(double x, double y, double epsilon) {
    if(fabs(x - y) < epsilon)
        return 1; //they are same
    return 0; //they are not same
}

void runRegressionTests() {
    testDegreesToDecimal();
    testNewCoords();
    testParseGPSMessage();
    testPolarToCartesianCoords();
    //testGPS();
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
    assert(compare_float(targetLat, 38.874897, 0.000001f) == 1);
    assert(compare_float(targetLong, -77.228306, 0.000001f) == 1);
}

void testParseGPSMessage() {
    double latResult = 0.0;
    double longResult = 0.0;
    char* testNMEASentence = "$GNGGA,202530.00,3852.76334,N,07713.69836,W,0,40,0.5,1097.36,M,-17.00,M,18,TSTR*61";
    parseGPSMessage(testNMEASentence, &latResult, &longResult);

    assert(compare_float(latResult, 38.879389, 0.000001f) == 1);
    assert(compare_float(longResult, -77.228306, 0.000001f) == 1);
}

void testPolarToCartesianCoords() {
    double xOffset = 0.0;
    double yOffset = 0.0;
    polarToCartesianCoords(500, 0, &xOffset, &yOffset);

    assert(compare_float(xOffset, 0.0, 0.000001f) == 1);
    assert(compare_float(yOffset, 500, 0.000001f) == 1);
}

//TEST NOT WORKING, WHILE LOOP RESULTING IN INFINITE RUNTIME WITH NO STOP CONDITION
void testGPS() {
    readGPS("testData/gpsTestData.txt");
    assert(gpsReadyFlag == 1);
    assert(compare_float(latitude, 38.879389, 0.000001f) == 1);
    assert(compare_float(longitude, -77.228306, 0.000001f) == 1);

    resetGlobalVariables();
}

//TEST NOT WORKING
void testGPSUnready() {
    readGPS("testData/gpsTestDataUnready.txt");
    assert(gpsReadyFlag == 0);
    assert(compare_float(latitude, 0.0, 0.000001f) == 1);
    assert(compare_float(longitude, 0.0, 0.000001f) == 1);

    resetGlobalVariables();
}