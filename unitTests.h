//
// Created by Justin Yang on 4/24/23.
//

#ifndef RANGEFINDER_UNITTESTS_H
#define RANGEFINDER_UNITTESTS_H
bool compare_float(double x, double y, double epsilon);
void runRegressionTests();
void testDegreesToDecimal();
void testNewCoords();
void testParseGPSMessage();
void testPolarToCartesianCoords();

#endif //RANGEFINDER_UNITTESTS_H
