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

#define GPIO_PATH_66 "/sys/class/gpio/gpio66" //Start/Stop Button

//Writes specified value to specified GPIO directory
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

///////DATA VARIABLES
//Calibration variables
int gpsReadyFlag = 0;
int rangeFinderReadyFlag = 0;
int compassReadyFlag = 0;
//GPS variables
double latitude = 0.0;
double longitude = 0.0;
//Rangefinder variables
double range = 0.0;
//Compass variables
double heading = 0.0;
//Output variables
double targetLatitude = 0.0;
double targetLongitude = 0.0;

//Mutexes
pthread_mutex_t gpsMutex;
pthread_mutex_t rangefinderMutex;
pthread_mutex_t compassMutex;
pthread_mutex_t targetLocMutex;

int main(void) {
    runRegressionTests();
	char buttonPort[25] = GPIO_PATH_66; //buttonPorts[0] is the start/stop

    (void) writeGPIO("/direction", buttonPort, "in");

    //Initialize mutexes
    (void) pthread_mutex_init(&gpsMutex, NULL);
    (void) pthread_mutex_init(&rangefinderMutex, NULL);
    (void) pthread_mutex_init(&compassMutex, NULL);
    (void) pthread_mutex_init(&targetLocMutex, NULL);

    // Create independent threads each of which will execute function
    pthread_t thread1, thread2, thread3, thread4;
    pthread_attr_t tattr1, tattr2, tattr3, tattr4;
    struct sched_param param1, param2, param3, param4;

    pthread_attr_init(&tattr1);
    pthread_attr_init(&tattr2);
    pthread_attr_init(&tattr3);
    pthread_attr_init(&tattr4);
    pthread_attr_getschedparam(&tattr1, &param1);
    pthread_attr_getschedparam(&tattr2, &param2);
    pthread_attr_getschedparam(&tattr3, &param3);
    pthread_attr_getschedparam(&tattr4, &param4);

    //Button priority is highest
    param1.sched_priority = 90;
    param2.sched_priority = 90;
    //Timer update thread is second highest
    param3.sched_priority = 80;
    //Display output has lowest priority
    param4.sched_priority = 20;

    pthread_attr_setschedparam(&tattr1, &param1);
    pthread_attr_setschedparam(&tattr2, &param2);
    pthread_attr_setschedparam(&tattr3, &param3);
    pthread_attr_setschedparam(&tattr4, &param4);

    //Button Thread
    (void) pthread_create( &thread1, &tattr1, (void*) getButtonPress, (void*) buttonPort);
    //Compass Thread
    (void) pthread_create( &thread2, &tattr2, (void *) bno055, NULL);
    //GPS Thread
    (void) pthread_create( &thread3, &tattr3, (void *) readGPS, NULL);
    //Rangefinder Thread
    (void) pthread_create( &thread4, &tattr4, (void *) rangeFinder, NULL);

    //Wait on button thread
    (void) pthread_join(thread1, NULL);

	return 0;
}

void getCalStatus() {
    struct bnocal bnoc;
    ////////////READ IMU SYSTEM CALIBRATION STATUS
    while (1) {
        get_calstatus(&bnoc);
        if(bnoc.scal_st == 3) {
            compassReadyFlag = 1;
            break;
        }
        usleep(100000);
    }
}

void tiltCompensatedCompass() {
    //https://toptechboy.com/9-axis-imu-lesson-10-making-a-tilt-compensated-compass-with-arduino/
    int res = 0;

    double thetaM;  //Measured
    double phiM;

    double theta; //pitch
    double phi;  //roll

    double thetaRad;
    double phiRad;

    double Xm;
    double Ym;
    double psi; //heading angle

    double dt = 0;
    struct timeval time;
    gettimeofday(&time, NULL);
    double millisecondsOld = ((double) time.tv_sec * 1000) + ((double) time.tv_usec / 1000);
    double millisecondsCurr = 0;

    while(1) {
        //Retrieve Accelerometer data
        struct bnoacc bnodAcc;
        res = get_acc(&bnodAcc);
        //Retrieve Gyroscope data
        struct bnogyr bnodGyr;
        res = get_gyr(&bnodGyr);
        //Retrieve Magnetometer data
        struct bnomag bnodMag;
        res = get_mag(&bnodMag);

        //Measured values normalized
        thetaM = -(atan2(bnodAcc.adata_x/9.8,bnodAcc.adata_z/9.8)*360.)/(2.*M_PI);
        phiM = -(atan2(bnodAcc.adata_y/9.8, bnodAcc.adata_z/9.8)*360.)/(2.*M_PI);

        gettimeofday(&time, NULL);
        millisecondsCurr = ((double) time.tv_sec * 1000.) + ((double) time.tv_usec / 1000.);
        dt = (millisecondsCurr - millisecondsOld)/1000.;  //dt is in SECONDS
        millisecondsOld = millisecondsCurr;
        //Low pass filter values for Gyroscope
        theta = (theta + bnodGyr.gdata_y * dt)*0.95 + thetaM * 0.05;
        phi = (phi - bnodGyr.gdata_x * dt)*0.95 + phiM * 0.95;

        //Converts degrees to radians because math.h trigonometry functions wants radians.
        phiRad = (phi/360.)*(2.*M_PI);
        thetaRad = (theta/360.)*(2.*M_PI);

        //Compensate accel/gyro tilt for magnetometer values
        Xm = bnodMag.mdata_x*cos(thetaRad) - bnodMag.mdata_y*sin(phiRad)*sin(thetaRad) + bnodMag.mdata_z*cos(phiRad)*sin(thetaRad);
        Ym = bnodMag.mdata_y*cos(phiRad) + bnodMag.mdata_z*sin(phiRad);

        //Outputs from 0 to 180, 0 to -180.  Need to convert to 0 to 360 degrees
        //conversion: angle = (angle + 360) % 360
        psi = fmod(((360*atan2(Ym, Xm))/(2*M_PI)) + 360, 360);  //HEADING IN DEGREES CONVERTED FROM RADIANS

        (void) pthread_mutex_lock(&compassMutex);
        heading = psi;
        (void) pthread_mutex_unlock(&compassMutex);

        usleep(10 * 1000); //Sleep for 10 milliseconds
    }
}

void bno055() {
    char senaddr[256] = "0x28";
    char i2c_bus[256] = I2CBUS;

    get_i2cbus(i2c_bus, senaddr);
    ////////SET MODE
    int res = set_mode(ndof_fmc);

    ////////CALIBRATION STATUS
    getCalStatus();

    ////////MAIN LOGIC
    tiltCompensatedCompass();
}

double degreesToDecimal(double degreeCoord) {
    double ddeg;
    double sec = modf(degreeCoord, &ddeg)*60;
    int deg = (int)(ddeg/100);
    int min = (int)(degreeCoord-(deg*100));

    double absdlat = round(deg * 1000000.);
    double absmlat = round(min * 1000000.);
    double absslat = round(sec * 1000000.);

    return round(absdlat + (absmlat/60) + (absslat/3600)) /1000000;
}

void parseGPSMessage(char* message, double* latResult, double* longResult) {
    if (strstr(message, "$GNGGA") != NULL) {
        double latRawValue = 0.0;
        char *ns;
        double longRawValue = 0.0;
        char *ew;

        char *p = message;
        p = strchr(p, ',')+1; //skip time value in GPS message

        //Latitude value
        p = strchr(p, ',')+1;
        latRawValue = atof(p);

        //Latitude hemisphere
        p = strchr(p, ',')+1;
        ns = &p[0];

        //Longitude value
        p = strchr(p, ',')+1;
        longRawValue = atof(p);

        //Longitude Hemisphere
        p = strchr(p, ',')+1;
        ew = &p[0];

        double latDegrees = (ns[0] == 'N') ? latRawValue : -1 * (latRawValue);
        double longDegrees = (ew[0] == 'E') ? longRawValue : -1 * (longRawValue);

        *latResult = degreesToDecimal(latDegrees);
        *longResult = degreesToDecimal(longDegrees);
    }
}

void readGPS() {
    int serialPort = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
    struct termios options;
    tcgetattr(serialPort, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL | IGNCR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serialPort, TCIFLUSH);
    tcsetattr(serialPort, TCSANOW, &options);

    char read_buf [256];
    double latResult = 0.0;
    double longResult = 0.0;
    //////////READ FROM GPS UART SERIAL PORT
    while(1) {
        char c;
        char *b = read_buf;
        while(1) {
            int n = read(serialPort, (void *) (&c), 1);
            if (n < 0) {

            } else {
                if (c == '\n') {
                    *b++ = '\0';
                    break;
                }
                *b++ = c;
            }
        }
        /////////////TEST DATA
        strcpy(read_buf, "$GNGGA,202530.00,3852.76334,N,07713.69836,W,0,40,0.5,1097.36,M,-17.00,M,18,TSTR*61");
        //////////////////////
        //Test coord origin: 38°52'45.8"N 77°13'41.9"W ||||||| 38.879389, -77.228306 |||||| 3852.76334,N,07713.69836,W
        char *p = read_buf;
        p = strchr(p, ',')+6;
        //GPS readiness status check.  Skip to GPS Fix Quality Indicator, which is greater than zero when GPS fix is obtained
        if(p[0] > 0) {
            gpsReadyFlag = 1;
        }
        else {
            continue;
        }
        parseGPSMessage(read_buf, &latResult, &longResult);
        (void) pthread_mutex_lock(&gpsMutex);
        latitude = latResult;
        longitude = longResult;
        (void) pthread_mutex_unlock(&gpsMutex);
    }
}

void rangeFinder() {
    unsigned char cmd1[] = { 0x80, 0x06, 0x03, 0x77 };         // Continuous Measurement Mode
    unsigned char cmd2[] = { 0x80, 0x06, 0x07, 0x73 };
    unsigned char cmd3[] = { 0x80, 0x06, 0x05, 0x01, 0x74 };   // LaserPointerOn
    unsigned char cmd4[] = { 0x80, 0x06, 0x05, 0x00, 0x75 };   // LaserPointerOff

    unsigned char cmd5[] = { 0xFA, 0x04, 0x0A, 0x00, 0xF8 };   // 2Hz
    unsigned char cmd6[] = { 0xFA, 0x04, 0x0A, 0x05, 0xF3 };   // 5Hz
    unsigned char cmd7[] = { 0xFA, 0x04, 0x0A, 0x0A, 0xEE };   // 10Hz
    unsigned char cmd8[] = { 0xFA, 0x04, 0x0A, 0x14, 0xE4 };   // 20Hz

    unsigned char cmd9[] = { 0xFA, 0x04, 0x09, 0x05, 0xF4 };    // 5m Range
    unsigned char cmd10[] = { 0xFA, 0x04, 0x09, 0x0A, 0xEF };   // 10m Range
    unsigned char cmd11[] = { 0xFA, 0x04, 0x09, 0x1E, 0xDB };   // 30m Range
    unsigned char cmd12[] = { 0xFA, 0x04, 0x09, 0x32, 0xC7 };   // 50m Range
    unsigned char cmd13[] = { 0xFA, 0x04, 0x09, 0x50, 0xA9 };   // 80m Range

    unsigned char cmd14[] = { 0xFA, 0x04, 0x0C, 0x02, 0xF4 };   // 0.1mm Resolution
    unsigned char cmd15[] = { 0xFA, 0x04, 0x0C, 0x01, 0xF5 };   // 1mm Resolution


    ////////////SETUP
    int serialPort = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    struct termios options;
    tcgetattr(serialPort, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL | IGNCR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serialPort, TCIFLUSH);
    tcsetattr(serialPort, TCSANOW, &options);
    printf("Beginning startup commands\n");

    printf("Resolution to 1mm\n");
    write(serialPort, cmd15, sizeof(cmd15));  //Set resolution to 1mm
    sleep(1);

    printf("Measurement Frequency\n");
    write(serialPort, cmd7, sizeof(cmd7));  //Set measurement frequency to 10Hz
    sleep(1);

    printf("Set Range\n");
    write(serialPort, cmd11, sizeof(cmd11));  //Set range to 10m
    sleep(1);

    printf("Laser on\n");
    write(serialPort, cmd3, sizeof(cmd3));  //Turn visible laser on
    sleep(1);

    printf("Continuous Measurement On\n");
    write(serialPort, cmd1, sizeof(cmd1));  //Turn continuous measurement on
    sleep(1);

    char read_buf [256];
    char test_buf [256];

    rangeFinderReadyFlag = 1;

    printf("Beginning read\n");
    while(1) {
        int n = read(serialPort, (void *) (&read_buf), 11);

        if (n < 0) {
            printf("Unresponsive\n");
        } else {
            strncpy(test_buf, read_buf + 3, 7);

            (void) pthread_mutex_lock(&rangefinderMutex);
            //range = strtod(test_buf, NULL);
            range = 500;
            (void) pthread_mutex_unlock(&rangefinderMutex);
        }
    }
}

void printCalibrationDisplay() {
    if(init_i2c_dev(I2C_DEV2_PATH, SSD1306_OLED_ADDR) == 0)
    {
        printf("(Main)i2c-2: Bus Connected to SSD1306\r\n");
    }
    else
    {
        printf("(Main)i2c-2: OOPS! Something Went Wrong\r\n");
        exit(1);
    }

    char gpsStatus[] = "GPS Status: ";
    char rangeFinderStatus[] = "Rangefinder Status: ";
    char compassStatus[] = "Compass Status: ";

    char calibratingStatus[] = "Calibrating";
    char readyStatus[] = "Ready";

    display_Init_seq();

    while(1) {
        clearDisplay();

        setTextSize(1);
        setTextColor(WHITE);
        setCursor(1, 0);

        print_strln(gpsStatus);
        if (gpsReadyFlag == 0) {
            print_strln(calibratingStatus);
        } else {
            print_strln(readyStatus);
        }
        println();
        print_strln(rangeFinderStatus);
        if (rangeFinderReadyFlag == 0) {
            print_strln(calibratingStatus);
        } else {
            print_strln(readyStatus);
        }
        println();
        print_strln(compassStatus);
        if (compassReadyFlag == 0) {
            print_strln(calibratingStatus);
        } else {
            print_strln(readyStatus);
        }
        Display();
        if(gpsReadyFlag == 1 && rangeFinderReadyFlag == 1 && compassReadyFlag == 1) {
            //printf("BREAKING\n");
            sleep(2);
            break;
        }
        sleep(1);
    }
    clearDisplay();
    setTextSize(1);
    setTextColor(WHITE);
    setCursor(1, 0);
    print_strln("Press button to");
    print_strln("receive target ");
    print_strln("location!");
    Display();
}

void polarToCartesianCoords(double r, double theta, double* x, double* y) {
    double adjTheta = fmod(90 - theta + 360, 360); //Since polar coordinates are x = 0 heading, have to rotate by 90 degrees counter-clock wise
    double thetaRadians = (adjTheta*2*M_PI)/360; //Need to convert to radians because cos() and sin() expect as such
    printf("range is: %f\n", r);
    printf("heading is: %f\n", theta);
    printf("dx is: %f\n", r * cos(thetaRadians));
    printf("dy is: %f\n", r * sin(thetaRadians));
    *x = r * cos(thetaRadians);
    *y = r * sin(thetaRadians);
}

void newCoords(double initLat, double initLong, double dx, double dy, double* targetLat, double* targetLong) {
    //https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
    double earthRadiusMeters = 6378 * 1000;
    double newLat = initLat + (dy/earthRadiusMeters) *  (180/M_PI);
    double newLong = initLong + ((dx/earthRadiusMeters) * (180/M_PI)/ cos(initLat * M_PI/180));

    *targetLat = newLat;
    *targetLong = newLong;
}

void getButtonPress(void *buttonPort) {
    uint32_t pressedFlag = 0;
    uint32_t signalSentFlag = 0;
    uint32_t gpioValue;
    double dx = 0.0;
    double dy = 0.0;
    char latStr[100];
    char longStr[100];
    printCalibrationDisplay();
    while(1) {
        gpioValue = readGPIO("/value", (char *) buttonPort);
        if(gpioValue == 1){
            //first press detected
            if(pressedFlag == 0) {
                pressedFlag = 1;

                printf("LATITUDE: %f\n", latitude);
                printf("LONGITUDE: %f\n", longitude);
                printf("RANGE %f\n", range);
                printf("COMPASS: %f\n", heading);

                (void) pthread_mutex_lock(&rangefinderMutex);
                (void) pthread_mutex_lock(&compassMutex);
                polarToCartesianCoords(range, heading, &dx, &dy);
                (void) pthread_mutex_unlock(&rangefinderMutex);
                (void) pthread_mutex_unlock(&compassMutex);

                (void) pthread_mutex_lock(&targetLocMutex);
                newCoords(latitude, longitude, dx, dy, &targetLatitude, &targetLongitude);
                sprintf(latStr, "%f", targetLatitude);
                sprintf(longStr, "%f", targetLongitude);
                (void) pthread_mutex_unlock(&targetLocMutex);

                clearDisplay();
                setTextSize(1);
                setTextColor(WHITE);
                setCursor(1, 0);
                print_strln("Target Latitude: ");
                print_strln(latStr);
                print_strln("Target Longitude: ");
                print_strln(longStr);
                Display();
            }
        }
        if(gpioValue == 0) {
            //if the button is let go after being pressed
            if(pressedFlag == 1) {
                pressedFlag = 0;
            }
        }
    }
}

static uint32_t readGPIO(char *filename, char *port) {
    FILE* fp; //create file pointer
    char fullFileName[100]; //store path and filename
    uint32_t val;
    (void) sprintf(fullFileName, "%s%s", port, filename); //write path/name
    fp = fopen(fullFileName, "r"); //open file for writing
    (void) fscanf(fp, "%d", &val);
    (void) fclose(fp);
    return val;
}

static void writeGPIO(char *filename, char *port, char *value) {
    FILE* fp; //create file pointer
    char fullFileName[100]; //store path and filename
    (void) sprintf(fullFileName, "%s%s", port, filename); //write path/name
    fp = fopen(fullFileName, "w+"); //open file for writing
    (void) fprintf(fp, "%s", value); // send value to the file
    (void) fclose(fp); //close the file using the file pointer
}
