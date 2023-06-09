#include "rangeFinder.h"

#define GPIO_PATH_66 "/sys/class/gpio/gpio66" //Start/Stop Button
#define GPS_PATH "/dev/ttyS1"


//Mutexes
pthread_mutex_t gpsMutex;
pthread_mutex_t rangefinderMutex;
pthread_mutex_t compassMutex;
pthread_mutex_t targetLocMutex;

int main(void) {
    ///////DATA VARIABLES
    //Calibration variables
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

    runRegressionTests();

    char buttonPort[25] = GPIO_PATH_66; //buttonPorts[0] is the start/stop
    char gpsPort[25] = GPS_PATH;

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
    //Compass requires updates
    param2.sched_priority = 80;
    //Timer update thread is second highest
    param3.sched_priority = 80;
    //Display output has lowest priority
    param4.sched_priority = 80;

    pthread_attr_setschedparam(&tattr1, &param1);
    pthread_attr_setschedparam(&tattr2, &param2);
    pthread_attr_setschedparam(&tattr3, &param3);
    pthread_attr_setschedparam(&tattr4, &param4);

    //Button Thread
    (void) pthread_create( &thread1, &tattr1, (void*) getButtonPress, (void*) buttonPort);
    //Compass Thread
    (void) pthread_create( &thread2, &tattr2, (void *) bno055, NULL);
    //GPS Thread
    (void) pthread_create( &thread3, &tattr3, (void *) readGPS, (void*) gpsPort);
    //Rangefinder Thread
    (void) pthread_create( &thread4, &tattr4, (void *) rangeFinder, NULL);

    //Wait on button thread
    (void) pthread_join(thread1, NULL);

	return 0;
}

/*
 * Retrieves device calibration status from bno055 IMU unit and sets the ready flag if applicable.
 *
 * To calibrate it, align the device to the x, y, and z axis for a few seconds each.  Do some figure 8 motions to align the magnetometer.
 *
*/
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

/*
 * Retrieves data from the IMU device and implements a tilt-compensated compass, or a compass that works no matter how the device is oriented,
 * as opposed to a compass that has to remain perfectly level at all times.
 *
 * Algorithm from this source: https://toptechboy.com/9-axis-imu-lesson-10-making-a-tilt-compensated-compass-with-arduino/
*/
void tiltCompensatedCompass() {
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

/*
 * Thread function to read from the bno055 IMU device.  Checks device status.
 *
*/
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

/*
 * Given a Lat/long coordinate in degree/minute format, converts to decimal
 *
 * Inputs:
 * double degreeCoord: Lat/long coordinate in degrees/minute format
 *
 * Outputs:
 * return: Lat/long coordinate in decimal format
*/
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

/*
 * Given a set of polar coordinates, returns the corresponding converted +-x, +-y offset values
 *
 * Inputs:
 * char* message: Pointer to the char array containing the NMEA sentence sent from the GPS device.
 *
 * Outputs:
 * double* latResult: The address used to store the parsed latitude.
 * double* longResult: The address used to store the parsed longitude.
*/
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

/*
 * Thread function to read input from GPS device, and to check ready state of device.  Only concerned with the GNGGA type of message in NMEA formatting. Saves device output in global variable.
 *
 * NOTE: due to hardware limitations(signal is hard to acquire indoors,
 * and the system setup is not convenient to transport and test outdoors), location is hardcoded
 *
*/
void readGPS(void *serialPort) {
    int gpsSerialPort = open((char *) serialPort, O_RDWR | O_NOCTTY);
    struct termios options;
    tcgetattr(gpsSerialPort, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR | ICRNL | IGNCR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(gpsSerialPort, TCIFLUSH);
    tcsetattr(gpsSerialPort, TCSANOW, &options);

    char read_buf [256];
    double latResult = 0.0;
    double longResult = 0.0;
    //////////READ FROM GPS UART SERIAL PORT
    while(1) {
        char c;
        char *b = read_buf;
        while(1) {
            int n = read(gpsSerialPort, (void *) (&c), 1);
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
        //Test coord origin: 38°52'45.8"N 77°13'41.9"W ||||||| 38.879389, -77.228306 |||||| 3852.76334,N,07713.69836,W
        strcpy(read_buf, "$GNGGA,202530.00,3852.76334,N,07713.69836,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR*61");
        //////////////////////
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

/*
 * Thread function to read and write to the Rangefinder device.  Saves device output to global variable.
 *
 * NOTE:  Value is currently hardcoded due to hardware limitations during testing.
*/
void rangeFinder() {
    unsigned char cmd1[] = { 0x80, 0x06, 0x03, 0x77 };         // Continuous Measurement Mode
    unsigned char cmd2[] = { 0x80, 0x06, 0x07, 0x73 };          //Single Measurement Mode
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
            //Hardcoded rangeFinder distance value due to testing limitations.
            range = 500;
            (void) pthread_mutex_unlock(&rangefinderMutex);
        }
    }
}

/*
 * Connects to the display.  Queries and displays device status.
*/
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

/*
 * Given a set of polar coordinates, returns the corresponding converted +-x, +-y offset values
 *
 * Inputs:
 * double r: Magnitude of polar vector in meters.  Uses the distance measurement from the rangefinder
 * double theta: Heading of polar vector in degrees.  Uses the heading measurement from the IMU compass
 *
 * Outputs:
 * double* x: The address used to put the x offset value in meters.
 * double* y: The address used to put the y offset value in meters.
*/
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

/*
 * Given your current position and an x and y offset, calculates the new coordinates
 *
 * Inputs:
 * double initLat: the current latitude of the GPS device in decimal format
 * double initLong: the current longitude of the GPS device in decimal format
 * double dx: the +-x offset value in meters to calculate from the current position.
 * double dy: the +-y offset value in meters to calculate from the current position.
 *
 * Outputs:
 * double* targetLat: the latitude of the target in decimal format
 * double* targetLong: the longitude of the target in decimal format
*/
void newCoords(double initLat, double initLong, double dx, double dy, double* targetLat, double* targetLong) {
    //https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
    double earthRadiusMeters = 6378 * 1000;
    double newLat = initLat + (dy/earthRadiusMeters) *  (180/M_PI);
    double newLong = initLong + ((dx/earthRadiusMeters) * (180/M_PI)/ cos(initLat * M_PI/180));

    *targetLat = newLat;
    *targetLong = newLong;
}

/*
 * Thread function to receive button input and write output to the display
 * Inputs:
 * void* buttonPort: address of the button GPIO port
*/
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
