#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <sys/utsname.h>
#include <stdint.h>
#include <time.h>
#include <float.h>
#include <sched.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include "SSD1306-OLED-display-driver-for-BeagleBone-master/SSD1306_OLED_Library/SSD1306_OLED.h"
#include "SSD1306-OLED-display-driver-for-BeagleBone-master/I2C_Library/I2C.h"
#include "pi-bno055-master/getbno055.h"
//comment out to live run
//#define DEBUG 1

#define GPIO_PATH_66 "/sys/class/gpio/gpio66" //Start/Stop Button

//Writes specified value to specified GPIO directory
static void writeGPIO(char *filename, char *port, char *value);

//Reads input to GPIO pin
static uint32_t readGPIO(char *filename, char *port);

//Primary button press detection
void getButtonPress(void *buttonPort);
void print_calstat();
void bno055();
void printDisplay();
void readGPS();
void rangeFinder();
void getBno055Info();
void getCalStatus();

pthread_mutex_t timerMutex;
float timerInMilliseconds;

pthread_mutex_t runningStateMutex;
int watchRunningState;

int main(void) {
    //arrays containing GPIO port definitions, representing the green and red lights, and the start/stop and reset buttons
	char buttonPort[25] = GPIO_PATH_66; //buttonPorts[0] is the start/stop

    #ifdef DEBUG
    (void) printf("DEBUG MODE\n");
    struct utsname sysInfo;
    (void) uname(&sysInfo);
    (void) printf("%s\n", sysInfo.sysname);
    (void) printf("%s\n", sysInfo.nodename);
    (void) printf("%s\n", sysInfo.machine);
    #else
    (void) writeGPIO("/direction", buttonPort, "in");
    #endif

    //Initialize mutexes
    (void) pthread_mutex_init(&runningStateMutex, NULL);
    (void) pthread_mutex_init(&timerMutex, NULL);

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
    //(void) pthread_create( &thread1, &tattr1, (void*) getButtonPress, (void*) buttonPort);
    //Thread
    //(void) pthread_create( &thread2, &tattr2, (void *) bno055, NULL);
    //GPS Thread
    (void) pthread_create( &thread3, &tattr3, (void *) readGPS, NULL);
    //Rangefinder Thread
    //(void) pthread_create( &thread4, &tattr4, (void *) rangeFinder, NULL);

    (void) pthread_join(thread3, NULL);

	return 0;
}

void print_calstat() {
    struct bnocal bnoc;
    /* -------------------------------------------------------- *
     *  Check the sensors calibration state                     *
     * -------------------------------------------------------- */
    int res = get_calstatus(&bnoc);
    if(res != 0) {
        printf("Error: Cannot read calibration state.\n");
        exit(-1);
    }

    /* -------------------------------------------------------- *
     *  Convert the status code into a status message           *
     * -------------------------------------------------------- */
    printf("Sensor System Calibration = ");
    switch(bnoc.scal_st) {
        case 0:
            printf("Uncalibrated\n");
            break;
        case 1:
            printf("Minimal Calibrated\n");
            break;
        case 2:
            printf("Mostly Calibrated\n");
            break;
        case 3:
            printf("Fully calibrated\n");
            break;
    }

    printf("    Gyroscope Calibration = ");
    switch(bnoc.gcal_st) {
        case 0:
            printf("Uncalibrated\n");
            break;
        case 1:
            printf("Minimal Calibrated\n");
            break;
        case 2:
            printf("Mostly Calibrated\n");
            break;
        case 3:
            printf("Fully calibrated\n");
            break;
    }

    printf("Accelerometer Calibration = ");
    switch(bnoc.acal_st) {
        case 0:
            printf("Uncalibrated\n");
            break;
        case 1:
            printf("Minimal Calibrated\n");
            break;
        case 2:
            printf("Mostly Calibrated\n");
            break;
        case 3:
            printf("Fully calibrated\n");
            break;
    }

    printf(" Magnetometer Calibration = ");
    switch(bnoc.mcal_st) {
        case 0:
            printf("Uncalibrated\n");
            break;
        case 1:
            printf("Minimal Calibrated\n");
            break;
        case 2:
            printf("Mostly Calibrated\n");
            break;
        case 3:
            printf("Fully calibrated\n");
            break;
    }
}

void getCalStatus() {
    struct bnocal bnoc;
    /* -------------------------------------------------------- *
     *  Read the sensors calibration state                      *
     * -------------------------------------------------------- */
    int res = get_calstatus(&bnoc);
    if(res != 0) {
        printf("Error: Cannot read calibration state.\n");
        exit(-1);
    }
    /* -------------------------------------------------------- *
     *  Read the sensors calibration offset                     *
     * -------------------------------------------------------- */
    res = get_caloffset(&bnoc);
    if(res != 0) {
        printf("Error: Cannot read calibration data.\n");
        exit(-1);
    }

    /* -------------------------------------------------------- *
     *  Print the calibration data line                         *
     * -------------------------------------------------------- */
    printf("sys [S:%d]", bnoc.scal_st);
    printf(" acc [S:%d ", bnoc.acal_st);
    printf("X:%d Y:%d Z:%d", bnoc.aoff_x, bnoc.aoff_y, bnoc.aoff_z);
    printf(" R:%d]", bnoc.acc_rad);

    printf(" mag [S:%d ", bnoc.mcal_st);
    printf("X:%d Y:%d Z:%d", bnoc.moff_x, bnoc.moff_y, bnoc.moff_z);
    printf(" R:%d]", bnoc.mag_rad);

    printf(" gyr [S:%d ", bnoc.gcal_st);
    printf("X:%d Y:%d Z:%d]\n", bnoc.goff_x, bnoc.goff_y, bnoc.goff_z);
}

void getBno055Info() {
    struct bnoinf bnoi;
    int res = get_inf(&bnoi);
    /* ----------------------------------------------------------- *
       * print the formatted output strings to stdout                *
       * ----------------------------------------------------------- */
    printf("\nBN0055 Information");
    printf("----------------------------------------------\n");
    printf("   Chip Version ID = 0x%02X\n", bnoi.chip_id);
    printf("  Accelerometer ID = 0x%02X\n", bnoi.acc_id);
    printf("      Gyroscope ID = 0x%02X\n", bnoi.gyr_id);
    printf("   Magnetoscope ID = 0x%02X\n", bnoi.mag_id);
    printf("  Software Version = %d.%d\n", bnoi.sw_msb, bnoi.sw_lsb);
    printf("   Operations Mode = "); print_mode(bnoi.opr_mode);
    printf("        Power Mode = "); print_power(bnoi.pwr_mode);
    printf("Axis Configuration = "); print_remap_conf(bnoi.axr_conf);
    printf("   Axis Remap Sign = "); print_remap_sign(bnoi.axr_sign);
    printf("System Status Code = "); print_sstat(bnoi.sys_stat);
    printf("System Clocksource = "); print_clksrc();

    printf("Accelerometer Test = ");
    if((bnoi.selftest >> 0) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf(" Magnetometer Test = ");
    if((bnoi.selftest >> 1) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf("    Gyroscope Test = ");
    if((bnoi.selftest >> 2) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf("MCU Cortex M0 Test = ");
    if((bnoi.selftest >> 3) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf(" System Error Code = ");
    switch(bnoi.sys_err) {
        case 0x00:
            printf("No Error\n");
            break;
        case 0x01:
            printf("Peripheral initialization error\n");
            break;
        case 0x02:
            printf("System initializion error\n");
            break;
        case 0x03:
            printf("Selftest result failed\n");
            break;
        case 0x04:
            printf("Register map value out of range\n");
            break;
        case 0x05:
            printf("Register map address out of range\n");
            break;
        case 0x06:
            printf("Register map write error\n");
            break;
        case 0x07:
            printf("BNO low power mode not available\n");
            break;
        case 0x08:
            printf("Accelerometer power mode not available\n");
            break;
        case 0x09:
            printf("Fusion algorithm configuration error\n");
            break;
        case 0x0A:
            printf("Sensor configuration error\n");
            break;
    }

    print_unit(bnoi.unitsel);

    printf("Sensor Temperature = ");
    if(bnoi.opr_mode > 0) {
        if((bnoi.unitsel >> 4) & 0x01) printf("%d°F\n", bnoi.temp_val);
        else printf("%d°C\n",bnoi.temp_val);
    }
    else  printf("no data in CONFIG mode\n");

    printf("\n----------------------------------------------\n");
    struct bnoaconf bnoac;
    if(get_acc_conf(&bnoac) == 0) print_acc_conf(&bnoac);

    printf("\n----------------------------------------------\n");
    print_calstat();
}

void bno055() {
    char senaddr[256] = "0x28";
    char i2c_bus[256] = I2CBUS;

    get_i2cbus(i2c_bus, senaddr);
    ////////SET MODE
    int res = set_mode(compass);
    if(res != 0) {
        printf("Error: could not set sensor mode \n");
        exit(-1);
    }

    ////////CALIBRATION STATUS
    struct bnocal bnoCalibrate;
    /* -------------------------------------------------------- *
     *  Read the sensors calibration state                      *
     * -------------------------------------------------------- */

    getBno055Info();
    getCalStatus();
    /*
    int gyrCalReady = 0;
    while(!gyrCalReady) {
        res = get_calstatus(&bnoCalibrate);
        if(res != 0) {
            printf("Error: Cannot read calibration state.\n");
            exit(-1);
        }
        printf(" gyr [S:%d ", bnoCalibrate.gcal_st);
        if(bnoCalibrate.gcal_st == 3) {
            gyrCalReady = 1;
        }
    }

    struct bnogyr bnod;
    while(1) {
        res = get_gyr(&bnod);
        if(res != 0) {
            printf("Error: Cannot read gyroscope data.\n");
            exit(-1);
        }

        printf("GYR %3.2f %3.2f %3.2f\n", bnod.gdata_x, bnod.gdata_y, bnod.gdata_z);
    }

    //struct bnoeul bnod;
    //res = get_eul(&bnod);
    //printf("EUL %3.4f %3.4f %3.4f\n", bnod.eul_head, bnod.eul_roll, bnod.eul_pitc);
    //struct bnomag bnodMag;
    //res = get_mag(&bnodMag);
    //printf("MAG %3.2f %3.2f %3.2f\n", bnodMag.mdata_x, bnodMag.mdata_y, bnodMag.mdata_z);
     */
}



void printDisplay() {
    /* Initialize I2C bus and connect to the I2C Device */
    if(init_i2c_dev(I2C_DEV2_PATH, SSD1306_OLED_ADDR) == 0)
    {
        printf("(Main)i2c-2: Bus Connected to SSD1306\r\n");
    }
    else
    {
        printf("(Main)i2c-2: OOPS! Something Went Wrong\r\n");
        exit(1);
    }

    display_Init_seq();

    /* Clear display */
    clearDisplay();

    setTextSize(1);
    setTextColor(WHITE);
    setCursor(1,0);
    print_strln("deeplyembedded.org");
    println();
    print_strln("Author:Vinay Divakar");
    println();
    println();
    print_strln("THANK YOU");

    Display();
}

void readGPS() {
    unsigned char cmd3[] = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28";

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

    printf("Antenna messages\n");
    write(serialPort, cmd3, sizeof(cmd3));  //Turn visible laser on
    sleep(1);

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
        /////////////TODO: MUTEX AND INFODUMP HERE
        printf("%s\n", read_buf);
        fflush(stdout);
        /////////////////////////////////////
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
    int continuousMeasurementMode = 1;
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
    printf("Beginning read\n");
    while(1) {
        while(1) {
            int n = read(serialPort, (void *) (&read_buf), 11);

            if (n < 0) {
                printf("Unresponsive\n");
            } else {
                printf("Raw data: %s\n", read_buf);
                strncpy(test_buf, read_buf + 3, 7);
                /////////////TODO: MUTEX AND INFODUMP HERE
                printf("Parsed: %s\n", test_buf);
                fflush(stdout);
                //////////////////////////////////
            }
        }
    }
}

void getButtonPress(void *buttonPort) {
    uint32_t pressedFlag = 0;
    uint32_t signalSentFlag = 0;
    uint32_t gpioValue;
    printDisplay();
    while(1) {
        gpioValue = readGPIO("/value", (char *) buttonPort);
        if(gpioValue == 1){
            //first press detected
            if(pressedFlag == 0) {
                pressedFlag = 1;
                //If the buttonPort corresponds with start/stop
                printf("hellosuip\n");
                fflush(stdout);
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