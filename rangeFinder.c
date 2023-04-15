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

//comment out to live run
//#define DEBUG 1

#define GPIO_PATH_66 "/sys/class/gpio/gpio66" //Start/Stop Button

//Writes specified value to specified GPIO directory
static void writeGPIO(char *filename, char *port, char *value);

//Reads input to GPIO pin
static uint32_t readGPIO(char *filename, char *port);

//Primary button press detection
void getButtonPress(void *buttonPort);

void readGPS();
void rangeFinder()

pthread_mutex_t timerMutex;
float timerInMilliseconds;

pthread_mutex_t runningStateMutex;
int watchRunningState;

struct termios tty;
int serialPort = -1;

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

    (void) pthread_create( &thread1, &tattr1, (void*) getButtonPress, (void*) buttonPort);
    (void) pthread_create( &thread3, &tattr3, (void *) readGPS, NULL);
    //(void) pthread_create( &thread4, &tattr4, (void *) displayTimerThread, NULL);

    (void) pthread_join(thread1, NULL);

	return 0;
}

void readGPS() {
    serialPort = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
    struct termios options;
    tcgetattr(serialPort, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serialPort, TCIFLUSH);
    tcsetattr(serialPort, TCSANOW, &options);

    char read_buf [256];

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
    byte cmd_2[] = { 0x80, 0x06, 0x02, 0x78 };         // Single Measurement Mode
    byte cmd_3[] = { 0x80, 0x06, 0x05, 0x01, 0x74 };   // LaserPointerOn
    byte cmd_4[] = { 0x80, 0x06, 0x05, 0x00, 0x75 };   // LaserPointerOff

    byte cmd_5[] = { 0xFA, 0x04, 0x09, 0x05, 0xF4 };   // 5m Range
    byte cmd_6[] = { 0xFA, 0x04, 0x09, 0x0A, 0xEF };   // 10m Range
    byte cmd_7[] = { 0xFA, 0x04, 0x09, 0x1E, 0xDB };   // 30m Range
    byte cmd_8[] = { 0xFA, 0x04, 0x09, 0x32, 0xC7 };   // 50m Range
    byte cmd_9[] = { 0xFA, 0x04, 0x09, 0x50, 0xA9 };   // 80m Range

    byte cmd_10[] = { 0xFA, 0x04, 0x0C, 0x02, 0xF4 };   // 0.1mm Resolution
    byte cmd_11[] = { 0xFA, 0x04, 0x0C, 0x01, 0xF5 };   // 1mm Resolution

    ////////////SETUP

}

void getButtonPress(void *buttonPort) {
    uint32_t pressedFlag = 0;
    uint32_t signalSentFlag = 0;
    uint32_t gpioValue;
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