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

//comment out to live run
//#define DEBUG 1

#define GPIO_PATH_66 "/sys/class/gpio/gpio66" //Start/Stop Button

//Reads input to GPIO pin
static uint32_t readGPIO(char *filename, char *port);

//Primary button press detection
void getButtonPress(void *buttonPort);

void msleep(long msec);
void updateTimerThread();
void displayTimerThread();

pthread_mutex_t timerMutex;
float timerInMilliseconds;

pthread_mutex_t runningStateMutex;
int watchRunningState;

int main(void) {
    //arrays containing GPIO port definitions, representing the green and red lights, and the start/stop and reset buttons
	char buttonPorts[25] = GPIO_PATH_66; //buttonPorts[0] is the start/stop

    #ifdef DEBUG
    (void) printf("DEBUG MODE\n");
    struct utsname sysInfo;
    (void) uname(&sysInfo);
    (void) printf("%s\n", sysInfo.sysname);
    (void) printf("%s\n", sysInfo.nodename);
    (void) printf("%s\n", sysInfo.machine);
    #else
    (void) writeLED("/direction", buttonPorts, "in");
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

    (void) pthread_create( &thread1, &tattr1, (void*) getButtonPress, (void*) buttonPorts[0]);
    //(void) pthread_create( &thread3, &tattr3, (void *) updateTimerThread, NULL);
    //(void) pthread_create( &thread4, &tattr4, (void *) displayTimerThread, NULL);

    (void) pthread_join(thread1, NULL);

	return 0;
}

//Sleep for the requested number of milliseconds
void msleep(long milliseconds) {
    struct timespec ts;

    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, &ts);
}

//Updates the timer counter every 10ms.  When timer reaches the max of float value, rolls over to 0.
void updateTimerThread() {
    while(1) {
        (void) pthread_mutex_lock(&runningStateMutex);
        if(watchRunningState == 1) {
            (void) pthread_mutex_lock(&timerMutex);
            //If the counter value + 10 would be greater than the maximum value of float, rollover to zero
            if(FLT_MAX - 10 < timerInMilliseconds) {
                timerInMilliseconds = 0.0f;
            }
            timerInMilliseconds = timerInMilliseconds + 10.0f;
            (void) pthread_mutex_unlock(&timerMutex);
        }
        (void) pthread_mutex_unlock(&runningStateMutex);
        //Maintain to resolution of 10 milliseconds
        msleep(10);
    }
}

//Outputs timer to stdout every 100ms when running state is 'on'
void displayTimerThread() {
    while(1) {
        (void) pthread_mutex_lock(&runningStateMutex);
        if (watchRunningState == 1) {
            (void) pthread_mutex_lock(&timerMutex);
            printf("%.1f\n", timerInMilliseconds/1000.0f);
            fflush(stdout);
            (void) pthread_mutex_unlock(&timerMutex);
        }
        (void) pthread_mutex_unlock(&runningStateMutex);
        //update terminal display every 100 milliseconds
        msleep(100);
    }
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
                if(signalSentFlag == 0) {
                    //If the buttonPort corresponds with start/stop
                    if(strcmp((char*) buttonPort,  GPIO_PATH_66) == 0) {
                        printf("hello\n");
                        fflush(stdout);
                        signalSentFlag = 1;
                        /*
                        (void) pthread_mutex_lock(&runningStateMutex);
                        //if the watch is currently running, stop it
                        if(watchRunningState == 1) {
                            (void) pthread_mutex_unlock(&runningStateMutex);
                            stopWatch();
                            signalSentFlag = 1;
                        }
                        //if the watch is currently stopped, start it
                        else if(watchRunningState == 0) {
                            (void) pthread_mutex_unlock(&runningStateMutex);
                            startWatch();
                            signalSentFlag = 1;
                        }*/
                    }
                }
            }
        }
        if(gpioValue == 0) {
            //if the button is let go after being pressed
            if(pressedFlag == 1) {
                pressedFlag = 0;
                signalSentFlag = 0;
            }
        }
        //Read buttons every 10 milliseconds
        msleep(10);
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