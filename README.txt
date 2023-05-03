Installation and Compilation Instructions

The following libraries were used:
https://github.com/deeplyembeddedWP/SSD1306-OLED-display-driver-for-BeagleBone
https://github.com/fm4dd/pi-bno055

Code can also be found here: https://github.com/yang9501/rangeFinder

The code is compiled with gcc ver. 8.3.0 and run on Linux kernel 4.19.94-ti-rt-r42.  In order to compile, run the following instruction:

gcc -pthread rangeFinder.c ./SSD1306-OLED-display-driver-for-BeagleBone-master/SSD1306_OLED_Library/SSD1306_OLED.c ./SSD1306-OLED-display-driver-for-BeagleBone-master/I2C_Library/I2C.c ./pi-bno055-master/i2c_bno055.c ./unitTests.c -lm -o rangeFinder

Then run the following to begin the program:

./rangeFinder
