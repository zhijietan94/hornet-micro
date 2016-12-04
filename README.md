# Hornet 2016 Micro Ino Code

## Description:
Ino code for 3rd party Micro that:
- controls grabber's servo
- receives I2C data from GY-87 IMU
- receives analog data from Pololu current sensor (measuring how much current is drawn from the power source)
- receives analog data from voltage divider circuit (measuring the voltage of the power source)
- displays power status using addressable RGB LED WS2812B
- displays operating status using addressable RGB LED WS2812B

## How to use:
- Ensure that you have the latest version of Arduino IDE installed (https://www.arduino.cc/)
- Download the micro.ino file and place it in a folder called "micro" (Folder name must be the same as the ino file name)
- Ensure you install the required libraries (Refer to Appendix A: Libraries below)
- Double click to open it with Arduino IDE, or from Arduino IDE : File > Open and locate the .ino file.
- Download to the 3rd party Micro while selecting "Board: Arduino Leonardo"
- Open Serial Monitor

# Credits
- Hornet 2016 Team
- Many thanks to https://github.com/jarzebski for his GY-87 MPU6050 Library and HMC5883L Library

# Appendix A: Libraries
- IMU: HMC5883L - https://github.com/jarzebski/Arduino-HMC5883L (cannot be found in Library Manager)
- IMU: MPU6050 - https://github.com/jarzebski/Arduino-MPU6050 (cannot be found in Library Manager)
- Addressable RGB LED: Adafruit NeoPixel Library - https://github.com/adafruit/Adafruit_NeoPixel (can download from Library Manager in Arduino IDE)
For information on how to install libraries, google it.
