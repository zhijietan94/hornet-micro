#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>

#define DEBUG false //Set to true to see detailed debugging print messages

//------POWER MANAGEMENT--------
#define CURR_SENSOR A2
#define CURR_OFFSET 530 //at this value when at 0A.
#define VOLT_SENSOR A3
#define VOLT_OFFSET 1130
#define ROLL_AVG_VALUE 40
#define BATT_MAX_VOLT 16.8
#define BATT_MIN_VOLT 14.5
#define BATT_LOW_THRESHOLD 15.5
#define BATT_MAX_CHARGE 21600 // 3600 sec * max 6A for 1 hour
#define BATT_LOW_CHARGE 5000

//------SERVO MANAGEMENT--------
#define PIN_SERVO 10
#define SERVO_HOLD 1500
#define SERVO_RELEASE 1900

//------LED STRIPS--------------
#define PIN_POWER_LED 8
#define PIN_STATUS_LED 7
#define NUM_LED_POWER 3
#define NUM_LED_STATUS 3
#define CLR_OFF 0
#define CLR_BLUE 1
#define CLR_GREEN 2
#define CLR_YELLOW 3
#define CLR_PINK 4
#define CLR_RED 5
#define CLR_WHITE 6

unsigned long serialTimer = 0;
unsigned long serialInterval = 100;
char msg[2];
boolean msgRead = false;

//------IMU MANAGEMENT----------
HMC5883L compass;
MPU6050 mpu;
// Timers
unsigned long gyroTimer = 0;
float gyroTimeStep = 0.01;
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float heading = 0;
float accelXYZ[3];

//------POWER MANAGEMENT--------
int curr_avg[ROLL_AVG_VALUE + 1];
int volt_avg[ROLL_AVG_VALUE + 1];
int curr_idx = 1;
int volt_idx = 1;
unsigned int curr_sum = 0;
unsigned int volt_sum = 0;
unsigned long power_timer = 0;
unsigned long time_interval = 1000;
float charge = BATT_MAX_CHARGE;
float voltage = 0;
float current = 0;

//------SERVO MANAGEMENT--------
boolean servo_release = false;
Servo servo;

//------LED STRIPS--------------
Adafruit_NeoPixel power_led = Adafruit_NeoPixel(NUM_LED_POWER, PIN_POWER_LED, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel status_led = Adafruit_NeoPixel(NUM_LED_STATUS, PIN_STATUS_LED, NEO_GRB + NEO_KHZ800);

void setup() {
  //Setup pins
  pinMode(CURR_SENSOR, INPUT);
  pinMode(VOLT_SENSOR, INPUT);

  //---------LED STRIPS---------------
  power_led.begin();
  power_led.setBrightness(255);
  power_led.show();
  setPowerLED(CLR_PINK);
  status_led.begin();
  status_led.setBrightness(255);
  status_led.show();
  setStatusLED(CLR_PINK);
  //---------END OF LED STRIPS--------

  //Wait for serial to be established
  while (!Serial) {
    Serial.begin(9600);
  }
  if (DEBUG) Serial.println("Serial established");
  //---------SERVO MANAGEMENT---------
  //servo.attach(PIN_SERVO);
  //servo.writeMicroseconds(SERVO_HOLD);
  //servo_release = false
  //-----END OF SERVO MANAGEMENT------

  //---------IMU MANAGEMENT-----------  
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    delay(50);
  }
  if (DEBUG) Serial.println("mpu established");
  delay(500);
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);
  while (!compass.begin()) {
    delay(50);
  }
  if (DEBUG) Serial.println("compass established");
  delay(500);
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(-154, -75);
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);

  //Accelerometer
  accelXYZ[0] = 0;
  accelXYZ[1] = 0;
  accelXYZ[2] = 0;
  //------END OF IMU MANAGEMENT-------
  
  setStatusLED(CLR_OFF);

  //---------POWER MANAGEMENT---------
  //Init rolling average arrays with equal value
  for (int i = 1; i <= ROLL_AVG_VALUE; i++) {
    curr_avg[i] = analogRead(CURR_SENSOR);
    curr_sum += curr_avg[i];
    volt_avg[i] = analogRead(VOLT_SENSOR);
    volt_sum += volt_avg[i];
  }
  //Wait for power source > 14.4V to be detected 
  if (DEBUG) Serial.println("Waiting to detect power source > 14.4V...");
  do {
      updateVolt();
      updateAvgVoltCurr();
  } while (voltage < BATT_MIN_VOLT);

  if (DEBUG) Serial.println("Detected power source, waiting for 2s to start up..");
  power_timer = millis();
  while (millis() - power_timer < 2000) {
    updateVolt();
    updateAvgVoltCurr();
  }

  if (DEBUG) Serial.println("Beginning operations...");
  //Initialise variable
  power_timer = millis();
  //Initialise battery charge count based on voltage
  setChargeUsingVoltage();
  //------------END OF POWER MANAGEMENT-------

  gyroTimer = millis();
  serialTimer = millis();
}

void loop() {
  if (millis() - serialTimer > serialInterval) {
    writeSerial();
    readSerial();
    serialTimer = millis();
  }
  if (millis() - power_timer > time_interval) {
    updateCharge();
    power_timer = millis();
  }
  if ((millis() - gyroTimer) > (gyroTimeStep * 1000)) {
    updateIMU();
    updateAvgVoltCurr();
    gyroTimer = millis();
  }
}

/*
 * Sends IMU information and Power (Voltage and Charge) over serial. 
 */
void writeSerial() {
  String toSend = String("");
  toSend = toSend + "Sh/" + getHeading() + "p/" + getPitch() + "r/" + getRoll() + "y/" + getYaw() + "ax/" + getAccelX() + "ay/" + getAccelY() + "az/" + getAccelZ() + "v/" + getVoltage() + "c/" + getCharge() + "E"; 
  Serial.println(toSend);
}

/*
 * Reads serial data from Odroid.
 * Expected data format: S**E, where * refers to a number between 0-9 (inclusive)
 * First number indicates the state of the vehicle (i.e. ready, launching, idle, etc)
 * Second number indicates the state of the servo (lock = 0, release = 1)
 * 
 * Data integrity is ensured by only accepting data of the exact format (prevent any loss of data in between/gaps)
 */
void readSerial() {
  if (Serial.available() > 0){
    //Reads starting character
    char sChar = Serial.read();
    if (sChar == 'S'){
      msgRead = true;
      //Read 2 data bytes
      for (int j=0;j<2;j++){
        char data = Serial.read();
        if (data >= '0' && data <= '9') {
          msg[j] = data; 
        } else {
          msgRead = false;
          break;
        }
      }
      //Read terminating character
      char eChar = Serial.read();
      if (eChar != 'E') {
        msgRead = false;
      }
    }
    
    if (msgRead) { //Message successfully read
      //Set status LED based on msg[0]
      setStatusLED((msg[0] - '0'));
      //Actuate servo depending on msg[1]
      if (servo_release == false && msg[1] == '1') {
        //servo.writeMicroseconds(SERVO_RELEASE);
        servo_release = true;
        if (DEBUG) Serial.println("Servo Released");
      }
      msgRead = false;
    }
  }
}

//------POWER MANAGEMENT--------
void updateCharge() {
  updateCurr();
  updateVolt();
  countCharge();

  if (DEBUG) {
    Serial.print(" || Current:");
    Serial.print(current);
    Serial.print(" || Voltage:");
    Serial.print(voltage);
    Serial.print(" || Charge:");
    Serial.println(charge);
  }
}

/*
 * Updates the rolling average of the current and voltage values
 */
void updateAvgVoltCurr() {

  //Remove oldest value
  curr_sum -= curr_avg[curr_idx];
  volt_sum -= volt_avg[volt_idx];

  //Replace oldest value with new value
  curr_avg[curr_idx] = analogRead(CURR_SENSOR);
  volt_avg[volt_idx] = analogRead(VOLT_SENSOR);

  //Add newest value
  curr_sum += curr_avg[curr_idx];
  volt_sum += volt_avg[volt_idx];

  //Increment index
  curr_idx = (curr_idx < ROLL_AVG_VALUE) ? curr_idx + 1 : 1;
  volt_idx = (volt_idx < ROLL_AVG_VALUE) ? volt_idx + 1 : 1;
}

/*
 * Sets the value of voltage to the rolling average
 */
void updateVolt() {
  //voltage = 0.0177 * (volt_sum / ROLL_AVG_VALUE) - 0.6665;  //Formula obtained by plotting in excel - Data: 17.0V 999 16.8V 988 15.8V 933 15.3 899 14.4 853 14.0V 830 
  float rawReading = map(volt_sum/ROLL_AVG_VALUE, 0, 1023, 0, 5000);
  voltage = rawReading / 5100 * (5100 + 15000) - VOLT_OFFSET; //5100 and 15000 are the resistor values
}

String getVoltage() {
  String value = String(voltage);
  return value;
}

/*
 * Sets the value of current to the rolling average
 */
void updateCurr() {
  current = (curr_sum / ROLL_AVG_VALUE - CURR_OFFSET) / (14.0 / 5) * 0.1;
}

String getCharge() {
  String value = String(charge);
  String output = String("");
  switch (value.length()) {
    case 2:
      output = "000" + value;
      break;

    case 3:
      output = "00" + value;
      break;

    case 4:
      output = "0" + value;
      break;

    default:
      output = value;
      break;
  }
  return output;
}

/*
 * Counts the amount of charge used during the last time_interval based on Charge = Current * Time
 * Sets the power LED lights to correspond to the existing voltage of the 
 */
void countCharge() {
  charge -= current * ((float)time_interval / 1000.0);
  if (voltage < BATT_MIN_VOLT) {
    setPowerLED(CLR_RED);
  } else if (voltage < BATT_LOW_THRESHOLD) {
    setPowerLED(CLR_YELLOW);
  } else {
    setPowerLED(CLR_GREEN);
  }
}

/*
 * This function sets the charge of the power source based on the existing voltage of the source.
 * Mapping range is pre-determined manually
 */ 
void setChargeUsingVoltage() {
  if (voltage < BATT_LOW_THRESHOLD) {
    charge = map(voltage * 1000, BATT_MIN_VOLT * 1000, BATT_LOW_THRESHOLD * 1000, 0, BATT_LOW_CHARGE);
  } else { //voltage normal range
    charge = map(voltage * 1000, BATT_LOW_THRESHOLD * 1000, BATT_MAX_VOLT * 1000, BATT_LOW_CHARGE, BATT_MAX_CHARGE);
  }
}
//----END OF POWER MANAGEMENT-------

//---------LED STRIPS---------------
void setPowerLED(int colour) {
  int rgbVal[3];
  switch (colour) {
    case CLR_GREEN:
      rgbVal[0] = 0;
      rgbVal[1] = 150;
      rgbVal[2] = 0;
      break;

    case CLR_YELLOW:
      rgbVal[0] = 150;
      rgbVal[1] = 150;
      rgbVal[2] = 0;
      break;

    case CLR_PINK:
      rgbVal[0] = 255;
      rgbVal[1] = 0;
      rgbVal[2] = 255;
      break;

    case CLR_RED:
      rgbVal[0] = 150;
      rgbVal[1] = 0;
      rgbVal[2] = 0;
      break;

    case CLR_BLUE:
      rgbVal[0] = 0;
      rgbVal[1] = 0;
      rgbVal[2] = 150;
      break;

    case CLR_WHITE:
      rgbVal[0] = 255;
      rgbVal[1] = 255;
      rgbVal[2] = 255;
      break;

    case CLR_OFF: //fall through
    default:
      rgbVal[0] = 0;
      rgbVal[1] = 0;
      rgbVal[2] = 0;
      break;
  }

  for (int i = 0; i < NUM_LED_POWER; i++) {
    power_led.setPixelColor(i, power_led.Color(rgbVal[0], rgbVal[1], rgbVal[2]));
  }
  power_led.show();
}

void setStatusLED(int colour) {
  int rgbVal[3];
  switch (colour) {
    case CLR_GREEN:
      rgbVal[0] = 0;
      rgbVal[1] = 150;
      rgbVal[2] = 0;
      break;

    case CLR_YELLOW:
      rgbVal[0] = 150;
      rgbVal[1] = 150;
      rgbVal[2] = 0;
      break;

    case CLR_PINK:
      rgbVal[0] = 255;
      rgbVal[1] = 0;
      rgbVal[2] = 255;
      break;

    case CLR_RED:
      rgbVal[0] = 150;
      rgbVal[1] = 0;
      rgbVal[2] = 0;
      break;

    case CLR_BLUE:
      rgbVal[0] = 0;
      rgbVal[1] = 0;
      rgbVal[2] = 150;
      break;
          
    case CLR_WHITE:
      rgbVal[0] = 255;
      rgbVal[1] = 255;
      rgbVal[2] = 255;
      break;
  
    case CLR_OFF: //fall through
    default:
      rgbVal[0] = 0;
      rgbVal[1] = 0;
      rgbVal[2] = 0;
      break;
  }

  for (int i = 0; i < NUM_LED_STATUS; i++) {
    status_led.setPixelColor(i, status_led.Color(rgbVal[0], rgbVal[1], rgbVal[2]));
  }
  status_led.show();
}
//------END OF LED STRIPS-------


//------IMU MANAGEMENT----------
void updateIMU() {
  //Update accelerometer
  Vector normAccel = mpu.readNormalizeAccel();
  accelXYZ[0] = normAccel.XAxis;
  accelXYZ[1] = normAccel.YAxis;
  accelXYZ[2] = normAccel.ZAxis;
  
  //Update Gyro
  Vector normG = mpu.readNormalizeGyro();
  pitch = pitch + normG.YAxis * gyroTimeStep;
  roll = roll + normG.XAxis * gyroTimeStep;
  yaw = yaw + normG.ZAxis * gyroTimeStep;

  if (DEBUG) {
    Serial.print(" || Accel[X]:");
    Serial.print(accelXYZ[0]);
    Serial.print(" || Accel[Y]:");
    Serial.print(accelXYZ[1]);
    Serial.print(" || Accel[Z]:");
    Serial.print(accelXYZ[2]);
    
    Serial.print(" || Pitch:");
    Serial.print(pitch);
    Serial.print(" || Roll:");
    Serial.print(roll);
    Serial.print(" || Yaw:");
    Serial.println(yaw);
  }
  
  //Update Compass
  Vector normC = compass.readNormalize();
  float headingRad = atan2(normC.YAxis, normC.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  headingRad += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (headingRad < 0) {
    headingRad += 2 * PI;
  }
  if (headingRad > 2 * PI) {
    headingRad -= 2 * PI;
  }
  heading = headingRad * 180 / M_PI;
}

/*
 * Formats the compass heading into a printable format of specific length
 */
String getHeading() {
  String degree = String(heading);
  String output = String("");
  switch (degree.length()) {
    case 4:
      output = "00" + degree;
      break;

    case 5:
      output = "0" + degree;
      break;

    default:
      output = degree;
      break;
  }
  return output;
}

/*
 * Formats the pitch angle into a printable format of specific length
 */
String getPitch() {
  String data = String(pitch);
  String output = processPYR(data);
  return output;
}

/*
 * Formats the roll angle into a printable format of specific length
 */
String getRoll() {
  String data = String(roll);
  String output = processPYR(data);
  return output;
}

/*
 * Formats the yaw angle into a printable format of specific length
 */
String getYaw() {
  String data = String(yaw);
  String output = processPYR(data);
  return output;
}

/*
 * Pads pyr data with negative/positive sign and leading zeros 
 */
String processPYR(String data) {
  String output = String("");
  if (data.startsWith("-")) {
    switch (data.length()) {
      case 5:
        output = "-00" + data.substring(1);
        break;

      case 6:
        output = "-0" + data.substring(1);
        break;

      default:
        output = "-" + data.substring(1);
        break;
    }
  } else {
    switch (data.length()) {
      case 4:
        output = "+00" + data;
        break;

      case 5:
        output = "+0" + data;
        break;

      default:
        output = "+" + data;
        break;
    }
  }
  return output;
}

/*
 * Formats accelX into a printable format of specific length
 */
String getAccelX() {
  String data = String(accelXYZ[0]);
  String output = processAccel(data);
  return output;
}

/*
 * Formats accelY into a printable format of specific length
 */
String getAccelY() {
  String data = String(accelXYZ[1]);
  String output = processAccel(data);
  return output;
}

/*
 * Formats accelZ into a printable format of specific length
 */
String getAccelZ() {
  String data = String(accelXYZ[2]);
  String output = processAccel(data);
  return output;
}

/*
 * Pads accel data with negative/positive sign and leading zeros 
 */
String processAccel(String data){
  String output = String("");
  if (data.startsWith("-")) {
    switch (data.length()) {
      case 5:
        output = "-0" + data.substring(1);
        break;

      default:
        output = "-" + data.substring(1);
        break;
    }
  } else {
    switch (data.length()) {
      case 4:
        output = "+0" + data;
        break;

      default:
        output = "+" + data;
        break;
    }
  }
  return output;
}

