#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG false //Set to true to see detailed debugging print messages

//------IMU MANAGEMENT----------
#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>
HMC5883L compass;
MPU6050 mpu;

//------POWER MANAGEMENT--------
#define CURR_SENSOR A2
#define CURR_OFFSET 530 //at this value when at 0A.
#define VOLT_OFFSET 290 //set this after running the battery. 
#define VOLT_AT_OFFSET 3.6 //analog reading 290 is at 5V. 
#define VOLT_SENSOR A3
#define ROLL_AVG_VALUE 20
#define BATT_MAX_VOLT 16.8
#define BATT_MIN_VOLT 14.4
#define BATT_LOW_THRESHOLD 15.0
#define BATT_HIGH_THRESHOLD 15.8
#define BATT_MAX_CHARGE 21600 // 3600 sec * max 6A for 1 hour
#define BATT_HIGH_CHARGE 20000
#define BATT_LOW_CHARGE 5000

//------SERVO MANAGEMENT--------
#define PIN_SERVO 9
#define SERVO_HOLD 40
#define SERVO_RELEASE 130

//------LED STRIPS--------------
#define PIN_POWER_LED 8
#define PIN_STATUS_LED 7
#define NUM_LED_POWER 3
#define NUM_LED_STATUS 3
#define CLR_GREEN 1
#define CLR_YELLOW 2
#define CLR_RED 3
#define CLR_BLUE 4
#define CLR_OFF 0

unsigned long serialTimer = 0;
unsigned long serialInterval = 100;
char msg[2];
boolean msgRead = false;

//------IMU MANAGEMENT----------
// Timers
unsigned long gyroTimer = 0;
float gyroTimeStep = 0.01;
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float heading = 0;

//------POWER MANAGEMENT--------
int curr_avg[ROLL_AVG_VALUE + 1];
int volt_avg[ROLL_AVG_VALUE + 1];
int curr_idx = 1;
int volt_idx = 1;
int curr_sum = 0;
int volt_sum = 0;
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

  //---------SERVO MANAGEMENT---------
  //servo.attach(PIN_SERVO);
  //servo.write(SERVO_HOLD);
  //-----END OF SERVO MANAGEMENT------

  //---------LED STRIPS---------------
  power_led.begin();
  power_led.show();
  setPowerLED(CLR_BLUE);
  status_led.begin();
  status_led.show();
  setStatusLED(CLR_YELLOW);
  //---------END OF LED STRIPS--------

  //Wait for serial to be established
  while (!Serial) {
    Serial.begin(115200);
  }

  //---------IMU MANAGEMENT-----------  
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G));
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);
  while (!compass.begin());
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  //------END OF IMU MANAGEMENT-------
  
  setStatusLED(CLR_GREEN);

  //---------POWER MANAGEMENT---------
  //Populate both arrays with equal value for start
  for (int i = 0; i < 10; i++) {
    curr_avg[i] = analogRead(CURR_SENSOR);
    curr_sum += curr_avg[i];
    volt_avg[i] = analogRead(VOLT_SENSOR);
    volt_sum += volt_avg[i];
  }
  //Wait for battery to be detected
    if (DEBUG) Serial.println("Waiting to detect battery...");
    do {
      updateVolt();
      updateAvgVoltCurr();
    } while (voltage < BATT_MIN_VOLT);

  if (DEBUG) Serial.println("Detected battery, waiting for 5s to start up..");
  power_timer = millis();
  while (millis() - power_timer < 5000) {
    updateVolt();
    updateAvgVoltCurr();
  }

  if (DEBUG) Serial.println("Starting counting mode..");
  //Initialise variable
  power_timer = millis();
  //Initialise battery charge count based on voltage
  setChargeUsingVoltage(voltage);
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
  toSend = toSend + "Sh/" + getHeading() + "p/" + getPitch() + "r/" + getRoll() + "y/" + getYaw() + "v/" + getVoltage() + "c/" + getCharge() + "E"; 
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
      //Activate servo
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
    Serial.print("Current:");
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
  voltage = (volt_sum / ROLL_AVG_VALUE - VOLT_OFFSET) / (275.0 / 5) + VOLT_AT_OFFSET;
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
void setChargeUsingVoltage(float voltage) {
  if (voltage > BATT_HIGH_THRESHOLD) {
    charge = map(voltage * 1000, BATT_HIGH_THRESHOLD * 1000, BATT_MAX_VOLT * 1000, BATT_HIGH_CHARGE, BATT_MAX_CHARGE);

    if (DEBUG) {
      Serial.print("high voltage :");
      Serial.println(charge);
    }
  }
  else if (voltage < BATT_LOW_THRESHOLD) {
    charge = map(voltage * 1000, BATT_MIN_VOLT * 1000, BATT_LOW_THRESHOLD * 1000, 0, BATT_LOW_CHARGE);

    if (DEBUG) {
      Serial.print("low voltage :");
      Serial.println(charge);
    }
  }
  else { //voltage normal range
    charge = map(voltage * 1000, BATT_LOW_THRESHOLD * 1000, BATT_HIGH_THRESHOLD * 1000, BATT_LOW_CHARGE, BATT_HIGH_CHARGE);

    if (DEBUG) {
      Serial.print("normal voltage :");
      Serial.println(charge);
    }
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

    case CLR_OFF:
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

    case CLR_OFF:
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
  //Update Gyro
  Vector normG = mpu.readNormalizeGyro();

  pitch = pitch + normG.YAxis * gyroTimeStep;
  roll = roll + normG.XAxis * gyroTimeStep;
  yaw = yaw + normG.ZAxis * gyroTimeStep;

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

String getPitch() {
  String data = String(pitch);
  String output = processPYR(data);
  return output;
}

String getRoll() {
  String data = String(roll);
  String output = processPYR(data);
  return output;
}

String getYaw() {
  String data = String(yaw);
  String output = processPYR(data);
  return output;
}

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
