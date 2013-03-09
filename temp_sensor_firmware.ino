/*
* Temp & Humidity Sensor 
* temp_sensor_firmware.ino
*
* Original author: Steven Casagrande (scasagrande@galvant.ca)
* 2013 
*
* This work is released under the Creative Commons Attribution-Sharealike 3.0 license.
* See http://creativecommons.org/licenses/by-sa/3.0/ or the included license/LICENSE.TXT file for more information.
*
* Attribution requirements can be found in license/ATTRIBUTION.TXT
*
* This project is designed to allow an Arduino to communicate with up to 100 
* Honeywell HumidIcon Digital Humidity and Temperature Sensors: HIH6130-021-001
*
* Holding down button attached to pin 4 on boot puts the device in admin mode where you can
* send a variety of commands.
* Valid admin commands:
*   add :    Used to add a brand new sensor to the chain. This will change the I2C address
*            from the default 39 (0x27) to the current number of sensors, plus 1.
*            For example, if you currently have 3 sensors, "add" will change the I2C address
*            of a new sensor to 4. 
*   count :  Change number of sensors attached. Useful if you wish to reduce the number of
             sensors attached, or if you are replacing your control board. Note that during
             normal operation, the board will query sensors with address from 1 to this value.
    change : Change the I2C address of a specific module. You specifiy the old and the new
             address, and the Arduino will take care of it.
    exit :   Exit admin mode and start normal operation.
*/

#include <LiquidCrystal.h>
#include <Wire.h>
#include <EEPROM.h>

#define SENSOR_PWR 5
#define CMD_MODE_BUTTON 4

#define EEPROM_CODE 0xA5
#define EEPROM_CODE_ADDRESS 0x00
#define EEPROM_SENSOR_COUNT_ADDRESS 0x01

#define MAX_SENSOR_COUNT 100

LiquidCrystal lcd(12, 11, 7, 8, 9, 10);

int sensorCount = 0;
byte eepromValue = 0;
String incomingCmd = "";
char incomingByte[10];

void setup()
{
  // Start hardware components
  Wire.begin();
  Serial.begin(9600);
  
  // Prep LCD
  lcd.begin(16,2); // 16x2 display
  lcd.home();
  
  // Check if EEPROM is valid
  eepromValue = EEPROM.read(EEPROM_CODE_ADDRESS);
  if (eepromValue == EEPROM_CODE) {
    sensorCount = EEPROM.read(EEPROM_SENSOR_COUNT_ADDRESS);
    Serial.println("Valid EEPROM");
    Serial.print("Current sensor count: ");
    Serial.print(sensorCount, DEC);
    Serial.print("\n");
  }
  else { // EEPROM not valid, so initialize it
    EEPROM.write(EEPROM_CODE_ADDRESS, EEPROM_CODE);
    EEPROM.write(EEPROM_SENSOR_COUNT_ADDRESS, 0x00); // Init with zero sensors
    sensorCount = 0;
  }
  
  // Init push buttons
  pinMode(CMD_MODE_BUTTON, INPUT);
  delay(1);
  
  // Here we will setup power to the sensor with one of the digital pins.
  // This will let us enter cmd mode before the window closes
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, LOW);
  
  int buttonState = digitalRead(CMD_MODE_BUTTON);
  if (buttonState) {
    adminMode();
  }
  
  // Start normal operation mode
  // Make sure sensor is powered
  digitalWrite(SENSOR_PWR, HIGH);
  
}

void adminMode()
{
  int i = 0;
  boolean inAdminMode = true;
  
  Serial.print("\n");    
  Serial.print("==Entering admin mode==");
  
  while(inAdminMode == true) {
    Serial.print("\n");
    Serial.println("Please enter a command:");
    incomingCmd = "";
    while (incomingCmd.equals("")) {
      incomingCmd = readLine();
    }
    Serial.print("Entered command: ");
    Serial.print(incomingCmd);
    Serial.print("\n");
  
    // --- Parse admin mode command --- //
    if (incomingCmd.substring(0,3).equalsIgnoreCase("add")) {
      menuAddSensor();
    }
    else if (incomingCmd.substring(0,5).equalsIgnoreCase("count")) {
      menuChangeSensorCount();
    }
    else if (incomingCmd.substring(0,6).equalsIgnoreCase("change")) {
      menuChangeAddress();
    }
    else if (incomingCmd.substring(0,4).equalsIgnoreCase("exit")) {
      Serial.println("Exiting admin mode.");
      inAdminMode = false;
    }
    else {
      Serial.println("Bad command.");
    }
  }
  
}

String readLine() {
  String value = "";
  for (int i = 0; Serial.available() > 0; i++) {
    value += (char)Serial.read();
    delay(5);
  }
  return value;
}

void menuAddSensor() {
  if(sensorCount >= MAX_SENSOR_COUNT) {
    Serial.println("System at maximum allowed sensor modules.");
  }
  else {
    Serial.println("Adding new sensor.");
    Serial.print("Current sensor count: ");
    Serial.println(sensorCount, DEC);
    Serial.println("This mode is for adding a brand new default sensor to the system.");
    Serial.print("New sensor will be given address of: ");
    Serial.println(sensorCount+1, DEC);
    if(confirm()) {
      sensorCount++;
      EEPROM.write(EEPROM_SENSOR_COUNT_ADDRESS, sensorCount);
      setConfigRegister(39, sensorCount);
      Serial.println("Sensor has been updated.");
    }
    else{
      Serial.println("Cancel adding new sensor.");
    }
  }
}

void menuChangeAddress() {
  int newAddress = -1;
  int oldAddress = -1;
  
  Serial.println("Change I2C address of attached sensor.");
  Serial.print("Enter old I2C address (new sensor default is 39): ");
  incomingCmd = "";
  while (incomingCmd.equals("")) {
    incomingCmd = readLine();
    incomingCmd.toCharArray(incomingByte,10);
    oldAddress = atoi((char*)(&(incomingByte[0])));
  }
  Serial.println(oldAddress, DEC);
  Serial.print("Enter new I2C address: ");
  incomingCmd = "";
  while (incomingCmd.equals("")) {
    incomingCmd = readLine();
    incomingCmd.toCharArray(incomingByte,10);
    newAddress = atoi((char*)(&(incomingByte[0])));
  }
  Serial.println(newAddress, DEC);
  
  setConfigRegister(oldAddress, newAddress);
}

void menuChangeSensorCount() {
  int newSensorCount = -1;
  
  Serial.println("Override current sensor count.");
  Serial.print("Current sensor count is at: ");
  Serial.println(sensorCount, DEC);
  Serial.println("Enter new number of sensors. Note you will need to have already set these up: ");
  incomingCmd = "";
  while (incomingCmd.equals("")) {
    incomingCmd = readLine();
    incomingCmd.toCharArray(incomingByte,10);
    newSensorCount = atoi((char*)(&(incomingByte[0])));
  }
  Serial.print("New sensor count is: ");
  Serial.println(newSensorCount, DEC);
  if (confirm()) {
    Serial.print("Sensor count confirmed for: ");
    Serial.println(newSensorCount, DEC);
    EEPROM.write(EEPROM_SENSOR_COUNT_ADDRESS, newSensorCount);
    sensorCount = newSensorCount;
  }
  else {
    Serial.println("Aborting changing sensor count.");
  }
}

boolean confirm() {
  boolean value = false;
  Serial.print("Confirm? (y/n): ");
  incomingCmd = "";
  while (incomingCmd.equals("")) {
    incomingCmd = readLine();
  }
  if (incomingCmd.substring(0,1).equalsIgnoreCase("y")) {
    value = true;
    Serial.println("y");
  }
  else {
    Serial.println("n");
  }
  return value;
}

void setConfigRegister(int oldAddress, int newAddress)
{
  Serial.print("Changing I2C addresses: ");
  Serial.print(oldAddress, DEC);
  Serial.print(" to ");
  Serial.println(newAddress, DEC);
  byte a, b, c; // Dummy temp variables
  byte start_cmd[] = {0xA0,0x00,0x00};
  byte fetch_status_register[] = {0x1C,0x00,0x00};
  byte start_norm[] = {0x80,0x00,0x00};
  byte write_status_register[] = {0x5C,0x15,0x01}; // 3rd byte is new I2C address
  
  // Power cycle sensor
  digitalWrite(SENSOR_PWR, LOW);
  delay(10); // Wait 10ms, let everything stabilize
  digitalWrite(SENSOR_PWR, HIGH); // Turn on power to the sensor
  delay(1); // Wait 1ms, putting us in the cmd mode window

  Wire.beginTransmission(oldAddress); // Start I2C on default sensor address
  Wire.write(start_cmd, 3); // cmd byte: START_CMD
  Wire.endTransmission();
  delayMicroseconds(100); // Delay to enter cmd mode
  
  // Now in CMD mode //
  Wire.beginTransmission(oldAddress);
  Wire.write(fetch_status_register,3); // Request data at address 0x1C: customer config register
  Wire.endTransmission();
  delayMicroseconds(100); // Delay for sensor to fetch register
  Wire.requestFrom(oldAddress, 3); // Read cmd status response (1byte) + eeprom fetch (2bytes)
  a = Wire.read();
  b = Wire.read();
  c = Wire.read();
  
  // Change I2C address in register 0x1C //
  write_status_register[2] = newAddress;
  Wire.beginTransmission(oldAddress);
  Wire.write(write_status_register,3);
  Wire.endTransmission();
  delay(12);
  
  // Enter normal operation mode //
  Wire.beginTransmission(newAddress);
  Wire.write(start_norm, 3);
  Wire.endTransmission();
  delay(50);
  
}

void getSensorData(int address, byte *a, byte *b, byte *c, byte *d)
{
  Wire.beginTransmission(address);
  Wire.endTransmission();
  
  delay(50); // 50ms delay to allow sensor to perform measurement
  
  Wire.requestFrom(address, 4);
  *a = Wire.read();
  *b = Wire.read();
  *c = Wire.read();
  *d = Wire.read();
}

void showSensorData(int address)
{
  byte aa,bb,cc,dd;
  float temperature = 0;
  float humidity = 0;
  
  getSensorData(address, &aa,&bb,&cc,&dd);
  
  temperature = ((cc * 256) + dd)/4;
  temperature = ( temperature/(16384-2) ) * 165 - 40;
  
  humidity = (((aa^B00111111) * 256) + bb);
  humidity = ( humidity/(16384-2) ) * 100;
  
  lcd.clear();
  lcd.print(temperature);
  lcd.print((char)223);
  lcd.print("C ");
  lcd.print(humidity);
  lcd.print((char)37);
  
  Serial.println(temperature, DEC);
  Serial.println(humidity, DEC);
  
}

void loop()
{
  for(int i = 1; i<= sensorCount;i++) {
    showSensorData(i);
    delay(5000);
  }
}
