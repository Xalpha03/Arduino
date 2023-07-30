
#include "Wire.h"
#include "AHT10.h"
#include "PIDController.h"

#define fan_Pin 4            // Fan pin for heat dissipation;
#define heat_Resistor_Pin 5  // Heating resistor for heat;
#define humidifier_Pin 6     // Humidifier for humidity;


//Set Control Assignment Variables
double Setpoint_T, ValueTemp, Output_T;
double Setpoint_H, ValueHum, Output_H;

//Specify the links and initial tuning parameters
double Kp_T = 2, Ki_T = 5, Kd_T = 1;
PIDController myPID_T;  // Create an instance of the temperature PID controller class, called myPID_T

double Kp_H = 2, Ki_H = 5, Kd_H = 1;
PIDController myPID_H;  // Create an instance of the humidity PID controller class, called myPID_T

AHT10 myAHT10(AHT10_ADDRESS_0X38);

PIDController pid_T;  // Create an instance of the temmperature PID controller class, called pid_T
PIDController pid_H;  // Create an instance of the humidity PID controller class, called pid_H


void setup() {

  Serial.begin(9600);
  pinMode(fan_Pin, OUTPUT);

  while (myAHT10.begin() != true) {
    Serial.println(F("AHT10 error"));  //(F()) saves string to flash & keeps dynamic memory free
  }

  Setpoint_T = 37.5;
  Setpoint_H = 67.5;

  // Temperature PID configuration
  pid_T.begin();
  pid_T.setpoint(Setpoint_T);
  pid_T.tune(Kp_T, Kd_T, Kp_T);
  pid_T.limit(0, 255);

  // Humidity PID configuration
  pid_H.begin();
  pid_H.setpoint(Setpoint_H);
  pid_H.tune(Kp_H, Kd_H, Kp_H);
  pid_H.limit(0, 255);
}

void loop() {

  uint8_t readStatus = 0;
  readStatus = myAHT10.readRawData();

  if (readStatus != AHT10_ERROR) {
    ValueTemp = myAHT10.readTemperature();
    ValueHum = myAHT10.readHumidity();

    Serial.print("temperature = "), Serial.print(ValueTemp), Serial.println(" °C");
    Serial.print("humidity = "), Serial.print(ValueHum), Serial.println(" %");

    Output_T = pid_T.compute(ValueTemp);
    analogWrite(heat_Resistor_Pin, Output_T);

    Output_H = pid_H.compute(ValueHum);
    analogWrite(humidifier_Pin, Output_H);

    Serial.print(Setpoint_T), Serial.println(Setpoint_T);
    Serial.print(Setpoint_H), Serial.println(Setpoint_H);

    Serial.print("Output_T"), Serial.println(Output_T);
    Serial.print("Output_H"), Serial.println(Output_H);


  } else {
    Serial.println("i2c error");
  }
}
