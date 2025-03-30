#include <Arduino.h>
#include <PID_v1.h>
#include <VescUart.h>

float current = 0.0;
float previousCurrent = 0.0;
float lerpSpeed = 0.1;

// Pin to read Analog input from linear potentiometer
int readPIN = A0;

// Pin to read reverse direction from button
int reversePIN = 3;

/** Initiate VescUart class */
VescUart UART;

double SetPoint, Input, Output;

double Kp=0.01, Ki=0.0, Kd=0.0;
PID pid(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(9600);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);

  pinMode(readPIN, INPUT);
  pinMode(reversePIN, INPUT);

  delay(1000);

  // UART.setDuty(0.2);

  pid.SetMode(AUTOMATIC);
}

void loop() {

  // float targetCurret = 5.0;

  // current = lerp(previousCurrent, targetCurret, lerpSpeed);
  // previousCurrent = current;

  // UART.setCurrent(0.0);

  bool reverse = digitalRead(reversePIN);
  
  float current = analogRead(readPIN);
  current = (current/1023) * 1.0;
  // current = ((current - 512) / 512) * 2

  SetPoint = current;

  pid.Compute();

  if (reverse)
    UART.setDuty(Output);
  else
    UART.setDuty(-Output);


  /** Call the function getVescValues() to acquire data from VESC */
  if ( UART.getVescValues() ) {
    Serial.print("PID Output: ");
    Serial.print(Output);
    Serial.print(", ");
    Serial.print("RPM: ");
    Serial.print(UART.data.rpm);
    Serial.print(", ");
    Serial.print("Voltage:  ");
    Serial.print(UART.data.inpVoltage);
    Serial.print(", ");
    Serial.print("Current:  ");
    Serial.print(UART.data.avgInputCurrent);
    Serial.print(", ");
    Serial.print("Target Current: ");
    Serial.print(current);
    Serial.print(", ");
    Serial.print("Power:    ");
    Serial.println(UART.data.inpVoltage * UART.data.avgInputCurrent);

    // Serial.print("Amp Hours: ");
    // Serial.println(UART.data.ampHours);
    // Serial.print("Tachometer: ");
    // Serial.println(UART.data.tachometerAbs);

    Input = UART.data.dutyCycleNow;
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  delay(500);
}

double lerp(double start, double end, double t)
{
  return (1.0 - t) * start + t * end;
}