#include <Arduino.h>
#include <VescUart.h>

// Pin to read Analog input from linear potentiometer
int readPIN = A0;

// Pin to read reverse direction from button
int reversePIN = 3;

/** Initiate VescUart class */
VescUart UART;

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
}

void loop() {

  /** Read the input from the button to reverse the current KAY probably add the shifter code here */
  bool reverse = digitalRead(reversePIN);
  
  /** Read The Input from the petentiometer and map the values from 0 - 1*/
  float duty = analogRead(readPIN);
  duty = (duty/1023) * 1.0;

  if (reverse)
    UART.setDuty(duty);
  else
    UART.setDuty(-duty);


  /** Call the function getVescValues() to acquire data from VESC */
  if ( UART.getVescValues() ) {
    Serial.print("RPM: ");
    Serial.print(UART.data.rpm);
    Serial.print(", ");
    Serial.print("Voltage:  ");
    Serial.print(UART.data.inpVoltage);
    Serial.print(", ");
    Serial.print("Current:  ");
    Serial.print(UART.data.avgInputCurrent);
    Serial.print(", ");
    Serial.print("Power:    ");
    Serial.println(UART.data.inpVoltage * UART.data.avgInputCurrent);
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  delay(500);
}

/** Lerp fuction to smoothly interpolate between two values */
double lerp(double start, double end, double t)
{
  return (1.0 - t) * start + t * end;
}