#include <Arduino.h>
#include <VescUart.h>
#include <ezButton.h>

// init the switches -------> limit switches put them wherever there is space and add pin #s
ezButton driveSwitch(2);
ezButton neutralSwitch(3);
ezButton reverseSwitch(4);

enum GearMode { DRIVE, NEUTRAL, REVERSE };
GearMode currentMode = NEUTRAL;

// Pin to read Analog input from linear potentiometer
int readPIN = A0;

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

  // if switches arent working, uncomment next 3 lines: ----> (Switch troubleshooting)
  // pinMode(driveSwitch, INPUT_PULLUP);
  // pinMode(nuetralSwitch, INPUT_PULLUP);
  // pinMode(reverseSwitch, INPUT_PULLUP);

  pinMode(readPIN, INPUT);
  pinMode(reversePIN, INPUT);

  driveSwitch.setDebounceTime(50);
  nuetralSwitch.setDebounceTime(50);
  reverseSwitch.setDebounceTime(50);

  delay(1000);
}

void loop() {
  driveSwitch.loop();
  neutralSwitch.loop();
  reverseSwitch.loop();
  
  /** Read The Input from the petentiometer and map the values from 0 - 1*/
  float duty = analogRead(readPIN);
  duty = (duty/1023) * 1.0;

  if (driveSwitch.wasPressed()) {
    currentMode = DRIVE;
    Serial.println("Mode swithched to : DRIVE");
  }
  if (neutralSwitch.wasPressed()) {
    currentMode = NEUTRAL;
    Serial.println("Mode swithched to : NUETRAL");
  }
  if (reverseSwitch.wasPressed()) {
    currentMode = REVERSE;
    Serial.println("Mode swithched to : REVERSE");
  }

  Serial.println("CURRENT MODE IS:" + currentMode);

  switch (currentMode) {
  case DRIVE:
    UART.setDuty(duty);
    break;

  case REVERSE:
    UART.setDuty(-duty);
    break;

  case NEUTRAL:
    UART.setDuty(0);
    break;
  }

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