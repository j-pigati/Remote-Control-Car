#include <SPI.h>  // Incude libraries for RF communication
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(9, 8);  // CE, CSN
const int buttonPin = 2;  // the number of the pushbutton pin
const int potPin1 = A0;  // Potentiometer connected to DC motor
const int potPin2 = A1; //Potentiomter connected to servo

int PotVal; //define changing variables
int MotorSpeed;

struct DataPacket {
  int PotVal1;
  int PotVal2;
  bool isButtonPressed;
};

DataPacket data;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(potPin1, INPUT);
  pinMode(potPin2, INPUT);
  radio.begin();
  radio.setChannel(105);
  radio.openWritingPipe(0xA1A1A1A1A1LL);
  radio.stopListening();
  Serial.begin(9600);
}

void loop() {
  data.isButtonPressed = digitalRead(buttonPin);
  data.PotVal1 = analogRead(potPin1);  //Read potentiometer value from pin A0 (DC)
  data.PotVal2 = analogRead(potPin2);  //Read potentiomter value from pin A1 (Servo)
  
  // Send data
  radio.write(&data, sizeof(data));

  if (data.isButtonPressed) {
    Serial.println(" Button is pressed - Message sent");
  } else {
    Serial.println(" Button is NOT pressed - Message sent");
  }
  Serial.print(" Pot_DC: ");
  Serial.print(data.PotVal1);
  Serial.print(" Pot_Servo: ");
  Serial.print(data.PotVal2);
}
