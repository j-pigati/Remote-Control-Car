#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(9, 8);  // CE, CSN
const int R_EN = 3;  //forward enable
const int L_EN = 4;  //reverse enable
const int R_PWM = 5;  //forward pwm
const int L_PWM = 6;  //reverse pwm
int MotorSpeed; 
int ServoAngle;
Servo SERVO;

struct DataPacket {
  int PotVal1;
  int PotVal2;
  bool isButtonPressed;
};

DataPacket data;

void setup() {
  SERVO.attach(10);  //Attach servo to pin 10

  pinMode(R_EN, OUTPUT);  //Setup pins as outputs
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  digitalWrite(R_EN, HIGH);  //Enable motor 
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_PWM, LOW);  //Ensure motor is off initially 
  digitalWrite(L_PWM, LOW);

  radio.begin();
  radio.setChannel(105);  //Connect to channel 105
  radio.openReadingPipe(0,0xA1A1A1A1A1LL);
  radio.startListening();
  Serial.begin(9600);
}

void loop() {
 if (radio.available()) {
    radio.read(&data, sizeof(data));
    
    ServoAngle = map(data.PotVal2, 0, 1023, 177, -3);
    SERVO.write(ServoAngle);

    if (data.PotVal1 >= 512) {                 //if potentiometer is greater than halfway, forwards
      MotorSpeed = map(data.PotVal1, 512, 1023, 0, 150);  //motor speed mapped to joystick 
      analogWrite(R_PWM, MotorSpeed);
      digitalWrite(L_PWM, LOW);

      Serial.print("Pot Value: "); 
      Serial.print(data.PotVal1);              //print potentiometer value
      Serial.print("    ");
      Serial.print("Forward Motor Speed: ");   //print the forward motor speed
      Serial.println(MotorSpeed);              //print the motor speed corresponding to the potentiometer value
      
    } else {                                   //if potentiometer is less than halfway, reverse
      MotorSpeed = map(data.PotVal1, 0, 512, 150, 0);
      analogWrite(L_PWM, MotorSpeed);
      digitalWrite(R_PWM, LOW);

      Serial.print("Pot Value: ");   
      Serial.print(data.PotVal1);              //print potentiometer value
      Serial.print("    ");
      Serial.print("Reverse Motor Speed: ");   //print reverse motor speed
      Serial.println(MotorSpeed);              //print the motor speed corresponding to the potentiometer value
    }
}
} 
