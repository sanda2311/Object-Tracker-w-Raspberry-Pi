#include <Servo.h>
Servo myServo;
void setup() {
  Serial.begin(9600);
  myServo.attach(9);
}
void loop() {
  if (Serial.available() > 0) {
    
    String data = Serial.readStringUntil('\n');
    int f = data.toInt();
    myServo.write(f);
    delay(100);
    //Serial.println(data);
  }
}
