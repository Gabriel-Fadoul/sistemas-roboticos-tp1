#include <Arduino.h>

volatile long ticks = 0;
void isrA() { if (digitalRead(2) == digitalRead(3)) ticks--; else ticks++; }
void isrB() { if (digitalRead(2) == digitalRead(3)) ticks++; else ticks--; }
void setup() {
pinMode(2, INPUT_PULLUP);
pinMode(3, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(2), isrA, CHANGE);
attachInterrupt(digitalPinToInterrupt(3), isrB, CHANGE);
Serial.begin(9600);
}

long lerTicks() { noInterrupts(); long t = ticks; interrupts(); return t; }


void loop() {
  Serial.println(ticks);
  delay(100);
  servo.writeMicroseconds(10);
}
