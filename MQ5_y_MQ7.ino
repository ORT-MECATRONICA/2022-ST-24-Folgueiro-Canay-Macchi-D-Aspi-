#include <ESP32AnalogRead.h>

#define MQ5pin 33
#define MQ7pin 32

float MQ5Value;
float MQ7Value;


void setup() {
  Serial.begin(9600);
}

void loop() {
  MQ5Value = analogRead(MQ5pin);
  Serial.print("Valor detectado por el sensor MQ5: ");
  Serial.print(MQ5Value);
  Serial.println("");

  MQ7Value = analogRead(MQ7pin);
  Serial.print("Valor detectado por el sensor MQ7: ");
  Serial.print(MQ7Value);
  Serial.println("");
}
