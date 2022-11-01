#include <ESP32AnalogRead.h>

#define TEMT6000_PIN 35

float TEMT6000Value;


void setup() {
  Serial.begin(9600);

}

void loop() {
  TEMT6000Value = analogRead(TEMT6000_PIN);
  Serial.print("Valor detectado por el sensor TEMT6000: ");
  Serial.println(TEMT6000Value);
}
