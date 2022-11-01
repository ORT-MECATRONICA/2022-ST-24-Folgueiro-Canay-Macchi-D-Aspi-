#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

float BMPTemp;
float BMPPress;
float bar;


void setup() {
  Serial.begin(9600);
}

void loop() {
  BMPTemp = bmp.readTemperature();
  BMPPress = bmp.readPressure();
  bar = BMPPress / 100000;

  Serial.print("Temperatura: ");
  Serial.print(BMPTemp);
  Serial.println(" °C");

  Serial.print("Presion: ");
  Serial.print(bar);
  Serial.println(" Bar");
}
