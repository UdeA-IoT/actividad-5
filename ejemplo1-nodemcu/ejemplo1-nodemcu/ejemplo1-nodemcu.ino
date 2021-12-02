// Ejemplo tomado de: https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/all#powering-the-thing

#define ESP8266_LED D5

void setup() 
{
  pinMode(ESP8266_LED, OUTPUT);
}

void loop() 
{
  digitalWrite(ESP8266_LED, HIGH);
  delay(500);
  digitalWrite(ESP8266_LED, LOW);
  delay(500);
}
