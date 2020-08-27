void setup()
{
  Serial.begin(9600);
  Serial.println("MiCS-5524 demo!");
}
 
void loop()
{
  int reading = analogRead(A0);
  Serial.println(reading);
  delay(100);
}
