void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  float sensorValue = analogRead(35);
  // print out the value you read:
  /*
 Serial.print("weight = ");
  Serial.print(sensorValue/4096*20);
 Serial.println(" N");
 */
Serial.print((4096-sensorValue)/4096*20);
Serial.println(" N");
  delay(1);        // delay in between reads for stability
}
