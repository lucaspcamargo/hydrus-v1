void setup() {
  // initialize serial communication
  Serial.begin(115200);
  // sending characters on serial port (visible if you display the serial monitor)
  Serial.println("Setup says: Hello World!");
}
 
void loop() {
  //sending characters the same way as setup function
  Serial.println("Loop says: Hello World!");
  //waiting a second
  delay(1000);
}