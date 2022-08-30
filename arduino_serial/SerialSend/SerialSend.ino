

String inData;
String *p;
void setup() {
  Serial.begin(115200);
 // p = &inData;
}

void loop() {
  while (Serial.available() > 0) {
    char recieved = Serial.read();
    inData += recieved;
         String function = inData.substring(0, 2);
         if (recieved == '\n') {
           Serial.println(inData + "UP");
         }
         //  inData = "";
        /* if (recieved == 'C') {
          inData = "";
          } */
   // inData = "";
  }
  // Serial.println();
 // inData = "";
}
