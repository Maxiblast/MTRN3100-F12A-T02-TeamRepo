#include <SD.h>
#include <SPI.h> 

File file;

const int FILE_LIMIT{1000}; // limit so that in weird situations nextAvailable() doesnt loop forever.

void setup() {
  Serial.begin(INSERT HERE); 
  setupLogger();
}

void loop() {

}


void setupLogger() {
  Serial.println("Initialising SD Card...");

  if (!SD.begin(4)) { //this line runs SD.begin() and if its output is false it prints "Failed". i think 4 is chip select pin. 
    Serial.println("Initialise SD Failed.");
    return;
  } else {
    Serial.println("Initialisation Successful.");
  }
  
  Serial.println("Creating File...");
  String fileName = "log" + String(nextAvailable()) + ".txt";
  file = SD.open(fileName.c_str(), FILE_WRITE);
  file.close();

  if (SD.exists(fileName.c_str())) {
    Serial.println("File Created.");
  }
  else {
    Serial.println("File Creation Failed.");
  }
}

//Finds the logX.txt for the largest X that is available. 
int nextAvailable() {
  int i{0};
  String str = String("log" + String(i) + ".txt");
  while (SD.exists(str.c_string()) && i < FILE_LIMIT) {
    i++;
    str = String("log" + String(i) + ".txt");
  }
  return i;
}
