#include <LittleFS.h>
#include "FS.h"

void setup() {

  
  Serial.begin(115200);
  LittleFS.begin();
  //LittleFS.format();

  //creatFile("/ds_plan.txt");
  
  // Read data from a file
  String data = readFile("/ds_plan.txt");
  Serial.println("Data from file: " + data);//*/

  
  
}

void loop() {
  // Your code here (if needed)
}


String readFile(const char* filename) {
  String data = "";

  File file = LittleFS.open(filename, "r");
  if (file) {
    while (file.available()) {
      data += (char)file.read();
    }
    file.close();
  } else {
    Serial.println("Failed to open file for reading.");
  }

  return data;
}
void creatFile(const char* filename) {
  File file = LittleFS.open(filename, "w");
  if (file) {
              file.print("Phi (rad): ");
              file.print('\t');
              file.print("PID(8 bit num) : ");
              file.print('\t');
              file.print("Theta (rad): ");
              file.print('\t');
              file.println("PID(8 bit num) : ");
              /*file.print('\t');
              file.print("Theta (rad): ");
              file.print('\t');
              
              file.println("Bearing (degree):");*/
              file.close();//*/
    Serial.println("File written successfully!");
  } else {
    Serial.println("Failed to open file for writing.");
  }
}
