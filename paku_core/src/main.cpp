#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting setup...");
  int result = myFunction(2, 3);

  for (int i = 0; i < 10; i++) {
    Serial.println(i);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello World by Jossu");

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}