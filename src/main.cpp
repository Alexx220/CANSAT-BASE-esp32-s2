#include <Arduino.h>
#include <HardwareSerial.h>

// Create a hardware serial instance for the GPS
HardwareSerial myGPS(1); // Use UART1 (you can change to 2 if needed)

void setup() {
  // Start the Serial Monitor at 115200 baud rate
  Serial.begin(115200);

  // Initialize GPS communication on UART1 (using pins GPIO 16 and 17)
  myGPS.begin(9600, SERIAL_8N1, 16, 17); // 9600 baud, 8 data bits, no parity, 1 stop bit
  
  // Wait for the serial to initialize
  while (!Serial) {
    ; // Wait for the serial port to connect

  }
   Serial.println("Bidirectional communication with ESP32 initialized!");
}











void loop() {
  // Read from GPS and send it to Serial Monitor
  if (myGPS.available()) {
    char gpsData = myGPS.read(); // Read data from GPS
    Serial.print(gpsData); // Print GPS data to Serial Monitor
  }

  // Optionally, you can send other debug messages over Serial
  Serial.println("ESP32 is running...");
  delay(2000); // Delay for readability
}
