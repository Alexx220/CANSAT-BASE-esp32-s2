#include <Arduino.h>
#include <TinyGPS++.h>
// Define GPS object
TinyGPSPlus gps;

// Define the serial port for GPS communication (e.g., Serial1, Serial2)
#define GPS_SERIAL Serial1 // You can change to Serial2 or another UART if necessary
// Define GPS baud rate
#define GPS_BAUD 9600


void setup() {
  Serial.begin(115200);  // Start Serial communication with the PC (for debugging)
  setupGPS();  // Initialize the GPS module
}


void loop() {
  processGPS();  // Read and process the GPS data
  sendvars2PC();
  delay(2000);  // Wait a bit before reading again
}     







void sendvars2PC(){
if (Serial.available() > 0) {
// saud feel free to structure the data sent as you like, as this is all BASE STATION dictates, not the pc you can just pass everything from the recivever in a pass through manner 
}
}







////////////////////////////
void setupGPS() {

  GPS_SERIAL.begin(GPS_BAUD);  // Start serial communication with the GPS module
  while (!GPS_SERIAL) {
    ; // Wait for GPS module to be ready
  }
  Serial.println("GPS initialized.");

}

// Function to read and process GPS data
void processGPS() {
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read()); // Feed GPS data to the TinyGPS++ object

    // Check if there's a new GPS fix (valid location)
    if (gps.location.isUpdated()) {
      // Print out the GPS data (latitude, longitude, altitude)
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);  // Prints with 6 decimals
      Serial.print(" Longitude= "); 
      Serial.print(gps.location.lng(), 6);  // Prints with 6 decimals
      Serial.print(" Altitude= "); 
      Serial.println(gps.altitude.meters()); // Prints altitude in meters
    }

    // You can add more data like speed, course, date, time, etc.
    if (gps.speed.isUpdated()) {
      Serial.print("Speed= ");
      Serial.println(gps.speed.kmph()); // Speed in km/h
    }

    if (gps.date.isUpdated()) {
      Serial.print("Date= ");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.println(gps.date.year());
    }

    if (gps.time.isUpdated()) {
      Serial.print("Time= ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.print(gps.time.second());
      Serial.println();
    }
  }
}
