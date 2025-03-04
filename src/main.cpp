#include <Arduino.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// Define the serial port for GPS communication (e.g., Serial1, Serial2)
#define GPS_SERIAL Serial1 // You can change to Serial2 or another UART if necessary
// Define GPS baud rate
#define GPS_BAUD 9600

// Time to delay before between loops
#define DELAY_s 0.1
#define DELAY_ms (DELAY_s * 1000)

// Radio CS pin
#define CS1 10
// Particle sensor CS pin
#define CS2 33

// Define radio reset pin
#define radio_rst 11

// Define Radio interrupt pin (G0)
#define radio_int 9

// Radio address for our radio module
#define FROM_address 0
#define TO_address 1


// For tracking acceleration in altitude
double last_altitude = 0.0;
double current_altitude = 0.0;

double last_altitude_updated = 0;
bool sensors_active = false;

// Define GPS object
TinyGPSPlus gps;

// Define radio driver object
RH_RF95 rf95(CS1, radio_int);

// Define another object to send data reliably
// use methods recvfromAck() and sendtoWait()
RHReliableDatagram data_transmitter(rf95, FROM_address);

////////////////////////////
void setupGPS() {

  GPS_SERIAL.begin(GPS_BAUD);  // Start serial communication with the GPS module
  while (!GPS_SERIAL) {} // Wait for GPS module to be ready
  Serial.println("GPS initialized.");

}

// Function to read and process GPS data
void processGPS() {
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read()); // Feed GPS data to the TinyGPS++ object

    // Check if there's a new GPS fix (valid location)
    if (gps.location.isUpdated()) {
      last_altitude = current_altitude;
      current_altitude = gps.altitude.meters();

      double falling_acceleration = (last_altitude - current_altitude) / last_altitude_updated;

      // Activating all required sensors
      if ((!sensors_active) && (1 < falling_acceleration)) {
        sensors_active = true;
        digitalWrite(26, HIGH);
      }
      
      last_altitude_updated = 0;
      //   // Print out the GPS data (latitude, longitude, altitude)
    //   Serial.print("Latitude= "); 
    //   Serial.print(gps.location.lat(), 6);  // Prints with 6 decimals
    //   Serial.print("Longitude= "); 
    //   Serial.print(gps.location.lng(), 6);  // Prints with 6 decimals
    //   Serial.print("Altitude= "); 
    //   Serial.println(gps.altitude.meters()); // Prints altitude in meters
    }

    // // You can add more data like speed, course, date, time, etc.
    // if (gps.speed.isUpdated()) {
    //   Serial.print("Speed= ");
    //   Serial.println(gps.speed.kmph()); // Speed in km/h
    // }

    // if (gps.date.isUpdated()) {
    //   Serial.print("Date= ");
    //   Serial.print(gps.date.day());
    //   Serial.print("/");
    //   Serial.print(gps.date.month());
    //   Serial.print("/");
    //   Serial.println(gps.date.year());
    // }

    // if (gps.time.isUpdated()) {
    //   Serial.print("Time= ");
    //   Serial.print(gps.time.hour());
    //   Serial.print(":");
    //   Serial.print(gps.time.minute());
    //   Serial.print(":");
    //   Serial.print(gps.time.second());
    //   Serial.println();
    // }
  }
}

void sendvars2PC(){
  // sending CANSAT info as json data
  char json_chars[128];

  sprintf(
    json_chars,
    "[%f, %f, %f]",
    gps.location.lat(), // Degrees
    gps.location.lng(), // Degrees
    current_altitude // Metres
  );

  // The RFM 9x only has a 252 byte buffer, sending in smaller chunks for safety (only useful if the data we are sending is larger than 252 bytes)
  for (int i = 0; i < sizeof(json_chars); i += 128) {
    data_transmitter.sendtoWait(((uint8_t*) json_chars), 128, TO_address);
  }
}

void setup() {
  // Configuring pins
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
//  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(radio_rst, OUTPUT);

  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
//  digitalWrite(CS1, LOW);
  digitalWrite(CS2, LOW);
  digitalWrite(radio_rst, HIGH);

  Serial.begin(115200);  // Start Serial communication with the PC (for debugging)
  setupGPS();  // Initialize the GPS module

  // Manual reset of the radio
  digitalWrite(radio_rst, LOW);
  delay(10);
  digitalWrite(radio_rst, HIGH);
  delay(10);

  rf95.init();
  rf95.setFrequency(433.92);
}


void loop() {
  processGPS();  // Read and process the GPS data
  sendvars2PC();
  delay(DELAY_ms);  // Wait a bit before reading again
  last_altitude_updated += DELAY_s;
}