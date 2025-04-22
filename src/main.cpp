#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <SparkFun_SGP40_Arduino_Library.h>
#include <Wire.h>
#include <GY521.h>
#include <ArduinoJson.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <sps30.h>

// Define GPS baud rate
#define GPS_BAUD 9600

// Time to delay before between loops
#define DELAY_ms 100

// Pin for Turning on/off peripherals
#define peripheral_pin 26

// Radio CS pin
#define CS1 10
// Particle sensor CS pin
#define CS2 33

// Define radio reset pin
#define radio_rst 11

// Define Radio interrupt pin (G0)
#define radio_int 9

// Radio address for our radio module
#define FROM_address 146
#define TO_address 173

#define DHTPIN 18

// For JSON fomatting
JsonDocument doc;

// For tracking acceleration in altitude
double last_altitude = 0.0;
double current_altitude = 0.0;
long last_altitude_updated;

// Storing whether the sensors are ready
bool sensors_active = false;

// Storing when the air quality sensor was last read
long last_air_quality_read = 0;

// Define the serial port for GPS communication (e.g., Serial1, Serial2)
// You can change to Serial2 or another UART if necessary
HardwareSerial gps_serial(1);

// Define GPS object
TinyGPSPlus gps;

// Define radio driver object
RH_RF95 rf95(CS1, radio_int);

// Define another object to send data reliably
// use methods recvfromAck() and sendtoWait()
RHReliableDatagram data_transmitter(rf95, FROM_address);

// Define DHT object for reading the temp/humidity
DHT dht(DHTPIN, DHT22);

// Define BMP object for air pressure
Adafruit_BMP280 bmp;

// Define air quality sensor object
SGP40 air_quality;

// Gyroscope gy-521 object
GY521 gyro;

// ISH sensor
SparkFun_ISM330DHCX ISM_sensor;

// Structs for storing all data for the ISM330DHCX
sfe_ism_data_t accel_data;
sfe_ism_data_t gyro_data;

// Magnometer
SFE_MMC5983MA magnometer;

void turn_on_sensors() {
  sensors_active = true;
  digitalWrite(peripheral_pin, HIGH);

  delay(100);
  
  Wire.begin(); // I2C initilisation

  delay(100);

  dht.begin();
  bmp.begin();

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500 /* Standby time. */
  );

  air_quality.begin();

  gyro.begin();

  gyro.setAccelSensitivity(1);
  gyro.setGyroSensitivity(1);

  ISM_sensor.begin();

	// Reset the device to default settings. This if helpful is you're doing multiple
	// uploads testing different settings. 
	ISM_sensor.deviceReset();

	// Wait for it to finish reseting
	while(!ISM_sensor.getDeviceReset()) { delay(100); }

  magnometer.begin();
  magnometer.softReset();

  sensirion_i2c_init();
  sps30_probe();
  sps30_set_fan_auto_cleaning_interval_days(4);
  sps30_start_measurement();
}

void setup_GPS() {
  gps_serial.begin(GPS_BAUD);  // Start serial communication with the GPS module
  while (!gps_serial) {} // Wait for GPS module to be ready
//  Serial.println("GPS initialized.");

  last_altitude_updated = millis();
}

void send_data(JsonDocument doc) { // Sending each sensors' data seperately to prevent hitting the 252 byte limit
  char json_chars[250];

  uint8_t num_bytes = serializeJson(doc, json_chars, 250);

  // The RFM 9x only has a 252 byte buffer (keep this in mind)
  data_transmitter.sendtoWait(((uint8_t*)json_chars), num_bytes, TO_address);

  doc.clear();
}

void setup() {
  // Configuring pins
  pinMode(peripheral_pin, OUTPUT);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(radio_rst, OUTPUT);

  digitalWrite(peripheral_pin, LOW);
  digitalWrite(CS1, LOW);
  digitalWrite(CS2, LOW);
  digitalWrite(radio_rst, HIGH);

  setup_GPS();  // Initialize the GPS module

  // Manual reset of the radio
  digitalWrite(radio_rst, LOW);
  delay(10);
  digitalWrite(radio_rst, HIGH);
  delay(10);

  rf95.init();
  rf95.setFrequency(433.92);  // Make sure this does not mess with others' frequencies

//  Serial.begin(9600);

//  turn_on_sensors();
  doc["sensor_status"] = sensors_active;  // Make sure the sensors are initially reported as their initial state

//  while (!Serial) {delay(100);}
}

void loop() { // Reading and sending all sensor data
  // sending CANSAT info as json data
  bool just_updated = false;
  while (gps_serial.available() > 0) {
    gps.encode(gps_serial.read()); // Feed GPS data to the TinyGPS++ object

    // Check if there's a new GPS fix (valid location)
    if (gps.location.isValid() && gps.location.isUpdated()) {
      just_updated = false;
      long current_time = millis();

      last_altitude = current_altitude;
      current_altitude = gps.altitude.meters();

      double falling_acceleration = ((last_altitude - current_altitude) * 1000.0) / (current_time - last_altitude_updated);

      // Activating all sensors
      if ((!sensors_active) && (1 < falling_acceleration)) {
        turn_on_sensors();
        doc["sensor_status"] = sensors_active;
      }

      last_altitude_updated = current_time;
    }
  }

  if (just_updated) {
    doc["GPS"]["latitude"] = gps.location.lat(); // Degrees GPS information
    doc["GPS"]["longitude"] = gps.location.lng(); // Degrees
    doc["GPS"]["altitude"] = current_altitude; // Metres
    send_data(doc);
  }

  if (sensors_active) {
    float t = dht.readTemperature(false); // DHT22 readings
    float h = dht.readHumidity();

    long current_time = millis();

    if (!(isnan(h) || isnan(t))) {
      doc["DHT"]["temp"] = t;  // Temp in degrees celcius
      doc["DHT"]["humidity"] = h; // Humidity as percent
      doc["DHT"]["heat_index"] = dht.computeHeatIndex(t, h, false);
      send_data(doc);

      doc["BMP"]["temp"] = bmp.readTemperature();  // Temp in degrees celcius BMP280 readings
      doc["BMP"]["pressure"] = bmp.readPressure();  // Atmospheric pressure in Pascals
      send_data(doc);

      // If it has taken at least one second, read the air quality again
      if (1000 <= (current_time - last_air_quality_read)) {
        doc["SGP"]["air_quality"] = air_quality.getVOCindex(h, t);  // VOC index  SGP40 readings
        last_air_quality_read = current_time;
        send_data(doc);
      }
    }

    if (gyro.read() == GY521_OK) {  // gy-521 readings
      doc["GY"]["temp"] = gyro.getTemperature();  // gyro has a temperature reading?

      doc["GY"]["angle_x"] = gyro.getAngleX();
      doc["GY"]["angle_y"] = gyro.getAngleY();
      doc["GY"]["angle_z"] = gyro.getAngleZ();

      doc["GY"]["accel_x"] = gyro.getAccelX();
      doc["GY"]["accel_y"] = gyro.getAccelY();
      doc["GY"]["accel_z"] = gyro.getAccelZ();

      doc["GY"]["gyro_x"] = gyro.getGyroX();
      doc["GY"]["gyro_y"] = gyro.getGyroY();
      doc["GY"]["gyro_z"] = gyro.getGyroZ();
      send_data(doc);
    }

    if (ISM_sensor.checkStatus()) { // ism330dhcx readings
      ISM_sensor.getAccel(&accel_data);
      ISM_sensor.getGyro(&gyro_data);

      doc["ISM"]["accel_x"] = accel_data.xData;
      doc["ISM"]["accel_y"] = accel_data.yData;
      doc["ISM"]["accel_z"] = accel_data.zData;

      doc["ISM"]["gyro_x"] = gyro_data.xData;
      doc["ISM"]["gyro_y"] = gyro_data.yData;
      doc["ISM"]["gyro_z"] = gyro_data.zData;
      send_data(doc);
    }

    if (magnometer.isConnected()) {
      doc["MMC"]["temp"] = magnometer.getTemperature();  // Can never have too many temperature sensors  mmc5983ma readings
      doc["MMC"]["x"] = (((double)magnometer.getMeasurementX()) - 131072.0) / 131072.0;
      doc["MMC"]["y"] = (((double)magnometer.getMeasurementY()) - 131072.0) / 131072.0;
      doc["MMC"]["z"] = (((double)magnometer.getMeasurementZ()) - 131072.0) / 131072.0;
      send_data(doc);
    }

    uint16_t data_ready;
    int16_t ret;
    ret = sps30_read_data_ready(&data_ready);

    if ((ret == 0) && data_ready) {
      sps30_measurement measurements;

      ret = sps30_read_measurement(&measurements);
      if (ret == 0) { // sps30 readings
        doc["SPS"]["PM1"] = measurements.mc_1p0;  // Particle mass concentration
        doc["SPS"]["PM2.5"] = measurements.mc_2p5;
        doc["SPS"]["PM4"] = measurements.mc_4p0;
        doc["SPS"]["PM10"] = measurements.mc_10p0;

        doc["SPS"]["NC0.5"] = measurements.nc_0p5;  // Particle number concentration
        doc["SPS"]["NC1"] = measurements.nc_1p0;
        doc["SPS"]["NC2.5"] = measurements.nc_2p5;
        doc["SPS"]["NC4"] = measurements.nc_4p0;
        doc["SPS"]["NC10"] = measurements.nc_10p0;

        doc["SPS"]["typical_particle_size"] = measurements.typical_particle_size;

        send_data(doc);
      }
    }
  }

  delay(DELAY_ms);  // Wait a bit before reading again
}