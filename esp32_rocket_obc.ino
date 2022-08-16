#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>

#include <Adafruit_BMP085.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#include <WiFi.h>
#include <esp_now.h>

#define RXD2 16 // ESP32 RX pin for GPS module
#define TXD2 17 // ESP32 TX pin for GPS module

#define SERVO_PIN 26 // ESP32 pin GIOP26 connected to servo motor

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float gps_lat;
  float gps_lon;
  float gps_alt;
  float gps_speed_mps;
  float bmp_temp;
  float bmp_alt;
  bool parachute_deploy;
} struct_message;

struct_message rocketTelemetry;

Servo servoMotor;
TinyGPS gps;
Adafruit_BMP085 bmp;

unsigned int base_pressure;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  if (!bmp.begin()) {
	  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
	while (1) {}
  }
  Serial.println("Altimeter calibration...");
  base_pressure = bmp.readPressure();

  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin
  // rotates from 0 degrees to 180 degrees
  for (int pos = 90; pos <= 110; pos += 1) {
    // in steps of 1 degree
    servoMotor.write(pos);
    delay(15); // waits 15ms to reach the position
  }
  for (int pos = 110; pos >= 90; pos -= 1) {
    // in steps of 1 degree
    servoMotor.write(pos);
    delay(15); // waits 15ms to reach the position
  }
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


}

void loop() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float bmp_temp, bmp_press, bmp_alt, bmp_rel_alt;
 

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon, falt, fmps;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    falt = gps.f_altitude();
    fmps = gps.f_speed_mps();
    flat = TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    flon = TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    falt = TinyGPS::GPS_INVALID_F_ALTITUDE ? 0.0 : falt, 6;
    fmps = TinyGPS::GPS_INVALID_F_SPEED ? 0.0 : fmps, 6;

    rocketTelemetry.gps_lat = flat;
    rocketTelemetry.gps_lon = flon;
    rocketTelemetry.gps_alt = falt;
    rocketTelemetry.gps_speed_mps = fmps;

    Serial.print("LAT=");
    Serial.print(flat);
    Serial.print(" LON=");
    Serial.print(flon);
    Serial.print(" ALT=");
    Serial.print(falt);
    Serial.print(" SPEED_MPS=");
    Serial.print(fmps);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  bmp_temp = bmp.readTemperature();
  bmp_press = bmp.readPressure();
  bmp_alt = bmp.readAltitude();
  bmp_rel_alt = bmp.readAltitude(base_pressure);

  Serial.print("Temperature = ");
  Serial.print(bmp_temp);
  Serial.print(" *C, ");

  Serial.print("Pressure = ");
  Serial.print(bmp_press);
  Serial.print(" Pa, ");

  Serial.print("Altitude = ");
  Serial.print(bmp_alt);
  Serial.print(" meters");

  Serial.print("Relative altitude = ");
  Serial.print(bmp_rel_alt);
  Serial.println(" meters");

  rocketTelemetry.bmp_alt = bmp_rel_alt;
  rocketTelemetry.bmp_temp = bmp_temp;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &rocketTelemetry, sizeof(rocketTelemetry));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

}

