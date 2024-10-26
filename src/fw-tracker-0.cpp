#include <Adafruit_GPS.h>
#include <oled-wing-adafruit.h>
#include "oled-wing-adafruit.h"

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
SYSTEM_THREAD(ENABLED);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

void update();
void particlePublish(String eventName, String data);
void updateDisplay(String data);
String buildTimeStamp(Adafruit_GPS GPS);
void initializePowerManagement();
void managePower();

unsigned long lastSerial = 0;
unsigned long lastPublish = 0;
unsigned long startFix = 0;
bool gettingFix = false;
int voltageReadPin = A1;
int batteryTogglePin = A2;
PinState mainPowerConnected = LOW;
unsigned long powerDownTimer = 0;

const unsigned long POWER_DOWN_TIMEOUT = 60000;
const unsigned long PUBLISH_PERIOD = 5000;
const unsigned long SERIAL_PERIOD = 5000;
const unsigned long MAX_GPS_AGE_MS = 10000;
OledWingAdafruit display;

void setup()
{
  initializePowerManagement();

  Serial.begin(115200);
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  display.setup();
  display.clearDisplay();
  display.display();
}

void loop()
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                       // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - lastSerial >= SERIAL_PERIOD)
  {
    lastSerial = millis();
    update();
  }

  managePower();
}

void update()
{
  if (GPS.fix)
  {
    if (gettingFix)
    {
      gettingFix = false;
      unsigned long elapsed = millis() - startFix;
      Serial.printlnf("%lu milliseconds to get GPS fix", elapsed);
    }

    String displayBuffer = String::format("(%f, %f)", GPS.latitudeDegrees, GPS.longitudeDegrees);
    updateDisplay(displayBuffer);

    // (lat,lng)|speed|altitude|angle|timestamp
    String event = String::format("%f|%f|%f|%f|%f|%s",
                                  GPS.latitudeDegrees,
                                  GPS.longitudeDegrees,
                                  GPS.speed,
                                  GPS.altitude,
                                  GPS.angle,
                                  buildTimeStamp(GPS).c_str());

    Serial.println(event);
    if (Particle.connected())
    {
      if (millis() - lastPublish >= PUBLISH_PERIOD)
      {
        lastPublish = millis();
        particlePublish("tracks.position.update", event);
      }
    }
  }
  else
  {
    if (!gettingFix)
    {
      gettingFix = true;
      startFix = millis();
    }
    updateDisplay("Getting GPS Fix...");
  }
}

void particlePublish(String eventName, String data)
{
  Particle.publish(eventName, data, 60, PRIVATE);
}

void updateDisplay(String data)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(data);
  display.display();
}

String buildTimeStamp(Adafruit_GPS GPS)
{
  String timeStamp;

  String hour = String::format("%s%d", GPS.hour < 10 ? "0" : "", GPS.hour);
  String minute = String::format("%s%d", GPS.minute < 10 ? "0" : "", GPS.minute);
  String second = String::format("%s%d", GPS.seconds < 10 ? "0" : "", GPS.seconds);
  String millisecond = String::format("%s%s%d", GPS.milliseconds < 10 ? "0" : "", GPS.milliseconds < 100 ? "0" : "", GPS.milliseconds);
  String date = String::format("20%d-%d-%d", GPS.year, GPS.month, GPS.day);
  return String::format("%sT%s:%s:%s.%s", date.c_str(), hour.c_str(), minute.c_str(), second.c_str(), millisecond.c_str());
}

void initializePowerManagement() {
  pinMode(batteryTogglePin, OUTPUT);
  pinMode(voltageReadPin, INPUT_PULLDOWN);
  digitalWrite(batteryTogglePin, HIGH);
  mainPowerConnected = (PinState)digitalRead(voltageReadPin);
}

void managePower() {
  mainPowerConnected = (PinState)digitalRead(voltageReadPin);
  if (!mainPowerConnected) {

    // Capture time power disconnected
    if (!powerDownTimer) {
      powerDownTimer = millis();
    }
    
    // If power is disconnected for POWER_DOWN_TIMEOUT, disconnect battery
    else if (millis() - powerDownTimer >= POWER_DOWN_TIMEOUT) {
      digitalWrite(batteryTogglePin, LOW);
    }
  }
  else {
    powerDownTimer = 0;
  }
}