#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample code tracks satellite elevations, printing each tracked
   satellite's PRN and elevation once per received GSV message.

   Elevation is parsed first-class from the $xxGSV sentences into
   gps.trackedSatellites[], so no per-field objects are needed. An earlier
   version of this example used TinyGPSCustom to reach it; that class no
   longer exists.

   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(RX) and 3(TX).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("SatElevTracker.ino"));
  Serial.println(F("Developed by Mikal Hart"));
  Serial.println();
}

void loop()
{
  // Dispatch incoming characters
  if (ss.available() > 0)
  {
    gps.encode(ss.read());

    // Reading value() is what clears the updated flag; testing isUpdated() alone would leave it
    // set and reprint the list on every character.
    if (gps.satellitesInView.isUpdated())
    {
      Serial.print(F("In view: "));
      Serial.println(gps.satellitesInView.value());

      for (int i = 0; i < TINYGPS_MAX_SATS; ++i)
        if (gps.trackedSatellites[i].prn != 0)
        {
          Serial.print(F("Sat "));
          Serial.print(gps.trackedSatellites[i].prn);
          Serial.print(F(" elevation "));
          Serial.print(gps.trackedSatellites[i].elevation);
          Serial.println(F(" deg"));
        }
      Serial.println();
    }
  }
}
