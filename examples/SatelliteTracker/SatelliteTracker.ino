#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample code monitors all the visible satellites: PRN, elevation,
   azimuth and signal-to-noise ratio, plus the total in view.

   All of it is parsed first-class from the $xxGSV sentences into
   gps.trackedSatellites[] and gps.satellitesInView, so no per-field objects
   are needed. Earlier versions of this example wired up sixteen TinyGPSCustom
   objects to reach the same data; that class no longer exists.

   Note that GSV is sent per constellation, each with its own talker ID
   (GPGSV, GLGSV, GAGSV, GBGSV), and this library matches on the last three
   characters -- so a multi-GNSS receiver fills the array from every
   constellation it tracks.

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

  Serial.println(F("SatelliteTracker.ino"));
  Serial.println(F("Monitoring satellite location and signal strength"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void loop()
{
  // Dispatch incoming characters
  if (ss.available() > 0)
  {
    gps.encode(ss.read());

    // satellitesInView updates once per GSV group, which makes it a good trigger
    if (gps.satellitesInView.isUpdated())
    {
      Serial.print(F("In view: "));
      Serial.println(gps.satellitesInView.value());

      Serial.print(F("PRN:  "));
      for (int i = 0; i < TINYGPS_MAX_SATS; ++i)
        if (gps.trackedSatellites[i].prn != 0)
        {
          Serial.print(gps.trackedSatellites[i].prn);
          Serial.print(F(" "));
        }
      Serial.println();

      Serial.print(F("SNR:  "));
      for (int i = 0; i < TINYGPS_MAX_SATS; ++i)
        if (gps.trackedSatellites[i].prn != 0)
        {
          Serial.print(gps.trackedSatellites[i].strength);
          Serial.print(F(" "));
        }
      Serial.println();

      Serial.print(F("Elev: "));
      for (int i = 0; i < TINYGPS_MAX_SATS; ++i)
        if (gps.trackedSatellites[i].prn != 0)
        {
          Serial.print(gps.trackedSatellites[i].elevation);
          Serial.print(F(" "));
        }
      Serial.println();

      Serial.print(F("Azim: "));
      for (int i = 0; i < TINYGPS_MAX_SATS; ++i)
        if (gps.trackedSatellites[i].prn != 0)
        {
          Serial.print(gps.trackedSatellites[i].azimuth);
          Serial.print(F(" "));
        }
      Serial.println();
      Serial.println();
    }
  }
}
