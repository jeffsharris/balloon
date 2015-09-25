#include <Adafruit_BMP085_U.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "RTClib.h"
#include <SD.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>

/*** Variables/constants pressure sensor ***/
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


/*** Variables/constants GPS module ***/

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

//SoftwareSerial mySerial(3, 2);

//Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
Adafruit_GPS GPS(&Serial1);


// Set GPSECHO to false to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  1

// Set GPSLOGGER to true to log the GPS data to the data logger built into the sensor
// This option can only be exercised if GPSECHO is also true
#define GPSLOGGER 1

// Set GPSERASE to true to erase whatever data is stored in the GPS module before logging commences
// This option can only be exercised if GPSLOGGER is also true
#define GPSERASE 0

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


/*** Variables/constants for the data logger ***/
RTC_DS1307 RTC;

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

#define LOG_INTERVAL     2000

// the logging file
File logfile;

// Reset the real time clock's time to be the current time of the computer
#define SET_CLOCK        1

// Echo data to serial port
#define ECHO_TO_SERIAL   1

void setup(void) 
{
  Serial.begin(9600);
  
  // initialize the SD card
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(10, 11, 12, 13)) {
    Serial.println(F("Card failed, or not present"));
  }
  Serial.println(F("card initialized."));

  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    Serial.println(F("couldnt create file"));
  }
  
  Serial.print(F("Logging to: "));
  Serial.println(filename);
    
  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.print("RTC failed");
  }

  if (! RTC.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
    #if SET_CLOCK
      RTC.adjust(DateTime(__DATE__, __TIME__));
    #endif
  } 

  
  /*** Initialization for the pressure sensor ***/
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print(F("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  /*** Initialization for the GPS module ***/
    // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  
  // You can adjust which sentences to have the module emit, below
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
  // Set the update rate
  // Note you must send both commands below to change both the output rate (how often the position
  // is written to the serial line), and the position fix rate.
  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  // Note the position can only be updated at most 5 times a second so it will lag behind serial output.
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  
  #if GPSLOGGER
  #if GPSERASE
  Serial.print("\nERASING GPS DATA");
  GPS.sendCommand(PMTK_LOCUS_ERASE_FLASH);
  #endif
  
  Serial.print("\nSTARTING GPS LOGGING....");
  if (GPS.LOCUS_StartLogger())
    Serial.println(" STARTED!");
  else
    Serial.println(" no response :(");
  #endif
  
  logfile.print("TimeOn (ms),UnixTime (ms),DateTime (PST),GPSTime (UTC),GPSFixQuality,Latitude,Longitude,Latitude (°),Longitude (°),Speed (knots),GPSAngle (°),GPSAltitude (cm),GPSSatellites,Pressure (hPa),Temp (°C),Altitude (m)\r\n");
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop(void) 
{
  /*** Query the GPS sensor for new readings ***/
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  


  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > LOG_INTERVAL) { 
    timer = millis();
    // log milliseconds since starting
    uint32_t m = millis();
    logfile.print(m);           // milliseconds since start   
    logUnixTime();
    logGPS();
    logPressure();
    logfile.print("\r\n");
    logfile.flush();
  }
}

void logGPS() {
    logGPSTime();
    logGPSFixQuality();
    if (GPS.fix) {
      logGPSLocation();
      logGPSSpeed();
      logGPSAngle();
      logGPSAltitude();
      logGPSSatellites();
      
    } else {
      logfile.print(",,,,,,,,");
    }
}

void logGPSSatellites() {
  logfile.print(",");
  logfile.print((int)GPS.satellites);
}

void logGPSAltitude() {
  logfile.print(",");
  logfile.print(GPS.altitude);
}


void logGPSAngle() {
  logfile.print(",");
  logfile.print(GPS.angle);
}

void logGPSSpeed() {
  logfile.print(",");
  logfile.print(GPS.speed);
}

void logGPSLocation() {
  logfile.print(",");
  logfile.print(GPS.latitude, 4);
  logfile.print(GPS.lat);
  logfile.print(",");
  logfile.print(GPS.longitude, 4);
  logfile.print(GPS.lon);
  logfile.print(",");
  logfile.print(GPS.latitudeDegrees, 4);
  logfile.print(",");
  logfile.print(GPS.longitudeDegrees, 4);
}

void logGPSFixQuality() {
 logfile.print(",");
 logfile.print((int)GPS.fixquality);
}

void logGPSTime() {
    logfile.print(",");
    logfile.print('"');
    logfile.print("20"); logfile.print(GPS.year, DEC);
    logfile.print("/");
    logfile.print(GPS.month, DEC);
    logfile.print("/");
    logfile.print(GPS.day, DEC);
    logfile.print(" ");
    logfile.print(GPS.hour, DEC);
    logfile.print(":");
    logfile.print(GPS.minute, DEC);
    logfile.print(':');
    logfile.print(GPS.seconds, DEC);
    logfile.print('"');
}

void logPressure() {
/* Get a new pressure sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    float temperature;
    bmp.getTemperature(&temperature);
 
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    logfile.print(",");
    logfile.print(event.pressure);
    logfile.print(",");
    logfile.print(temperature);
    logfile.print(",");
    logfile.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
  }
  else
  {
    Serial.println(F("Sensor error"));
    logfile.print(",,,");
  }
}

void logUnixTime() {
    DateTime now = RTC.now();
    // log time
    logfile.print(",");
    logfile.print(now.unixtime()); // seconds since 1/1/1970
    logfile.print(",");
    logfile.print('"');
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print('"');
}

