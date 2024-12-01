#include <cstring>

#include "GPS.hpp"
#include "common.hpp"
#define MAXLINELENGTH 120

volatile char line1[MAXLINELENGTH];
volatile char line2[MAXLINELENGTH];
volatile uint16_t lineidx = 0;
volatile char *currentline;
volatile char *lastline;
volatile bool recvdflag;
volatile bool inStandbyMode;

using namespace std::chrono;
UnbufferedSerial *gps_Serial;
extern Thread i2cSensorsThread;
static std::uint32_t rcvFlag;
extern EventFlags event_flags;  // first defined in main.cpp

Queue<GPSData, 1> gpsQueue;
GPSData data;

void gpsThreadFunction(void) {
  gps_Serial = new UnbufferedSerial(PA_9, PA_10, 9600);
  GPS myGPS(gps_Serial);
  static Timer refresh_Timer;
  static const int refresh_Time = 1500;

  static char c;

  myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  myGPS.sendCommand(PGCMD_ANTENNA);

  refresh_Timer.start();  // starts the clock on the timer

  while (true) {

    c = myGPS.read();  // queries the GPS
    if (myGPS.newNMEAreceived()) {
      if (!myGPS.parse(myGPS.lastNMEA())) {
        continue;
      }
    }
    if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >=
        refresh_Time) {
      refresh_Timer.reset();

      if (myGPS.fixquality) {
        data.hour = myGPS.hour + 1;
        data.minute = myGPS.minute;
        data.seconds = myGPS.seconds;
        data.latitude = myGPS.latitude;
        data.longitude = myGPS.longitude / 100;
        data.lat = myGPS.lat;
        data.lon = myGPS.lon;
        data.altitude = myGPS.altitude;
        data.fixquality = myGPS.fixquality;
        data.satellites = myGPS.satellites;
        gpsQueue.try_put(&data);
      }
    }
  }
}

bool GPS::parse(char *nmea) {
  // do checksum check

  // first look if we even have one
  if (nmea[strlen(nmea) - 4] == '*') {
    uint16_t sum = parseHex(nmea[strlen(nmea) - 3]) * 16;
    sum += parseHex(nmea[strlen(nmea) - 2]);

    // check checksum
    for (uint8_t i = 1; i < (strlen(nmea) - 4); i++) {
      sum ^= nmea[i];
    }
    if (sum != 0) {
      // bad checksum :(
      // return false;
    }
  }

  // look for a few common sentences
  if (strstr(nmea, "$GPGGA")) {
    // found GGA
    char *p = nmea;
    // get time
    p = strchr(p, ',') + 1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod((double)timef, 1.0) * 1000;

    // parse out latitude
    p = strchr(p, ',') + 1;
    latitude = atof(p);

    p = strchr(p, ',') + 1;
    if (p[0] == 'N')
      lat = 'N';
    else if (p[0] == 'S')
      lat = 'S';
    else if (p[0] == ',')
      lat = 0;
    else
      return false;

    // parse out longitude
    p = strchr(p, ',') + 1;
    longitude = atof(p);

    p = strchr(p, ',') + 1;
    if (p[0] == 'W')
      lon = 'W';
    else if (p[0] == 'E')
      lon = 'E';
    else if (p[0] == ',')
      lon = 0;
    else
      return false;

    p = strchr(p, ',') + 1;
    fixquality = atoi(p);

    p = strchr(p, ',') + 1;
    satellites = atoi(p);

    p = strchr(p, ',') + 1;
    HDOP = atof(p);

    p = strchr(p, ',') + 1;
    altitude = atof(p);
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    geoidheight = atof(p);
    return true;
  }
  if (strstr(nmea, "$GPRMC")) {
    // found RMC
    char *p = nmea;

    // get time
    p = strchr(p, ',') + 1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    //milliseconds = fmod((double)timef, 1.0) * 1000;

    p = strchr(p, ',') + 1;
    // Serial.println(p);
    if (p[0] == 'A')
      fix = true;
    else if (p[0] == 'V')
      fix = false;
    else
      return false;

    // parse out latitude
    p = strchr(p, ',') + 1;
    latitude = atof(p);

    p = strchr(p, ',') + 1;
    if (p[0] == 'N')
      lat = 'N';
    else if (p[0] == 'S')
      lat = 'S';
    else if (p[0] == ',')
      lat = 0;
    else
      return false;

    // parse out longitude
    p = strchr(p, ',') + 1;
    longitude = atof(p);

    p = strchr(p, ',') + 1;
    if (p[0] == 'W')
      lon = 'W';
    else if (p[0] == 'E')
      lon = 'E';
    else if (p[0] == ',')
      lon = 0;
    else
      return false;

    // speed
    p = strchr(p, ',') + 1;
    speed = atof(p);

    // angle
    p = strchr(p, ',') + 1;
    angle = atof(p);

    p = strchr(p, ',') + 1;
    uint32_t fulldate = atof(p);
    day = fulldate / 10000;
    month = (fulldate % 10000) / 100;
    year = (fulldate % 100);

    // we dont parse the remaining, yet!
    return true;
  }

  return false;
}

char GPS::read(void) {
  char c = 0;

  if (paused) return c;

  if (!gpsSerial->readable()) return c;
  gpsSerial->read(&c, 1);

  if (c == '$') {
    currentline[lineidx] = 0;
    lineidx = 0;
  }
  if (c == '\n') {
    currentline[lineidx] = 0;

    if (currentline == line1) {
      currentline = line2;
      lastline = line1;
    } else {
      currentline = line1;
      lastline = line2;
    }

    lineidx = 0;
    recvdflag = true;
  }

  currentline[lineidx++] = c;
  if (lineidx >= MAXLINELENGTH) lineidx = MAXLINELENGTH - 1;

  return c;
}

GPS::GPS(UnbufferedSerial *ser) {
  common_init();
  gpsSerial = ser;
}

// Initialization code used by all constructor types
void GPS::common_init(void) {
  gpsSerial = NULL;
  recvdflag = false;
  paused = false;
  lineidx = 0;
  currentline = line1;
  lastline = line2;

  hour = minute = seconds = year = month = day = fixquality = satellites =
      0;                // uint8_t
  lat = lon = mag = 0;  // char
  fix = false;          // bool
  milliseconds = 0;     // uint16_t
  latitude = longitude = geoidheight = altitude = speed = angle = magvariation =
      HDOP = 0.0;  // float
}

void GPS::begin(int baud) {
  gpsSerial->baud(baud);
  thread_sleep_for(10);
}

void GPS::sendCommand(const char *str) { gpsSerial->write(str, strlen(str)); }

bool GPS::newNMEAreceived(void) { return recvdflag; }

void GPS::pause(bool p) { paused = p; }

char *GPS::lastNMEA(void) {
  recvdflag = false;
  return (char *)lastline;
}

// read a Hex value and return the decimal equivalent
uint8_t GPS::parseHex(char c) {
  if (c < '0') return 0;
  if (c <= '9') return c - '0';
  if (c < 'A') return 0;
  if (c <= 'F') return (c - 'A') + 10;

  return 0;
}

bool GPS::waitForSentence(const char *wait4me, uint8_t max) {
  char str[20];

  uint8_t i = 0;
  while (i < max) {
    if (newNMEAreceived()) {
      char *nmea = lastNMEA();
      strncpy(str, nmea, 20);
      str[19] = 0;
      i++;

      if (strstr(str, wait4me)) return true;
    }
  }

  return false;
}

bool GPS::LOCUS_StartLogger(void) {
  sendCommand(PMTK_LOCUS_STARTLOG);
  recvdflag = false;
  return waitForSentence(PMTK_LOCUS_LOGSTARTED);
}

bool GPS::LOCUS_ReadStatus(void) {
  sendCommand(PMTK_LOCUS_QUERY_STATUS);

  if (!waitForSentence("$PMTKLOG")) return false;

  char *response = lastNMEA();
  uint16_t parsed[10];
  uint8_t i;

  for (i = 0; i < 10; i++) parsed[i] = -1;

  response = strchr(response, ',');
  for (i = 0; i < 10; i++) {
    if (!response || (response[0] == 0) || (response[0] == '*')) break;
    response++;
    parsed[i] = 0;
    while ((response[0] != ',') && (response[0] != '*') && (response[0] != 0)) {
      parsed[i] *= 10;
      char c = response[0];
      if (isdigit(c))
        parsed[i] += c - '0';
      else
        parsed[i] = c;
      response++;
    }
  }
  LOCUS_serial = parsed[0];
  LOCUS_type = parsed[1];
  if (isalpha(parsed[2])) {
    parsed[2] = parsed[2] - 'a' + 10;
  }
  LOCUS_mode = parsed[2];
  LOCUS_config = parsed[3];
  LOCUS_interval = parsed[4];
  LOCUS_distance = parsed[5];
  LOCUS_speed = parsed[6];
  LOCUS_status = !parsed[7];
  LOCUS_records = parsed[8];
  LOCUS_percent = parsed[9];

  return true;
}

// Standby Mode Switches
bool GPS::standby(void) {
  if (inStandbyMode) {
    return false;  // Returns false if already in standby mode, so that you do
                   // not wake it up by sending commands to GPS
  } else {
    inStandbyMode = true;
    sendCommand(PMTK_STANDBY);
    // return waitForSentence(PMTK_STANDBY_SUCCESS);  // don't seem to be fast
    // enough to catch the message, or something else just is not working
    return true;
  }
}

bool GPS::wakeup(void) {
  if (inStandbyMode) {
    inStandbyMode = false;
    sendCommand("");  // send byte to wake it up
    return waitForSentence(PMTK_AWAKE);
  } else {
    return false;  // Returns false if not in standby mode, nothing to wakeup
  }
}

int GPS::_putc(int value) {
  gpsSerial->write((void *)&value, 1);
  return value;
}

int GPS::_getc() {
  char c = 0;
  if (!gpsSerial->readable()) return c;
  gpsSerial->read(&c, 1);
  return (int)c;
}