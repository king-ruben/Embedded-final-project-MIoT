#include "Sensors.hpp"
// GPS

static I2C i2c(SDA, SCL);  // I2C1
static std::uint32_t rcvFlag;

extern EventFlags event_flags;  // first defined in main.cpp
extern BusOut ledRGB;

// Acc
Queue<float[3], 1> accQueue;
static float accValues[3];

// RGB
Queue<int[5], 1> rgbQueue;
static int measureValues[5];
static int maxColour;
static const int sensor_addr = (41 << 1);
static int red_value, green_value, blue_value;

// Temp and humidity
Queue<float[2], 1> tempHumidityQueue;
static float measureValuesTempHumidity[2];

static uint8_t rx_buff[8];
static uint8_t tx_buff[2];

static uint32_t rhData;
static int32_t tData;

// Analog sensors
Queue<float[2], 1> lightAndSoilQueue;
static float measureValuesLightAndSoil[2];

static float lightPercentage;
static float soilMoisturePercentage;
static AnalogIn light(LIGHT_SENSOR_PIN);
static AnalogIn soilMoisture(SOIL_MOISTURE_PIN);

// Normal mode:
// store every hour the temp, humidity, light and soil: mean, min and max values
static uint8_t hourCounter = 0;  // 0..120
static float ligtMem[120];
static float soilMem[120];
static float tempMem[120];
static float humidityMem[120];
static float xMem[120];
static float yMem[120];
static float zMem[120];

static float lightMin, lightMax, lightMean;
static float soilMin, soilMax, soilMean;
static float tempMin, tempMax, tempMean;
static float humidityMin, humidityMax, humidityMean;
static float xMin, xMax, xMean;
static float yMin, yMax, yMean;
static float zMin, zMax, zMean;
static int howManyTimesRed, howManyTimesGreen, howManyTimesBlue;
extern std::string mostRepeatedColour;

extern GPSData *receivedData;
static uint8_t data[2] = {REG_CTRL_REG_1, 0x01};

// ADVANCED
static DigitalOut soil_power(SOIL_POWER_PIN, 1);

void i2cSensorsThreadFunction(void) {
  /// Accel
  writeRegs(data, 2);

  // RGB
  connectRgbSensor(100 * 1000);  // Hz
  initRgbSensor();

  // Temp&hum
  check();

  while (true) {
    rcvFlag = event_flags.wait_any(FLAG_TEST | FLAG_NORMAL | FLAG_ADVANCED);

    // Acc
    accValues[0] = getAccX();
    accValues[1] = getAccY();
    accValues[2] = getAccZ();

    // RGB
    measureClearRedGreenBlue();

    // Temp
    measureTemp();
    measureHumidity();

    // Analog
    lightPercentage = (light.read() * 100);
    measureValuesLightAndSoil[0] = lightPercentage;

    soilMoisturePercentage = (soilMoisture.read() * 100);
    measureValuesLightAndSoil[1] = soilMoisturePercentage;

    switch (rcvFlag) {
      case FLAG_TEST:
       // ledRGB.write(0);
        soil_power = 1;
        accQueue.try_put(&accValues);
        rgbQueue.try_put(&measureValues);
        tempHumidityQueue.try_put(&measureValuesTempHumidity);
        lightAndSoilQueue.try_put(&measureValuesLightAndSoil);
        break;

      case FLAG_NORMAL:
        //ledRGB.write(0);
        soil_power = 1;
        ligtMem[hourCounter] = lightPercentage;
        soilMem[hourCounter] = soilMoisturePercentage;
        tempMem[hourCounter] = (float)(tData / 1000.0f);
        humidityMem[hourCounter] = (float)(rhData / 1000.0f);

        xMem[hourCounter] = accValues[0];
        xMem[hourCounter] = accValues[1];
        zMem[hourCounter] = accValues[2];
        hourCounter++;

        if (hourCounter == 120) {  // 3600s/30s

          calculateStatistics(ligtMem, 120, lightMin, lightMax, lightMean);
          calculateStatistics(soilMem, 120, soilMin, soilMax, soilMean);
          calculateStatistics(tempMem, 120, tempMin, tempMax, tempMean);
          calculateStatistics(humidityMem, 120, humidityMin, humidityMax,
                              humidityMean);
          calculateStatistics(xMem, 120, xMin, xMax, xMean);
          calculateStatistics(yMem, 120, yMin, yMax, yMean);
          calculateStatistics(zMem, 120, zMin, zMax, zMean);

          mostRepeatedColour = dominantColour_str(
              measureValues[1], measureValues[2], measureValues[3]);

          showStats();
          if (receivedData) {
            printf("\r\n-----> GPS\r\n");
            printf("Time: %d:%d:%d\r\n", receivedData->hour,
                   receivedData->minute, receivedData->seconds);
            printf("Location: %5.2f%c, %5.2f%c\r\n", receivedData->latitude,
                   receivedData->lat, receivedData->longitude,
                   receivedData->lon);
            printf("Altitude: %5.2f\r\n", receivedData->altitude);
            printf("Fix Quality: %d\r\n", receivedData->fixquality);
            printf("Satellites: %d\r\n", receivedData->satellites);
          }
          // Limits
          checkLimits(lightPercentage, LIGHT_MIN_LIMIT, LIGHT_MAX_LIMIT,
                      "Light");
          checkLimits(soilMoisturePercentage, SOIL_MIN_LIMIT, SOIL_MAX_LIMIT,
                      "Soil Moisture");
          checkLimits((float)(tData / 1000.0f), TEMP_MIN_LIMIT, TEMP_MAX_LIMIT,
                      "Temperature");
          checkLimits((float)(rhData / 1000.0f), HUMIDITY_MIN_LIMIT,
                      HUMIDITY_MAX_LIMIT, "Humidity");
          checkLimits(accValues[0], ACC_MIN_LIMIT, ACC_MAX_LIMIT, "Accel X");
          checkLimits(accValues[1], ACC_MIN_LIMIT, ACC_MAX_LIMIT, "Accel Y");
          checkLimits(accValues[2], ACC_MIN_LIMIT, ACC_MAX_LIMIT, "Accel Z");

          hourCounter = 0;  // rst
        }
        break;

      case FLAG_ADVANCED:
        soil_power = 1;
        doNormal();
        soil_power = 0;
        showStats();
        __WFI();
        break;

      default:
        break;
    }
  }
}

// Accelerometer
uint8_t getWhoAmI() {
  uint8_t who_am_i = 0;
  readRegs(REG_WHO_AM_I, &who_am_i, 1);
  return who_am_i;
}

float getAccX() { return (float(getAccAxis(REG_OUT_X_MSB)) / 4096.0); }

float getAccY() { return (float(getAccAxis(REG_OUT_Y_MSB)) / 4096.0); }

float getAccZ() { return (float(getAccAxis(REG_OUT_Z_MSB)) / 4096.0); }

void getAccAllAxis(float *res) {
  res[0] = getAccX();
  res[1] = getAccY();
  res[2] = getAccZ();
}

int16_t getAccAxis(uint8_t addr) {
  int16_t acc;
  uint8_t res[2];
  readRegs(addr, res, 2);

  acc = (res[0] << 6) | (res[1] >> 2);
  if (acc > UINT14_MAX / 2) {
    acc -= UINT14_MAX;
  }
  return acc;
}

void readRegs(int addr, uint8_t *data, int len) {
  char t[1] = {(char)addr};
  i2c.write(MMA8451_I2C_ADDRESS, t, 1, true);
  i2c.read(MMA8451_I2C_ADDRESS, (char *)data, len);
}

void writeRegs(uint8_t *data, int len) {
  i2c.write(MMA8451_I2C_ADDRESS, (char *)data, len);
}
/// end acc functions

// RGB
void initRgbSensor(void) {
  // Initialize color sensor
  char timing_register[2] = {129, 0};
  i2c.write(sensor_addr, timing_register, 2, false);

  char control_register[2] = {143, 0};
  i2c.write(sensor_addr, control_register, 2, false);

  char enable_register[2] = {128, 3};
  i2c.write(sensor_addr, enable_register, 2, false);
}

void measureClearRedGreenBlue(void) {
  // Clear
  const char clear_reg[1] = {148};
  char clear_data[2] = {0, 0};
  i2c.write(sensor_addr, clear_reg, 1, true);
  i2c.read(sensor_addr, clear_data, 2, false);

  int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
  measureValues[0] = clear_value;

  // Red
  const char red_reg[1] = {150};
  char red_data[2] = {0, 0};
  i2c.write(sensor_addr, red_reg, 1, true);
  i2c.read(sensor_addr, red_data, 2, false);

  red_value = ((int)red_data[1] << 8) | red_data[0];
  measureValues[1] = red_value;

  ThisThread::sleep_for(10ms);

  // Green
  const char green_reg[1] = {152};
  char green_data[2] = {0, 0};
  i2c.write(sensor_addr, green_reg, 1, true);
  i2c.read(sensor_addr, green_data, 2, false);

  green_value = ((int)green_data[1] << 8) | green_data[0];
  measureValues[2] = green_value;

  // Blue
  const char blue_reg[1] = {154};
  char blue_data[2] = {0, 0};
  i2c.write(sensor_addr, blue_reg, 1, true);
  i2c.read(sensor_addr, blue_data, 2, false);

  blue_value = ((int)blue_data[1] << 8) | blue_data[0];
  measureValues[3] = blue_value;

  // for test mode
  measureValues[4] =
      dominantColour_int(measureValues[1], measureValues[2], measureValues[3]);
  //   rgbIntegratedLED = !rgbIntegratedLED; // flash sensor LED
}

bool connectRgbSensor(int freq) {
  // UNUSED(freq);
  // i2c.frequency(freq);
  bool connected;
  const char id_regval[1] = {146};
  char data[1] = {0};

  i2c.write(sensor_addr, id_regval, 1, true);
  i2c.read(sensor_addr, data, 1, false);

  if (data[0] == 68) {
    connected = true;
  } else {
    connected = false;
  }

  return connected;
}

std::string dominantColour_str(int r, int g, int b) {
  if ((r >= g) && (r >= b)) {
    return "RED";
  } else if ((g >= r) && (g >= b)) {
    return "GREEN";
  } else {
    return "BLUE";
  }
}

int dominantColour_int(int r, int g, int b) {
  if ((r >= g) && (r >= b)) {
    return RED;
  } else if ((g >= r) && (g >= b)) {
    return GREEN;
  } else {
    return BLUE;
  }
}

// Temp
bool measureTemp(void) {
  tx_buff[0] = READ_TEMP;
  if (i2c.write(ADDR, (char *)tx_buff, 1) != 0) return 0;
  if (i2c.read(ADDR, (char *)rx_buff, 2) != 0) return 0;

  tData = ((uint32_t)rx_buff[0] << 8) + (rx_buff[1] & 0xFC);
  tData = (((tData) * 21965L) >> 13) - 46850;
  measureValuesTempHumidity[0] = (float)(tData / 1000.0f);
  return 1;
}

bool measureHumidity(void) {
  tx_buff[0] = READ_RH;
  if (i2c.write(ADDR, (char *)tx_buff, 1, true) != 0) return 0;
  ThisThread::sleep_for(20ms);
  if (i2c.read(ADDR, (char *)rx_buff, 2, false) != 0) return 0;
  rhData = ((uint32_t)rx_buff[0] << 8) + (rx_buff[1] & 0xFC);
  rhData = (((rhData) * 15625L) >> 13) - 6000;
  measureValuesTempHumidity[1] = (float)(rhData / 1000.0f);
  // printf("rH stored: %.2f\r\n", (float)measureValuesTempHumidity[1]);
  return 1;
}

bool check(void) {
  tx_buff[0] = READ_ID2_1;
  tx_buff[1] = READ_ID2_2;
  if (i2c.write(ADDR, (char *)tx_buff, 2) != 0) return 0;
  if (i2c.read(ADDR, (char *)rx_buff, 8) != 0) return 0;

  if (rx_buff[0] == DEVICE_ID) return true;
  return false;
}

// Normal mode things

void calculateStatistics(float *data, int size, float &min, float &max,
                         float &mean) {
  min = *std::min_element(data, data + size);
  max = *std::max_element(data, data + size);
  mean = std::accumulate(data, data + size, 0.0f) / size;
}

void showStats(void) {
  printf("\r\n");
  printf("| Sensor      | Min   | Max   | Mean  | Unit   |\r\n");
  printf("|-------------|-------|-------|-------|--------|\r\n");
  printf("| Light       | %5.2f | %5.2f | %5.2f | %%      |\r\n", lightMin,
         lightMax, lightMean);
  printf("| Soil        | %5.2f | %5.2f | %5.2f | %%      |\r\n", soilMin,
         soilMax, soilMean);
  printf("| Temperature | %5.2f | %5.2f | %5.2f | °C     |\r\n", tempMin,
         tempMax, tempMean);
  printf("| Humidity    | %5.2f | %5.2f | %5.2f | %%      |\r\n", humidityMin,
         humidityMax, humidityMean);
  printf("| X           | %5.2f | %5.2f | %5.2f | m/s^2  |\r\n", xMin, xMax,
         xMean);
  printf("| Y           | %5.2f | %5.2f | %5.2f | m/s^2  |\r\n", yMin, yMax,
         yMean);
  printf("| Z           | %5.2f | %5.2f | %5.2f | m/s^2  |\r\n", zMin, zMax,
         zMean);

  printf("\r\n");
  printf("| Dominant colour: %s\r\n", mostRepeatedColour.c_str());
}

bool stringEquals(const char *str1, const char *str2) {
  int i = 0;
  while (str1[i] != '\0' && str2[i] != '\0') {
    if (str1[i] != str2[i]) {
      return false;  // Los caracteres son diferentes
    }
    i++;
  }
  // Comprueba que ambas cadenas han llegado al final
  return (str1[i] == '\0' && str2[i] == '\0');
}

bool doOnce = true;
void checkLimits(float value, float minLimit, float maxLimit,
                 const char *sensorName) {
  bool outOfBounds = (value < minLimit || value > maxLimit);

  if (outOfBounds) {
    printf("Warning: %s value %.2f is out of bounds (%.2f - %.2f)\r\n",
           sensorName, value, minLimit, maxLimit);

    // Set LED color based on sensor type using a single integer value
    if (stringEquals(sensorName, "Temperature")) {
      ledRGB.write(RED);  // Rojo para Temperatura
    } else if (stringEquals(sensorName, "Humidity")) {
      ledRGB.write(BLUE);  // Azul para Humedad
    } else if (stringEquals(sensorName, "Light")) {
      ledRGB.write(GREEN);  // Verde para Luz
    } else if (stringEquals(sensorName, "Soil Moisture")) {
      ledRGB.write(YELLOW);  // Amarillo (Rojo + Verde) para Humedad de suelo
    } else if (stringEquals(sensorName, "Accel X") ||
               stringEquals(sensorName, "Accel Y") ||
               stringEquals(sensorName, "Accel Z")) {
      ledRGB.write(WHITE);  // Blanco (Rojo + Verde + Azul) para Aceleración
                            // fuera de rango
    }
  } else {
      printf("Sensor inside the stablished limits\r\n");
  }
}

void doNormal(void) {
  //     // Storing
  soil_power = 1;
  ligtMem[hourCounter] = lightPercentage;
  soilMem[hourCounter] = soilMoisturePercentage;
  tempMem[hourCounter] = (float)(tData / 1000.0f);
  humidityMem[hourCounter] = (float)(rhData / 1000.0f);

  xMem[hourCounter] = accValues[0];
  xMem[hourCounter] = accValues[1];
  zMem[hourCounter] = accValues[2];
  hourCounter++;

  if (hourCounter == 120) {  // 3600s/30s

    calculateStatistics(ligtMem, 120, lightMin, lightMax, lightMean);
    calculateStatistics(soilMem, 120, soilMin, soilMax, soilMean);
    calculateStatistics(tempMem, 120, tempMin, tempMax, tempMean);
    calculateStatistics(humidityMem, 120, humidityMin, humidityMax,
                        humidityMean);
    calculateStatistics(xMem, 120, xMin, xMax, xMean);
    calculateStatistics(yMem, 120, yMin, yMax, yMean);
    calculateStatistics(zMem, 120, zMin, zMax, zMean);

    mostRepeatedColour = dominantColour_str(measureValues[1], measureValues[2],
                                            measureValues[3]);

    showStats();
    if (receivedData) {
      printf("\r\n-----> GPS\r\n");
      printf("Time: %d:%d:%d\r\n", receivedData->hour, receivedData->minute,
             receivedData->seconds);
      printf("Location: %5.2f%c, %5.2f%c\r\n", receivedData->latitude,
             receivedData->lat, receivedData->longitude, receivedData->lon);
      printf("Altitude: %5.2f\r\n", receivedData->altitude);
      printf("Fix Quality: %d\r\n", receivedData->fixquality);
      printf("Satellites: %d\r\n", receivedData->satellites);
    }
    // Limits
    checkLimits(lightPercentage, LIGHT_MIN_LIMIT, LIGHT_MAX_LIMIT, "Light");
    checkLimits(soilMoisturePercentage, SOIL_MIN_LIMIT, SOIL_MAX_LIMIT,
                "Soil Moisture");
    checkLimits((float)(tData / 1000.0f), TEMP_MIN_LIMIT, TEMP_MAX_LIMIT,
                "Temperature");
    checkLimits((float)(rhData / 1000.0f), HUMIDITY_MIN_LIMIT,
                HUMIDITY_MAX_LIMIT, "Humidity");
    checkLimits(accValues[0], ACC_MIN_LIMIT, ACC_MAX_LIMIT, "Accel X");
    checkLimits(accValues[1], ACC_MIN_LIMIT, ACC_MAX_LIMIT, "Accel Y");
    checkLimits(accValues[2], ACC_MIN_LIMIT, ACC_MAX_LIMIT, "Accel Z");

    hourCounter = 0;  // rst
  }
}