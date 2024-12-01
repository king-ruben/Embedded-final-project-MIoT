#include "common.hpp"
#include <cmath> // floor()
#include "GPS.hpp"

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////    ACCELEROMETER  //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define MMA8451_I2C_ADDRESS (0x1D << 1)  // Sensor address
// Registers
#define REG_WHO_AM_I      0x0D
#define REG_CTRL_REG_1    0x2A
#define REG_OUT_X_MSB     0x01
#define REG_OUT_Y_MSB     0x03
#define REG_OUT_Z_MSB     0x05

// Others
#define UINT14_MAX 16383

// Function headers
void calculateStatistics(float *data, int size, float &min, float &max, float &mean);
void i2cSensorsThreadFunction(void);
void accelMeasureThreadFunction(void);
void readRegs(int addr, uint8_t *data, int len);
void writeRegs(uint8_t *data, int len);
void getAccAllAxis(float *res);
uint8_t getWhoAmI();
float getAccX();
float getAccY();
float getAccZ();
int16_t getAccAxis(uint8_t addr);

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////    RGB SENSOR  //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void rgbMeasureThreadFunction(void);
bool connectRgbSensor(int freq);
bool connectRgbSensor(void);
void initRgbSensor(void);
void measureClearRedGreenBlue(void);

//Others:
std::string dominantColour_str(int r, int g, int b);
int dominantColour_int(int r, int g, int b);
void calcMostFrequentRGBColour(void);
std::string mostFreqColour2String(int r, int g, int b);
void checkLimits(float value, float minLimit, float maxLimit, const char* sensorName);
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////    TEMP. HUMIDITY ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void tempHumidityThreadFunction(void);
bool measureHumidity();
bool measureTemp();
bool check();


// Others:
void showStats(void);
bool stringEquals(const char* str1, const char* str2);
void checkLimits(float value, float minLimit, float maxLimit, const char* sensorName);
void doNormal(void);

/** Si7012 Read Temperature Command */
#define READ_TEMP        0xE0 /* Read previous T data from RH measurement command*/
/** Si7012 Read RH Command */
#define READ_RH          0xE5 /* Perform RH (and T) measurement. */
/** Si7012 Read ID */
#define READ_ID1_1       0xFA
#define READ_ID1_2       0x0F
#define READ_ID2_1       0xFC
#define READ_ID2_2       0xC9
/** Si7012 Read Firmware Revision */
#define READ_FWREV_1     0x84
#define READ_FWREV_2      0xB8
/** I2C device address for Si7021 */
#define ADDR    0x80
/** I2C device frequency for Si7021 */
#define FREQ    100000
/** Device ID value for Si7021 */
#define DEVICE_ID 0x15

// Limits
#define LIGHT_MIN_LIMIT 1.0f
#define LIGHT_MAX_LIMIT 90.0f
#define SOIL_MIN_LIMIT 1.0f
#define SOIL_MAX_LIMIT 90.0f
#define TEMP_MIN_LIMIT 10.0f
#define TEMP_MAX_LIMIT 30.0f
#define HUMIDITY_MIN_LIMIT 0.0f
#define HUMIDITY_MAX_LIMIT 100.0f
#define ACC_MIN_LIMIT -1.5f // Adjust for accelerometer's range
#define ACC_MAX_LIMIT 1.5f   // Adjust for accelerometer's range
