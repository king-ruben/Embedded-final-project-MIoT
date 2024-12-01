#ifndef __cplusplus
#error C++ compiler needed!
#endif

#include <stdio.h>
#include <string.h>

#include <cstdint>

#include "Sensors.hpp"
#include "mbed.h"

static BusOut onBoardLEDs(LED1, LED2, LED3);
BusOut ledRGB(PB_13, PB_14, PB_15);

// Function headers
void buttonISR(void);
void printData(int mode);

InterruptIn userButton(USER_BUTTON_PIN);

Thread i2cSensorsThread;
static Thread gpsThread;

EventFlags event_flags;
static enum modes { TEST = 0, NORMAL = 1, ADVANCED = 2 } mode;

// Acc
extern Queue<float[3], 1> accQueue;
float (*accMeasured)[3];

// RGB
extern Queue<int[5], 1> rgbQueue;
int (*rgbMeasured)[5];

// Temp
extern Queue<float[2], 1> tempHumidityQueue;
static float (*measureValuesTempHumidity)[2];

// Analog
extern Queue<float[2], 1> lightAndSoilQueue;
static float (*measureValuesLightAndSoil)[2];

// GPS
extern Queue<GPSData, 1> gpsQueue;
GPSData *receivedData;

std::string mostRepeatedColour;

int main(void) {
  printf("Start\r\n");
  gpsThread.start(gpsThreadFunction);
  ledRGB.write(0);  // Apagar LEDs RGB
  onBoardLEDs.write(0);
  userButton.fall(&buttonISR);

  // Iniciar el hilo de sensores
  i2cSensorsThread.start(i2cSensorsThreadFunction);

  constexpr int INIT_WAIT_TIME = 4;  // Segundos
  printf("Waiting for sensors to initialize (%d seconds)...\r\n",
         INIT_WAIT_TIME);
  ThisThread::sleep_for(INIT_WAIT_TIME * 1s);

  // Mensaje final después del tiempo de espera
  printf("Initialization wait complete. Starting main loop.\r\n");

  // Variable para indicar si los sensores están listos
  bool sensorsReady = false;

  // FSM
  while (true) {
    switch (mode) {
      case TEST:
        onBoardLEDs.write(1);  // LED1 encendido
        event_flags.set(FLAG_TEST);

        // Solo intentar leer si los sensores ya están listos
        if (!sensorsReady) {
          sensorsReady = true;  // Se establece la bandera después de la espera
          continue;  // No intentar leer sensores durante la inicialización
        }

        if (!accQueue.try_get(&accMeasured)) {
          printf("Can't read from acc\n");
        }
        if (!tempHumidityQueue.try_get(&measureValuesTempHumidity)) {
          printf("Can't read temp sensor\n");
        }
        if (!rgbQueue.try_get(&rgbMeasured)) {
          printf("Can't read rgb sensor\n");
        }
        if (!lightAndSoilQueue.try_get(&measureValuesLightAndSoil)) {
          printf("Can't read soil sensor\n");
        }

        if (!gpsQueue.try_get(&receivedData)) {
          printf("Can't read GPS\n");
        }

        ledRGB.write((*rgbMeasured)[4]);  // Color RGB LED según sensor

        printData(TEST);
        ThisThread::sleep_for(TEST_MODE_SYNC_RATE);
        break;

      case NORMAL:
        onBoardLEDs.write(2);  // LED2 encendido
        event_flags.set(FLAG_NORMAL);
        printData(NORMAL);
        ThisThread::sleep_for(NORMAL_MODE_SYNC_RATE);
        break;

      case ADVANCED:
        onBoardLEDs.write(4);  // LED3 encendido
        event_flags.set(FLAG_ADVANCED);
        printData(ADVANCED);
        ThisThread::sleep_for(ADVANCED_MODE_SYNC_RATE);
        break;
    }
  }
  return 0;
}

// T->N->A->T ...
void buttonISR() {
  if (mode == TEST) {
    mode = NORMAL;
  } else if (mode == NORMAL) {
    mode = ADVANCED;
  } else {
    mode = TEST;
  }
}

void printData(int mode) {
  switch (mode) {
    case TEST:
      if (accMeasured) {
        printf("\r\n-------> Accelerometer values:\n");
        printf("  X: %.2f m/s^2\r\n", (*accMeasured)[0]);
        printf("  Y: %.2f m/s^2\r\n", (*accMeasured)[1]);
        printf("  Z: %.2f m/s^2\r\n", (*accMeasured)[2]);
      }

      if (rgbMeasured) {
        printf("\r\n------> RGB values:\n");
        printf("  Clear:    %d\r\n", (*rgbMeasured)[0]);
        printf("  Red:      %d\r\n", (*rgbMeasured)[1]);
        printf("  Green:    %d\r\n", (*rgbMeasured)[2]);
        printf("  Blue:     %d\r\n", (*rgbMeasured)[3]);
        mostRepeatedColour = dominantColour_str(
              (*rgbMeasured)[0], (*rgbMeasured)[1], (*rgbMeasured)[2]);
        printf(" Dominant colour: %s\r\n", mostRepeatedColour.c_str());
      }

      if (measureValuesTempHumidity) {
        printf("\r\n------> Temperature and Humidity:\r\n");
        printf("  Temperature: %.2f C\r\n", (*measureValuesTempHumidity)[0]);
        printf("  Humidity:    %.2f %%\r\n", (*measureValuesTempHumidity)[1]);
      }

      if (measureValuesLightAndSoil) {
        printf("\r\n------> Light and Soil Moisture\r\n");
        printf("  Light percentage: %.2f %%\r\n",
               (*measureValuesLightAndSoil)[0]);
        printf("  Soil percentage:  %.2f %%\r\n",
               (*measureValuesLightAndSoil)[1]);
      }

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
      break;

    case NORMAL:;
      ;  // print is done inside sensors.cpp for normal mode
      break;

    case ADVANCED:
      printf("=== ADVANCED MODE ===\r\n");
      break;
  }
}