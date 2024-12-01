// common.hpp
#pragma once

#include "mbed.h"
#include "stdlib.h" // for uint8_t etc
#include <numeric>
#include <string>

// LED RGB
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define WHITE   7
#define ALL_OFF 0

// RGB SENSOR
#define SCL            PB_8
#define SDA            PB_9
#define RGB_SENSOR_LED PH_0

// RGB LED
#define R_PIN PB_13
#define G_PIN PB_14
#define B_PIN PB_15

// Accelerometer
#define ACCEL_SDA  PB_14
#define ACCEL_SCL  PB_13

// SOIL/MOISTURE SENSOR
#define SOIL_MOISTURE_PIN PA_4
#define SOIL_POWER_PIN    PA_8

// User button
#define USER_BUTTON_PIN PB_2

// Mode sync rate
#define TEST_MODE_SYNC_RATE     2s
#define NORMAL_MODE_SYNC_RATE   100ms // change
#define ADVANCED_MODE_SYNC_RATE 2s

// Light sensor
#define LIGHT_SENSOR_PIN PA_0

//GPS
#define GPS_TX PA_9
#define GPS_RX PA_10


const uint32_t FLAG_TEST =        0x01;
const uint32_t FLAG_NORMAL =      0x02;
const uint32_t FLAG_ADVANCED =    0x04;