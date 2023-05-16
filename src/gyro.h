/*
MIT License

CopyRight (c) 2023 Charlie Wu.
Copyright (c) 2021 Haoran Wang.
Copyright (c) 2020 Steffen S.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <mbed.h>

// Register addresses
#define WHO_AM_I 0x0F // device identification register

#define CTRL_REG_1 0x20 // control register 1
#define CTRL_REG_2 0x21 // control register 2
#define CTRL_REG_3 0x22 // control register 3
#define CTRL_REG_4 0x23 // control register 4
#define CTRL_REG_5 0x24 // control register 5

#define STATUS_REG 0x27 // status register

#define OUT_X_L 0x28 // X-axis angular rate data Low
#define OUT_X_H 0x29 // X-axis angular rate data high
#define OUT_Y_L 0x2A // Y-axis angular rate data low
#define OUT_Y_H 0x2B // Y-axis angular rate data high
#define OUT_Z_L 0x2C // Z-axis angular rate data low
#define OUT_Z_H 0x2D // Z-axis angular rate data high

#define FIFO_CTRL_REG 0x2E // FIFO control register
#define FIFO_SRC_REG 0x2F  // FIFO status control register

#define INT1_CFG 0x30 // interrupt 1 configuration register
#define INT1_SRC 0x31 // interrupt 1 source register
#define INT1_TSH_XH 0x32 // interrupt 1 threshold X register high
#define INT1_TSH_XL 0x33 // interrupt 1 threshold X register low
#define INT1_TSH_YH 0x34 // interrupt 1 threshold Y register high
#define INT1_TSH_YL 0x35 // interrupt 1 threshold Y register low
#define INT1_TSH_ZH 0x36 // interrupt 1 threshold Z register high
#define INT1_TSH_ZL 0x37 // interrupt 1 threshold Z register low
#define INT1_DURATION 0x38 // interrupt 1 duration register

// Output data rate selections and cutoff frequencies
#define ODR_100_CUTOFF_12_5 0x00
#define ODR_100_CUTOFF_25 0x10
#define ODR_200_CUTOFF_12_5 0x40
#define ODR_200_CUTOFF_25 0x50
#define ODR_200_CUTOFF_50 0x60
#define ODR_200_CUTOFF_70 0x70
#define ODR_400_CUTOFF_20 0x80
#define ODR_400_CUTOFF_25 0x90
#define ODR_400_CUTOFF_50 0xa0
#define ODR_400_CUTOFF_110 0xb0
#define ODR_800_CUTOFF_30 0xc0
#define ODR_800_CUTOFF_35 0xd0
#define ODR_800_CUTOFF_50 0xe0
#define ODR_800_CUTOFF_110 0xf0

// High pass filter selections (high pass filter mode disabled)
#define ODR_100_HIGH_PASS_8 0x00
#define ODR_200_HIGH_PASS_15 0x00
#define ODR_400_HIGH_PASS_30 0x00
#define ODR_800_HIGH_PASS_56 0x00

// Interrupt configurations
#define INT1_ENB 0x80 // Interrupt enable on the INT1 pin
#define INT1_BOOT 0x40 // Boot status available on INT1 pin
#define INT1_ACT 0x20 // Interrupt active configuration on INT1 pin
#define INT1_OPEN 0x10 // INT1 pin configuration
#define INT1_LATCH 0x02 // Latch interrupt request on INT1_SRC register
#define INT1_ZHIE 0x20 // Enable interrupt generation on Z high event
#define INT1_ZLIE 0x10 // Enable interrupt generation on Z low event
#define INT1_YHIE 0x08 // Enable interrupt generation on Y high event
#define INT1_YLIE 0x04 // Enable interrupt generation on Y low event
#define INT1_XHIE 0x02 // Enable interrupt generation on X high event
#define INT1_XLIE 0x01 // Enable interrupt generation on X low event
#define INT2_DRDY 0x08 // Data ready on DRDY/INT2 pin

// Fullscale selections
#define FULL_SCALE_245 0x00      // full scale 245 dps
#define FULL_SCALE_500 0x10      // full scale 500 dps
#define FULL_SCALE_2000 0x20     // full scale 2000 dps
#define FULL_SCALE_2000_ALT 0x30 // full scale 2000 dps

// Sensitivities in dps/digit
#define SENSITIVITY_245 0.00875f // 245 dps typical sensitivity
#define SENSITIVITY_500 0.0175f  // 500 dps typical sensitivity
#define SENSITIVITY_2000 0.07f   // 2000 dps typical sensitivity

// Convert constants
#define MY_LEG 1              // put board on left leg 0.8m above ground
#define DEGREE_TO_RAD 0.0175f // rad = dgree * (pi / 180)

#define POWERON 0x0f  // turn gyroscope
#define POWEROFF 0x00 // turnoff gyroscope

#define SAMPLE_TIME_20 20
#define SAMPLE_INTERVAL_0_05 0.005f

// Initialization parameters
typedef struct
{
    uint8_t conf1;       // output data rate
    uint8_t conf3;       // interrupt configuration
    uint8_t conf4;       // full sacle selection
} Gyroscope_Init_Parameters;

// Raw data
typedef struct
{
    int16_t x_raw; // X-axis raw data
    int16_t y_raw; // Y-axis raw data
    int16_t z_raw; // Z-axis raw data
} Gyroscope_RawData;

// Calibrated data
typedef struct
{
    int16_t x_calibrated; // X-axis calibrated data
    int16_t y_calibrated; // Y-axis calibrated data
    int16_t z_calibrated; // Z-axis calibrated data
} Gyroscope_CalibratedData;

// Write IO
void WriteByte(uint8_t address, uint8_t data);

// Read IO
void GetGyroValue(Gyroscope_RawData *rawdata);

// Gyroscope calibration
void CalibrateGyroscope(Gyroscope_RawData *rawdata);

// Gyroscope initialization
void InitiateGyroscope(Gyroscope_Init_Parameters *init_parameters, Gyroscope_RawData *init_raw_data);

// Data conversion: raw -> dps
float ConvertToDPS(int16_t rawdata);

// Data conversion: dps -> m/s
float ConvertToVelocity(int16_t rawdata);

// Calculate distance from raw data array;
float GetDistance(int16_t arr[]);

// Get calibrated data
void GetCalibratedRawData();

// Turn off the gyroscope
void PowerOff();