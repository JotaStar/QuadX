#ifndef _MAIN_H_
#define _MAIN_H_


#include <util/atomic.h>
#include "../../core/Arduino.h"
#include "../../core/HardwareSerial.h"
#include "../../core/libraries/HardwareI2C/HardwareI2C.h"
#include "../../core/libraries/I2C/I2C.h"
#include "../../core/libraries/SPI/SPI.h"
#include "../../core/libraries/RF22/RF22.h"
#include "../../core/libraries/RadioProtocol/RadioProtocol.h"
#include "../../core/libraries/MPU6050/MPU6050.h"
#include "../../core/libraries/HMC5883L/HMC5883L.h"
#include "../../core/libraries/PID/PID.h"
#include "../../core/libraries/PWMT1/PWMT1.h"
//#include "../../core/libraries/PWMT2/PWMT2.h"

HardwareSerial hwdSerial;
SPIClass hwdSPI;
RF22 rf22;
RadioProtocol rfProtocol;
MPU6050 mpu;
HMC5883L mag;

static uint32_t FAILSAFE_TIMEOUT_US = 300000;

//RC
#define RC_CHANNELS 6
#define RC_MIN 1080	
#define RC_MAX 1880
#define RC_HALF ((RC_MAX + RC_MIN) / 2)

//CHANNEL MAPPING
#define THRO 0		//3 Throttle
#define YAW 1		//4 Yaw
#define PITCH 2		//2 Pitch
#define ROLL 3		//1 Roll
#define AUX1 4		//5 Aux1
#define AUX2 5		//6 Aux2


//ESC CONFIGURATION
#define ESC_FL_PIN A0
#define ESC_FR_PIN A1
#define ESC_BR_PIN A2
#define ESC_BL_PIN A3

#define ESC_NUMBER 4
#define ESC_FL 0
#define ESC_FR 1
#define ESC_BR 2
#define ESC_BL 3

#define ESC_MIN 1080
#define ESC_MAX 1880
#define ESC_HALF ((ESC_MAX + ESC_MIN) / 2)
#define ESC_ARM_DELAY 1000



static short ORIENTATION = 1; // ALLOWED VALUES 1 OR -1


//PID
#define YAW_PID_INFLUENCE	180
#define PITCH_PID_INFLUENCE 50
#define ROLL_PID_INFLUENCE	50

#define PID_I_LIMIT 30

#define PITCH_P_VAL 4.0f
#define PITCH_I_VAL 0.0f
#define PITCH_D_VAL 0.0f

#define ROLL_P_VAL 4.0f
#define ROLL_I_VAL 0.0f
#define ROLL_D_VAL 0.0f

#define YAW_P_VAL 5.0f
#define YAW_I_VAL 0.0f
#define YAW_D_VAL 0.0f

#endif