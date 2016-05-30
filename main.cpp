#include "main.h"

extern HardwareSerial hwdSerial;
extern MPU6050 mpu;
extern HMC5883L mag;

#define DEBUG

//Complementary Filter
#define ALPHA_MAX 0.92f
#define ALPHA_MIN (1.0f - ALPHA_MAX)

#define MPU6050

//SYSTEM VARIABLES
bool systemReady = false;
bool armed = false;
bool rxIsStable = true;
byte arming = 0;


//TIMING
uint32_t lastSamplePeriod = 0;
float loopTime = 0;

//PROTOCOL
bool continueSending = true;

bool rfIsStable = false;
uint32_t rfLastTime1 = 0;
uint32_t rfLastTime2 = 0;

//MPU6050
double ypr[3];   //Yaw, Pitch, Roll   
uint8_t gyroAddress = 0x6B;

//RX
int channel[RC_CHANNELS];

volatile uint16_t lastChannel1, lastChannel2, lastChannel3, lastChannel4, lastChannel5, lastChannel6;
volatile uint16_t receiverInputChannel1 = RC_HALF;
volatile uint16_t receiverInputChannel2 = RC_HALF;
volatile uint16_t receiverInputChannel3 = RC_MIN;
volatile uint16_t receiverInputChannel4 = RC_HALF;
volatile uint16_t receiverInputChannel5 = RC_HALF;
volatile uint16_t receiverInputChannel6 = RC_HALF;
volatile uint32_t timerChannel1, timerChannel2, timerChannel3, timerChannel4, timerChannel5, timerChannel6;

volatile uint32_t now;


//PID
double P_x;
double P_y;
double P_z;

double I_x;
double I_y;
double I_z;

double D_x;
double D_y;
double D_z;

double rot_x; 
double rot_y;

int radRotate;
int radTiltTB;
int radTiltLR;

double xAdder;
double yAdder;

//IMU
int16_t ax, ay, az, gx, gy, gz;
int16_t mx, my, mz;

float accX;
float accY;
float accZ;
float wantedAccZ;

float gyroX;
float gyroY;
float gyroZ;

float deltaTime = 0;
float lastDeltaTime = 0;

//MOTORS
int vFrontLeft, vFrontRight;
int vBackLeft, vBackRight;


float ACC_X_OFFSET = 0;
float ACC_Y_OFFSET = 0;
float ACC_Z_OFFSET = 0;

float GYRO_X_OFFSET = 0;
float GYRO_Y_OFFSET = 0;
float GYRO_Z_OFFSET = 0;

long imuLastSampleTime = micros();
long imuLastComputeTime = 0;



char command[100];
char commandId[10];
byte commandIndex;

//FUNCTION DECLARATION
bool initHS();
bool initRX();
bool initIMU();
bool initPWM();
void computeRC();
void computeIMU();
void computePID();
void computeMotors();
void printData();
void checkForArmESC();
void armESC();
void disarmESC();
void calculateFlatPosition();
void checkForCalculateFlatPosition();
void processCommand(char* command);
void readSerial();

int main()
{
	init();

	systemReady = initHS();
	systemReady &= initRX();
	systemReady &= initIMU();
	systemReady &= initPWM();

	calculateFlatPosition();

	if (systemReady)
	{
		hwdSerial.println("System Ready.");
	}
	else
	{
		hwdSerial.println("System Error.");
		delay(100);
		return 1;
	}

	//GET SAMPLE PERIOD
	static uint32_t lastSample10ms = 0;
	static uint32_t lastSample20ms = 0;
	static uint32_t lastSample50ms = 0;
	static uint32_t lastSample100ms = 0;
	static uint32_t lastSampleAux = 0;

	for (;;)
	{
		uint32_t now = lastSamplePeriod = micros();

		//CHECK IF STICKS ARE ARMING OR DISARMING QUAD
		checkForArmESC();

		//CHECK IF STICK ASK FOR FLAT POSITION
		checkForCalculateFlatPosition();

		//READ COMMANDS
		readSerial();

		//COMPUTE IMU
		computeIMU();

		//COMPUTE RC
		computeRC();

		//COMPUTE PID
		computePID();

		//COMPUTE MOTORS
		computeMotors();

		//TASKS IN 50MS
		if (now - lastSample100ms > 100000)
		{
			//PRINT DATA
			printData();

			lastSample50ms = now;
		}
	}

	return 0;
}

bool initHS()
{
	hwdSerial.begin(57600);
	hwdSerial.flush();

	delay(50);

	return true;
}

bool initRX()
{
	pinMode(4, INPUT);   // set Pin as Input (default)
	digitalWrite(4, HIGH);  // enable pullup resistor
	pinMode(5, INPUT);   // set Pin as Input (default)
	digitalWrite(5, HIGH);  // enable pullup resistor
	pinMode(6, INPUT);   // set Pin as Input (default)
	digitalWrite(6, HIGH);  // enable pullup resistor
	pinMode(7, INPUT);   // set Pin as Input (default)
	digitalWrite(7, HIGH);  // enable pullup resistor
	pinMode(8, INPUT);   // set Pin as Input (default)
	digitalWrite(8, HIGH);  // enable pullup resistor
	pinMode(9, INPUT);   // set Pin as Input (default)
	digitalWrite(9, HIGH);  // enable pullup resistor

	//Enable INTs for RX
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT20);
	PCMSK2 |= (1 << PCINT21);
	PCMSK2 |= (1 << PCINT22);
	PCMSK2 |= (1 << PCINT23);

	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT0);
	PCMSK0 |= (1 << PCINT1);

	return true;
}

bool initIMU()
{
	I2C::begin();

	delay(50);

	//MPU6050
	mpu.initialize(&hwdSerial);

	mpu.setI2CMasterModeEnabled(false);
	mpu.setI2CBypassEnabled(true);

	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
	mpu.setSleepEnabled(false);
	
	//HMC5883L
	mag.initialize();

	bool baroReady = true; // baro.testConnection();
	bool mpuReady = mpu.testConnection();
	bool magReady = mag.testConnection();

	if (!mpuReady || !magReady)
	{
		hwdSerial.println("IMU connection error");
		delay(50);

		return false;
	}

	hwdSerial.println("IMU connection successful");
	delay(50);

	return true;
}

bool initPWM()
{
	bool calibrate = false;

	PWMT1::init(ESC_NUMBER);

	hwdSerial.println("PWM Ready");
	delay(50);

	if (!calibrate)
	{
		PWMT1::attach(ESC_FL_PIN, ESC_FL, ESC_MIN);
		PWMT1::attach(ESC_FR_PIN, ESC_FR, ESC_MIN);
		PWMT1::attach(ESC_BR_PIN, ESC_BR, ESC_MIN);
		PWMT1::attach(ESC_BL_PIN, ESC_BL, ESC_MIN);
	}

	if (calibrate) //calibrate
	{
		hwdSerial.println("Calibrating ESC.");
		delay(50);

		PWMT1::attach(ESC_FL_PIN, ESC_FL, ESC_MAX);
		PWMT1::attach(ESC_FR_PIN, ESC_FR, ESC_MAX);
		PWMT1::attach(ESC_BR_PIN, ESC_BR, ESC_MAX);
		PWMT1::attach(ESC_BL_PIN, ESC_BL, ESC_MAX);


		hwdSerial.println("ESC PINs with MAX VALUE. Power Up ESCs in next 5 secs!");

		digitalWrite(13, HIGH);
		delay(2000);

		hwdSerial.println("ESC PINs with MIN VALUE.");
		digitalWrite(13, LOW);

		PWMT1::setChannel(ESC_FL, ESC_MIN);
		PWMT1::setChannel(ESC_FR, ESC_MIN);
		PWMT1::setChannel(ESC_BR, ESC_MIN);
		PWMT1::setChannel(ESC_BL, ESC_MIN);
		
		delay(3000);
	}

	hwdSerial.println("ESC calibration done.");

	hwdSerial.println("Testing motors. Order must be FrontLeft, FrontRight, BackRight, BackLeft.");
	delay(250);

	//TEST MOTORS
	PWMT1::setChannel(ESC_FL, ESC_MIN + 80);
	delay(250); 
	PWMT1::setChannel(ESC_FR, ESC_MIN + 80);
	delay(250);
	PWMT1::setChannel(ESC_BR, ESC_MIN + 80);
	delay(250);
	PWMT1::setChannel(ESC_BL, ESC_MIN + 80);
	delay(2000);
	return true;
}

void readSerial()
{
	char charRecv;
	while (hwdSerial.available())
	{
		charRecv = hwdSerial.read();

		if (charRecv == '\n')
		{
			// Termino el string
			command[commandIndex] = '\0';
			processCommand(command);

			//Inicializo el string
			commandIndex = 0;
			command[0] = '\0';
		}
		else
		{
			command[commandIndex] = charRecv;
			commandIndex++;
		}
	}
}

void processCommand(char* command)
{
	//SPEED 0 700
	char* commandSplit = strtok(command, " ");
	char* param1;
	char *param2;

	while (commandSplit != 0)
	{

		if (strcmp(commandSplit, "SPEED") == 0)
		{
			commandSplit = strtok(0, " ");
			param1 = commandSplit;
			//commandSplit = strtok(0, " ");
			//param2 = commandSplit;

			//motors.motorSpeed[0] = atoi(param2);
			//motors.motorSpeed[1] = atoi(param2);
			//motors.motorSpeed[2] = atoi(param2);


			receiverInputChannel3 = atoi(param1);
			hwdSerial.print("SPEED\t");
			hwdSerial.println(receiverInputChannel3);
			
			//motors.setMotorSpeed(atoi(param1), atoi(param2));
		}
		else if (strcmp(commandSplit, "BT") == 0)
		{
			commandSplit = strtok(0, " ");
			param1 = commandSplit;
			//writeBluetooth(param1);
		}

		commandSplit = strtok(0, " ");
	}
}

void computeIMU()
{
	static long startTime = 0;
	static long rightNow = 0;

	wantedAccZ = wantedAccZ + radRotate * deltaTime * 2;
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	accX = ax - ACC_X_OFFSET;
	accY = ay - ACC_Y_OFFSET;
	accZ = az - ACC_Z_OFFSET;

	//Gyro Raw to Deg/Sec
	gyroX = (-gx - GYRO_X_OFFSET) / 65.5;
	gyroY = (-gy - GYRO_Y_OFFSET) / 65.5;
	gyroZ = (-gz - GYRO_Z_OFFSET) / 65.5;

	//hwdSerial.print(accX);
	//hwdSerial.print("\t");
	//hwdSerial.print(accY);
	//hwdSerial.print("\t");
	//hwdSerial.print(accZ);
	//hwdSerial.print("\t");

	//hwdSerial.print(gyroX);
	//hwdSerial.print("\t");
	//hwdSerial.print(gyroY);
	//hwdSerial.print("\t");
	//hwdSerial.print(gyroZ);
	//hwdSerial.print("\t\t");

	//Get angles from accelerometer in degrees
	accX = (atan2(accX, sqrt(sq(accY) + sq(accZ))) + PI) * RAD_TO_DEG - 180;
	accY = (atan2(accY, sqrt(sq(accX) + sq(accZ))) + PI) * RAD_TO_DEG - 180;

	//hwdSerial.print(accX);
	//hwdSerial.print("\t");
	//hwdSerial.print(accY);
	//hwdSerial.print("\t\t");

	deltaTime = ((float)micros() - lastDeltaTime) / 1000000;

	// Calculate angles using Complementary Filter (Accel & Gyro)
	ypr[0] = ypr[0] + gyroZ * deltaTime;
	ypr[1] = ALPHA_MAX * (ypr[1] + (gyroX * deltaTime)) + (ALPHA_MIN * accX);
	ypr[2] = ALPHA_MAX * (ypr[2] + (gyroY * deltaTime)) + (ALPHA_MIN * accY);

	lastDeltaTime = micros();

	//hwdSerial.print(ypr[0]);
	//hwdSerial.print("\t");
	//hwdSerial.print(ypr[1]);
	//hwdSerial.print("\t");
	//hwdSerial.print(ypr[2]);
	//hwdSerial.println("\t");


}

void computeRC()
{
	//channel[THRO] = 1310;
	//channel[YAW] = 1500;
	//channel[PITCH] = 1500;
	//channel[ROLL] = 1500;

	//RX

	rxIsStable = !((receiverInputChannel1 < RC_MIN || receiverInputChannel1 > RC_MAX) ||
		(receiverInputChannel2 < RC_MIN || receiverInputChannel2 > RC_MAX) ||
		(receiverInputChannel3 < RC_MIN || receiverInputChannel3 > RC_MAX) ||
		(receiverInputChannel4 < RC_MIN || receiverInputChannel4 > RC_MAX));


	//If lost signal of the radio, set THRO to 0 and sticks to center.
	if (!rxIsStable)
	{
		channel[THRO] = RC_MIN;
		channel[YAW] = RC_HALF;
		channel[PITCH] = RC_HALF;
		channel[ROLL] = RC_HALF;
	}
	else
	{
		channel[THRO] = receiverInputChannel3;
		channel[YAW] = receiverInputChannel4;
		channel[PITCH] = receiverInputChannel2;
		channel[ROLL] = receiverInputChannel1;
		channel[AUX1] = receiverInputChannel5;
		channel[AUX2] = receiverInputChannel6;
	}

	//RX	
	channel[THRO] = (receiverInputChannel3 < RC_MIN) ? RC_MIN : receiverInputChannel3;
	channel[YAW] = (receiverInputChannel4 <  RC_MIN) ? RC_MIN : receiverInputChannel4;
	channel[PITCH] = (receiverInputChannel2 < RC_MIN) ? RC_MIN : receiverInputChannel2;
	channel[ROLL] = (receiverInputChannel1 < RC_MIN) ? RC_MIN : receiverInputChannel1;

	channel[THRO] = (receiverInputChannel3 > RC_MAX) ? RC_MAX : receiverInputChannel3;
	channel[YAW] = (receiverInputChannel4 > RC_MAX) ? RC_MAX : receiverInputChannel4;
	channel[PITCH] = (receiverInputChannel2 > RC_MAX) ? RC_MAX : receiverInputChannel2;
	channel[ROLL] = (receiverInputChannel1 > RC_MAX) ? RC_MAX : receiverInputChannel1;

	//Compute sticks yaw (rotate), pitch (TiltTopBack), roll (TiltLeftRight)
	radRotate = map((float)channel[YAW], (float)RC_MIN, (float)RC_MAX, (float)-YAW_PID_INFLUENCE, (float)YAW_PID_INFLUENCE);
	radTiltTB = map((float)channel[PITCH], (float)RC_MIN, (float)RC_MAX, (float)-PITCH_PID_INFLUENCE, (float)PITCH_PID_INFLUENCE);
	radTiltLR = map((float)channel[ROLL], (float)RC_MIN, (float)RC_MAX, (float)-ROLL_PID_INFLUENCE, (float)ROLL_PID_INFLUENCE);
}

void computePID()
{
	//P Value
	P_x = (ypr[2] + radTiltLR) * 2.4;
	P_y = (ypr[1] + radTiltTB) * 2.4;

	//I Value
	I_x = I_x + (ypr[2] + radTiltLR) * deltaTime * 3.7;
	I_y = I_y + (ypr[1] + radTiltTB) * deltaTime * 3.7;

	//D Value
	D_x = gyroX * 0.7;
	D_y = gyroY * 0.7;

	//YAW 
	P_z = (ypr[0] + wantedAccZ) * 2.0;
	I_z = I_z + (ypr[0] + wantedAccZ) * deltaTime * 0.8;
	D_z = gyroZ * 0.3;


	if (P_z > YAW_PID_INFLUENCE) P_z = YAW_PID_INFLUENCE;
	if (P_z < -YAW_PID_INFLUENCE) P_z = -YAW_PID_INFLUENCE;

	//Stop I;
	if (I_x > PID_I_LIMIT) I_x = PID_I_LIMIT;
	if (I_x < -PID_I_LIMIT) I_x = -PID_I_LIMIT;

	if (I_y > PID_I_LIMIT) I_y = PID_I_LIMIT;
	if (I_y < -PID_I_LIMIT) I_y = -PID_I_LIMIT;
	
	if (I_z > PID_I_LIMIT) I_z = PID_I_LIMIT;
	if (I_z < -PID_I_LIMIT) I_z = -PID_I_LIMIT;

	xAdder = P_x + I_x + D_x;
	yAdder = P_y + I_y + D_y;
}

void computeMotors()
{
	vFrontLeft = channel[THRO] + P_z + I_z + D_z + xAdder + yAdder;
	vFrontRight = channel[THRO] - P_z - I_z - D_z - yAdder + xAdder;
	vBackLeft = channel[THRO] + P_z + I_z + D_z - xAdder - yAdder;
	vBackRight = channel[THRO] - P_z - I_z - D_z + yAdder - xAdder;

	if (vFrontLeft < ESC_MIN) vFrontLeft = ESC_MIN;
	if (vFrontLeft > ESC_MAX) vFrontLeft = ESC_MAX;

	if (vFrontRight < ESC_MIN) vFrontRight = ESC_MIN;
	if (vFrontRight > ESC_MAX) vFrontRight = ESC_MAX;

	if (vBackLeft < ESC_MIN) vBackLeft = ESC_MIN;
	if (vBackLeft > ESC_MAX) vBackLeft = ESC_MAX;

	if (vBackRight < ESC_MIN) vBackRight = ESC_MIN;
	if (vBackRight > ESC_MAX) vBackRight = ESC_MAX;


	PWMT1::setChannel(ESC_FL, vFrontLeft);
	PWMT1::setChannel(ESC_FR, vFrontRight);
	PWMT1::setChannel(ESC_BR, vBackRight);
	PWMT1::setChannel(ESC_BL, vBackLeft);
}

void printData()
{
	//hwdSerial.print(armed);
	//hwdSerial.print("\t\t");

	//hwdSerial.print(arming);
	//hwdSerial.print("\t\t");

	hwdSerial.print(ypr[0]);
	hwdSerial.print("\t");
	hwdSerial.print(ypr[1]);
	hwdSerial.print("\t");
	hwdSerial.print(ypr[2]);
	hwdSerial.print("\t\t");

	//hwdSerial.print(imuLastComputeTime);
	//hwdSerial.print("\t\t");

	//hwdSerial.print(receiverInputChannel1);
	//hwdSerial.print("\t");
	//hwdSerial.print(receiverInputChannel2);
	//hwdSerial.print("\t");
	//hwdSerial.print(receiverInputChannel3);
	//hwdSerial.print("\t");
	//hwdSerial.print(receiverInputChannel4);
	//hwdSerial.print("\t\t");

	hwdSerial.print(channel[THRO]);
	hwdSerial.print("\t");
	hwdSerial.print(channel[YAW]);
	hwdSerial.print("\t");
	hwdSerial.print(channel[PITCH]);
	hwdSerial.print("\t");
	hwdSerial.print(channel[ROLL]);
	hwdSerial.print("\t");
	hwdSerial.print(channel[AUX1]);
	hwdSerial.print("\t");
	hwdSerial.print(channel[AUX2]);
	hwdSerial.print("\t\t");

	hwdSerial.print(radRotate);
	hwdSerial.print("\t");
	hwdSerial.print(radTiltTB);
	hwdSerial.print("\t");
	hwdSerial.print(radTiltLR);
	hwdSerial.print("\t\t");

	hwdSerial.print(vFrontLeft);
	hwdSerial.print("\t");
	hwdSerial.print(vFrontRight);
	hwdSerial.print("\t");
	hwdSerial.print(vBackLeft);
	hwdSerial.print("\t");
	hwdSerial.print(vBackRight);
	hwdSerial.print("\t\t");

	//hwdSerial.print(rxIsStable);
	//hwdSerial.print("\t\t");

	hwdSerial.println(loopTime);
}

void checkForArmESC()
{
	static bool lastCheck = false;

	if (channel[THRO] < ESC_MIN + 100 && channel[YAW] < ESC_MIN + 100)
	{
		if (!armed && lastCheck)
		{
			//NO estaba armado y pido armarlo
			armESC();

			lastCheck = false;
			return;
		}
		else
		{
			lastCheck = true;
		}
	}


	if (channel[THRO] < ESC_MIN + 100 && channel[YAW] > ESC_MAX - 100)
	{
		if (armed && lastCheck)
		{
			//Estaba armado y pido desarmarlo
			disarmESC();
			lastCheck = false;
			return;
		}
		else
		{
			lastCheck = true;
		}
	}
}

void checkForCalculateFlatPosition()
{
	static bool lastCheck = false;

	if (channel[AUX1] > ESC_MAX - 50)
	{
		if (lastCheck)
		{
			//NO estaba armado y pido armarlo
			calculateFlatPosition();
			lastCheck = false;
			return;
		}
		else
		{
			lastCheck = true;
		}
	}
}

void armESC()
{
	PWMT1::attach(ESC_FL_PIN, ESC_FL, ESC_MIN);
	PWMT1::attach(ESC_FR_PIN, ESC_FR, ESC_MIN);
	PWMT1::attach(ESC_BR_PIN, ESC_BR, ESC_MIN);
	PWMT1::attach(ESC_BL_PIN, ESC_BL, ESC_MIN);
	armed = true;
}

void disarmESC()
{
	PWMT1::detach(ESC_FL);
	PWMT1::detach(ESC_FR);
	PWMT1::detach(ESC_BR);
	PWMT1::detach(ESC_BL);
	armed = false;
}

void calculateFlatPosition()
{
	int numSamples = 20;
	long axSum = 0, aySum = 0, azSum = 0;
	long gxSum = 0, gySum = 0, gzSum = 0;

	for (int i = 0; i < numSamples; i++)
	{
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		
		axSum += ax;
		aySum += ay;
		azSum += az;

		gxSum += gx;
		gySum += gy;
		gzSum += gz;
	}

	ACC_X_OFFSET = axSum / numSamples;
	ACC_Y_OFFSET = aySum / numSamples;
	//ACC_Z_OFFSET = azSum / numSamples;

	GYRO_X_OFFSET = gxSum / numSamples;
	GYRO_Y_OFFSET = gySum / numSamples;
	GYRO_Z_OFFSET = gzSum / numSamples;

	hwdSerial.print(ACC_X_OFFSET);
	hwdSerial.print("\t");
	hwdSerial.print(ACC_Y_OFFSET);
	hwdSerial.print("\t");
	hwdSerial.print(ACC_Z_OFFSET);
	hwdSerial.println("\t");

	hwdSerial.print(GYRO_X_OFFSET);
	hwdSerial.print("\t");
	hwdSerial.print(GYRO_Y_OFFSET);
	hwdSerial.print("\t");
	hwdSerial.print(GYRO_Z_OFFSET);
	hwdSerial.println("\t");


}

ISR(TWI_vect)
{
	I2C::signalISR();
}

ISR(TIMER1_COMPA_vect)
{
	noInterrupts();
	PWMT1::signalISR();
	interrupts();
}

ISR(PCINT2_vect)
{
	noInterrupts();

	now = micros();

	//PIN4
	if (PIND & (1 << PD4))											
	{                                       						
		if (lastChannel1 == 0)										
		{
			lastChannel1 = 1;
			timerChannel1 = now;
		}
	}
	else if (lastChannel1 == 1)
	{
		lastChannel1 = 0;
		receiverInputChannel1 = now - timerChannel1;
	}

	//PIN5
	if (PIND & (1 << PD5))											
	{                                      							
		if (lastChannel2 == 0)										
		{
			lastChannel2 = 1;
			timerChannel2 = now;
		}
	}
	else if (lastChannel2 == 1)
	{
		lastChannel2 = 0;
		receiverInputChannel2 = now - timerChannel2;
	}

	//PIN 6
	if (PIND & (1 << PD6)) 											
	{                                       						
		if (lastChannel3 == 0)										
		{
			lastChannel3 = 1;
			timerChannel3 = now;
		}
	}
	else if (lastChannel3 == 1)
	{
		lastChannel3 = 0;
		receiverInputChannel3 = now - timerChannel3;
	}

	//PIN7
	if (PIND & (1 << PD7))
	{																
		if (lastChannel4 == 0)										
		{															
			lastChannel4 = 1;
			timerChannel4 = now;
		}
	}
	else if (lastChannel4 == 1)
	{
		lastChannel4 = 0;
		receiverInputChannel4 = now - timerChannel4;
	}

	interrupts();
}

ISR(PCINT0_vect)
{
	noInterrupts();

	now = micros();

	//PIN8
	if (PINB & (1 << PB0))
	{																
		if (lastChannel5 == 0)										
		{															
			lastChannel5 = 1;
			timerChannel5 = now;
		}
	}
	else if (lastChannel5 == 1)
	{
		lastChannel5 = 0;
		receiverInputChannel5 = now - timerChannel5;
	}
	//PIN9
	if (PINB & (1 << PB1))
	{																
		if (lastChannel6 == 0)										
		{															
			lastChannel6 = 1;
			timerChannel6 = now;
		}
	}
	else if (lastChannel6 == 1)
	{
		lastChannel6 = 0;
		receiverInputChannel6 = now - timerChannel6;
	}

	interrupts();
}