#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"

#include <time.h>
#include <math.h>

#define PCF_ADDRESS	0x38
#define MPU_ADDRESS	0x68
#define BUS_I2C		0
#define SCL 		14
#define SDA 		12

//					mask	returned value
#define button1		0x20	// 0b ??0? ????
#define button2		0x10	// 0b ???0 ????
#define button3		0x80	// 0b 0??? ????
#define button4		0x40	// 0b ?0?? ????
#define clr_btn		0xf0

#define led1 		0xfe	// 0b ???? ???0
#define led2 		0xfd	// 0b ???? ??0?
#define led3 		0xfb	// 0b ???? ?0??
#define led4 		0xf7	// 0b ???? 0???
#define leds_off	0xff

#define gpio_wemos_led	2

// 		defines for the accelometer
#define G_CONST 	16384
#define G_TO_MS		9.84
#define MAX_IDLE	5

typedef struct vect {
	float x;
	float y;
	float z;
} vector;

typedef struct vect_uint8 {
	uint8_t x;
	uint8_t y;
	uint8_t z;
} uint8_vector;

typedef struct vect_int16 {
	int16_t x;
	int16_t y;
	int16_t z;
} int16_vector;

vector acceleration;
vector acceleration_old;
vector speed;
vector position;

vector cal;

uint8_vector self_test;

// to count the steps until last time idle
uint8_t unchanged_counter = 0;

//	function to reset vector values back to 0
void reset_vector(vector *vect) {
	vect->x = 0.0f; vect->y = 0.0f; vect->z = 0.0f;
}

void reset_int16_vector(int16_vector *vect) {
	vect->x = 0; vect->y = 0; vect->z = 0;
}

typedef enum {
	MPU9250_SELF_TEST_X_ACCEL = 0x0d,
	MPU9250_SELF_TEST_Y_ACCEL = 0x0e,
	MPU9250_SELF_TEST_Z_ACCEL = 0x0f,
	MPU9250_ACCEL_X = 0x3b,
	MPU9250_ACCEL_Y = 0x3d,
	MPU9250_ACCEL_Z = 0x3f,
	MPU9250_TEMP = 0x41,
	MPU9250_GYRO_X = 0x43,
	MPU9250_GYRO_Y = 0x45,
	MPU9250_GYRO_Z = 0x47,
	MPU9250_GYRO_CONFIG = 0x1B
} mpu9250_quantity;

enum {
	SSI_UPTIME, SSI_FREE_HEAP, SSI_LED_STATE
};


// write byte to PCF on I2C bus
void write_byte_pcf(uint8_t data) {
	i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
}

// read byte from PCF on I2C bus
uint8_t read_byte_pcf() {
	uint8_t data;

	i2c_slave_read(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
	return data;
}

// check for pressed buttons
void pcf_task(void *pvParameters) {

	uint8_t pcf_byte;

	// turn off all leds
	write_byte_pcf(leds_off);

	while (1) {

		pcf_byte = read_byte_pcf();

		if ((pcf_byte & button3) == 0) {
			// clear buttons states and turn off led 3
			write_byte_pcf((pcf_byte | ~led3) | clr_btn);

			// button 4 is pressed
		} else if ((pcf_byte & button4) == 0) {
			// blink led 4
			write_byte_pcf(pcf_byte ^ ~led4);
		}

		// check again after 200 ms
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// read 2 bytes from MPU-9250 on I2C bus
int16_t read_bytes_mpu(mpu9250_quantity quantity) {

	// high and low byte of quantity
	uint8_t data_high, data_low;
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_high, 1);
	register_address++;
	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_low, 1);

	return (int16_t) (data_high << 8) + data_low;
}

uint8_t read_bytes_mpu_config(mpu9250_quantity quantity) {

	// high and low byte of quantity
	uint8_t data;
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data, 1);

	return data;
}

// read 1 byte from MPU-9250 on I2C bus
uint8_t read_byte_mpu(mpu9250_quantity quantity) {

	uint8_t data;
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data, 1);

	return data;
}

// get an absolute difference between x1 and x2
float absf(float x1, float x2) {
	if(x1 > x2)
		return x1 - x2;
	else
		return x2 - x1;	
}

void write_bytes_mpu(mpu9250_quantity quantity, uint8_t data) {

	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_write(BUS_I2C, MPU_ADDRESS, &register_address, &data, 1);
}

void init_vectors() {

	reset_vector(&acceleration);
	reset_vector(&acceleration_old);
	reset_vector(&speed);
	reset_vector(&position);
	reset_vector(&cal);

	// read the self_test values - unused
	self_test.x = read_byte_mpu(MPU9250_SELF_TEST_X_ACCEL);
	self_test.y = read_byte_mpu(MPU9250_SELF_TEST_Y_ACCEL);
	self_test.z = read_byte_mpu(MPU9250_SELF_TEST_Z_ACCEL);
}

// converts the accelometer output to m/s^2 - moved to unity
float convert_to_accel(int16_t output) {
	return (output * G_TO_MS) / G_CONST;
}

// Returns POSITIVE(!!) (index+mod)%3
int8_t index_mod(uint8_t index, int8_t mod) {
	uint8_t size = 3;	
	int8_t i = (index+mod)%size;
	if (i < 0) i += size;
	return i;
}


// check MPU-9250 sensor values
void mpu_task(void *pvParameters) {

	// Runs about every 3ms with no delay

	double rotationX = 0;
	double rotationX_delta = 0;
	double rotationY = 0;
	double rotationY_delta = 0;
	double rotationZ = 0;
	double rotationZ_delta = 0;

	//uint16_t taskDelay = 5;
	//uint16_t taskDelaySmoothing = 10; //used before for estimating deltaTime (taskDelay + smoothing) ≃ real delay (delta time)

	double deltaThreshold = 0.1;

	int16_t driftCorrectionX = 0;
	int16_t driftCorrectionY = 0;
	int16_t driftCorrectionZ = 0;

	uint16_t calibrationSteps = 0;

	// turn off Wemos led
	gpio_write(gpio_wemos_led, 1);

	// Set Gyro config
	/*
	FS_SEL=0 131 LSB/(º/s)
	FS_SEL=1 65.5 LSB/(º/s)
	FS_SEL=2 32.8 LSB/(º/s)
	FS_SEL=3 16.4 LSB/(º/s)
	*/
	uint8_t gyro_config = read_bytes_mpu_config(MPU9250_GYRO_CONFIG) & 0b11100111;
	//int8_t gyro_config = read_bytes_mpu_config(MPU9250_GYRO_CONFIG) | 0b00011000;
	write_bytes_mpu(MPU9250_GYRO_CONFIG, gyro_config);
	//double gyro_scaling = 16.4;
	double gyro_scaling = 131;

	double historyX[3] = {0.0, 0.0, 0.0};
	double historyY[3] = {0.0, 0.0, 0.0};
	double historyZ[3] = {0.0, 0.0, 0.0};
	uint8_t historyX_index = 0;
	uint8_t historyY_index = 0;
	uint8_t historyZ_index = 0;

	uint32_t oldTime = xTaskGetTickCount();
	uint32_t deltaTime = 0;

	while (1) {

		// GYRO_OUT = gyro_sensitivity * angular_rate
		// gyro_sensitivity = 131 LSB/ (stopinje/s)

		
		taskENTER_CRITICAL();

		deltaTime = xTaskGetTickCount() - oldTime;
		oldTime += deltaTime;

		int16_t gyroX = (int16_t) read_bytes_mpu(MPU9250_GYRO_X) / gyro_scaling;
		//rotationX_delta = (gyroX * ((taskDelay+taskDelaySmoothing) / 1000.0)) - driftCorrectionX;
		rotationX_delta = (gyroX * (deltaTime / 1000.0)) - driftCorrectionX;
		//historyX[historyX_index++] = rotationX_delta;
		//historyX_index %= 3;
		//if (!(historyX[index_mod(historyX_index, -2)] == 0.0 && historyX[historyX_index] == 0.0))
		//	rotationX += historyX[index_mod(historyX_index, -1)];		
		//if (fabs(rotationX_delta) >= deltaThreshold && !(historyX[index_mod(historyX_index, -2)] == 0.0 && historyX[historyX_index] == 0.0))
		//	rotationX += rotationX_delta;

		int16_t gyroY = (int16_t) read_bytes_mpu(MPU9250_GYRO_Y) / gyro_scaling;
		//rotationY_delta = (gyroY * ((taskDelay+taskDelaySmoothing) / 1000.0)) - driftCorrectionY;
		rotationY_delta = (gyroY * (deltaTime / 1000.0)) - driftCorrectionY;
		//historyY[historyY_index++] = rotationY_delta;
		//historyY_index %= 3;
		//if (!(historyY[index_mod(historyY_index, -2)] == 0.0 && historyY[historyY_index] == 0.0))
		//	rotationY += historyY[index_mod(historyY_index, -1)];
		//if (fabs(rotationY_delta) >= deltaThreshold && !(historyY[index_mod(historyY_index, -2)] == 0.0 && historyY[historyY_index] == 0.0))
		//	rotationY += rotationY_delta;
		
		int16_t gyroZ = (int16_t) read_bytes_mpu(MPU9250_GYRO_Z) / gyro_scaling;
		//rotationZ_delta = (gyroZ * ((taskDelay+taskDelaySmoothing) / 1000.0)) - driftCorrectionZ;
		rotationZ_delta = (gyroZ * (deltaTime / 1000.0)) - driftCorrectionZ;
		//historyZ[historyZ_index++] = rotationZ_delta;
		//historyZ_index %= 3;
		//if (!(historyZ[index_mod(historyZ_index, -2)] == 0.0 && historyZ[historyZ_index] == 0.0))			
		//	rotationZ += historyZ[index_mod(historyZ_index, -1)];
		//if (fabs(rotationZ_delta) >= deltaThreshold && !(historyZ[index_mod(historyZ_index, -2)] == 0.0 && historyZ[historyZ_index] == 0.0))
		//	rotationZ += rotationZ_delta;
		
		//printf("ABS TEST %f | %f\n", rotationZ_delta, fabs(rotationZ_delta));

		// ACCEL start

		acceleration.x = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_X));
		acceleration.y = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Y));
		acceleration.z = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Z));
		
		// ACCEL end

		taskEXIT_CRITICAL();

		// ACCEL start
		
		// change of speed
		speed.x = acceleration.x * (deltaTime / 1000.0);
		speed.y = acceleration.y * (deltaTime / 1000.0);
		speed.z = acceleration.z * (deltaTime / 1000.0);

		// change of position - delta
		position.x = speed.x * (deltaTime / 1000.0);
		position.y = speed.y * (deltaTime / 1000.0);
		position.z = speed.z * (deltaTime / 1000.0);

		// ACCEL end

		//printf("GYRO: %d\n", read_bytes_mpu(MPU9250_ACCEL_Z));
		//printf("GYRO X: %d\n", gyroX);
		//printf("GYRO Y: %d\n", gyroY);
		//printf("GYRO Z: %d\n", gyroZ);
		//printf("ROTATION X DELTA: %f\n", rotationX_delta);
		//printf("ROTATION Y DELTA: %f\n", rotationY_delta);
		//printf("ROTATION Z DELTA: %f\n", rotationZ_delta);
		//printf("ROTATION X: %f\n", rotationX);
		//printf("ROTATION Y: %f\n", rotationY);
		//printf("ROTATION Z: %f\n", rotationZ);
		//printf("CONFIG: %d\n", read_bytes_mpu_config(MPU9250_GYRO_CONFIG) & 0b00011000);
		//printf("Calibration Steps: %d\n", calibrationSteps);
		//printf("%d %d %d\n", index_mod(historyZ_index, -2), index_mod(historyZ_index, -1), historyZ_index);
		//printf("%d\n", xTaskGetTickCount());
		printf("%f %f %f %f %f %f\n", rotationX_delta, rotationY_delta, rotationZ_delta, position.x, position.y, position.z);
		//printf("%d\n", oldTime);
		//printf("*********\n");



		// CALIBRATION -- is this necessary??
		// GYRO Y has constant 0.06 when still
		/*if (calibrationSteps > 0) {
			calibrationSteps--;
			if (rotationX_delta != 0) {
				driftCorrectionX += rotationX_delta;
			}
			if (rotationY_delta != 0) {
				driftCorrectionY += rotationY_delta;
			}
			if (rotationZ_delta != 0) {
				driftCorrectionZ += rotationZ_delta;
			}
			gpio_write(gpio_wemos_led, 0);
			if (calibrationSteps == 0) {
				gpio_write(gpio_wemos_led, 1);
			}
		}*/		

		
		//vTaskDelay(pdMS_TO_TICKS(taskDelay));

	}
}

void user_init(void) {

	uart_set_baud(0, 115200);
	i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
	// fix i2c driver to work with MPU-9250
	gpio_enable(SCL, GPIO_OUTPUT);

	// turn off Wemos led
	gpio_enable(gpio_wemos_led, GPIO_OUTPUT);
	gpio_write(gpio_wemos_led, 1);

	// reset all usable vectors to 0
	init_vectors();

	// create pcf task
	xTaskCreate(pcf_task, "PCF task", 1000, NULL, 2, NULL);

	// create mpu task
	xTaskCreate(mpu_task, "MPU-9250 task", 1000, NULL, 2, NULL);
}

