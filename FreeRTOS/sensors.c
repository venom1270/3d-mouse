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
#define G_TO_MS		984
#define MAX_IDLE	20

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

vector new_accel;
vector acceleration;
vector acceleration_old;
vector speed;
vector position;

vector gyro;

vector cal;
vector idle_acc[MAX_IDLE];

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
	INT_ENABLE = 0x38,
	INT_STATUS = 0x3a,
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

	return (data_high << 8) + data_low;
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

void process_accel_mpu(vector *vect_acc, vector *vect_old) {

	// Set the sensitivity of our movement - if we don't breach this value, then don't change acceleration
	float threshold = 50.0f;	// [m/s^2]
	uint8_t pcf_byte;

	// if acceleration doesn't change in 3 consecutive measures, reset it
	if (absf(new_accel.x - cal.x, vect_old->x) > threshold) {
		new_accel.x = new_accel.x - cal.x;
		// update both acc vectors x
		vect_acc->x = new_accel.x;
		vect_old->x = new_accel.x;

		// reset change counter
		unchanged_counter = 0;
	} else {
		idle_acc[unchanged_counter % MAX_IDLE].x = new_accel.x;
		vect_acc->x = 0.0f;
	}

	if (absf(new_accel.y - cal.y, vect_old->y) > threshold) {
		new_accel.y = new_accel.y - cal.y;
		// update both acc vectors y
		vect_acc->y = new_accel.y;
		vect_old->y = new_accel.y;

		// reset change counter
		unchanged_counter = 0;
	} else {
		idle_acc[unchanged_counter % MAX_IDLE].y = new_accel.y;
		vect_acc->y = 0.0f;
	}

	if (absf(new_accel.z - cal.z, vect_old->z) > threshold) {
		new_accel.z = new_accel.z - cal.z;
		// update both acc vectors
		vect_acc->z = new_accel.z;
		vect_old->z = new_accel.z;

		// reset change counter
		unchanged_counter = 0;
	} else {
		idle_acc[unchanged_counter % MAX_IDLE].z = new_accel.z;
		vect_acc->z = 0.0f;
	}

	if (unchanged_counter >= MAX_IDLE) {
		// in this case we assume tablet is not moving, so we got to reset the acceleration and speed to zero		
		reset_vector(vect_acc);
		reset_vector(vect_old);
		reset_vector(&speed);

		if(unchanged_counter == MAX_IDLE) {
			
			pcf_byte = read_byte_pcf();
			write_byte_pcf(pcf_byte & led2);
			// do calibration
			reset_vector(&cal);

			int i;
			for(i = 0; i < MAX_IDLE; i++) {
				cal.x += idle_acc[i].x;
				cal.y += idle_acc[i].y;
				cal.z += idle_acc[i].z;
			}

			cal.x /= MAX_IDLE;
			cal.y /= MAX_IDLE;
			cal.z /= MAX_IDLE;

			write_byte_pcf(pcf_byte | leds_off);

		}

	}

	unchanged_counter++;
}

// check MPU-9250 sensor values
void mpu_task(void *pvParameters) {

	// Runs about every 6ms with no delay

	double rotationX_delta = 0;
	double rotationY_delta = 0;
	double rotationZ_delta = 0;

	int16_t driftCorrectionX = 0;
	int16_t driftCorrectionY = 0;
	int16_t driftCorrectionZ = 0;

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

	uint32_t oldTime = xTaskGetTickCount();
	uint32_t deltaTime = 0;

	//double historyX[20];
	//uint8_t historyIndexX = 0;

	//double historyY[20];
	//uint8_t historyIndexY = 0;

	//vector history[20];
	//uint8_t historyIndex = 0;

	float accelZ = 0;

	while (1) {

		// GYRO_OUT = gyro_sensitivity * angular_rate
		// gyro_sensitivity = 131 LSB/ (stopinje/s)

		if(read_byte_mpu(INT_STATUS) & 0x01) {
			taskENTER_CRITICAL();

			deltaTime = xTaskGetTickCount() - oldTime;
			oldTime += deltaTime;

			// Read ACCEL and GYRO

			gyro.x = (int16_t) read_bytes_mpu(MPU9250_GYRO_X) / gyro_scaling;
			gyro.y = (int16_t) read_bytes_mpu(MPU9250_GYRO_Y) / gyro_scaling;
			gyro.z = (int16_t) read_bytes_mpu(MPU9250_GYRO_Z) / gyro_scaling;

			new_accel.x = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_X));
			new_accel.y = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Y));
			new_accel.z = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Z));

			accelZ = new_accel.z / 100.0;

			taskEXIT_CRITICAL();

			// Calculate 

			rotationX_delta = (gyro.x * (deltaTime / 1000.0)) - driftCorrectionX;
			rotationY_delta = (gyro.y * (deltaTime / 1000.0)) - driftCorrectionY;
			rotationZ_delta = (gyro.z * (deltaTime / 1000.0)) - driftCorrectionZ;

			// ACCEL start
			
			process_accel_mpu(&acceleration, &acceleration_old);

			// change of speed
			/*speed.x += acceleration.x * (deltaTime / 1000.0);
			speed.y += acceleration.y * (deltaTime / 1000.0);
			speed.z += acceleration.z * (deltaTime / 1000.0);

			// change of position - delta
			position.x += speed.x * (deltaTime / 1000.0);
			position.y += speed.y * (deltaTime / 1000.0);
			position.z += speed.z * (deltaTime / 1000.0);
			*/
			
			//printf("CONFIG: %d\n", read_bytes_mpu_config(MPU9250_GYRO_CONFIG) & 0b00011000);
			//printf("%d\n", xTaskGetTickCount());
			//printf("%f %f %f\n", acceleration.x, acceleration.y, acceleration.z);
			//printf("%f %f %f %f %f %f\n", rotationX_delta, rotationY_delta, rotationZ_delta, position.x, position.y, position.z);
			printf("%f %f %f %f %f %f %d %f\n", rotationX_delta, rotationY_delta, rotationZ_delta, acceleration.x, acceleration.y, acceleration.z, deltaTime, accelZ);
		}

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

	// enable data ready interupt
	write_bytes_mpu(INT_ENABLE, 0x01);

	// create pcf task
	xTaskCreate(pcf_task, "PCF task", 1000, NULL, 2, NULL);

	// create mpu task
	xTaskCreate(mpu_task, "MPU-9250 task", 1000, NULL, 2, NULL);
}

