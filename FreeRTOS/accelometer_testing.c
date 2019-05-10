#include <stdio.h>
#include <math.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"
#include "bmp280/bmp280.h"

#define G_CONST 	16384
#define G_TO_MS		9.84
#define MAX_IDLE	10
#define MAX_HISTORY	10

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

typedef struct vect_int {
	int x;
	int y;
	int z;
} int_vector;

typedef enum {
	BMP280_TEMPERATURE, BMP280_PRESSURE
} bmp280_quantity;

typedef enum {
	SMPLRT_DIV = 0x19,
	FIFO_EN = 0x23,
	FIFO_COUNT = 0x72,
	FIFO_R_W = 0x74,
	MPU9250_SELF_TEST_X_ACCEL = 0x0d,
	MPU9250_SELF_TEST_Y_ACCEL = 0x0e,
	MPU9250_SELF_TEST_Z_ACCEL = 0x0f,
	ACCEL_CONFIG = 0x1c,
	ACCEL_CONFIG2 = 0x1d,
	MPU9250_ACCEL_X = 0x3b,
	MPU9250_ACCEL_Y = 0x3d,
	MPU9250_ACCEL_Z = 0x3f,
	MPU9250_TEMP = 0x41,
	MPU9250_GYRO_X = 0x43,
	MPU9250_GYRO_Y = 0x45,
	MPU9250_GYRO_Z = 0x47,
	USER_CTRL = 0x6a,
	XA_OFFSET_H = 0x77,
	YA_OFFSET_H = 0x7a,
	ZA_OFFSET_H = 0x7d
} mpu9250_quantity;

typedef enum {
	SMPLR_DIV_1KH = 0x00,
	USER_ENABLE_FIFO = 0x40,
	USER_RESET_FIFO = 0x0C,
	FIFO_DISABLE = 0x00,
	FIFO_ENABLE_ACC = 0x08,
	FIFO_ENABLE_GYR = 0x70,
	FIFO_ENABLE_AG = 0x78
} fifo_control;

// just reference, for now
typedef enum {
  AFS_2G = 0x00,
  AFS_4G = 0x01,
  AFS_8G = 0x02,
  AFS_16G = 0x03
} Ascale;

enum {
	SSI_UPTIME, SSI_FREE_HEAP, SSI_LED_STATE
};

void calibrate_raw();

bmp280_t bmp280_dev;

vector acceleration;
vector acceleration_old;
vector speed;
vector position;

vector cal;
vector idle_acc[MAX_IDLE];
vector acc_hist[MAX_HISTORY];

uint8_t Ares = 0;

uint8_vector self_test;

int16_vector accel_raw;

uint8_t history_idx = 0;
uint8_t unchanged_counter = 0;

TaskHandle_t task1_handle, task2_handle;

void reset_vector(vector *vect) {
	vect->x = 0.0f; vect->y = 0.0f; vect->z = 0.0f;
}

void reset_int16_vector(int16_vector *vect) {
	vect->x = 0; vect->y = 0; vect->z = 0;
}

void reset_int_vector(int_vector *vect) {
	vect->x = 0; vect->y = 0; vect->z = 0;
}

// write byte to PCF on I2C bus
void write_byte_pcf(uint8_t data) {
	i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
}

void write_byte_mpu(uint8_t data, mpu9250_quantity quantity) {
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_write(BUS_I2C, MPU_ADDRESS, &register_address, &data, 1);
}

// read byte from PCF on I2C bus
uint8_t read_byte_pcf() {
	uint8_t data;

	i2c_slave_read(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
	return data;
}

void print_vector(int16_vector *vector) {
	printf("%d %d %d\n", vector->x, vector->y, vector->z);
}

void print_vector_int(int_vector *vector) {
	printf("%d %d %d\n", vector->x, vector->y, vector->z);
}

// read 2 bytes from MPU-9250 on I2C bus
int16_t read_bytes_mpu(mpu9250_quantity quantity) {

	// high and low byte of quantity
	uint8_t data_high, data_low;
	uint8_t register_address = (uint8_t) quantity;

	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_high, 1);
	register_address++;
	i2c_slave_read(BUS_I2C, MPU_ADDRESS, &register_address, &data_low, 1);

	return (int16_t) ((int16_t)(data_high << 8) | data_low);
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

// converts the accelometer output to m/s^2
float convert_to_accel(int16_t output) {
	return output * G_TO_MS / G_CONST;
}

void process_accel_mpu(vector *vect_acc, vector *vect_old) {

	// Set the sensitivity of our movement - if we don't breach this value, then don't change acceleration
	float threshold = 50.0f;	// [m/s^2]
	uint8_t pcf_byte;

	vector new_accel;
	reset_vector(&new_accel);

	int h;
	for(h = 0; h < MAX_HISTORY; h++) {
		new_accel.x += acc_hist[h].x;
		new_accel.y += acc_hist[h].y;
		new_accel.z += acc_hist[h].z;
	}

	new_accel.x /= MAX_HISTORY;
	new_accel.y /= MAX_HISTORY;
	new_accel.z /= MAX_HISTORY;

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
}

// check MPU-9250 sensor values
void mpu_task(void *pvParameters) {

	while (1) {
		
		acc_hist[history_idx].x = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_X));
		acc_hist[history_idx].y = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Y));
		acc_hist[history_idx].z = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Z));

		history_idx = (history_idx + 1) % MAX_HISTORY;

		process_accel_mpu(&acceleration, &acceleration_old);
		
		vector speed_new;

		// new speed is calculated with a trapezoid formula for integration
		speed_new.x = speed.x + ((acceleration.x + acceleration_old.x)/2 * 0.01);
		speed_new.y = speed.y + ((acceleration.y + acceleration_old.y)/2 * 0.01);
		speed_new.z = speed.z + ((acceleration.z + acceleration_old.z)/2 * 0.01);

		// position is a linear formula for integration
		position.x = position.x + ((speed.x + speed_new.x)/2 * 0.01);
		position.y = position.y + ((speed.y + speed_new.y)/2 * 0.01);
		position.z = position.z + ((speed.z + speed_new.z)/2 * 0.01);

		speed = speed_new;

		// printf("Acceleration\n");
		// printf("X: %f, Y: %f, Z: %f\n",
		// 	acceleration.x, acceleration.y, acceleration.z);

		// printf("Speed\n");
		// printf("X: %f, Y: %f, Z: %f\n",
		// 	speed.x, speed.y, speed.z);

		// printf("Self test\n");
		// printf("X: %d, Y: %d, Z: %d\n",
		// 	self_test.x, self_test.y, self_test.z);
		printf("Position\n");
		printf("X: %f, Y: %f, Z: %f\n",
			position.x, position.y, position.z);

		// check again after 10 ms
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void calibrate_raw() {

	uint8_t pcf_byte;

	int16_vector curr_accel;

	int16_vector min_accel;
	int16_vector max_accel;
	int_vector avg_accel;

	uint8_t CAL_MAX = 200;

	while(1) {

		// turn on the LED to let user know the calibration is going on
		pcf_byte = read_byte_pcf();
		write_byte_pcf((pcf_byte & led1) | clr_btn);

		reset_int_vector(&avg_accel);
		reset_int16_vector(&min_accel);
		reset_int16_vector(&max_accel);

		// insure user gets the memo before we start the calibration
		vTaskDelay(200);

		uint8_t counter = 0;
		for(counter = 0; counter < CAL_MAX; counter ++) {
			curr_accel.x = read_bytes_mpu(MPU9250_ACCEL_X);
			curr_accel.y = read_bytes_mpu(MPU9250_ACCEL_Y);
			curr_accel.z = read_bytes_mpu(MPU9250_ACCEL_Z);

			if(counter == 0) {
				min_accel.x = curr_accel.x;
				min_accel.y = curr_accel.y;
				min_accel.z = curr_accel.z;

				max_accel.x = curr_accel.x;
				max_accel.y = curr_accel.y;
				max_accel.z = curr_accel.z;
			} else {
				if (curr_accel.x < min_accel.x) {
					min_accel.x = curr_accel.x;
				} else if (max_accel.x < curr_accel.x) {
					max_accel.x = curr_accel.x;
				}

				if (curr_accel.y < min_accel.y) {
					min_accel.y = curr_accel.y;
				} else if (max_accel.y < curr_accel.y) {
					max_accel.y = curr_accel.y;
				}

				if (curr_accel.z < min_accel.z) {
					min_accel.z = curr_accel.z;
				} else if (max_accel.z < curr_accel.z) {
					max_accel.z = curr_accel.z;
				}
			}

			avg_accel.x = avg_accel.x + curr_accel.x;
			avg_accel.y = avg_accel.y + curr_accel.y;
			avg_accel.z = avg_accel.z + curr_accel.z;
		}

		avg_accel.x = avg_accel.x / CAL_MAX;
		avg_accel.y = avg_accel.y / CAL_MAX;
		avg_accel.z = avg_accel.z / CAL_MAX;

		printf("AVG:\n");
		print_vector_int(&avg_accel);
		printf("MAX:\n");
		print_vector(&max_accel);
		printf("MIN:\n");
		print_vector(&min_accel);

		// turn off the LED to let user know he's free to do as he pleases
		pcf_byte = read_byte_pcf();
		write_byte_pcf(pcf_byte | leds_off);
		
		// give user 3 sec to do random movement
		vTaskDelay(pdMS_TO_TICKS(4000));
	}
}

// check for pressed buttons
void pcf_task(void *pvParameters) {

	uint8_t pcf_byte;

	// turn off all leds
	write_byte_pcf(leds_off);

	while (1) {

		pcf_byte = read_byte_pcf();

		// button 1 is pressed
		if ((pcf_byte & button1) == 0) {
			// clear buttons states and toggle led 1
			//write_byte_pcf((pcf_byte & led1) | clr_btn);

			// TODO: run the calibration for accelometer
			if(!task2_handle)
				xTaskCreate(calibrate_raw, "Calibration task", 1000, NULL, 1, &task2_handle);
			else
				vTaskResume(task2_handle);

			// button 2 is pressed
		} else if ((pcf_byte & button2) == 0) {
			// clear buttons states and turn on led 2
			//write_byte_pcf((pcf_byte & led2) | clr_btn);
			
			// prevent suspending yourself
			if(task2_handle)
				vTaskSuspend(task2_handle);

			printf("=================\n");
			printf("SELF-test params:\n");
			printf("%d %d %d\n",
					self_test.x, self_test.y, self_test.z);
			printf("=================\n");

			// button 3 is pressed
		} else if ((pcf_byte & button3) == 0) {
			// clear buttons states and turn off led 3

			//write_byte_pcf((pcf_byte & led3) | clr_btn);

			// prevent suspending yourself
			if(task2_handle)
				vTaskSuspend(task2_handle);

			printf("Changing res to %d\n", (++Ares % 4));

			pcf_byte = read_byte_mpu(ACCEL_CONFIG);
			pcf_byte = pcf_byte & ~0x18; // clear bytes for accel scale
			pcf_byte = pcf_byte | Ares << 3;
			write_byte_mpu(pcf_byte, ACCEL_CONFIG);

			if(task2_handle)
				vTaskResume(task2_handle);

			// button 4 is pressed
		} else if ((pcf_byte & button4) == 0) {
			// blink led 4
			//write_byte_pcf(pcf_byte & led4);

			// read offset values - just for fun
			int16_vector offsetsA;
			offsetsA.x = read_bytes_mpu(XA_OFFSET_H);
			offsetsA.y = read_bytes_mpu(YA_OFFSET_H);
			offsetsA.z = read_bytes_mpu(ZA_OFFSET_H);

			printf("=================\n");
			printf("Offset params on-board:\n");
			printf("%d %d %d\n",
					offsetsA.x, offsetsA.y, offsetsA.z);
			printf("=================\n");
		}

		// check again after 200 ms
		vTaskDelay(pdMS_TO_TICKS(200));
	}
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

	// prefill history
	int h;
	for(h = 0; h<MAX_HISTORY; h++) {
		acc_hist[h].x = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_X));
		acc_hist[h].y = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Y));
		acc_hist[h].z = convert_to_accel(read_bytes_mpu(MPU9250_ACCEL_Z));
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

	init_vectors();

	// create pcf task
	//xTaskCreate(pcf_task, "PCF task", 1000, NULL, 2, NULL);

	// create mpu task
	xTaskCreate(mpu_task, "MPU-9250 task", 1000, NULL, 2, &task1_handle);

	vTaskDelay(pdMS_TO_TICKS(100));
}