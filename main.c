/*
 * main.c
 *
 * Copyright (C) 2018 Carnegie Mellon University
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Written by:
 * Anh Luong <anhluong@cmu.edu>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "BME680_driver/bme680.h"
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>

#define SPI_PATH 						"/dev/spidev1.0"

static uint32_t mode 		= 0;
static uint8_t bits 		= 8;
static uint32_t speed 		= 10000000; //10MHz
static uint16_t delay_us 	= 10;

static int fd;

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
	usleep(period * 1000);
}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */

	int status;
	uint8_t txbuf[len+1];
	uint8_t rxbuf[len+1];

	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)txbuf,
		.rx_buf = (unsigned long)rxbuf,
		.len = headerLength+readlength,
		.delay_usecs = delay_us,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	txbuf[0] = reg_addr;

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(status < 0)
		return -1;

	int j;
	for(j = 0; j < len; j++)
	{
		readBuffer[j] = rxbuf[j+1];
	}

    return rslt;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */

	int status;
	
	uint8_t txbuf[len+1];
	uint8_t rxbuf[len+1];

	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)txbuf,
		.rx_buf = (unsigned long)rxbuf,
		.len = headerLength+bodylength,
		.delay_usecs = delay_us,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	txbuf[0] = reg_addr;

	int j;
	for(j = 0; j < len; j++)
	{
		txbuf[j+1] = headerBuffer[j];
	}

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(status < 0)
		return -1;

    return rslt;
}

void bme680_config()
{
	uint8_t set_required_settings;

	/* Set the temperature, pressure and humidity settings */
	gas_sensor.tph_sett.os_hum = BME680_OS_2X;
	gas_sensor.tph_sett.os_pres = BME680_OS_4X;
	gas_sensor.tph_sett.os_temp = BME680_OS_8X;
	gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

	/* Set the remaining gas sensor settings and link the heating profile */
	gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	/* Create a ramp heat waveform in 3 steps */
	gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
	gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

	/* Select the power mode */
	/* Must be set before writing the sensor configuration */
	gas_sensor.power_mode = BME680_FORCED_MODE; 

	/* Set the required sensor settings needed */
	set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
		| BME680_GAS_SENSOR_SEL;
		
	/* Set the desired sensor configuration */
	rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);

	/* Set the power mode */
	rslt = bme680_set_sensor_mode(&gas_sensor);

	/* Get the total measurement duration so as to sleep or wait till the
	 * measurement is complete */
	uint16_t meas_period;
	bme680_get_profile_dur(&meas_period, &gas_sensor);
	user_delay_ms(meas_period); /* Delay till the measurement is ready */
}

/**
 * Application entry point.
 */
int main(void)
{
	// Init as SPI driver
	struct bme680_dev gas_sensor;

	/* You may assign a chip select identifier to be handled later */
	gas_sensor.dev_id = 0;
	gas_sensor.intf = BME680_SPI_INTF;
	gas_sensor.read = user_spi_read;
	gas_sensor.write = user_spi_write;
	gas_sensor.delay_ms = user_delay_ms;

	int8_t rslt = BME680_OK;
	rslt = bme680_init(&gas_sensor);

	// // Read sensor data
	// struct bme680_field_data data;
	
	// while(1) 
	// {
	// 	rslt = bme680_get_sensor_data(&data, &gas_sensor);

	// 	printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
	// 		data.pressure / 100.0f, data.humidity / 1000.0f );
	// 	/* Avoid using measurements from an unstable heating setup */
	// 	if(data.status & BME680_GASM_VALID_MSK)
	// 		printf(", G: %d ohms", data.gas_resistance);
		
	// 	printf("\r\n");
	// }

	return 0;

}