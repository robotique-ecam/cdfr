
/*
* This file is part of VL53L1 Platform
*
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

#include "vl53l1_platform.h"

/* Globals ------------------------------------------------------------------*/

#define VL53L1_MAX_I2C_XFER_SIZE 512

static uint8_t buffer[VL53L1_MAX_I2C_XFER_SIZE + 2];/* GLOBAL I2C comm buff */

static int i2c_hdl = -1;
static int st_tof_dev = -1;

static int8_t Linux_I2CRead(uint8_t *buff, uint8_t len)
{
	int ret;

	ret = read(i2c_hdl, buff, len);
	if (ret != len) {
		printf("Read failed with %d\n", ret);
		return -1;
	}
	return 0;
}

static int8_t Linux_I2CWrite(uint8_t *buff, uint8_t len)
{
	int ret;

	ret = write(i2c_hdl, buff, len);
	if (ret != len) {
		printf("Write failed with %d\n", ret);
		return -1;
	}
	return 0;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t RegisterAddress,
		uint8_t *pdata, uint32_t count)
{
	int8_t Status = 0;

	if ((count + 1) > VL53L1_MAX_I2C_XFER_SIZE)
		return -1;
	buffer[0] = RegisterAddress >> 8;
	buffer[1] = RegisterAddress & 0xFF;

	memcpy(&buffer[2], pdata, count);
	Status = Linux_I2CWrite(buffer, (count + 2));
	return Status;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t RegisterAddress,
		uint8_t *pdata, uint32_t count)
{
	int8_t Status = 0;

	if ((count + 1) > VL53L1_MAX_I2C_XFER_SIZE)
		return -1;

	buffer[0] = RegisterAddress >> 8;
	buffer[1] = RegisterAddress & 0xFF;

	Status = Linux_I2CWrite((uint8_t *) buffer, (uint8_t) 2);
	if (!Status) {
		pdata[0] = RegisterAddress;
		Status = Linux_I2CRead(pdata, count);
	}
	return Status;
}

int8_t VL53L1X_UltraLite_Linux_I2C_Init(uint16_t dev,
		int i2c_adapter_nr, uint8_t i2c_Addr)
{
	/* Open the I2C on Linux */
	char i2c_hdlname[20];
	(void)dev;

	printf("I2C Bus number is %d\n", i2c_adapter_nr);

	snprintf(i2c_hdlname, 19, "/dev/i2c-%d", i2c_adapter_nr);
	i2c_hdl = open(i2c_hdlname, O_RDWR);
	if (i2c_hdl < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		printf(" Open failed, returned value = %d, i2c_hdl_name = %s\n",
				i2c_hdl, i2c_hdlname);
		perror("Open bus ");
		return -1;
	}
	if (ioctl(i2c_hdl, I2C_SLAVE, i2c_Addr) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		return -1;
	}

	return 0;
}

int8_t VL53L1X_UltraLite_Linux_Interrupt_Init(void)
{
	char *st_tof_dev_name = "st_tof_dev";

	st_tof_dev = open("/dev/st_tof_dev", O_RDONLY);
	if (st_tof_dev == -1) {
		printf("Failed to open %s\n", st_tof_dev_name);
		return -1;
	}

	return 0;
}

int8_t VL53L1X_UltraLite_WaitForInterrupt(int IoctlWfiNumber)
{
	int Status = 0;

	Status = ioctl(st_tof_dev, IoctlWfiNumber);
	return (Status < 0 ? -1 : 0);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
	int8_t Status = 0;
	uint16_t my_data;

	Status = VL53L1_ReadMulti(dev, index, (uint8_t *) &my_data, 2);
	my_data = ntohs(my_data);
	*data = my_data;
	return Status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
	int8_t Status = 0;
	uint32_t my_data;

	Status = VL53L1_ReadMulti(dev, index, (uint8_t *) &my_data, 4);
	my_data = ntohl(my_data);
	*data = my_data;
	return Status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
	return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
	return VL53L1_WriteMulti(dev, index, (uint8_t *) &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
	data = htons(data);
	return VL53L1_WriteMulti(dev, index, (uint8_t *) &data, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
	data = htonl(data);
	return VL53L1_WriteMulti(dev, index, (uint8_t *) &data, 4);
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
	(void)dev;
	usleep(wait_ms * 1000);

	return 0;
}
