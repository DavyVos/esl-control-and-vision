#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include "jiwy_controller.h"

#include "soc_system.h"

int main(int argc, char **argv)
{
	int fd = 0; // file descriptor

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0)
	{
		perror("Couldn't open /dev/mem\n");
		return -1;
	}
	uint32_t *base      = NULL;
	uint32_t *pitch_enc = NULL;
	uint32_t *yaw_enc   = NULL;
	if (base == MAP_FAILED)
	{
		perror("Couldn't map bridge.");
		close(fd);
		return -1;
	}
	// Avalon maps a contiguous memory area
	// Pan = Yaw
	// Pitch = Tilt
    uint32_t *encoderTilt = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_ENCODER_BUS_0_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_ENCODER_BUS_0_BASE);
    uint32_t *encoderPan  = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_ENCODER_BUS_1_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_ENCODER_BUS_1_BASE);
    uint32_t *pwmTilt     = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_PWMBUS_0_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_PWMBUS_0_BASE);
    uint32_t *pwmPan      = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_PWMBUS_1_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_PWMBUS_1_BASE);

	Jiwy jiwy;
	Jiwy_Init(&jiwy, encoderPan, encoderTilt, pwmPan, pwmTilt);

	Jiwy_CalibrateTilt(&jiwy);
	Jiwy_CalibratePan(&jiwy);

	//SetTargetPan(&jiwy, 0.5);
	//SetTargetTilt(&jiwy, 0.5);

	uint8_t enable = 0;
	uint8_t dir = 0;
	uint8_t duty = 90;

	while (1)
	{
		uint32_t pwmPanValue  = *((uint32_t *)pwmPan);
		uint32_t pwmTiltValue = *((uint32_t *)pwmTilt);
		uint32_t panValue     = *((uint32_t *)encoderPan);
		uint32_t tiltValue    = *((uint32_t *)encoderTilt);
		printf("PWM(Tilt: %u, pan: %u), Encoder(Tilt: %u, Pan: %u)\n", pwmTiltValue, pwmPanValue, tiltValue, panValue);
		//Update();
		usleep(100);
	}

	close(fd);
	return 0;
}
