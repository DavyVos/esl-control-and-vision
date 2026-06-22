#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include "jiwy_controller.h"

#include "soc_system.h"

int main()
{
	int fd = 0; // file descriptor

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0)
	{
		perror("Couldn't open /dev/mem\n");
		return -1;
	}

	// Avalon maps a contiguous memory area
	// Pan = Yaw
	// Pitch = Tilt
    uint32_t *encoderTilt = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_ENCODER_BUS_0_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_ENCODER_BUS_0_BASE);
    uint32_t *encoderPan  = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_ENCODER_BUS_1_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_ENCODER_BUS_1_BASE);
    uint32_t *pwmTilt     = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_PWMBUS_0_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_PWMBUS_0_BASE);
    uint32_t *pwmPan      = (uint32_t *)mmap(NULL, HPS_0_ARM_A9_0_PWMBUS_1_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HPS_0_ARM_A9_0_PWMBUS_1_BASE);

	if (encoderTilt == MAP_FAILED)
	{
		perror("Couldn't map bridge.");
		close(fd);
		return -1;
	}

	Jiwy jiwy;
	Jiwy_Init(&jiwy, encoderPan, encoderTilt, pwmPan, pwmTilt);

	Jiwy_CalibrateTilt(&jiwy);
	Jiwy_CalibratePan(&jiwy);

	jiwy.tilt_target = 0.5;
	jiwy.pan_target = 0.5;

	while (1)
	{
		uint32_t pwmPanValue  = *((uint32_t *)pwmPan); 
		uint32_t pwmTiltValue = *((uint32_t *)pwmTilt);
		printf("tilt_min: %i, tilt_max: %i, pan_min: %i, pan_max: %i Tilt pwm: %u, pan pwm: %u), Encoder(Tilt: %f, Pan: %f\n", 
			jiwy.tiltMin, jiwy.tiltMax, jiwy.panMin, jiwy.panMax, pwmTiltValue, pwmPanValue, Jiwy_getTilt(&jiwy), Jiwy_getPan(&jiwy));
		Jiwy_Update(&jiwy);
		usleep(10000);
	}

	close(fd);
	return 0;
}
