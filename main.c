#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include "gestionMouvements.h"
#include"spi_comm.h"

#define CENTMILIS 100
#define SERIALCONFIG 115200,0,0,0,
#define TRUE 1

static void serial_start(void)
{
	//serial config pour la communication
	static SerialConfig ser_cfg = {
	   SERIALCONFIG
	};
//document changement
	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

	//initialisation systeme
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    spi_comm_start();

    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
    VL53L0X_start();	//init pour le decteur de distance
    startDetecteur();
    /* Infinite loop. */
    while (TRUE) {

        //waits until a result must be sent to the computer
    wait_send_to_computer();
    chThdSleepMilliseconds(CENTMILIS);
    }
}

//surveille la stack

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
