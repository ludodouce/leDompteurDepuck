/*
 * detecteurinfra.c
 *
 *  Created on: 29 avr. 2022
 *      Author: ludo
 */

#include "detecteurinfra.h"

#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include "ch.h"
#include <chprintf.h>
#include <audio_processing.h>
#include "selector.h"

#define INITIAL_SPEED 100
#define SPEED_CHOREE 1000


static THD_WORKING_AREA(waDetecteurDistance, 256);
static THD_FUNCTION(DetecteurDistance, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
			uint16_t distance = 0;

	       int selecteur = get_selector();

	      //selection du mode choregraphie ou mode Cirque
	     if(selecteur == 5){
	    	laChoreeDeReggaeton();
	     }

	     else{

	    	 distance = VL53L0X_get_dist_mm(); //receive distance in mm from the Thread of the ToF
	    	 float speed = get_speed_coeff();
	    	 bool direction = get_direction(); //receive direction, going left or right, from the Thread of audio processing

	        if ((distance < 55) && speed!=0){

	        	if (direction) {
	        	 right_motor_set_speed(-INITIAL_SPEED); //turn right of 90deg
	        	 left_motor_set_speed(INITIAL_SPEED);
	        	} else {
	        	 right_motor_set_speed(INITIAL_SPEED); //turn left of 90deg
	        	 left_motor_set_speed(-INITIAL_SPEED);
	        	}
	        	 chThdSleepMilliseconds(3270);
	        } else {
	        	left_motor_set_speed(INITIAL_SPEED*speed); //going forward with speed depending on the frequency (100step/s by default)
	        	right_motor_set_speed(INITIAL_SPEED*speed);
	        }
	     }
    }

	chThdSleepMilliseconds(100);
}

void startDetecteur(void){
	chThdCreateStatic(waDetecteurDistance, sizeof(waDetecteurDistance), NORMALPRIO, DetecteurDistance, NULL);
	}


void laChoreeDeReggaeton(void){
	chThdSleepMilliseconds(3000);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);
	chThdSleepMilliseconds(165);   //toure de 45 degr�s

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(1400);
	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE); //derri�re
	chThdSleepMilliseconds(1400);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE); //tour de 90 degr�s
	chThdSleepMilliseconds(340);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(1400);
	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE);  //derri�re
	chThdSleepMilliseconds(1400);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);
	chThdSleepMilliseconds(165);   //toure de 45 degr�s

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(1400);
	right_motor_set_speed(-SPEED_CHOREE); //derri�re
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(1400);
	right_motor_set_speed(-SPEED_CHOREE); //derri�re
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(1400);
	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(1400);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);  //tour et quart
	chThdSleepMilliseconds(1630);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(1400);
	right_motor_set_speed(-SPEED_CHOREE); //derri�re
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(1400);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE); //tour de 180 degr�s
	chThdSleepMilliseconds(670);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(1400);
	right_motor_set_speed(-SPEED_CHOREE); //derri�re
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(1400);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);  //tour et quart
	chThdSleepMilliseconds(2970);

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	chThdSleepMilliseconds(3000);

}
