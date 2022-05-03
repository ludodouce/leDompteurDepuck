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

#define INITIAL_SPEED 100

static BSEMAPHORE_DECL(peutTourner, TRUE);

static THD_WORKING_AREA(waDetecteurDistance, 256);
static THD_FUNCTION(DetecteurDistance, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
			uint16_t distance = 0;
	        distance = VL53L0X_get_dist_mm();
	        float speed = get_speed_coeff();
	        if ((distance < 55) && speed!=0){
	        	 right_motor_set_speed(-INITIAL_SPEED);
	        	 left_motor_set_speed(INITIAL_SPEED);
	             //chprintf((BaseSequentialStream*)&SD3, "Hey! position = %d\n", valActuelle);
	        	 chThdSleepMilliseconds(3262);
	        } else {
	        	left_motor_set_speed(INITIAL_SPEED*speed);//attention utile pour debuguer mais � enlever quand on mixe les threads
	        	right_motor_set_speed(INITIAL_SPEED*speed);
	            //chprintf((BaseSequentialStream*)&SD3, "speed_coeff = %d \n", speed_coeff);
	        }
    }
	chThdSleepMilliseconds(100);
}

//static THD_WORKING_AREA(waTourne, 512);
//static THD_FUNCTION(Tourne, arg){
//	chRegSetThreadName(__FUNCTION__);
//	    (void)arg;
//
//	    while(1){
//	    	//chBSemSignal(&peutTourner);
//	     	chBSemWait(&peutTourner);
//	     	right_motor_set_speed(-100);
//	  }
//}

void startDetecteur(void){
	chThdCreateStatic(waDetecteurDistance, sizeof(waDetecteurDistance), NORMALPRIO, DetecteurDistance, NULL);
	//chThdCreateStatic(waTourne, sizeof(waTourne), NORMALPRIO, Tourne, NULL);
}
