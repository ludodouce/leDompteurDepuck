/*
 * detecteurinfra.c
 *
 *  Created on: 29 avr. 2022
 *      Author: ludo
 */

#include "detecteurinfra.h"

#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>

static BSEMAPHORE_DECL(peutTourner, TRUE);

static THD_WORKING_AREA(waDetecteurDistance, 128);
static THD_FUNCTION(DetecteurDistance, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
			uint16_t distance = 0;
	        distance = VL53L0X_get_dist_mm();

	        if (distance < 55){

	        	chBSemWait(&peutTourner);
	        	//chprintf((BaseSequentialStream*)&SD3, "Hey! position = %d\n", valActuelle);


	        } else {
	        	left_motor_set_speed(100);//attention utile pour debuguer mais à enlever quand on mixe les threads
	        	right_motor_set_speed(100);

	        //chprintf((BaseSequentialStream*)&SD3, "distance = %d \n", distance);
	        }
    }
	chThdSleepMilliseconds(100);
}

static THD_WORKING_AREA(waTourne, 128);
static THD_FUNCTION(Tourne, arg){
	chRegSetThreadName(__FUNCTION__);
	    (void)arg;

	    while(1){

	    	chBSemSignal(&peutTourner);

			uint16_t nombrePourFaireUnTour = 2000;
		    uint16_t valActuelle = 0;

		    while (nombrePourFaireUnTour >= valActuelle){
		    right_motor_set_speed(-100);
		    valActuelle+=1;

		}
	  }
}

void startDetecteur(void){
	chThdCreateStatic(waDetecteurDistance, sizeof(waDetecteurDistance), NORMALPRIO, DetecteurDistance, NULL);
	chThdCreateStatic(waTourne, sizeof(waTourne), NORMALPRIO+2, Tourne, NULL);
}
