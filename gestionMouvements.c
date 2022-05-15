/*
 * gestionsMouvements.c
 *
 *  Created on: 29 avr. 2022
 *      Author: Ludo et Marius
 */

#include "gestionMouvements.h"

#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include "ch.h"
#include <audio_processing.h>
#include "selector.h"
#include <stdbool.h>

#define THREADSIZE 256
#define INITIAL_SPEED 100
#define SPEED_CHOREE 1000
#define ZERO 0
#define SELECTOR 5
#define DISTANCE_SEUIL 55
#define TEMPS_NONANTE_DEGRES_INITIAL_SPEED 3270
#define CENTMS 100

//gestion des tempos
#define SPEED_OFF 0
#define TEMPO_ROULER 472
#define ATTENTE_DEMARRER 3000
#define TEMPS_QUARANTECINQ_DEGRES 165
#define TEMPS_TOURS_DEGRES  2970
#define TEMPS_CENTQUATREVINGT_DEGRES 670
#define TEMPS_NONANTE_DEGRES_CHOREE_SPEED 340
#define TEMPO_PAUSE_NONANTE 200
#define TEMPO_PAUSE_QUARANTECINQ 290

static bool stop_audio = false;
static BSEMAPHORE_DECL(Stop_Audio, FALSE); //sem pour eviter d'utiliser les ressources de processaudio pour rien

static THD_WORKING_AREA(waMouvementsEtModes, THREADSIZE);
static THD_FUNCTION(MouvementsEtModes, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
			uint16_t distance = ZERO;

	       int selecteur = get_selector();

	      //selection du mode choregraphie ou mode Cirque
	     if(selecteur == SELECTOR){
	    	 stop_audio = true;
	    	laChoreeDeReggaeton();

	     }else{
		     chBSemSignal(&Stop_Audio); //signal pour reutiliser process audio
		     stop_audio = false;
	    	 distance = VL53L0X_get_dist_mm(); //receive distance in mm from the Thread of the ToF
	    	 float speed = get_speed_coeff();
	    	 bool direction = get_direction(); //receive direction, going left or right, from the Thread of audio processing

	        if ((distance < DISTANCE_SEUIL) && speed!=ZERO){

	        	if (direction) {
	        	 right_motor_set_speed(-INITIAL_SPEED); //turn right of 90deg
	        	 left_motor_set_speed(INITIAL_SPEED);
	        	} else {
	        	 right_motor_set_speed(INITIAL_SPEED); //turn left of 90deg
	        	 left_motor_set_speed(-INITIAL_SPEED);
	        	}
	        	 chThdSleepMilliseconds(TEMPS_NONANTE_DEGRES_INITIAL_SPEED);
	        } else {
	        	left_motor_set_speed(INITIAL_SPEED*speed); //going forward with speed depending on the frequency (100step/s by default)
	        	right_motor_set_speed(INITIAL_SPEED*speed);
	        }
	     }
    }

}

void startDetecteur(void){ //declaration du thread
	chThdCreateStatic(waMouvementsEtModes, sizeof(waMouvementsEtModes), NORMALPRIO, MouvementsEtModes, NULL);
	}


void laChoreeDeReggaeton(void){ //toute la choree
	chThdSleepMilliseconds(ATTENTE_DEMARRER);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);
	chThdSleepMilliseconds(TEMPS_QUARANTECINQ_DEGRES);   //toure de 45 degres

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(TEMPO_ROULER);
	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE); //derriere
	chThdSleepMilliseconds(TEMPO_ROULER);

	right_motor_set_speed(SPEED_OFF);
	left_motor_set_speed(SPEED_OFF);		//tempo pause
	chThdSleepMilliseconds(TEMPO_PAUSE_NONANTE);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE); //tour de 90 degres
	chThdSleepMilliseconds(TEMPS_NONANTE_DEGRES_CHOREE_SPEED);

	right_motor_set_speed(SPEED_OFF);
	left_motor_set_speed(SPEED_OFF);
	chThdSleepMilliseconds(TEMPO_PAUSE_NONANTE);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(TEMPO_ROULER);
	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE);  //derriere
	chThdSleepMilliseconds(TEMPO_ROULER);

	right_motor_set_speed(SPEED_OFF);
	left_motor_set_speed(SPEED_OFF);
	chThdSleepMilliseconds(TEMPO_PAUSE_QUARANTECINQ);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);
	chThdSleepMilliseconds(TEMPS_QUARANTECINQ_DEGRES);   //toure de 45 degres

	right_motor_set_speed(SPEED_OFF);
	left_motor_set_speed(SPEED_OFF);		//tempo pause
	chThdSleepMilliseconds(TEMPO_PAUSE_QUARANTECINQ);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(TEMPO_ROULER);
	right_motor_set_speed(-SPEED_CHOREE); //derriere
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(TEMPO_ROULER);

	right_motor_set_speed(SPEED_OFF);
	left_motor_set_speed(SPEED_OFF);
	chThdSleepMilliseconds(TEMPO_PAUSE_NONANTE);

	right_motor_set_speed(-SPEED_CHOREE); //derriere
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(TEMPO_ROULER);
	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(TEMPO_ROULER);

	right_motor_set_speed(SPEED_OFF);
	left_motor_set_speed(SPEED_OFF);		//tempo pause
	chThdSleepMilliseconds(TEMPO_PAUSE_NONANTE);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);  //nonante
	chThdSleepMilliseconds(TEMPS_NONANTE_DEGRES_CHOREE_SPEED);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(TEMPO_ROULER);
	right_motor_set_speed(-SPEED_CHOREE); //derriere
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(TEMPO_ROULER);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(-SPEED_CHOREE); //tour de 180 degres
	chThdSleepMilliseconds(TEMPS_CENTQUATREVINGT_DEGRES);

	right_motor_set_speed(SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE); //devant
	chThdSleepMilliseconds(TEMPO_ROULER);
	right_motor_set_speed(-SPEED_CHOREE); //derriere
	left_motor_set_speed(-SPEED_CHOREE);
	chThdSleepMilliseconds(TEMPO_ROULER);

	right_motor_set_speed(-SPEED_CHOREE);
	left_motor_set_speed(SPEED_CHOREE);  //tour et quart
	chThdSleepMilliseconds(TEMPS_TOURS_DEGRES);

	right_motor_set_speed(SPEED_OFF);
	left_motor_set_speed(SPEED_OFF);

	chThdSleepMilliseconds(ATTENTE_DEMARRER);
}

void get_StopAudioSem(void){
	chBSemWait(&Stop_Audio);
}

_Bool get_stopAudio(void){
	return stop_audio;
}
