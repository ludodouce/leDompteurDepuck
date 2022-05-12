/*
 * detecteurinfra.h
 *
 *  Created on: 29 avr. 2022
 *      Author: ludo
 */

#ifndef GESTIONMOUVEMENTS_H_
#define GESTIONMOUVEMENTS_H_

void startDetecteur(void); //start the thread for detectDistance
void laChoreeDeReggaeton(void); //fonction de la chor�e
void get_StopAudioSem(void);
_Bool get_stopAudio(void);


#endif /* GESTIONMOUVEMENTS_H_ */
