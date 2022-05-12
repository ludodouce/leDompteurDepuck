#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>
#include <detecteurinfra.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
static float speed_coeff;
static bool direction;
static int t;


#define MIN_VALUE_THRESHOLD	30000
#define IN_BETWEEN_FRONT 15000
#define IN_BETWEEN_BACK 20000
#define GREEN 0,100,0


#define MIN_FREQ		60	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	32	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	32	//406Hz
#define MAX_FREQ		80	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

#define SELECTORVALUE 5
#define LEDON 1
#define NORMINDEXMAX -1
#define ZERO 0
#define COEFFDEUX 2
#define COEFFSPEED 3
#define NORM_INDEX_MIN_LEFT 65
#define NORM_INDEX_MAX_LEFT 70
#define LEFT_VALUE_MIN 100000
#define SPEED_ON 1
#define SPEED_OFF 0
#define DEUXMILLEMILIS 2000
#define MICRO_NB 4
#define TAILLEPOUREVITERSURCHARGE 8

/* Function to turn on and off leds and rgb leds (LED1..LED8) depending on the direction
 * of the source of the sound by comparing the intensity perceived by M1 (back mic), M2 (left mic), M3 (right mic), M4 (front mic)
 */
void led_mon(float left, float right, float front, float back) {
	//fonction qui g�re la gestion de l'allumage des LEDs suivant la provenance du son
	//seulement si le mode chor�e n'est pas activ�e
	if ((right > left)){	//compare l'intensit�
		direction = true;	//choisit la direction o� tourner
			if (front > back) {	//compare l'intensit�
				if (abs(right-front)<IN_BETWEEN_FRONT){
					clear_leds();	//efface les LEDs deja allumee puis allume la led desiree
					set_rgb_led(LED2,GREEN);
				} else if (front > right) {
					clear_leds();
					set_led(LED1,LEDON);
				} else {
					clear_leds();
					set_led(LED3,LEDON);
				}
			} else {
				if ((abs(right-back)<IN_BETWEEN_BACK)){
					clear_leds();
					set_rgb_led(LED4,GREEN);
				} else if (back > right) {
					clear_leds();
					set_led(LED5,LEDON);
				} else {
					clear_leds();
					set_led(LED3,LEDON);
				}
			}
		} else if ((right < left)) {
			direction = false;		//retient de tourner dans l'autre direction
			if (front > back) {
						if (abs(front-left)<IN_BETWEEN_FRONT) {
							clear_leds();
							set_rgb_led(LED8,GREEN);
						} else if (front > left) {
							clear_leds();
							set_led(LED1,LEDON);
						} else {
							clear_leds();
							set_led(LED7,LEDON);
						}
					} else {
						if ((abs(left-back)<IN_BETWEEN_BACK)) {
							clear_leds();
							set_rgb_led(LED6,GREEN);
						} else if (back > left) {
							clear_leds();
							set_led(LED5,LEDON);
						} else {
							clear_leds();
							set_led(LED7,LEDON);
						}
					}
		} else {
			clear_leds();
		}
}

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void sound_remote(float* data_L, float* data_R, float* data_F, float* data_B){
	float max_norm_left = MIN_VALUE_THRESHOLD;
	float max_norm_front = MIN_VALUE_THRESHOLD;
	float max_norm_back = MIN_VALUE_THRESHOLD;
	float max_norm_right = MIN_VALUE_THRESHOLD;

	int16_t max_norm_index_left = NORMINDEXMAX;
	static bool allumer = false; //pour que le robot roule ou non

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data_L[i] > max_norm_left){
			max_norm_left = data_L[i];
			max_norm_index_left = i;
		}
		if(data_R[i] > max_norm_right){
			max_norm_right = data_R[i];
		}
		if(data_F[i] > max_norm_front){
			max_norm_front = data_F[i];
				}
		if(data_B[i] > max_norm_back){
			max_norm_back = data_B[i];
		}
	}

//pour trouver l'intensité au niveau des micros et au niveau de la led par
	//rapport au deux micros pour former les zones (rapport + justesse du code).
	led_mon(max_norm_left,max_norm_right,max_norm_front,max_norm_back);

	//condition pour les fr�quences pour allumer le robot

	if((max_norm_index_left >= NORM_INDEX_MIN_LEFT && max_norm_index_left <= NORM_INDEX_MAX_LEFT) && max_norm_left > LEFT_VALUE_MIN){ // 1015.625Hz et 1093.75Hz
		//fonctionne seulement s'il n'est pas dans le mode chor�e
		allumer = !allumer;
		speed_coeff=allumer;
		chThdSleepMilliseconds(DEUXMILLEMILIS);
		}
	if (allumer) {
		speed_coeff=SPEED_ON;		//envoie la vitesse pour allumer ou eteindre le robot
	} else {
		speed_coeff=SPEED_OFF;
	}
	if ((max_norm_index_left > NORM_INDEX_MAX_LEFT) && allumer) { //accelere le robot en fonction de la vitesse entendue
			speed_coeff = COEFFSPEED*max_norm_index_left/MIN_FREQ;
		}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = ZERO;
	static uint8_t mustSend = ZERO;
	bool stop=get_stopAudio();

	if(stop){
		get_StopAudioSem();
	}

	//loop to fill the buffers
	for(uint16_t i = ZERO ; i < num_samples ; i+=MICRO_NB){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = ZERO;
		micLeft_cmplx_input[nb_samples] = ZERO;
		micBack_cmplx_input[nb_samples] = ZERO;
		micFront_cmplx_input[nb_samples] = ZERO;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (COEFFDEUX * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (COEFFDEUX * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > TAILLEPOUREVITERSURCHARGE){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = ZERO;
		} //ICI y'a des bailles � enlever
		nb_samples = ZERO;
		mustSend++;

		sound_remote(micLeft_output,micRight_output,micFront_output,micBack_output);
		t=t+1;
		chprintf((BaseSequentialStream*)&SD3, "time_audio = %d \n", t);
	}
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float get_speed_coeff(void) {
	return speed_coeff;
} 								//nos publics fonctions pour communiquer la vitesse et la diretion
bool get_direction(void) {
	return direction;
}
float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
