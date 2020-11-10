/*
 * config.h
 *
 *  Created on: 31 mars 2016
 *      Author: Nirgal
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//Choix de la cible utilis�e.
#define BLUEPILL 1

//_______________________________________________________
//Configuration des broches utilis�es...
//Ces macros permettent d'utiliser dans le code des noms explicites (LED_GREEN, ...)

#if BLUEPILL
	#define LED_BC	GPIOB, GPIO_PIN_0
#endif

//Choisir les broches pour l'UART1, parmi ces deux possibilit�s :
//#define UART1_ON_PB6_PB7
#define UART1_ON_PA9_PA10

//Choisir les broches pour l'UART2, parmi ces deux possibilit�s :
#define UART2_ON_PA2_PA3
//#define UART2_ON_PD5_PD6

//Choisir les broches pour l'UART3, parmi ces deux possibilit�s :
#define UART3_ON_PB10_PB11
//#define UART3_ON_PD8_PD9

//_______________________________________________________
//Configuration des diff�rentes couleurs utilis�es

#define WHITE	0x555555
#define RED	    0x005500
#define YELLOW	0x55aa00
#define OFF     0x000000


#define USE_MATRIX_LED			1
#define USE_MATRIX_LED_32_32	1

//Configuration du nombre de led de notre ruban
#define NBLED   50


#define USE_ADC					0
	//Ces configurations permettent d'activer les entr�es analogiques souhait�es.
	//16 entr�es analogiques peuvent �tre activ�es maximum.
	//2 entr�es analogiques doivent �tre activ�es minimum. (Vref est un choix possible si vous utilisez une seule entr�e)
	#define USE_AN0			1	//Broche correspondante : PA0
	#define USE_AN1			1	//Broche correspondante : PA1
	#define USE_AN2			0	//Broche correspondante : PA2	//Sur la Nucleo, cette broche n'est pas c�bl�e !
	#define USE_AN3			0	//Broche correspondante : PA3	//Sur la Nucleo, cette broche n'est pas c�bl�e !
	#define USE_AN4			0	//Broche correspondante : PA4
	#define USE_AN5			0	//Broche correspondante : PA5
	#define USE_AN6			0	//Broche correspondante : PA6
	#define USE_AN7			0	//Broche correspondante : PA7
	#define USE_AN8			0	//Broche correspondante : PB0
	#define USE_AN9			0	//Broche correspondante : PB1
	#define USE_AN10		0	//Broche correspondante : PC0	//Sur la Bluepill, cette broche n'est pas c�bl�e !
	#define USE_AN11		0	//Broche correspondante : PC1	//Sur la Bluepill, cette broche n'est pas c�bl�e !
	#define USE_AN12		0	//Broche correspondante : PC2	//Sur la Bluepill, cette broche n'est pas c�bl�e !
	#define USE_AN13		0	//Broche correspondante : PC3	//Sur la Bluepill, cette broche n'est pas c�bl�e !
	#define USE_AN14		0	//Broche correspondante : PC4	//Sur la Bluepill, cette broche n'est pas c�bl�e !
	#define USE_AN15		0	//Broche correspondante : PC5	//Sur la Bluepill, cette broche n'est pas c�bl�e !
	#define USE_AN16		0	//Capteur de temp�rature interne
	#define USE_AN17		1	//Vref


#define USE_BSP_TIMER			0 //Utilisation de stm32f1_timer.c/h


#endif /* CONFIG_H_ */
