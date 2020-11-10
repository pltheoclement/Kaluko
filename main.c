/**
  ******************************************************************************
  * @file    main.c
  * @author  Samuel Poiraud Modifi� par Th�ophane Durand et R�mi Gandrillon
  * @version V2.0 Kaluko edition
  * @date    28/01/2019
  * @brief   Fonction main
  ******************************************************************************
*/

//Librairies fournies de base
#include "stm32f1xx_hal.h"
#include "stm32f1xx_nucleo.h"
#include "stm32f1_uart.h"
#include "stm32f1_sys.h"
#include "macro_types.h"
#include "stm32f1_gpio.h"
#include "stm32f1_timer.h"
#include "test.h"

//Librairies ajout�es pour notre projet
#include "gps.h" //Permet la gestion des trams re�ues par le GPS
#include "WS2812S.h" //Permet la gestion du ruban de led
#include "systick.h" //Permet les innterruptuins � certain intervales de temps


/**
 * Structure � 4 champs pour la gestion de la machine � �tat.
 * Cette sutructure peut contenir tous les �tats de la machine � �tat
 */
typedef enum
{
	LED_CLIGNO_GAUCHE,
	LED_CLIGNO_DROITE,
	LED_ETEINT,
	LED_NOMINAL
}led_state_e;
static volatile led_state_e led_state;

/**
 * Variable globale qui prend comme valeur les charcact�res envoy�s par le t�l�phone
 */
static volatile uint8_t commandeBluetooth;

/**
 * D�claration du prototype des fonctions (d�taill�es plus bas)
 */
void process_ms(void);
void state_machine(uint8_t modeFonctionnement);
uint8_t bluetoothControl(void);
void gps(void);

/**
 * Fonction principale du programme
 */
int main(void)
{
	HAL_Init();			//Initialisation de la couche logicielle HAL (Hardware Abstraction Layer)
	BSP_GPIO_Enable();	//Activation des p�riph�riques GPIO
	SYS_ClockConfig();		//Configuration des horloges.

	//ajoute la fonction process_ms � la liste des fonctions � appeler � chaque ms.
	Systick_add_callback_function(&process_ms);


	/**
	 * Initialisation du port de la led de bluetooth
	 * Lorsque la connection est �tablie cette LED s'allume, sinon elle clignotte
	 */
	BSP_GPIO_PinCfg(LED_BC,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);

	/**
	 * Initialisation des pins Enable des haut-parleurs
	 * On fixe cette valeur � 1 d�s l'initialisation du programme pour pouvoir � tout instant mettre de la musique par les haut-parleurs
	 */
	BSP_GPIO_PinCfg(GPIOB, GPIO_PIN_12,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

	//Initialisation de l'UART1 � la vitesse de 115200 bauds/secondes (92kbits/s) PB6 : Tx  | PB7 : Rx.
	UART_init(UART3_ID,9600); //L'UART 3 est branch� au GPS pour recevoir les trams des sat�lites
	UART_init(UART2_ID,9600); //L'UART 2 est connect� au bluetooth de controle, afin de recevoir les informations du t�l�phone
	UART_init(UART1_ID,9600); //L'UART 1 n'est finalement pas utilis�, il est connect� au module bluetooth audio

	//"Indique que les printf sortent vers le p�riph�rique UART2."
	SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

	//On initialise le ruban de LEDs
	LED_MATRIX_init();
	uint8_t i;
	uint32_t pixels[NBLED];

	//On �teint le ruban de LED pour ne pas avoir des "restes" des derni�res instructions donn�es
	for(i = 0; i < NBLED; i++)
	{
		pixels[i] = OFF;
	}
	LED_MATRIX_display(pixels, NBLED);

	//On initilaise le caract�re � 0 pour les donn�es re�ues par le bluetooth.
	commandeBluetooth = '0';

	//On initialise le mode de fonctionnement � n pour �tre d�s le d�but en mode nominal
	uint8_t modeFonctionnement = 'n';

	//Tache du fond d programme
	while(1)
	{

		modeFonctionnement = bluetoothControl();  //On lit la valeur re�ue par le bluetooth
		state_machine(modeFonctionnement);		//On d�finit l'�tat de la machine � �tat en fonction de la donn�e re�ue par bluetooth
		gps();		//On lit les donn�es re�ues par le GPS et on les traite
	}
}

/**
 * Fonction d�crivant la machine � �tat de notre syst�me
 */
void state_machine(uint8_t modeFonctionnement)
{
	/**
	 * Structure d�crvant les �tats possibles du syst�me
	 */
	typedef enum
	{
		INIT = 0,
		NOMINAL,
		CLIGNODROITE,
		CLIGNOGAUCHE
	}state_e;

	//On initialise l'�tat de la machine � INIT
	static state_e state = INIT;

	/**
	 * D�roulement de la machine � �tat.
	 * On de tous les �tats on peut passer � tous les autres.
	 *Exc�pt� l'�tat INIT qui n'est appel� � l'initialisation du programme
	 */
	switch(state)
	{
		case INIT:
			state = NOMINAL;
			break;

		case NOMINAL:
			switch(modeFonctionnement)
			{
				case 'g':
					state = CLIGNOGAUCHE;
					break;
				case 'd':
					state = CLIGNODROITE;
					break;
				default:
					break;
			}
			led_state = LED_NOMINAL;
			break;

		case CLIGNOGAUCHE:
			switch(modeFonctionnement)
			{
				case 'n':
					state = NOMINAL;
					break;
				case 'd':
					state = CLIGNODROITE;
					break;
				default:
					break;
			}
			led_state = LED_CLIGNO_GAUCHE;
			break;

		case CLIGNODROITE:
				switch(modeFonctionnement)
				{
					case 'n':
						state = NOMINAL;
						break;
					case 'g':
						state = CLIGNOGAUCHE;
						break;
					default:
						break;
				}
				led_state = LED_CLIGNO_DROITE;
				break;
		default:
			break;
	}

	//Commande de la led de controle pour la t�moin lumineux de la connection bluetooth
	if(modeFonctionnement == 'b')
	{
		commandeBluetooth = 'b';
		HAL_GPIO_WritePin(LED_BC, 1);
	}
	else if(modeFonctionnement == 'y')
	{
		commandeBluetooth = '0';
		HAL_GPIO_WritePin(LED_BC, 0);
	}
	else if(modeFonctionnement == 's'){

	}
}


/**
 * R�c�ption des don�es GPS et traiement pour �tre envoy�es au t�l�phone
 */
void gps(void)
{
		uint8_t c;
		static gps_datas_t gps_datas;
		while(UART_data_ready(UART3_ID))
		{
			c = UART_get_next_byte(UART3_ID);
			if(GPS_process_rx(c, &gps_datas) == TRAME_GPRMC)
			{
				printf("%lf, %lf\n",gps_datas.latitude_deg, gps_datas.longitude_deg);
			}

		}
}

/**
 * Fonction aui aurait permis de traiter le signal audio (augmenter volume, play, pause ...)
 */
/*void controlAudio(void)
{

}*/

/**
 * Fonction permettant de r�cup�rer le charact�re envoy� par le t�l�phone
 */
uint8_t bluetoothControl(void)
{
	uint8_t c;
	c = UART_getc(UART2_ID);
	return c;
}

/**
 * Fonction appel�e toutes les milisecondes
 * Elle g�re le clignottement et l'allumage des led
 */
void process_ms(void)
{
	static uint32_t t_led = 0;
	//static uint32_t num_led = 0;
	t_led++; //Variable permettant de compter le temps elle s'incr�mente toutes les milisecondes

	static uint32_t pixels[NBLED];

		if(t_led == 200)
		{
			if(led_state == LED_NOMINAL)
			{
				for(int i = 0; i < NBLED; i++){ //Allumage de l'arri�re du casque en rouge en mode nominal
					if(i <= 9 || i >= 40){
						pixels[i] = RED;
					}
					else if(i >= 15 && i <= 34){ //Allumage de l'avant du casque en blanc en mode nominal
						pixels[i] = WHITE;
					}
					else{  //Les leds sur le cot� restent �teintes
						pixels[i] = OFF;
					}
				}
			}
			else if(led_state == LED_CLIGNO_GAUCHE)  //Allumage de du cot� gauche du casque en jaune en mode cligno gauche
			{
				for(int i = 0; i < NBLED; i++){
					if(i <= 2 || i >= 47){
						pixels[i] = RED;
					}
					else if(i >= 22 && i <= 27){ //Allumage de du cot� avant du casque en blanc en mode cligno gauche
						pixels[i] = WHITE;
					}
					else if(i > 27 && i < 47){ //Allumage de du cot� arri�re du casque en rouge en mode cligno gauche
						pixels[i] = YELLOW;
					}
					else if(i > 2 && i < 22){ //Rest edes leds �teintes
						pixels[i] = OFF;
					}
				}
			}
			else if(led_state == LED_CLIGNO_DROITE)
			{
				for(int i = 0; i < NBLED; i++){
					if(i <= 2 || i >= 47){
						pixels[i] = RED;
					}
					else if(i >= 22 && i <= 27){
						pixels[i] = WHITE;
					}
					else if(i > 2 && i < 22){
						pixels[i] = YELLOW;
					}
					else if(i > 27 && i < 47){
						pixels[i] = OFF;
					}
				}
			}
			if(commandeBluetooth != 'b') //Clignottement du t�moin lumineux lorsque la connection bluetooth n'est pas faite
			{
				HAL_GPIO_WritePin(LED_BC, 0);
			}
			LED_MATRIX_display(pixels, NBLED);
		}

		if(t_led == 400)
		{
			t_led = 0;
			if(led_state == LED_NOMINAL)  //On �teint toutes les leds pour l'effet de clignottement
			{
				for(int i = 0; i < NBLED; i++){
					pixels[i] = OFF;
				}
			}
			else if(led_state == LED_CLIGNO_GAUCHE)  //On �teint que les leds aunes pour faire clignotter le cot� du casuqe sans faire clignotter l'avant et l'arri�re
			{
				for(int i = 0; i < NBLED; i++){
					if(i > 27 && i < 47){
						pixels[i] = OFF;
					}
				}
			}
			else if(led_state == LED_CLIGNO_DROITE)
			{
				for(int i = 0; i < NBLED; i++){
					if(i > 2 && i < 22){
						pixels[i] = OFF;
					}
				}
			}
			if(commandeBluetooth != 'b')  //Clignottement du t�moin lumineux lorsque la connection bluetooth n'est pas faite
			{
				HAL_GPIO_WritePin(LED_BC, 1);
			}
			LED_MATRIX_display(pixels, NBLED);
		}
}
