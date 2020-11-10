/**
  ******************************************************************************
  * @file    main.c
  * @author  Samuel Poiraud Modifié par Théophane Durand et Rémi Gandrillon
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

//Librairies ajoutées pour notre projet
#include "gps.h" //Permet la gestion des trams reçues par le GPS
#include "WS2812S.h" //Permet la gestion du ruban de led
#include "systick.h" //Permet les innterruptuins à certain intervales de temps


/**
 * Structure à 4 champs pour la gestion de la machine à état.
 * Cette sutructure peut contenir tous les états de la machine à état
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
 * Variable globale qui prend comme valeur les charcactères envoyés par le téléphone
 */
static volatile uint8_t commandeBluetooth;

/**
 * Déclaration du prototype des fonctions (détaillées plus bas)
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
	BSP_GPIO_Enable();	//Activation des périphériques GPIO
	SYS_ClockConfig();		//Configuration des horloges.

	//ajoute la fonction process_ms à la liste des fonctions à appeler à chaque ms.
	Systick_add_callback_function(&process_ms);


	/**
	 * Initialisation du port de la led de bluetooth
	 * Lorsque la connection est établie cette LED s'allume, sinon elle clignotte
	 */
	BSP_GPIO_PinCfg(LED_BC,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);

	/**
	 * Initialisation des pins Enable des haut-parleurs
	 * On fixe cette valeur à 1 dés l'initialisation du programme pour pouvoir à tout instant mettre de la musique par les haut-parleurs
	 */
	BSP_GPIO_PinCfg(GPIOB, GPIO_PIN_12,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

	//Initialisation de l'UART1 à la vitesse de 115200 bauds/secondes (92kbits/s) PB6 : Tx  | PB7 : Rx.
	UART_init(UART3_ID,9600); //L'UART 3 est branché au GPS pour recevoir les trams des satélites
	UART_init(UART2_ID,9600); //L'UART 2 est connecté au bluetooth de controle, afin de recevoir les informations du téléphone
	UART_init(UART1_ID,9600); //L'UART 1 n'est finalement pas utilisé, il est connecté au module bluetooth audio

	//"Indique que les printf sortent vers le périphérique UART2."
	SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

	//On initialise le ruban de LEDs
	LED_MATRIX_init();
	uint8_t i;
	uint32_t pixels[NBLED];

	//On éteint le ruban de LED pour ne pas avoir des "restes" des dernières instructions données
	for(i = 0; i < NBLED; i++)
	{
		pixels[i] = OFF;
	}
	LED_MATRIX_display(pixels, NBLED);

	//On initilaise le caractère à 0 pour les données reçues par le bluetooth.
	commandeBluetooth = '0';

	//On initialise le mode de fonctionnement à n pour être dés le début en mode nominal
	uint8_t modeFonctionnement = 'n';

	//Tache du fond d programme
	while(1)
	{

		modeFonctionnement = bluetoothControl();  //On lit la valeur reçue par le bluetooth
		state_machine(modeFonctionnement);		//On définit l'état de la machine à état en fonction de la donnée reçue par bluetooth
		gps();		//On lit les données reçues par le GPS et on les traite
	}
}

/**
 * Fonction décrivant la machine à état de notre système
 */
void state_machine(uint8_t modeFonctionnement)
{
	/**
	 * Structure décrvant les états possibles du système
	 */
	typedef enum
	{
		INIT = 0,
		NOMINAL,
		CLIGNODROITE,
		CLIGNOGAUCHE
	}state_e;

	//On initialise l'état de la machine à INIT
	static state_e state = INIT;

	/**
	 * Déroulement de la machine à état.
	 * On de tous les états on peut passer à tous les autres.
	 *Excépté l'état INIT qui n'est appelé à l'initialisation du programme
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

	//Commande de la led de controle pour la témoin lumineux de la connection bluetooth
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
 * Récéption des donées GPS et traiement pour être envoyées au téléphone
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
 * Fonction permettant de récupérer le charactère envoyé par le téléphone
 */
uint8_t bluetoothControl(void)
{
	uint8_t c;
	c = UART_getc(UART2_ID);
	return c;
}

/**
 * Fonction appelée toutes les milisecondes
 * Elle gère le clignottement et l'allumage des led
 */
void process_ms(void)
{
	static uint32_t t_led = 0;
	//static uint32_t num_led = 0;
	t_led++; //Variable permettant de compter le temps elle s'incrémente toutes les milisecondes

	static uint32_t pixels[NBLED];

		if(t_led == 200)
		{
			if(led_state == LED_NOMINAL)
			{
				for(int i = 0; i < NBLED; i++){ //Allumage de l'arrière du casque en rouge en mode nominal
					if(i <= 9 || i >= 40){
						pixels[i] = RED;
					}
					else if(i >= 15 && i <= 34){ //Allumage de l'avant du casque en blanc en mode nominal
						pixels[i] = WHITE;
					}
					else{  //Les leds sur le coté restent éteintes
						pixels[i] = OFF;
					}
				}
			}
			else if(led_state == LED_CLIGNO_GAUCHE)  //Allumage de du coté gauche du casque en jaune en mode cligno gauche
			{
				for(int i = 0; i < NBLED; i++){
					if(i <= 2 || i >= 47){
						pixels[i] = RED;
					}
					else if(i >= 22 && i <= 27){ //Allumage de du coté avant du casque en blanc en mode cligno gauche
						pixels[i] = WHITE;
					}
					else if(i > 27 && i < 47){ //Allumage de du coté arrière du casque en rouge en mode cligno gauche
						pixels[i] = YELLOW;
					}
					else if(i > 2 && i < 22){ //Rest edes leds éteintes
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
			if(commandeBluetooth != 'b') //Clignottement du témoin lumineux lorsque la connection bluetooth n'est pas faite
			{
				HAL_GPIO_WritePin(LED_BC, 0);
			}
			LED_MATRIX_display(pixels, NBLED);
		}

		if(t_led == 400)
		{
			t_led = 0;
			if(led_state == LED_NOMINAL)  //On éteint toutes les leds pour l'effet de clignottement
			{
				for(int i = 0; i < NBLED; i++){
					pixels[i] = OFF;
				}
			}
			else if(led_state == LED_CLIGNO_GAUCHE)  //On éteint que les leds aunes pour faire clignotter le coté du casuqe sans faire clignotter l'avant et l'arrière
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
			if(commandeBluetooth != 'b')  //Clignottement du témoin lumineux lorsque la connection bluetooth n'est pas faite
			{
				HAL_GPIO_WritePin(LED_BC, 1);
			}
			LED_MATRIX_display(pixels, NBLED);
		}
}
