/*
 * utilitaires.h
 *
 *  Created on: 23 juil. 2017
 *      Author: Tocqueville
 */

#ifndef UTILITAIRES_H_
#define UTILITAIRES_H_

#include "Board.h"


//  -------------------------------------   Global Define ----------------------------------------



/*------------------
   ENTREES
--------------------*/


#define BP0                   0           // BP droite
#define BP1                   1           // BP gauche
#define BP2                   2           // BP

/*------------------
   ENTREES ANALOGIQUES
--------------------*/



/*------------------
   SORTIES PWM
--------------------*/



/*------------------
   SORTIES DIGITALES
--------------------*/

#define LED_cpu      0  // cpu
#define SORTIE_1     1  // Sortie 1
#define SPOT_All     1  // ou LED IR

#ifdef C10_CAMERA_PIR   // liste sorties
    #define Charge_TP    1
    #define Enab_ESP     2
#endif

#ifdef C10_sonde_HUmSol  // liste sortie
    #define Power_sonde 1
    #define Power_sonde2 2
#endif

#define adress_filter                  0x03

#ifdef CC13xx
  #define DELAY_MS(i)      (Task_sleep((i) * (1000 / Clock_tickPeriod)))
  #define DELAY_US(i)      (Task_sleep(i / Clock_tickPeriod))
#endif
#ifdef ESP32
  #define DELAY_MS(i)      vTaskDelay( i / portTICK_RATE_MS)  // 1tick=10ms
  #define DELAY_US(i)      vTaskDelay( i /1000 / portTICK_RATE_MS)
#endif

// Define
#define large_data

#define nb_erreurs_enregistrees  20

#define depass_tab                  0x60  // + 0x0F  // code erreur

#define FLASH_PAGE_SIZE   128  // TODO 128 mots de 16bits  256  // 16 pages (de 256 octets) par secteurs (de 4Koctets) (256 secteurs)
#define EXT_FLASH_DATE_SIZE     4  // multiple de 4
#define FLASH_ADDRESS(page, offset) (((page) << 8) + (offset))

#ifndef MIN
  #define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
  #define MAX(X, Y) (((X) < (Y)) ? (Y) : (X))
#endif

#define T_BIT(x,y) (x & (0x1 << (y)))

#ifdef CC13xx
    #define MS_to_TICK(x)       (x *(1000/Clock_tickPeriod))      // *1000/10=*100  10 micro-secondes
    #define NB_SECONDES(x)      (x=Seconds_get())        // depuis 1970
    #define NB_TICKS            Clock_getTicks()
    #define ECRIT_SECONDES(x)   Seconds_set(x.tv_sec)
    #define CLOCK_START(h)      Clock_start(h)
    #define CLOCK_STOP(h)       Clock_stop(h)
    #define CLOCK_RESTART_TIMEOUT(h,x)  {Clock_setTimeout(h, x*(1000/Clock_tickPeriod));  Clock_start(h);}  // x en ms
    #define CLOCK_RESTART_PERIOD(h,x)  {Clock_stop(h); Clock_setPeriod(h, x*(1000/Clock_tickPeriod));  Clock_start(h);}  // x en ms
    //#define CLOCK_RESTART_TIMEOUT(h,x) {h=0; h=x;}
    #define TIMER_HANDLE        Clock_Handle
    #define TIMER_ARG           UArg
#endif
#ifdef ESP32
    #define MS_to_TICK(x)       (x / portTICK_RATE_MS)     //  /10           // 10 milli-secondes
    #define NB_SECONDES(x)      (time((time_t *) &x))
    #define NB_TICKS            esp_timer_get_time()/10
    #define ECRIT_SECONDES(x)   settimeofday(&x, NULL)
    #define CLOCK_START(h)      xTimerStart( h, 0 )
    #define CLOCK_STOP(h)       xTimerStop( h, 0 )
    #define CLOCK_RESTART_TIMEOUT(h,x)  (xTimerChangePeriod (h, x / portTICK_PERIOD_MS, 0)); // inclus start
    #define TIMER_HANDLE        TimerHandle_t
    #define TIMER_ARG           TimerHandle_t
#endif

#define SEUIL_ARRET_CHARGE 4150   // en milli-volts

//  -------------------------------------   Type declarations  ----------------------------------------

// ENTREES
typedef struct
{
//  GPIO_TypeDef*     entree_port;
  uint32_t          entree_pin;        // GPIO
  uint16_t          comptage;          // comptage en ms de 0 au max
  uint16_t          max;                // seuil max
  uint8_t           etat;                // 0 ou 1
  uint8_t           chgt_etat;           // 0 pas de chgt, 1: désactivation, 2: activation
  uint8_t           activ_manuelle;      // 0 : activé à 0   1:activé à 1
  uint16_t          duree_activ_manu;    // durée en 100ms de l'activation (max 2h)
  uint8_t           configure;
} etat_entree;

// SORTIES
typedef struct
{
  uint32_t          sortie_pin;    // GPIO
  uint8_t           etat_initial;
  uint8_t           configure;    // 0:sortie non configuree   1:sortie utilisable et configuree
  uint8_t           consigne;
  uint16_t          duree;
  uint8_t           etat;
  uint8_t           nb_flash;
  uint8_t           etat_ref;
  uint8_t           inversion;  // 1: inversion de la sortie
  TIMER_HANDLE      h_clock;     // handle clock
} S_SortieTor;

// SORTIES  EVannes
typedef struct
{
  TIMER_HANDLE      h_clock;     // handle clock
} S_SortieEV;

// Sortie PWM
typedef struct
{
    uint8_t     consigne;
    uint8_t     nb_flash;
    uint16_t    duree; // duree totale restante
    uint8_t     etat;  // 0:eteint 1:pause 2:allume 3:2�ton
    uint16_t     periode1;
    uint8_t     duty1;
    uint8_t     periode2;
    uint8_t     duty2;
    uint8_t     duree_chgt;  // duree avant changement
    TIMER_HANDLE      h_clock;     // handle clock
    #ifdef CC13xx
      PWM_Handle        h_pwm;       // handle pwm
    #endif
} EtatPwm;

typedef struct
{
    int16_t temp;
    uint16_t hygro;
} S_Temp;

//  -------------------------------------   Extern  ---------------------------------------

#ifdef DEBUG_TIME
    #define MAX_Index 30
    extern uint32_t time_cpu[], etat_detail[];
    extern uint8_t etat_cpu[], index_time, trigger_time;
#endif

extern  uint8_t code_erreur, err_donnee1, err_donnee2;
extern  S_SortieTor SortieTor[];

extern uint16_t voltage_panneau;
extern int16_t Temp_actuelle;   // Temp au centieme de degr�

extern uint8_t mode_arret;  // 0:marche  1:reduit1  2:reduit2  3:reduit3  4:arret total

//  -------------------------------------   Global functions prototypes  ----------------------------------------


void ES_init(void);
void Clignot_sortie(uint8_t num);
void Clignot_buzzer();

 void envoi_code_erreur (void);        // envoi tout le temps l'erreur au PC
 void raz_erreur(void);

 void Turn_on_LED(uint8_t);
 void Turn_off_LED(uint8_t);
 void ACTIV_SORTIE(char unsigned, char unsigned, int unsigned);
 void ETAT_SORTIE(char unsigned);  // change l'etat des sorties
 void SORTIE_S_Callback(TIMER_ARG pxTimer);
 void SORTIE_EV_Callback(TIMER_ARG pxTimer);
 void SORTIE_PWM_Callback(TIMER_ARG pxTimer );

 void ACTIV_PWM( char unsigned,char unsigned, int unsigned, int unsigned, char unsigned);
 void ETAT_PWM(uint8_t num);
 void init_PWM(void);
 void allume_PWM(uint8_t num);
 void eteint_PWM(uint8_t num);
 char unsigned  lecture_etat_sortie(uint8_t);
 void message_lecture_etat_sortie(uint8_t);
 void hysteresis_entrees(void);
 void activation_entree (char unsigned a, char unsigned duree, char unsigned valeur);
 void dec_activation_entree_manuelle (void);

 void modif_keep_alive (uint8_t type, uint8_t periode) ;

 int16_t mesure_volt_vref (void);
 void envoi_voltage_bas (void);
 uint32_t mesure_temperature32(void);
 uint8_t mesure_temperature(S_Temp *Stemp);
 uint16_t mesure_voltage(void);
 uint8_t mesure_niv_batterie(uint16_t volt, int16_t temp);
 uint8_t conv_volt_batt (uint16_t voltage_batt, uint8_t temp_mesure_t);
 uint8_t lecture_volt_batterie(uint16_t * voltage_bat, uint16_t * voltage_sol);

 void envoi_int_decimal (uint16_t val, char unsigned position);
 void envoi_char_decimal (char unsigned val, char unsigned position); // envoie 1 char en d�cimal 0 � 255
 char unsigned comp_message (char unsigned * mess1, char unsigned * mess2);


 uint8_t lecture_analog(uint8_t num, uint16_t * adcVolt);

 struct timeval NB_MICROSEC(void);

 uint8_t Part_Identif(void);
 void ecrit_date_rtc(void);
 void ecrit_heure_rtc(void);
 void lecture_date_rtc(void);
 void lecture_heure_rtc(void);
 void conversion_sec_date(time_t jour, uint8_t position);  //  écrit la date dans message à la position
 void conversion_sec_heure(uint32_t second, uint8_t position, bool decimal);  //  écrit l'heure dans message à la position
 void SetupSetAonRtcSubSecIncOK (uint32_t subSecInc );
 void code_dec_hex (uint8_t* ad_entree, uint8_t* ad_sortie, uint8_t nb_hex);
 void code_hex_dec (uint8_t* ad_entree, uint8_t* ad_sortie, uint8_t nb_octet_hex, uint8_t type);
 uint16_t  decodage16_message_in (uint8_t nombre, uint8_t position);
 int16_t  decodage16i_message_in (uint8_t nombre, uint8_t position);

// uint8_t ajout_2d_float_message (float valeur, uint8_t nb_decimale, uint8_t position);
// void envoi_2d_float_message (double valeur, uint8_t nb_decimale, uint8_t position);

 uint8_t ajout_float_multi_valeurs(float * valeur, uint8_t nb_decimale, uint8_t num, uint8_t nb, uint8_t fixe);
 uint8_t ajout_float_message (float valeur, uint8_t nb_decimale, uint8_t num, uint8_t fixe);
 uint8_t ajout_int16_multi_valeurs(int16_t * valeur, uint8_t num, uint8_t nb, uint8_t fixe);
 uint8_t ajout_int16_message (int16_t valeur, uint8_t pos, uint8_t fixe);
 uint8_t ajout_int8_message (int8_t valeur, uint8_t pos, uint8_t fixe);


#endif /* UTILITAIRES_H_ */
