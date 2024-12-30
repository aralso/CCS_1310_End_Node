/*
 * Copyright (c) 2016
 * All rights reserved.
 *
 * TEMPERATURE-HYGRO-Mouvement :
 *
B0c : Chgt mode : arret <-> marche
B0l :
B0d : serie->radio->flash
B1c : interrog passerelle
B1l : lecture température
B1d :
B0+B1 :hexa<->texte

Télécommande :
    Bouton du haut : court:activation centrale
    Bouton du milieu : court:allumage spot(P) 70 secondes & spot solaire(1)   Double:Réception code tournant
    Bouton du bas : court désactivation centrale sécurisée    double : désactivation no sécurisée

 *
 * : Bouton0 court : Arret/marche clock - (Temp - Mouv - FFT)
 * : Bouton0 long  : interrogation passerelle + Bip
 * : Bouton0 double  : mode d'envoi :  0:envoi port série 'R'  1:envoi radio 'L'   2:enregis Flash
 * : Bouton1 court  : lecture unique
                           Si Temp  : ->H Temp
                           Si Mouv  : chgt val brut Z
                           Si FFT   : captation FFT
 * : Bouton1 long  : Changement mode principal du capteur (Temp-> mouv-> FFT)
 * : Bouton1 double : changement mode secondaire (mouv:10->13)
 * : Bouton0 + Bouton 1 : Hexa-texte


TELECOMMANDE :
Bouton 0        appui court :  Activation centrale (PMC2) (1 bip long si ok, 2 bip long si déja, 1 bip court:désactivé)

Bouton 1        appui court :  Desactivation centrale (PMC0 - 1 bip si ok, 2 bip court si déja)

Bouton 2        appui court :  Activation partielle (PMC2I0)
                double appui: Envoi du code tournant


CAPTEUR (S&T):
Bouton 0        appui court :  envoi 1 valeur capteur (temperature/luminosité,..)
                appui long  :  raz index flash & ram (avec 3 bips)
                double appui:  réception messages du concentrateur

Bouton 1
                appui court :  activation capteur (2 bips)
                appui long  :  désactivation capteur (1 bip)
                double appui:  Envoi niveau de réception à L

2 Boutons simultanés
                appui       : transfert flash de la ram (avec bip long)*/


/***** Includes *****/
#include <DmRadioTask.h>
#include <DmNodeTask.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Seconds.h>
#include <time.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"

#ifdef NODE

#include "sensors/SensorI2C.h"
#include "sensors/SensorOpt3001.h"
#include "sensors/SensorBmp280.h"
#include "sensors/SensorHdc1000.h"
#include "sensors/SensorMpu9250.h"
#include "sensors/SensorTmp007.h"
#include "extflash/ExtFlash.h"

#include "RadioProtocol.h"
#include "utilitaires.h"
#include "comm_message.h"

#include <ti/drivers/Watchdog.h>
#include <ti/sysbios/hal/Seconds.h>

#include <ti/sysbios/knl/Swi.h>

#include <ti/drivers/Power.h>
#include <ti/devices/CC13x0/driverlib/AON_BATMON.h>



 /***** Defines Utilisateur *****/


//   **********   CONFIGURATION MODULE ***************


//#define TELECOMMANDE     // activation telecommande code tournant



// Periodicité keep-alive, mise à l'heure, interrogation concentrateur

#define TEST_PRESENCE_CONCENTRATEUR  0 // 0 : pas de test,   1:periode 10 secondes
#define TEST_ENVOI_MESSAGES  0  // 0:pas de test d'envoi periodique   x:periode en seconde

 /*  **********  CONFIGURATION CAPTEURS ****************/

// Affichage sur écran ou enregistrement sur Flash
#define MODE_PRINCIPAL       0   //0   0:temp  1:Mouv  2:FFT
#define MODE_AFFICH_ENREG    1   //1   0:envoi par port série 'R'  1:envoi par radio 'L'   2:enregis Flash
#define MODE_HEXA_TEXTE      1   //0   0:hexa   1:texte

#define MAX_RAM_TOURNANTE 20   // Nb max de valeurs enregistrées dans la ram

// Capteur ESP32-Camera
#define PIR_DUREE_RAZ  8  // en minutes
#define PIR_CPT_MAX    5  // nb max dans le temps recedent

// Define pour Capteur MPU
#define ENREG_QUAT    0     // 0:pas enreg sur flash  1:enregistré sur flash

// -----------  Capteur temperature hygrometrie -----------------
// Consommation capteur temp (50kbps):
// conso en sleep : 0,7uA (v1.5.1 sans capteur hall) 2,5uA (v1.4.5)
// Watchdog chaque 20 sec (3,3mA pdt 1,5ms) : 0,25uA
// recharge chaque 0,7seconde : 2,3mA pdt 200us  : 0,7uA
// 1 interrogation concentrateur chaque 12h : (13mA pdt 26ms et 6mA pendant 125ms)     : 0,025uA
// Keepalive à 50kbps chaque 24h : 28mA pdt 4,6ms puis 6mA pdt 18ms : 0,003uA
// 1 mesure de temp chaque 10 minute : ( 3,5mA pdt 3ms  et 180uA pdt 12ms) : 0,020uA
// 1 envoi RF chaque 2 heures (13mA pdt 125ms pour 6 mesures)   : 0,225uA
// mode=5 (balise synchro) toutes les 30 secondes, fenetre=4ms=>9ms=>11,3ms à 4mA => 1,5uA
// TOTAL : 3,5uA =  7 ans en mode ecoute


// -----------  Camera ESP32 -----------------
// Consommation :
// PIR+circuits:17uA
// conso en sleep : 0,7uA
// recharge chaque seconde : 0,5uA
// Watchdog 0,5uA
// 1 interrogation concentrateur chaque 12h : (13mA pdt 26ms et 6mA pendant 125ms)     : 0,025uA
// Keepalive à 50kbps chaque 24h : 28mA pdt 4,6ms puis 6mA pdt 18ms : 0,003uA
// mode=5 (balise synchro) toutes les 10 secondes, fenetre=4ms => 5uA
// TOTAL : 25uA

// niveau batterie : 3,990V (80%) arret
// augmentation voltage +0,010v => +1% de charge batterie

// -----------  Commande electrovannes -----------------
// avec radio 50kbps:
// conso en sleep : 0,7uA
// recharge chaque 0,7seconde : 2mA pdt 200us  : 0,5uA
// Watchdog chaque 20 sec (3,3mA pdt 1,5ms) : 0,25uA
// 1 interrogation concentrateur chaque 12h : (13mA pdt 26ms et 6mA pendant 125ms)     : 0,025uA
// Keepalive à 50kbps chaque 24h : 28mA pdt 4,6ms puis 6mA pdt 18ms : 0,003uA
// mode=5 (balise synchro) toutes les 20 secondes, fenetre=4ms=>9ms=>11,3ms à 4mA => 2,25uA
// TOTAL : 4uA =  30 ans avec CR2470

// -----------  Sonde humidité sol -----------------
// avec radio 50kbps:
// mode=5 (balise synchro) toutes les 30 secondes, fenetre=4ms=>9ms=>11,3ms à 4mA => 1,5uA
// conso en sleep : 0,7uA
// recharge chaque 0,7seconde : 2mA pdt 200us  : 0,5uA
// Watchdog chaque 20 sec (3,3mA pdt 1,5ms) : 0,25uA
// 1 envoi sonde par RF chaque heure (13mA pdt 30ms)   : 0,1uA
// 1 interrogation concentrateur chaque 12h : (13mA pdt 26ms et 6mA pendant 125ms)     : 0,025uA
// Keepalive à 50kbps chaque 24h : 28mA pdt 4,6ms puis 6mA pdt 18ms : 0,003uA
// TOTAL : 3,1uA =  38 ans avec CR2470

// -----------  Capteur Luminosite -----------------
#ifdef CAPTEUR_LUMI
    #define LUMI_ACTIF       1   // 0:inactif  1:actif
    #define LUMI_PERIOD      5   // periode de mesure en secondes      (max:39000)
    #define LUMI_PERIOD_MIN    1  // periode minimum d'envoi de valeur (chaque x mesures) (max:255)
    #define LUMI_POURC_MIN   15.0   // ecart en pourcentage de lux
    #define LUMI_ECART_MIN   10.0   // ecart min de lux
#endif

// -----------  Capteur Pression -----------------
#ifdef CAPTEUR_PRESS
    #define PRESS_ACTIF       1     // 0:inactif  1:actif
    #define PRESS_PERIOD      10    // 600 periode de mesure en secondes      (max:39000)
    #define PRESS_PERIOD_MIN  1     // 6 periode minimum d'envoi de valeur (chaque x mesures) (max:255)
    #define PRESS_ECART_MIN   1    // 1 ecart en  hPa  (max:255) 1atm=1013hpa
#endif


//  CONFIGURATION SPOT SOLAIRE
#define SPOT_ALLUMAGE_JOUR  5           // durée en 0,1s d'allumage  PESJ5
#define SPOT_ALLUMAGE_NUIT  300         // durée en s d'allumage     PESN300
#define SPOT_ALLUMAGE_NUIT_SEUIL  20    // durée en 10mv de seuil : 10=>100mV   PESS20


/***** Defines *****/


#define BV(n)               (1 << (n))

#define TIMER_100MS                           100

    #define NODE_TASK_STACK_SIZE 1010  // 800  max constaté

#define NODE_TASK_PRIORITY   2 //2

#define NODE_EVENT_ALL                  0xFFFFFFFF
#define EVENT_RECEP_MESSAGE_UART        (uint32_t)(1 << 0)  // 1
#define EVENT_SEND_MESSAGE_UART         (uint32_t)(1 << 1)  // 2
#define EVENT_TIMER_PWM0                (uint32_t)(1 << 2)  // 4
#define EVENT_TIMER_PWM1                (uint32_t)(1 << 3)  //
#define EVENT_MESSAGE_RF_A_ENVOYER      (uint32_t)(1 << 4)  // 10
#define EVENT_FLASH_A_ENVOYER           (uint32_t)(1 << 5)  // 20
#define EVENT_RAM_A_ENVOYER             (uint32_t)(1 << 6)  // 40
#define NODE_EVENT_BOUTTON_0            (uint32_t)(1 << 7)  // 80
#define NODE_EVENT_BOUTTON_1            (uint32_t)(1 << 8)  // 100
#define NODE_EVENT_2BOUTTONS_APPUI      (uint32_t)(1 << 9)
#define NODE_EVENT_BOUTTON_2            (uint32_t)(1 << 10)
#define NODE_EVENT_READ_SENSOR_TEMP     (uint32_t)(1 << 11) // 4 0000
#define EVENT_ACTION_PERIODIQUE         (uint32_t)(1 << 12)
#define NODE_EVENT_HORS_PORTEE          (uint32_t)(1 << 13) // 10 0000
#define NODE_EVENT_MAJ_HEURE            (uint32_t)(1 << 14)
#define EVENT_FIN_ENVOI_RF              (uint32_t)(1 << 15) // 40 0000
#define NODE_EVENT_READ_SENSOR_LUMI     (uint32_t)(1 << 16)
#define NODE_EVENT_SIMPLE_MESSAGE       (uint32_t)(1 << 17)
#define NODE_EVENT_READ_SENSOR_PRESS    (uint32_t)(1 << 17)  // DOUBLON
#define EVENT_KEEP_ALIVE                (uint32_t)(1 << 18)
#define EVENT_VIDAGE_TAMPON             (uint32_t)(1 << 19)
#define ENREG_EEPROM                    (uint32_t)(1 << 20)
#define EVENT_TEST_PASSERELLE_DISPO     (uint32_t)(1 << 21)
#define NODE_EVENT_ENVOI_VOLTAGE        (uint32_t)(1 << 22)
#define EVENT_TIMER_S                   (uint32_t)(1 << 23)  // 80 0000
#define NODE_EVENT_SEND_CAPTEUR         (uint32_t)(1 << 24)
#ifdef C10_ElecVanne
    #define EVENT_TIMER_EV              (uint32_t)(1 << 25)  // 100 0000
    #define EVENT_PROG_EV               (uint32_t)(1 << 26)
#endif
#ifdef C10_Compteur
   #define NODE_Fin_timer_compteur      (uint32_t)(1 << 25)
#endif

#ifdef TACHE_NODE_WDT
    #define EVENT_TACHE_NODE_WDT        (uint32_t)(1 << 27)
#endif
#define NODE_EVENT_MESURE_CHARGE_BATT   (uint32_t)(1 << 28)
#define EVENT_NODE_EXTINCT_ESP          (uint32_t)(1 << 29)
#define NODE_ESP32                      (uint32_t)(1 << 30)
#define NODE_MESS_LONG                  (uint32_t)(1 << 31)  // test

//#define NODE_EVENT_UPDATE_LCD           (uint32_t)(1 << 1)

// Codes erreur
    // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
    // Erreurs 20 a 7F : 4 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
    // Erreurs 80 a FF : tout le temps (Appli > 0xD0)
#define erreur_capteur      0x70
#define erreur_flash_pleine 0x71
#define timout_envoi_rf     0x72
#define erreur_flash_occupe 0x73
#define erreur_stack_overfl 0x74
#define erreur_esp_wlf      0x75
#define erreur_esp          0x76

/******************** Type declarations ******************/


/*********************** Variable declarations ******************/

// test

uint8_t Config_boutons;
uint8_t vers_hard;

uint8_t ram_tampon[GROUPES_CAR_MAX];
uint8_t index_ram_nb, index_ram_car, MS_init;
uint32_t timestamp_init;

// Timers
uint16_t timer_inter=0;
uint8_t timer_ev_inter=0;

#ifdef OPT_CAPTEUR
    struct Capteurs_R capt[NB_CAPTEURS];
#endif

uint8_t mode_principal, mode_periodique;
uint8_t mode_affich_enreg;
uint8_t mode_hexa_texte;
uint8_t stack_over;

// variables capteur temp-hygro
uint8_t temp_cpt;
uint16_t temp_last;
uint16_t Temp_period_U;  // period unitaire
uint8_t Temp_ecart_min;  // Ecart min pour mesurer temperature
uint8_t Temp_period_min;      // period min de prise de mesure
uint8_t Temp_groupes;    // individuel ou en groupes
int16_t G_OFFSET_T, G_OFFSET_H;
uint8_t forcage_envoi_capteur;

// variables envoi en groupe
uint8_t Groupes_nb;     // nb max de mesures envoyees en meme temps
uint16_t Groupes_temps; // Temps max de retention des donnees

//Periode Keep alive
uint8_t  Temps_interrog_maj;    // Nombre de fois 6 heures
uint16_t Temps_interrog_min;    // 6 heures
uint8_t  Temps_interrog_maj_cpt;

uint8_t cam_type, sd_file, download_file;
uint16_t m_long_periode_e;

uint8_t arret_radio;

// Variables temporelles
uint16_t compteur_maj_heure; // interrogation periodique : Nb d'heures avant interrogation
uint8_t compteur_PHLS;
uint8_t  cpt_pass_intero_conc;  // Compteur intero conc
uint32_t validite;       // validite pour mise a l'heure

// variables capteur luminosite
uint8_t lumi_cpt;
float lumi_last;

// variables capteur pression
uint8_t press_cpt;
uint32_t press_last;

uint16_t param1;

// variables capteur
#ifdef C10_Compteur    // definition variables
    uint32_t cpt_somme_cpt;  // compteur sommé depuis l'origine, par tick
    uint16_t conso_instant;  // conso instantanée en deci_litres/h
    uint32_t time_dernier_tick_10ms;  // dernier tick en 10ms
    uint8_t conso_valide;  // 0:pas d'info  1:valide
    uint32_t timer_debut_conso; // en 10ms
    uint8_t fuite;
    uint8_t nb_clock_cpt;
#endif

#ifdef C10_Cent_Meteo   // definition variables
    // Pluviometre : envoi à chaque heure, ou quand 255 est atteint
    uint32_t cpt_somme_pluv;  // compteur sommé de la dernière heure, par tick
    uint32_t cpt_somme_vent;  // compteur sommé de la dernière heure, par tick
    //uint32_t time_dernier_tick_100ms;  // dernier tick en 100ms
    //uint16_t time_conso_min;  // val min de la dernière heure entre deux tick (en 100ms)
#endif

#ifdef FLASH_EXTERNE
    //uint8_t timeout_envoi;
    extern struct Pile_Fl_S Pile_F[NB_PILES_F];
    uint8_t Valid_flash;
#endif

char unsigned index_node; // test
uint8_t nb_capteurs;
bool option_envoi_reg;
uint32_t code_tournant_telec;
uint16_t capt_ram_tournante[MAX_RAM_TOURNANTE];
uint8_t index_ram_tournante;

uint8_t hors_portee_timing;   // conserve le temps de la clock hors_portee


static Task_Params nodeTaskParams;
Task_Struct nodeTask;    /* not static so you can see in ROV */
static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;  /* not static so you can see in ROV */
static Event_Handle nodeEventHandle;

// Données capteur
uint8_t lecture_capteur;  // capteur à lire

uint8_t type_capteur;    // 'L' ou 'T' ou 'H'

#define CAPTEUR_CPU   // pour CPU sinon SCS
bool activation_capteur;    // actif / inactif
uint16_t period_capteur;    // en secondes   maximum 18 heures
uint8_t envoi_data_capteur; // 0: pas d'envoi, 1:envoi toues les 10, 2:envoi syst T, 3:envoi syst T+H
uint8_t dest_data_capteur;
uint16_t ValueBrut, Value2Brut;
float FloatValue, Float2Value;
uint32_t count = 0;  // used to control display output rate


uint16_t compteur_option_envoi;

#ifdef BATT_BMS
    uint16_t voltage_panneau_sol;
    uint16_t voltage_batt_prec;
    int16_t temp_precedent, temp_mesure;
    uint8_t batt_degrad;
    uint8_t batt_soc100;
    uint32_t batt_soh;    // viellissement 3300uAh à 0 util + calendaire (en 10nAh)
    uint32_t batt_soh_cal;  // cumul du vieillismeent calendaire en 10nAh
    uint32_t batt_soh_cycle;   // cumul du viellissement utilisation en 10nAh
    uint8_t  batt_soc2;  //soc1:direct  soc2:iteratif
    uint32_t batt_soc_int;
#endif
    uint8_t batt_soc1;

uint8_t eve_esp32;

uint8_t period_type_reg;
uint8_t forcage_envoi_rf;  // 0:n'enoie que si le node pas bloqué  1:n'envoie que si le blocage correspond au timing  2:envoie dans tous les cas

//uint32_t time_prec_sec;
//uint16_t time_prec_milli;
//uint8_t annul_suivant=0;

//uint8_t  buzzer_actif;

uint32_t secondes_mem;
uint8_t activ_led_reponse;

// Enregistrement en Flash par longueur fixe
#ifdef RAM_TAMPON_SIZE
    static uint16_t Ram_tampon_flash [RAM_TAMPON_SIZE];
    static uint16_t Ram_tampon_index;     // 0 à 256
    static uint16_t Flash_page_index;   // page à remplir 0 à 4095
    char unsigned index_r, index_lg;
    uint16_t * adresse_ram_index;
    uint16_t  adresse_page_index;
    uint16_t  adresse_page_index_fin;
    bool date_imp;
    static uint32_t Heure_debut;  // 1=1 minutes
#endif


// gestion de l'ESP32
#ifdef C10_CAMERA_PIR  //definition variables
    uint8_t OrCam;  // état pour machine d'état
    uint16_t OrCam_param1;
    uint8_t OrCam_param2, OrCam_param3, OrCam_essai;
    uint16_t OrCam_debut;
    uint8_t ESP32_etat, ESP32_statut;

    uint8_t camera_debug=0;
    uint32_t date_derniere_video, date_derniere_photo;
    uint8_t ecart_video[12], ecart_photo[30];

    uint8_t esp_mess_cptp;
    uint32_t esp_mess_tps;

    // niveaux de batterie :
    uint8_t esp_bat[5] =    {10, 20, 40, 60, 90}; // rien, mess, photo stock, photo, video, HQ
    uint8_t esp_pho[6] = { 70, 85,  88, 91, 96, 100};   //
    uint8_t esp_vid[6] = { 70,  70,  81, 86, 96, 100};   //100:40s  90:6min  86:1h  81:1j  73:1an
    //uint8_t esp_pho[6] = { 0, 20,  30, 60, 60, 100};   //
    //uint8_t esp_vid[6] = { 0,  0,  10, 30, 60, 100};   //

#endif

// Chaque Clock : 40 octets RAM et 80 octets en Flash

/* Clock pour mise à jour periodique de l'heure */
Clock_Struct MajHeureClock;     /* not static so you can see in ROV */
Clock_Handle MajHeureClockHandle;

 /* Timer 30s pour eviter d'avoir trop de detection radar */
 Clock_Struct DetectionClock;     /* not static so you can see in ROV */
 Clock_Handle TimerDetectionClockHandle;

/* Timer initial pour mesure du voltage batterie */
Clock_Struct TimerVoltageInitial;     /* not static so you can see in ROV */
//static Clock_Handle TimerVoltageInitialHandle;

#ifdef CAPTEUR_TEMP
    /* Clock for the tache Capteur Temp-hygro */
    Clock_Struct sensorTempClock;     /* not static so you can see in ROV */
    static Clock_Handle sensorTempClockHandle;
#endif

#ifdef CAPTEUR_LUMI
    /* Clock for the tache Capteur Lumi */
    Clock_Struct sensorLumiClock;     /* not static so you can see in ROV */
    static Clock_Handle sensorLumiClockHandle;
#endif

#ifdef CAPTEUR_PRESS
    /* Clock for the tache Capteur Pression */
    Clock_Struct sensorPressClock;     /* not static so you can see in ROV */
    static Clock_Handle sensorPressClockHandle;
#endif

#ifdef PERIODE_CAPTEUR
/* Clock for the tache Capteur Pluviometrie ou sonde humidite*/
    Clock_Struct sensorCapteurClock;     /* not static so you can see in ROV */
    static Clock_Handle sensorCapteurClockHandle;
#endif

#ifdef C10_Compteur    // definition timer
/* Clock for the tache compteur eau : timeout suite à arret tick */
    Clock_Struct sensorCapteurTimeout;     /* not static so you can see in ROV */
    static Clock_Handle sensorCapteurTimeoutHandle;
#endif

#ifdef RAM_TAMPON_SIZE
/* Clock 100ms pour envoyer des infos RAM */
Clock_Struct TimerRamClock;     /* not static so you can see in ROV */
static Clock_Handle TimerRamClockHandle;
#endif

#ifdef PILE_RF_FLASH
    /* Clock 100ms pour envoyer des infos Flash */
    Clock_Struct TimerFlashClock;     /* not static so you can see in ROV */
    Clock_Handle TimerFlashClockHandle;
#endif

// Timer pour sleep de l'Uart et ESP
#ifdef C10_CAMERA_PIR   // difinition timer
        Clock_Struct ESP32_sleepTimerClock;     /* not static so you can see in ROV */
        static Clock_Handle ESP32_sleepTimerClockHandle;
#endif

/* Clock pour test periodique pour envoyer messages stockés en flash */
Clock_Struct Test_mess_Clock;     /* not static so you can see in ROV */
static Clock_Handle Test_mess_ClockHandle;

#ifdef TEMPS_INTERROG_MIN
    /* Clock en heures pour keep-alive / interrogation concentrateur */
    Clock_Struct Inter_ConcClock;     /* not static so you can see in ROV */
    static Clock_Handle Inter_ConcClockHandle;
#endif

#ifdef OPT_CAPTEUR
/* Timer pour vidage tampon si temps d'envoi trop long */
Clock_Struct RetenClock;     /* not static so you can see in ROV */
static Clock_Handle RetenClockHandle;
#endif

/* Timer pour node hors portee du concentrateur */
Clock_Struct Hors_portee_Clock;     /* not static so you can see in ROV */
static Clock_Handle Hors_portee_timerHandle;


#ifdef CAPTEUR_TEMP
  /* Timer pour prise de mesure instantanée et envoi */
  Clock_Struct TimerTemp1mesure;     /* not static so you can see in ROV */
  static Clock_Handle TimerTemp1mesureHandle;
#endif

#ifdef C10_CAMERA_PIR   // definition Clock
  /* Timer pour mesure voltage batterie */
  Clock_Struct TimerBattmesure;     /* not static so you can see in ROV */
  //static Clock_Handle TimerBattmesureHandle;
#endif

#ifdef TACHE_NODE_WDT
  /* Clock pour eviter watchdog, si la tache node n'est pas activee */
  Clock_Struct WdtNodeClock;     /* not static so you can see in ROV */
  //static Clock_Handle WdtNodeClockHandle;
#endif

// Test : Clock pour envoyer message long
Clock_Struct TestMessLongClock;     /* not static so you can see in ROV */
static Clock_Handle TestMessLongClockHandle;

#ifdef C10_ElecVanne   // definition clocks
      Clock_Struct SORTIE_EV0_Clock;     /* not static so you can see in ROV */
      Clock_Struct SORTIE_EV1_Clock;     /* not static so you can see in ROV */
      Clock_Struct SORTIE_EV2_Clock;     /* not static so you can see in ROV */
      static Clock_Handle EV_prog_ClockHandle;
      Clock_Struct EV_prog_Clock;        /* not static so you can see in ROV */
      void EV_prog_Callback(TIMER_ARG pxTimer);
#endif

// Variables allumage spot solaire

#ifdef SPOT_TOR_PWM
  uint8_t Spot_allumage_jour;           // durée en 0,1s d'allumage
  uint16_t Spot_allumage_nuit;         // durée en s d'allumage
  uint8_t Spot_allumage_nuit_seuil;    // durée en 10mv de seuil : 10=>100mV
#endif

#ifdef C10_ElecVanne    // definition variables
     S_SortieEV SortieEV[NB_EVanne];
     uint8_t EV_prog_periode;
     uint8_t EV_prog_duree;
     uint8_t EV_prog_timer;
     uint8_t EV_prog_timer_rest;

#endif

/************** Extern variables ******************/

extern uint8_t i2c_valide, val_reset;
extern struct Radio_S Radio;
extern S_SortieTor SortieTor[];
extern uint8_t b0_appui, b1_appui;


#if BOUTON_CC == 3  // 3e bouton
    extern uint8_t b2_appui;
#endif

extern uint8_t periode_activ_RX;  // (mode 4,5), en seconde

// variables MPU

#ifdef CAPTEUR_MPU
extern uint8_t capteur_mpu_init;
extern uint8_t config_mpu;
#endif

extern  char unsigned  code_erreur;
extern  char unsigned err_donnee1, err_donnee2;

extern uint8_t phase_radio_dest;

extern struct RemoteNode knownNodes[];
extern struct Message_RF_Packet latestRxPacket;
extern struct RadioOperation currentRadioOperation;


extern UART_Handle hUart;
extern PIN_Handle ledPinHandle;         // Led
extern PIN_Handle buttonPinHandle;      // Bouttons

extern uint16_t volt_mesure_bas;
extern uint32_t prec_tick_voltage;

extern  EtatPwm    Pwm[NB_PWM];  // 1 ou 2


/***************** Local Prototypes *****************/

static void nodeTaskFunction(UArg arg0, UArg arg1);
static void packetReceivedCallback(struct Message_RF_Packet* packet, int8_t rssi);
void activation_mode_principal(uint8_t new);
void activation_mode_periodique(uint8_t new);

//static void updateLcd(void);
void MajHeureCallback(UArg arg0);
void sensorTempCallback(UArg arg0);
void sensorLumiCallback(UArg arg0);
void sensorPressCallback(UArg arg0);
void sensorCapteurCallback(UArg arg0);
void sensorCapteurTimeoutCallback(UArg arg0);
void TimerFlashCallback(UArg arg0);
void TimerRamCallback(UArg arg0);
void Test_mess_Callback(UArg arg0);
void Hors_portee_timer_Callback (UArg arg0);      // fin du timer pour renvoyer TX suite hors_portée
void Inter_ConcCallback(UArg arg0);
void RetenCallback(UArg arg0);
void ESP32_sleepTimerClockCallback(UArg arg0);

//void SORTIE_S_Callback(UArg arg0);
void SORTIE_PWM_Callback(UArg arg0);
void DetectionCallback(UArg arg0);
void TimerVoltageInitialCallback (UArg arg0);
void TimerTemp1mesureHandleCallback (UArg arg0);
void TimerBattmesureHandleCallback (UArg arg0);
void WdtNodeClockCallback(UArg arg0);

void TestMessLongClockCallback(UArg arg0);

void traitement_data_capteur(void);
void Envoi_data_capteur (void);
void envoi_ram_message_date ( uint16_t * adresse_ram , unsigned char lg_ram, unsigned char type);
void envoi_ram_message (void);
void envoi_flash_message_date( uint16_t debut_flash, uint16_t fin_flash, char unsigned type);
void envoi_flash_message_mpu (void);
void envoi_flash_message (void);
void Envoi_une_valeur(uint16_t valeur);
void ajout_ram_tournante (uint16_t Value_brute);
void envoi_ram_valeur (uint8_t ca);
void lecture_press(uint8_t flag, uint8_t *data, int16_t *temper, uint16_t *press);
void vidage_tampon(void);
void Calibration_T_hygro(void);
uint8_t check_long_index_flash(uint8_t longu);
void envoi_capt(char, char,  uint8_t * chaine);
void envoi_soc_soh();
void envoie_mesure_compteur_eau ( uint8_t type);  // 0:hexa  1:texte
void envoie_mesure_pluviometre ( uint8_t type);  // 0:hexa  1:texte
uint8_t mesure_temper_val (int16_t * temp_mes);

uint16_t extinction_esp32(void);  // duree en 5 secondes
void allumage_esp32(uint16_t secondes);
uint8_t calcul_pir_esp(void);
void err_esp_extinct ( uint8_t code);
void allumage_led_pwm(void);

uint16_t transf_duree_ev(uint8_t duree);
uint16_t transf_periode_ev(uint8_t duree);
void activ_evanne(uint8_t num, uint16_t duree);
uint32_t tps_batt(uint8_t batterie_niv, uint8_t type);

/***** Function definitions *****/

// Fonction qui fait planter le Cortex
void writeToAddress (uintptr_t *addr, int val)
{
    *(int *) addr = val;   // writeToAddress ( (void*) 19,4)
}


void NodeTask_init(void)
{

    StatutOps = 0b11;  // Bit0:PIR  Bit1:PIR+Cam
    #ifdef DEBUG_M
        StatutOps = 0b11;  // inactif
        camera_debug=2;    // 0:normal 1:PIR 2:PIR+cam absent 3:PIR+CAM
    #endif

    stack_over=0;
    arret_radio=0;

    uint8_t retc;
    for (retc=0; retc < PAR_VAR_MAX; retc++)  par8_var[retc]=0;
    for (retc=0; retc < PAR_VAR_MAX; retc++)  par16_var[retc]=0;

#ifdef OPT_CAPTEUR
      i2c_valide = 0;

      index_ram_nb = 0;
      index_ram_car = 0;
      mode_principal = MODE_PRINCIPAL;      //  mode temp, mouv, fft
      mode_periodique = MODE_PERIODIQUE;                  //  mode arret ou periodique
      mode_affich_enreg = MODE_AFFICH_ENREG;  // mode liaison serie, liaison RF, enreg flash
      mode_hexa_texte   = MODE_HEXA_TEXTE;    // mode hexa ou texte
#endif

      Config_boutons = CONFIG_BOUTONS;

// Initialisation variables capteurs
#ifdef CAPTEUR_TEMP
     temp_cpt = 0;
     temp_last = 0;
     Temp_actuelle = -100;  // -1°C
#endif


#ifdef CAPTEUR_LUMI
     lumi_cpt = 0;
     lumi_last = 1;
#endif

     #ifdef CAPTEUR_PRESS
     press_cpt = 0;
     press_last = 0;
#endif


#ifdef SPOT_TOR_PWM
  Spot_allumage_jour  = SPOT_ALLUMAGE_JOUR;           // durée en 0,1s d'allumage
  Spot_allumage_nuit = SPOT_ALLUMAGE_NUIT;         // durée en s d'allumage
  Spot_allumage_nuit_seuil = SPOT_ALLUMAGE_NUIT_SEUIL;    // durée en 10mv de seuil : 10=>100mV
#endif

#ifdef C10_sonde_HUmSol   // init variables
  par16_var[0] = TPS_ALLUM;
#endif

#ifdef C10_ElecVanne  // init variables
  par16_var[0] = TPS_ALLUM;
  par16_var[1] = DUREE_VANNE_BP;
  EV_prog_periode = 0;
  EV_prog_duree = 0;
  EV_prog_timer = 0;
  EV_prog_timer_rest = 0;

  #if PRGM_VANNE
    EV_prog_periode = PRGM_VANNE & 0xF;
    EV_prog_duree = (PRGM_VANNE>>4) & 0xF;
  #endif


#endif

    /* Create event used internally for Node state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);

    Clock_Params clkParams;
    Clock_Params_init (&clkParams);

    // Création Clock/Timer :  Clock:Periode et Timeout=1    Timer: Periode=0 et Timeout

    /* setup pour mise à jour initiale/periodique de l'heure */
    clkParams.period = 10 *(1000 * 1000 / Clock_tickPeriod);  // 10 secondes puis 1 heure
    clkParams.startFlag = FALSE;
    Clock_construct(&MajHeureClock, MajHeureCallback, 9 * (1000 * 1000 / Clock_tickPeriod), &clkParams);  // decalage 9 secondes
    MajHeureClockHandle = Clock_handle(&MajHeureClock);

    // setup initial pour timer detection radar
    clkParams.period = 0;  // 30 secondes
    clkParams.startFlag = FALSE;
    Clock_construct(&DetectionClock,DetectionCallback, 30 * (1000 * 1000 / Clock_tickPeriod), &clkParams);  // decalage 30 secondes
    TimerDetectionClockHandle = Clock_handle(&DetectionClock);


    /* setup Timer initial pour mesure du voltage batterie */
    clkParams.startFlag = TRUE;
    Clock_construct(&TimerVoltageInitial, TimerVoltageInitialCallback, 7 *(1000 * 1000 / Clock_tickPeriod), &clkParams);
    //TimerVoltageInitialHandle = Clock_handle(&TimerVoltageInitial);

  #ifdef CAPTEUR_TEMP
    // setup clock pour capteur temp-hygro
    clkParams.period = TEMP_PERIOD * 1000 * (1000 / Clock_tickPeriod);
    clkParams.startFlag = FALSE;
    Clock_construct(&sensorTempClock, sensorTempCallback, clkParams.period, &clkParams);
    sensorTempClockHandle = Clock_handle(&sensorTempClock);
    Calibration_T_hygro();
  #endif

   #ifdef CAPTEUR_LUMI
    // setup clock pour capteur luminosite
    clkParams.period = LUMI_PERIOD * 1000 * (1000 / Clock_tickPeriod);
    clkParams.startFlag = FALSE;
    Clock_construct(&sensorLumiClock, sensorLumiCallback, clkParams.period, &clkParams);
    sensorLumiClockHandle = Clock_handle(&sensorLumiClock);
  #endif

#ifdef CAPTEUR_PRESS
     // setup clock pour capteur pression
     clkParams.period = PRESS_PERIOD * 1000 * (1000 / Clock_tickPeriod);
     clkParams.startFlag = FALSE;
     Clock_construct(&sensorPressClock, sensorPressCallback, clkParams.period, &clkParams);
     sensorPressClockHandle = Clock_handle(&sensorPressClock);
#endif

#ifdef PERIODE_CAPTEUR
     // setup clock pour capteur
     clkParams.period = PERIODE_CAPTEUR * 1000 * (1000 / Clock_tickPeriod);  // en secondes
     clkParams.startFlag = FALSE;
     Clock_construct(&sensorCapteurClock, sensorCapteurCallback, clkParams.period, &clkParams);
     sensorCapteurClockHandle = Clock_handle(&sensorCapteurClock);
#endif

#ifdef C10_Compteur     // Init Timer pour timeout non detection tick suivant
     clkParams.period = 0;
     clkParams.startFlag = FALSE;
     Clock_construct(&sensorCapteurTimeout, sensorCapteurTimeoutCallback, 1, &clkParams);
     sensorCapteurTimeoutHandle = Clock_handle(&sensorCapteurTimeout);

#endif

#ifdef RAM_TAMPON_SIZE
    // setup test clock pour envoi Ram
    clkParams.period = RADIO_PROTOCOLE * 100 * (1000 / Clock_tickPeriod);   // 100ms
    clkParams.startFlag = FALSE;
    Clock_construct(&TimerRamClock, TimerRamCallback, 1, &clkParams);
    TimerRamClockHandle = Clock_handle(&TimerRamClock);
#endif

#ifdef PILE_RF_FLASH
    // setup test clock pour envoi Flash
    clkParams.period = RADIO_PROTOCOLE * 50 * (1000 / Clock_tickPeriod);   // 625bps:3sec  5kbps:300ms   50kbps:30ms
    clkParams.startFlag = FALSE;
    Clock_construct(&TimerFlashClock, TimerFlashCallback, clkParams.period, &clkParams);
    TimerFlashClockHandle = Clock_handle(&TimerFlashClock);

#endif

    /* setup timer repetition suite hors portee*/
    clkParams.period = 60 * 1000 * (1000 / Clock_tickPeriod);
    clkParams.startFlag = FALSE;
    Clock_construct(&Hors_portee_Clock, Hors_portee_timer_Callback, 1, &clkParams);
    Hors_portee_timerHandle = Clock_handle(&Hors_portee_Clock);
    hors_portee_timing=0;   // conserve le temps de la clock hors_portee

    #ifdef TEMPS_INTERROG_MIN
        // setup clock pour keep-alive / interrogation concentrateur (demande message en attente)
        clkParams.period =  ((uint32_t)TEMPS_INTERROG_MIN  * 1000 *(1000 / Clock_tickPeriod));
        clkParams.startFlag = FALSE;
        Clock_construct(&Inter_ConcClock, Inter_ConcCallback, clkParams.period, &clkParams);
        Inter_ConcClockHandle = Clock_handle(&Inter_ConcClock);
    #endif

#ifdef OPT_CAPTEUR
    // setup clock pour envoi donnees si temps de retention trop long
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&RetenClock, RetenCallback, 1, &clkParams);
    RetenClockHandle = Clock_handle(&RetenClock);
#endif

    // setup clock pour test de verif dispo passerelle, si message flash en attente
    clkParams.period = TEST_ENVOI_MESSAGES * 1000 * (1000 / Clock_tickPeriod);
    clkParams.startFlag = FALSE;
    Clock_construct(&Test_mess_Clock, Test_mess_Callback, 1000 * (1000 / Clock_tickPeriod), &clkParams);
    Test_mess_ClockHandle = Clock_handle(&Test_mess_Clock);


   #ifdef    CAPTEUR_TEMP
    // setup Timer pour prise de mesure instantanée de température et envoi
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&TimerTemp1mesure, TimerTemp1mesureHandleCallback,1, &clkParams);
    TimerTemp1mesureHandle = Clock_handle(&TimerTemp1mesure);
  #endif

   /* Clock pour eviter watchdog, si la tache node n'est pas activee */
   #ifdef TACHE_NODE_WDT
    clkParams.period = TACHE_NODE_WDT * 60 * 1000 * (1000 / Clock_tickPeriod);
    clkParams.startFlag = TRUE;
    Clock_construct(&WdtNodeClock, WdtNodeClockCallback,clkParams.period, &clkParams);
    //WdtNodeClockHandle = Clock_handle(&WdtNodeClock);
   #endif

  // Timer pour sleep de l'esp
  #ifdef C10_CAMERA_PIR  // init timer
        // setup Timer pour prise de mesure instantanée de température et envoi
        clkParams.period = 0;
        clkParams.startFlag = FALSE;
        Clock_construct(&ESP32_sleepTimerClock, ESP32_sleepTimerClockCallback,1, &clkParams);
        ESP32_sleepTimerClockHandle = Clock_handle(&ESP32_sleepTimerClock);

  #endif

#ifdef BATT_BMS
    // setup Timer pour mesure charge batterie
    clkParams.period = BATT_MES_PERIODE * 60 * 1000 * (1000 / Clock_tickPeriod);
    clkParams.startFlag = TRUE;
    Clock_construct(&TimerBattmesure, TimerBattmesureHandleCallback,clkParams.period, &clkParams);
    //TimerBattmesureHandle = Clock_handle(&TimerBattmesure);

    batt_soh = BATT_SOC0;
    batt_soh_cal=0;
    batt_soh_cycle=0;
    batt_degrad=0;
    temp_mesure = 15;
    temp_precedent = 15;
#endif

#ifdef C10_ElecVanne   // init clocks
    clkParams.arg = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&SORTIE_EV0_Clock, SORTIE_EV_Callback, 1, &clkParams);
    SortieEV[0].h_clock = Clock_handle(&SORTIE_EV0_Clock);
    #if NB_EVanne >1
        clkParams.arg = 1;
        Clock_construct(&SORTIE_EV1_Clock, SORTIE_EV_Callback, 1, &clkParams);
        SortieEV[1].h_clock = Clock_handle(&SORTIE_EV1_Clock);
    #endif
    #if NB_EVanne >2
        clkParams.arg = 2;
        Clock_construct(&SORTIE_EV2_Clock, SORTIE_EV_Callback, 1, &clkParams);
        SortieEV[2].h_clock = Clock_handle(&SORTIE_EV2_Clock);
    #endif

    clkParams.arg = 0;
    Clock_construct(&EV_prog_Clock, EV_prog_Callback, 1, &clkParams);
    EV_prog_ClockHandle = Clock_handle(&EV_prog_Clock);

#endif


    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    nodeTaskParams.instance->name = "Node";
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);

    forcage_envoi_rf=0;
    code_tournant_telec = 0xD4A6A59B + (My_Address << 16) + My_Address;  // code tournant initial


    #ifdef MICROPHONE
             PDMCC26XX_init();
    #endif

 // setup Timer pour test d'envoi message long
 clkParams.period =  5 * 1000 * (1000 / Clock_tickPeriod);  // 5 secondes
 clkParams.startFlag = FALSE;
 Clock_construct(&TestMessLongClock, TestMessLongClockCallback,clkParams.period, &clkParams);
 TestMessLongClockHandle = Clock_handle(&TestMessLongClock);


}



//chgt mode a chaque appui long sur bouton 0:liaison série -> 1:envoi RF -> 2enregistrement Flash
// gestion mode veille/off/on du capteur MPU
void increment_mode_affich_enreg(void)  // bouton 0 double
{
    mode_affich_enreg++ ;
    if (mode_affich_enreg > 2) mode_affich_enreg = 0;
    #ifndef MODE_SILENCE
    if (!mode_affich_enreg)
        ACTIV_PWM ( 0,1, 10, 1500,3);  // 1 bip long
    else
        ACTIV_PWM ( 0,4, 6*mode_affich_enreg, 1500, 3);  // 1-2 bip court
    #endif
    //ACTIV_SORTIE (LED_cpu, 2 , 6*(mode_affich_enreg+1));
}

// hexa (0) <-> texte (1)
void increment_mode_envoi_mpu(void)  // bouton 0 long
{
    mode_hexa_texte++ ;
    if (mode_hexa_texte > 1) mode_hexa_texte = 0;
    ACTIV_SORTIE(LED_cpu, 2 , 6*(mode_hexa_texte+1));
}

void Task_message_recu(void)
{
    Event_post(nodeEventHandle, EVENT_RECEP_MESSAGE_UART);
}

void Task_envoie_message(void)
{
    Event_post(nodeEventHandle, EVENT_SEND_MESSAGE_UART);
}

// envoi les messages RF qui sont dans le Buffer TX
void mess_RF_dans_bufferTX()
{
    Event_post(nodeEventHandle, EVENT_MESSAGE_RF_A_ENVOYER);
}

void appui_boutton_0(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_BOUTTON_0);
}

void appui_boutton_1(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_BOUTTON_1);
}

void appui_boutton_2(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_BOUTTON_2);
}

void appui_2bouttons(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_2BOUTTONS_APPUI);
}

void event_action_periodique(uint8_t type_action_periodique)
{
    period_type_reg = type_action_periodique;
    Event_post(nodeEventHandle, EVENT_ACTION_PERIODIQUE);
}


// modifie : Periode_balise_Ecoute_RX, periode envoie donne capteur, periode keepalive(niveau pile)

void chgt_mode_arret( uint8_t nouv)   // 0:marche  1:reduit1  2:reduit2  3:reduit3  4:arret total
{
    #ifdef C10_sonde_HUmSol   // mode hiver
        if (!nouv)   // full operationnel
        {
            // periode nominale balise ecoute, si mode_rx=5
            if (MODE_RX==5)
                chgt_mode_radio (2, 5, PERIODE_ACTIV_RX);
            CLOCK_RESTART_PERIOD(sensorCapteurClockHandle, PERIODE_CAPTEUR*1000);  // timer nominal de mesure sonde
            modif_keep_alive(1, TEMPS_INTERROG_MAJ);
        }
        else if (nouv == 3)   //  reduit 3 : mode hiver
        {
            // diminution periode balise ecoute, si mode_rx=5 => 1 minute => 0,75uA
            // diminution intervalle keepalive (*7)
            if (MODE_RX==5)
                chgt_mode_radio (2, 5, 60);
            CLOCK_RESTART_PERIOD(sensorCapteurClockHandle, 3600*1000);  // mesure chaque 24heures pour verif si en terre
            modif_keep_alive(1, TEMPS_INTERROG_MAJ*7);
        }
        else if (nouv == 4)   //  arret total
        {
            // arret balise ecoute, si mode_rx=5
            if (MODE_RX==5)
                chgt_mode_radio (2, 6, PERIODE_ACTIV_RX);
            CLOCK_STOP(sensorCapteurClockHandle);  // arret envoi mesure capteur
            modif_keep_alive(0, TEMPS_INTERROG_MAJ);
        }
    #endif

    #ifdef C10_Compteur   // mode arret
        if (!nouv)   // full operationnel
        {
            // periode nominale balise ecoute, si mode_rx=5
            if (MODE_RX==5)
                chgt_mode_radio (2, 5, PERIODE_ACTIV_RX);
            modif_keep_alive(1, TEMPS_INTERROG_MAJ);
        }
        else if (nouv == 3)   //  reduit 3 : mode hiver
        {
            // diminution periode balise ecoute, si mode_rx=5 => 1 minute => 0,75uA
            // diminution intervalle keepalive (*7)
            if (MODE_RX==5)
                chgt_mode_radio (2, 5, 60);
            modif_keep_alive(1, TEMPS_INTERROG_MAJ*7);
        }
        else if (nouv == 4)   //  arret total
        {
            // arret balise ecoute, si mode_rx=5
            if (MODE_RX==5)
                chgt_mode_radio (2, 6, PERIODE_ACTIV_RX);
            modif_keep_alive(0, TEMPS_INTERROG_MAJ);  // arret keepalive
        }
    #endif

    mode_arret = nouv;
}

// type : 0:arret  1:marche
//periode MAJ :
void modif_keep_alive (uint8_t type, uint8_t periode)
{
    Clock_stop(Inter_ConcClockHandle);
    Temps_interrog_min = TEMPS_INTERROG_MIN;
    Temps_interrog_maj = periode;
    Temps_interrog_maj_cpt = Temps_interrog_maj;
    Clock_setPeriod (Inter_ConcClockHandle, (uint32_t) Temps_interrog_min * (1000  / Clock_tickPeriod * 1000));
    Clock_setTimeout (Inter_ConcClockHandle, (uint32_t) Temps_interrog_min * (1000  / Clock_tickPeriod * 1000));
    if (type)
        Clock_start(Inter_ConcClockHandle);
}


#ifdef OPT_CAPTEUR

// return 0:ok  1:err
uint8_t lecture_temphdc1000(void)
{
    uint8_t ret = 1;
    uint8_t delai_temp;
    if (!i2c_valide)
        SensorI2C_open();
    SensorBmp280_enable(false);  // sans l'activation du capteur de pression, il y a une erreur i2c
    SensorHdc1000_start();

    // delai d'attente plus important à basse temperature
    delai_temp=15;
    if (Temp_actuelle<1000) delai_temp = 20;
    if (Temp_actuelle<0)    delai_temp = 30;
    DELAY_MS(delai_temp);

    ValueBrut=1;
    Value2Brut=1;
    SensorHdc1000_read(&ValueBrut, &Value2Brut);  // Temp et hygro
    SensorHdc1000_convert ( ValueBrut, Value2Brut,  &FloatValue, &Float2Value);
    FloatValue  += G_OFFSET_T/100;
    Float2Value += G_OFFSET_H/100;

    if (ValueBrut == 1)  // mauvaise lecture
    {
        ValueBrut =  21845;
        Value2Brut = 32768;
        FloatValue  = 15;  // Temp par défaut  : 15 degrés
        Float2Value = 50;  // Hygro par défaut : 50%
    }
    else
    {
        Temp_actuelle = FloatValue*100;
        ret = 0;
    }
    SensorI2C_close();
    return ret;
}

void lecture_lumi(uint8_t flag)
{
    if (!i2c_valide)
        SensorI2C_open();
    SensorOpt3001_enable(true);
    DELAY_MS(120);
    SensorOpt3001_read (&ValueBrut);
    if (flag)
        FloatValue = SensorOpt3001_convert  ( ValueBrut);
    //SensorOpt3001_enable(false);
    SensorI2C_close();
}

void lecture_press(uint8_t flag, uint8_t *data, int16_t *temper, uint16_t *press)
{
    int32_t temper32=1500;
    uint32_t press32=100000;

    if (!i2c_valide)
        SensorI2C_open();
    SensorBmp280_enable(true);
    DELAY_MS(120);
    SensorBmp280_read (data);
    if (flag)
        SensorBmp280_convert  ( data, &temper32, &press32);
    *temper = temper32;
    *press  = press32 - 50000;
    //SensorBmp280_enable(false);
    SensorI2C_close();
}

/*********************************************************************
 * @fn      configureRelay
 *
 * @brief   Re-configure IO-pin if Hall Effect Sensor detected.
 *
 * @descr   SensorTags with PCB revision 1.5.0 and higher are equipped
 *          with a Hall Effect Sensor instead of a Reed Relay. The application
 *          needs to take this into account because the polarity of Hall
 *          Effect Sensor is active low, whereas the Reed Relay is active high.
 *
 * @return  none
 */

/*static void configureRelay(void)
{
  uint8_t value = PIN_getInputValue(CC1350STK_RELAY);  // Board_RELAY);

  if (value > 0) // Pin is high, assume Hall Effect Sensor
  {
    PIN_Config pinCfg;
    PIN_Config pinCfgBm;

    // Reconfigure as pull-up + inverted output. Turn off hysteresis.
    pinCfg = PIN_ID(CC1350STK_RELAY) | PIN_PULLUP | PIN_INV_INOUT;
    pinCfgBm = PIN_BM_PULLING | PIN_BM_INV_INOUT | PIN_BM_HYSTERESIS;

    //PIN_setConfig(hGpioPin, pinCfgBm, pinCfg);
  }
  // else: Pin is low. Assume Reed Relay - no reconfiguration required
}*/


char sensor_init()
{
    char unsigned ret;

    ret = 0;
    //configureRelay();  // raz Hall effect sensor


    if (SensorI2C_open())
    {
      #ifdef C10_CAMERA_PIR  //  initialisation capteur HDC1000
        if (!SensorHdc1000_init())
            ret++  ;          // Humidity  Adresse=0x43

      #else // sensortag CC1350
        // Put unused external sensors and flash into Sleep
        if ( !SensorBmp280_init())
            ret++;            // Pressure Sensor
        SensorBmp280_enable(false);

        // Sensor infrared temp (uniquement sur v4)
        #if VERS_HARD == 4
            // Infrared Thermopile Sensor
            if ( !SensorTmp007_enable(false) )
                ret++;
        #endif

        // Init Light sensor
        SensorOpt3001_init();          // Adresse=0x45
        SensorOpt3001_enable(false);

        // Temp & humidity sensor
        if (!SensorHdc1000_init())
            ret++  ;          // Humidity  Adresse=0x43

        //SensorI2C_close();   // test
        //SensorI2C_open();
        /*SensorHdc1000_start();
        DELAY_MS(15);
        SensorHdc1000_read(&ValueBrut, &Value2Brut);
        SensorHdc1000_convert ( ValueBrut, Value2Brut,  &FloatValue, &Float2Value);*/

        // init capteur MPU
        SensorMpu9250_init();   // 223ms
        #ifdef CAPTEUR_MPU
                init_capteur_mpu();  // 407 ms
        #endif
        sensorMpuSleep();
        SensorMpu9250_powerOff();
      #endif
    }
    else ret=9;
    SensorI2C_close();

    #ifdef FLASH_EXTERNE
        if ( !ExtFlash_open()  )
            ret=ret+0x10;   // pb SPI
        ExtFlash_close();
        if (ret&0x10)
        {
            if ( !ExtFlash_open()  )
                ret=ret+0x10;   // pb SPI
            ExtFlash_close();
        }
    #endif


return ret;
}
#endif


static void nodeTaskFunction(UArg arg0, UArg arg1)
{
    uint8_t retc;

    message_test[0]=0;
    message_test[1]=0;

    /* Register a packet received callback with the radio task */
    RadioTask_registerPacketReceivedCallback(packetReceivedCallback);

    addNewNode(RADIO_CONCENTRATOR_ADDRESS);

    DELAY_MS(1000);

#ifdef C10_CAMERA_PIR  // init variables
    uint8_t k;
    for (k=0; k<12; k++)  ecart_video[k]=0;
    for (k=0; k<12; k++)  ecart_photo[k]=0;
    m_long_periode_e = 80;  // 15=150ms
    OrCam=0;
    OrCam_essai=0;
    ESP32_etat=0;
    ESP32_statut=0;;
    esp_mess_cptp=0;
    esp_mess_tps=0;

#endif

  // mesure voltage initial et batt_soc1 initial
  uint16_t volt_mesure;
  volt_mesure = mesure_voltage();
  //temp_mesure = mesure_temperature();
  //batt_soc1 = mesure_niv_batt(volt_mesure, temp_mesure);

  // volt=844/3300:100%   Volt=588/2300:0%
  if (volt_mesure < 2300)
      batt_soc1 = 0;
  else
      batt_soc1 = (volt_mesure - 2300) / 10;
  if (batt_soc1>100) batt_soc1=100;

#ifdef BATT_BMS
    batt_soc1 = conv_volt_batt (volt_mesure, 25);
    voltage_batt_prec = volt_mesure;
    voltage_panneau_sol = volt_mesure;

    batt_soc_int = (BATT_SOC0 / 100) * batt_soc1;
    batt_soc2 = batt_soc1;

    if (batt_soc1 > 98)    // Arret a 4,17V
        ACTIV_SORTIE( Charge_TP, 0, 0 );  // batterie full => desactivation du circuit de charge
#endif

    if (batt_soc1 < Niv_batt_tres_bas)  // arret radio
    {
        arret_radio=1;
    }

#ifdef CAPTEUR_MPU
    capteur_mpu_init=0;
#endif

    strcpy((char *)message, "LSinit- :  - ");  // init
    #ifdef CC1350_carte
        message[7]= VERS_HARD +'0';
    #endif
        Mess_statut = 0x10;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé

#ifdef OPT_CAPTEUR

    retc = sensor_init();
    message[9]= (retc >> 4) +'0';
    message[10]= (retc & 0xF) +'0';

    DELAY_MS(1000);

#ifdef FLASH_EXTERNE
    // lecture index_flash (index relatif) en EEPROM des sections 0 et 1
    uint8_t buff_r[8], i, j;
    Valid_flash=1;
    if ((retc & 0xF0) == 0x20)
        Valid_flash=0;
    else
    {
        EEPROM_init();
        i=0;
        j=0;
        for (i=0; i<NB_PILES_F; i++)
        {
            EEPROM_read(8*i, buff_r, 8);
            memcpy (&Pile_F[i].head, buff_r, 4);
            memcpy (&Pile_F[i].tail, buff_r+4, 4);
            if (((Pile_F[i].head>>12) >= Pile_F[i].taille) || \
            ((Pile_F[i].tail>>12) >= Pile_F[i].taille<<12))

            /*uint8_t sec_head, sec_tail;
            sec_head = Pile_F[i].head >> 12;
            sec_tail = Pile_F[i].tail >> 12;
            if ( (sec_head >= Pile_F[i].taille) || (sec_tail >= Pile_F[i].taille) \
                 || (Pile_F[i].head & 1) || (Pile_F[i].tail & 1))*/
            {
                Pile_F[i].head = 0;
                Pile_F[i].tail = 0;
                j=1;
            }
        }
        if (j)
            sauve_eeprom_index_flash();
    }
#endif
#endif

    //DELAY_MS(4000);

    // Si batterie trop faible ou reset=Brownout
    if (((batt_soc1 < 10) && (batt_soc1)) || ((val_reset>=2) && (val_reset<=4)))
    {
        DELAY_MS(60000); // attente 1 minute
    }
    else
    {
        #ifdef DEBUG_M
            ACTIV_SORTIE(LED_cpu, 1, 20);  // Clignotement led à l'initialisation
        #else
            #ifndef NO_LED0
                ACTIV_SORTIE(LED_cpu, 2, 10);  // Clignotement led à l'initialisation
            #endif
        #endif
    }
    message[12]=val_reset+'0';
    if (batt_soc1>=Niv_batt_tres_bas)
        envoi_message (); // apres lecture des index flash


    option_envoi_reg = 0;
    #ifdef OPTION_ENVOI_REG
      option_envoi_reg = 1;
    #endif
    dest_data_capteur = 'L';
    envoi_data_capteur = 0;

    #ifdef OPT_CAPTEUR
        activation_capteur= ACTIV_CAPTEUR;

        // variables envoi en groupe
        Groupes_nb = GROUPES_NB_MAX;     // nb max de mesures envoyees en meme temps
        Groupes_temps = GROUPES_TEMPS_MAX; // Temps max de retention des donnees
    #endif



// initialisation capteur temperature/hygrometrie
#ifdef CAPTEUR_TEMP
    Temp_period_U = TEMP_PERIOD;  // period unitaire
    Temp_ecart_min = TEMP_ECART_MIN;  // Ecart min pour mesurer temperature
    Temp_period_min = TEMP_PERIOD_MIN;      // period min de prise de mesure
    Temp_groupes = TEMP_GROUPES;    // individuel ou en groupes
#endif

#ifdef C10_sonde_HUmSol   // demarrage clock 1 heure
    CLOCK_START(sensorCapteurClockHandle);  // timer 1 heure
#endif

#ifdef C10_Compteur   // Init variables
    cpt_somme_cpt = 0;
    conso_instant = 0;  // conso instantanée en litres/h
    time_dernier_tick_10ms = 0;
    timer_debut_conso = 0;
    fuite=0;
    nb_clock_cpt =0;

    /*struct timeval tv;  // microsecondes
    tv = NB_MICROSEC();  // seconds et microsecondes
    time_dernier_tick_10ms =  (tv.tv_sec % (3600*24*365) )*100 + tv.tv_usec/10000;  // raz au 1er janvier   2530 = 25,30sec*/
#endif

#ifdef C10_Cent_Meteo   // init variables
    // Pluviometre : envoi à chaque heure
    cpt_somme_pluv = 0;
    cpt_somme_vent = 0;
    //time_conso_min = 0;
    //time_dernier_tick_100ms=0;
    CLOCK_START(sensorCapteurClockHandle);  // timer 1 heure
#endif

    forcage_envoi_capteur=0;

    activ_led_reponse=0;

    // mise a l'heure initiale
    struct tm sTime = {0};
    sTime.tm_mday =   1;  // 1er
    sTime.tm_mon =   0;  // janvier
    sTime.tm_year =   20 + 100;  // since 1900
    time_t seconds = mktime(&sTime);  // 01/01/2020 = 3755203200 sec = 43464 jours
    Seconds_set(seconds);
    strcpy((char *)message, "QHLS");  // Demande date et heure au concentrateur
    Mess_statut = 0x22;  // 0x24  bit1-2:reenvoi(00:non, 01:1 fois, 10:5 fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
    validite = Seconds_get();
    envoi_message ();
    compteur_PHLS=3;
    compteur_maj_heure = CPT_MAJ_HEURE;    // interrogation periodique
    Clock_start(MajHeureClockHandle);  // Démarrage de la maj periodique heure QHLS


    //  Keep-alive / interrogation concentrateur
    #ifdef TEMPS_INTERROG_MIN
        Temps_interrog_maj = TEMPS_INTERROG_MAJ;
        Temps_interrog_min = TEMPS_INTERROG_MIN;
        Temps_interrog_maj_cpt = Temps_interrog_maj; // valeur initiale
        cpt_pass_intero_conc = PERIOD_PASS_INTERO_CONC; // Interrogation pour message concentrateur
        if (TEMPS_INTERROG_MIN)
            Clock_start(Inter_ConcClockHandle);
    #endif

    // Demarrage clock pour test periodique de presence passerelle (toutes les 10 secondes) : pour dechargement mémoire
    if (TEST_PRESENCE_CONCENTRATEUR)
        Clock_start(Test_mess_ClockHandle);


    if (mode_periodique)  // activation a l'initialisation de la température. Le mouv et FFt se fait dans la tache sensormpu
    {
        if (mode_principal == 0) // activation clock temp
        {
            #ifdef CAPTEUR_TEMP
                    Clock_start(sensorTempClockHandle);
            #endif
            #ifdef CAPTEUR_PRESS
                if (PRESS_ACTIF)
                    Clock_start(sensorPressClockHandle);
            #endif
            #ifdef CAPTEUR_LUMI
                if (LUMI_ACTIF)
                    Clock_start(sensorLumiClockHandle);
            #endif
        }

    }
    activation_mode_periodique(mode_periodique); // 260ms = 825ms -> 1085ms
    activation_mode_principal(mode_principal);

    #ifdef C10_ElecVanne  // demarrage programme à l'initialisation => 6 heures apres init
        if ((EV_prog_periode) && (EV_prog_duree))
        {
            EV_prog_timer = transf_periode_ev(EV_prog_periode);
            EV_prog_timer_rest = 1;
            Clock_setPeriod(EV_prog_ClockHandle, (uint32_t)  6*3600 * (1000 / Clock_tickPeriod) * 1000); // 6 heures
            Clock_setTimeout(EV_prog_ClockHandle, (uint32_t)  6*3600 * (1000 / Clock_tickPeriod) * 1000); // 6 heures
            Clock_start(EV_prog_ClockHandle);
            //uint16_t duree_sec;   // pas de demarrage immediat
            //duree_sec = transf_duree_ev(EV_prog_duree);
            //activ_evanne (0, duree_sec);
        }
    #endif


    while (1)
    {
        #ifdef WATCHDOG
            watchdog_tache[0]=0; // tache asleep
            Watchdog_clear(watchdogHandle);
        #endif


        // Wait for event
        // plusieurs events peuvent être activés en meme temps (pas ordonnés, le meme evenement n'est actif qu'une fois)
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);


        #ifdef WATCHDOG
            watchdog_tache[0]=1; // tache active
            watchdog_cpt_activ[0]++;   // compteur d'activation
            watchdog_cpt_sleep[0]=0;   // compteur de sleep
        #endif

        #if (DEBUG_TIME&0b11) == 0b01
        if ((trigger_time) && (index_time <MAX_Index))
        {
            etat_cpu[index_time] = 70;
            etat_detail[index_time] = events ;
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
        }
        #endif


           #ifdef PILE_RF_FLASH
           // Teste periodiquement si la passerelle est dispo, si message en attente
           if (events & EVENT_TEST_PASSERELLE_DISPO)
           {
               if ((Pile_F[0].tail != Pile_F[0].head))
                       demarr_lect_flash();
           }
           #endif

        #ifdef Uart2
            /* Message reçu, à traiter */
            if (events & EVENT_RECEP_MESSAGE_UART)
            {
                reception_message_Uart2();
            }
            if (events & EVENT_SEND_MESSAGE_UART)
            {
                transmission_message_Uart2();
            }
        #endif

        if (events & EVENT_FIN_ENVOI_RF)  // Fin (echec ou succes de l'envoi RF)
        {
        }

        // Si le timer hors de portee => renvoi les messages
        if (events & NODE_EVENT_HORS_PORTEE)
        {
            // trouve 1er message à envoyer
            retc = trouve_mess_RF_envoyer(phase_radio_dest, 1);
            // si ok, envoie ce message
            if (retc)
            {
                //currentRadioOperation.statut = 1;
                Radio_debut_tx();
            }
            else  // revient en mode RX
            {

            }
        }


        /* Si demande de message a envoyer (en attente ou immediat) */
        if (events & EVENT_MESSAGE_RF_A_ENVOYER)
        {
            Radio_debut_tx();
        }

        // message à traiter
        if (events & NODE_EVENT_SIMPLE_MESSAGE)
        {
            traitement_rx(latestRxPacket.length);
        }


        if (events & EVENT_ACTION_PERIODIQUE)
        {
            if (period_type_reg == 2)  // Envoi message pour test RSSI
            {
                strcpy((char *)message, "QRLRxL");  // Envoi au PC le niveau de réception du concentrateur RSSI
                envoi_message ();
            }
            if (period_type_reg == 1)  // Envoi température au PC en clair
            {
                #ifdef CAPTEUR_TEMP
                lecture_temphdc1000();
                strcpy((char *)message, "LCAP:");
                strcpy((char *)message+5, "Temp: xx.xx");
                ajout_float_message(FloatValue, 2, 10,0);
                envoi_message ();
                #endif
            }
        }

        #ifdef RAM_TAMPON_FLASH
        if (events & EVENT_RAM_A_ENVOYER)
        {
            envoi_ram_message();
        }
        #endif

        #ifdef PILE_RF_FLASH
        if (events & EVENT_FLASH_A_ENVOYER)
        {
            envoi_flash_message_mpu();
        }
        #endif

        #ifdef BATT_BMS
        if (events & NODE_EVENT_MESURE_CHARGE_BATT)       // ---------  LECTURE CHARGE BATTERIE chaque 10 minutes
        {
            // mesure voltage batterie, panneau et temperature

            // mesure temperature interne et validation mesure
            uint8_t temp_valide = mesure_temper_val(&temp_mesure);

            // mesure voltage batterie et panneau solaire
            uint16_t voltage_batt;
            uint16_t voltage_panneau;
            uint8_t ret = lecture_volt_batterie (&voltage_batt, &voltage_panneau);

            int16_t ecart_voltage;
            ecart_voltage = voltage_batt -voltage_batt_prec;
            if (ecart_voltage<0) ecart_voltage = -ecart_voltage;

            #ifdef DEBUG_M
                if (ecart_voltage > 100)     // si variation de plus de 10% de Soc de la batterie (OCV) (0,1V)
                {
                    strcpy((char *)message, "LErrVolB:");
                    ajout_int16_message(voltage_batt, 9, 0);
                    ajout_int16_message(voltage_batt_prec, 15, 0);
                    envoi_message();
                }
            #endif

            // correction voltage_batterie par la temperature
            // relation Soc-OCV-temp : reference 25°C
            // à 0°C  : >3,9V:id  3,9V à 3,5V:-6%  <3,5V:-10%
            // à 10°C : >3,9V:id  3,9V à 3,5V:-3%  <3,5V:-5%
            uint16_t voltage_batt_cor = voltage_batt;
            uint16_t voltage_batt_prec_cor = voltage_batt_prec;

            // relation Soc100 en fonction de température : ref 25°C
            // 0,5% par degré °C
            // 10°C : -7%
            // 0°C  : -12%
            // -10°C : -18%

            batt_soc100 = 100;
            if (temp_mesure < 25)
            {
                batt_soc100 = 88 - temp_mesure/2;  // 100 - (25-Temp)*0,5
                if (voltage_batt < 3500)  // (25-Temp)*5/12
                {
                    voltage_batt_cor += ((voltage_batt/10) * (25-temp_mesure))/24;
                }
                else if (voltage_batt < 3900)
                    voltage_batt_cor += ((voltage_batt/12) * (25-temp_mesure))/12;

                if (voltage_batt_prec < 3500)  // (25-Temp)*5/12
                    voltage_batt_prec_cor += ((voltage_batt_prec/10) * (25-temp_mesure))/24;
                else if (voltage_batt_prec < 3900)
                    voltage_batt_prec_cor += ((voltage_batt_prec/12) * (25-temp_mesure))/12;
            }

            // régulation de la charge, composant TP4056
            // IDEAL : SOC73% à 3,98V  3,9V=SOC65% => 4000cycles
            // hysteresis de 50mv = 6% de SOC = 200mAh

            if ((voltage_batt_cor > SEUIL_ARRET_CHARGE) && (voltage_batt_prec_cor > SEUIL_ARRET_CHARGE))   // arret de la charge si Vbatt >3,9V (70% Soc)
            {
                ACTIV_SORTIE( Charge_TP, 0, 0 );
            }
            if ((voltage_batt_cor < (SEUIL_ARRET_CHARGE-200 )) && (voltage_batt_prec_cor < (SEUIL_ARRET_CHARGE-200)))
                // redemarrage de la charge si Vbatt = 20% de moins   < 3,90V (70% Soc)
            {
                ACTIV_SORTIE( Charge_TP, 1, 0 );
            }

            voltage_batt_prec = voltage_batt;


            // viellissement calendaire :
            // par temperature : à 0°C, SOC80%, 250j  => -2,5% (1,3%à40%SOC, 4%à100%SOC)
            //                   à 25°C, SOC80%, 250j => -5%   (2,7%à40%SOC, 7% à 100%SOC)
            //                   à 30°C, SOC80%, 250j => -7%
            //                   à 45, SOC80%, 250j => -20%
            // => <25°:-2,5-0,1/°  >25:15-0,8/°
            // par SOC,30°C : à 100% degrad -7% (ou l'inverse)
            //              à 50% degrad -6%%  hypothese de SOC constant, égal à 50%
            //              à 30% degrad -5%
            if (temp_valide)
            {
                if (temp_mesure < 25)  // degrad en pour mille
                    batt_degrad = temp_mesure + 2 ;
                else
                    batt_degrad = 8 * temp_mesure  - 150;
                if (temp_mesure > 50) batt_degrad = 255;
                if (temp_mesure < -25) batt_degrad = 0 ;
            }
            uint16_t degrad_soh;
            degrad_soh = ((batt_soh / 1000 / 250 / 24 ) * batt_degrad ) / 6;
            batt_soh_cal += degrad_soh;
            batt_soh -= degrad_soh;

            #ifdef DEBUG_M  // affichage de la degradation soh de la batterie
               /* strcpy((char *)message, "LDegradTemp:");
                ajout_int16_message(temp_mesure, 12, 0);
                ajout_int16_message(batt_degrad, 18, 0);
                ajout_int16_message(degrad_soh, 24, 0);
                envoi_message(); */
            #endif

            // Calcul 1 : SOC evaluation directe en fonction des paramètres physico-chimiques
            // relation Soc-OCV :pente 7mV/%  4V=90% 3,9V=70% 3,8V(50%) 3,7V=30%  3,44V=8%  3,25V=10%  3V=2%
            // Optimum : cycles de 70%(3,9V) à 50%(3,8V)

            batt_soc1 = conv_volt_batt (voltage_batt, temp_mesure);


            // Calcul 2 : evaluation progressive en fonction de la charge instantanée
            // si l'ESP est sous tension => on utilise la mesure précédente
            // toutes les dix minutes : charge panneau ou décharge 18uA

            int16_t intensite;
            if (voltage_panneau >= voltage_batt)  // charge
            {
                intensite = (voltage_panneau - voltage_batt) * 5;   // 1v=>5mA  =  1mV => 5uA
            }
            else  // decharge
            {
                intensite = -18;  // decharge de 18uA
            }
            batt_soc_int += intensite/6*100;  // toutes les 10 minutes
            batt_soc2 = batt_soc_int / (batt_soh / 10000) / batt_soc100;

            // Raz sur batterie pleine ou vide
            if (voltage_batt_cor < 2800)
            {
                if (batt_soc2 > 20)
                {
                    message[0]='L';
                    envoi_soc_soh();
                    batt_soc_int = 0;
                    batt_soc2=0;
                }
            }
            if (voltage_batt > 3800)
            {
                if (batt_soc2 < 60)
                {
                    message[0]='L';
                    envoi_soc_soh();
                    batt_soc_int = batt_soh;
                    batt_soc2=90;
                }
            }

            // si batterie a niveau tres bas => arret Radio ecoute RX toutes les 5 secondes
            if ((arret_radio==1) && (batt_soc1 > Niv_batt_tres_bas))  // reprise
            {
                reprise_ecoute_radio();
                arret_radio=0;
            }
            if ((arret_radio==0) && (batt_soc1 < Niv_batt_tres_bas))  // arret
            {
                arret_ecoute_radio();
                arret_radio=1;
            }
        }
        #endif

        // eve_esp32 : bit0:radar bit1:radar autre   bit2:liste SD
        if (events & NODE_ESP32)
        {
            if (eve_esp32 & 1)  // detection radar interne
            {
            }
            if (eve_esp32 & 2)  // detection autre detecteur
            {

            }
            if (eve_esp32 & 4)  { }
        }


        #ifdef OPT_CAPTEUR

        /* Read the CPU sensor and send the value */
        if (events & NODE_EVENT_READ_SENSOR_TEMP)         // *********   LECTURE CAPTEUR  TEMP HYGRO **********************
        {
          #ifdef CAPTEUR_TEMP
            uint8_t ret;
            ret = lecture_temphdc1000();  // 0:ok  1:err
            if (!ret)
            {
                temp_cpt++;
                if ( (temp_cpt >= Temp_period_min) || ( abs(ValueBrut - temp_last) > Temp_ecart_min ) || (forcage_envoi_capteur))    // envoi-enreg
                {
                    int16_t resultat[2];
                    uint8_t chaine[20];
                    temp_last = ValueBrut;
                    temp_cpt = 0;
                    chaine[0] = 14;  // longueur avec cet octet  lg, M, Em, T, type, TS(5), val(4)
                    resultat[0] = FloatValue*100;  // temp en centieme de degres
                    resultat[1] = Float2Value*100; // hygro en centieme de pourcent
                    memcpy (chaine+10, resultat, 4);
                    envoi_enreg_resultat ('T', chaine);
                }
                forcage_envoi_capteur=0;
            }
          #endif
        }
     /*   if (events & NODE_EVENT_READ_SENSOR_PRESS)         // ********   LECTURE CAPTEUR  PRESSION  **********************
        {
           #ifdef CAPTEUR_PRESS
            uint8_t DataBrut[6];
            int16_t TemperN;
            uint16_t PressN;  // Offset de 50000
            lecture_press(1, DataBrut, &TemperN, &PressN);
            press_cpt++;
            if ( (press_cpt >= PRESS_PERIOD_MIN) || ( abs(PressN - press_last) > PRESS_ECART_MIN ))    // envoi-enreg
            {
                uint8_t chaine[20];
                press_last = PressN;  // Pression = 101325 => PressN=21325 (offset=80000) uint16
                press_cpt = 0;
                chaine[0] = 14;  // longueur avec cet octet

                memcpy (chaine+10, &PressN, 2);
                memcpy (chaine+12, &TemperN, 2);
                envoi_enreg_resultat ('P', chaine);
            }
          #endif
        }  */

        if (events & NODE_EVENT_READ_SENSOR_LUMI)         // ********   LECTURE CAPTEUR  LUMI  **********************
        {
           #ifdef CAPTEUR_LUMI
            lecture_lumi(1);
            lumi_cpt++;
            float lumi_ratio;
            if (FloatValue > lumi_last) lumi_ratio = FloatValue / lumi_last;
            else  lumi_ratio = lumi_last / FloatValue;
            lumi_ratio = (lumi_ratio - 1) * 100;
            if ( (lumi_cpt >= LUMI_PERIOD_MIN) || (( lumi_ratio > LUMI_POURC_MIN ) && ( abs(FloatValue-lumi_last) > LUMI_ECART_MIN )))    // envoi-enreg
            {
                uint8_t chaine[20];
                lumi_last = FloatValue;
                lumi_cpt = 0;
                chaine[0] = 14;  // longueur avec cet octet
                ((float*) (chaine+10))[0]  = FloatValue;  // cast array 8 -> array float
                envoi_enreg_resultat ('L', chaine);
            }
          #endif
        }
        #endif

        #ifdef TACHE_NODE_WDT
            if (events & EVENT_TACHE_NODE_WDT)  // tache qui ne fait rien, permet d'activer la tache, si aucune autre  tache
                // n'active la boucle, pour eviter le WDT
            {
            }
        #endif

        //writeToAddress ( (void*) 19,4);  // pour Test : fonction qui fait planter le CPU

        if (events & NODE_EVENT_BOUTTON_0)  // 0:long  1:court  2:double  3:3fois
        {

            #ifdef C50_T_v4
                strcpy((char *)message, "1B0: ");
                message[4] = b0_appui+'0' ;
                envoi_message ();
             #endif


            #ifdef C10_ElecVanne    // appui bouton : inversion vanne
                uint8_t evanne = 0;
                if (b0_appui )   // 1 , 2 ou 3
                {
                    //envoi_info_balise();

                   // HWREG (AON_SYSCTL_BASE + 4) |= (uint32_t) (1<<31);  // RESET!!

                    evanne = b0_appui-1;  // numero de la vanne : 1->0  2->1  3->2
                    if (evanne < NB_EVanne)
                    {
                        if (Clock_isActive(SortieEV[evanne].h_clock))   // lecture etat electrovanne
                            activ_evanne (evanne, 0);  // arret
                        else
                        {
                            uint8_t duree_v;
                            duree_v = par16_var[1]; // activation en minutes
                            activ_evanne (evanne, duree_v*60);
                            strcpy((char *)message, "L EVanne: -  ");
                            message[9] = evanne +'0' ;
                            ajout_int8_message( duree_v, 11,2);
                            envoi_message ();
                        }
                    }
                }
            #endif

            #ifdef C10_CAMERA_PIR   // Detection PIR
                /*
                type_pir=0  Batterie très basse => rien
                type_pir=1  PIR pas actif  => rien
                type_pir=2  camera deja en train de filmer => rien
                type_pir=4  camera inactive => rien
                type_pir=5  message
                Detec PIR:xx dans les 3 minutes apres video => message
                type_pir=6  Video

                 */
              uint8_t type_pir=0;
              uint8_t action=0;
              if (batt_soc1 > Niv_batt_tres_bas)
              {
               type_pir=1;
               if (T_BIT(StatutOps,0))  // detection active
               {
                 type_pir=2;
                 ACTIV_SORTIE(LED_cpu, 1, 1);  // activation Led N°0, mode:1, duree 0.1s

                 if (!OrCam)  // si au repos
                 {
                   type_pir=4;
                   action=1;
                   if (T_BIT(StatutOps,1))  // detection camera active
                   {
                       type_pir=5;
                       action = calcul_pir_esp();
                       if (action == 2)  // photo/video
                       {
                           type_pir=6;
                           // WEV402  Video/Photo x:cam_format, y:duree, z:envoi
                           // allumage ESP 10 secondes puis attente message WK
                           if (camera_debug != 1)
                               allumage_esp32(10);
                           else
                               OrCam=0;
                           allumage_led_pwm();  // allumage LED IR si nuit
                       }
                   }

                   if (action == 1)  // message uniquement : max 5 en 8 minutes
                   {
                       uint32_t tps;
                       struct timeval tv;  // microsecondes
                       tv = NB_MICROSEC();  // seconds et microsecondes
                       tps = tv.tv_sec;
                       if ( tps - esp_mess_tps > (PIR_DUREE_RAZ*60))  // raz apres X minutes
                       {
                           esp_mess_tps = tps;  // raz intervalle
                           if ( esp_mess_cptp > PIR_CPT_MAX)  // max  en X minutes
                               OrCam_param1 = esp_mess_cptp;  // compteur
                           else
                               OrCam_param1 = 0;  // compteur
                           esp_mess_cptp = 0;
                       }
                       else  //  detection en rafale, dans un intervalle court
                       {
                           esp_mess_cptp ++;
                           OrCam_param1 = esp_mess_cptp;  // compteur
                           if ( esp_mess_cptp > PIR_CPT_MAX)  // max 6 en 5 minutes
                               action = 0;
                       }

                       if (action == 1)
                       {
                           strcpy((char *)message, "L Detec PIR:   ");
                           message[12] = (OrCam_param1 / 10) + '0';  // compteur sur les 3 dernieres minutes
                           message[13] = (OrCam_param1 % 10) + '0';
                           envoi_message ();
                       }
                       //Seconds_Time ts;  // nanosecondes
                       //Seconds_getTime(&ts);  // seconds et nanosecondes
                       //conversion_sec_date (ts.secs, 12);
                       //conversion_sec_heure ( ts.secs, 12,0);
                       //message[20] = car_fin_trame;
                   }
                 }
               }
              }
              if (camera_debug)  // actif avec WEDB1
              {
                  if (type_pir<2)
                    ACTIV_SORTIE(LED_cpu, 6, 1);  // activation Led N°0, mode:6, duree 0.01s
                  if (action != 1)
                      strcpy((char *)message, "L Dbg-Detect PIR:     ");
                  if (type_pir==6)
                  {
                      message[19]=OrCam_param1; // Format
                      message[20]=OrCam_param2; // Duree
                      message[21]=OrCam_param3; // Envoi
                      if (Pwm[1].consigne) message[22]='I';  // Allumage LEd IR
                  }
                  message[17] = type_pir + '0';
                  envoi_message ();
              }

            #endif


            if (b0_appui == 1)        // *********************  BOUTON 0   COURT   ****************************
            {
                #ifdef C50_T_v4
                    strcpy((char *)message, "1B0court ");
                    message[4] = b0_appui+'0' ;
                    envoi_message ();
                 #endif


                #ifdef TELECOMMANDE   // Telecommande (bouton du milieu : allumage spot)
                   strcpy((char *)message, "PXE31Z");  // Allumage spot(3) pendant 73secondes(Z)
                     //   Ack / reenvoi 2fois / pas sup si echec / RX apres
                   Mess_statut = 0x20;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                   envoi_message ();
                   ACTIV_SORTIE(LED_cpu, 1, 1);  // activation Led N°0, mode:1, duree 0.1s
                   strcpy((char *)message, "1XS159");  // Allumage spot solaire (node:1) pendant 50 secondes(5)
                   Mess_statut = 0x20;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                   envoi_message ();
                #endif

                #ifdef C10_Compteur   // compteur : nouveau tick      envoi si modif de plus de 5%
                   CLOCK_STOP (sensorCapteurTimeoutHandle);
                   cpt_somme_cpt++;   // ajout de 1 a l'index
                   // calcul du temps depuis le tick precedent (en 10ms)
                   uint32_t nouveau_tick, ecart_tick;
                   struct timeval tv;  // microsecondes
                   tv = NB_MICROSEC();  // seconds et microsecondes
                   nouveau_tick = (tv.tv_sec % (3600*24*365) )*100 + tv.tv_usec/10000;
                   ecart_tick = nouveau_tick - time_dernier_tick_10ms;  // en 10ms

                   if (ecart_tick > 3*3600*100)  // si ecart > 3h alors debordement clock => double clock
                   {
                       // a partir de 3 fois (en ms) le temps => arret
                       nb_clock_cpt=100;
                       Clock_setPeriod (sensorCapteurTimeoutHandle, (uint32_t) ((ecart_tick/100) * 30) * (1000  / Clock_tickPeriod));
                       Clock_setTimeout (sensorCapteurTimeoutHandle, (uint32_t) ((ecart_tick/100) * 30) * (1000  / Clock_tickPeriod));
                   }
                   else
                   {
                       nb_clock_cpt=0;
                       Clock_setPeriod (sensorCapteurTimeoutHandle, (uint32_t) ((ecart_tick) * 30) * (1000  / Clock_tickPeriod));
                       Clock_setTimeout (sensorCapteurTimeoutHandle, (uint32_t) ((ecart_tick) * 30) * (1000  / Clock_tickPeriod));
                   }
                   Clock_start(sensorCapteurTimeoutHandle);

                   uint16_t new_conso_instant;
                   new_conso_instant = (3600 * 10 * 100) / ecart_tick;   // en l/h - 10 litres par tick
                   if (!new_conso_instant) new_conso_instant = 1;  // minimum 1

                   // si precedente conso_instant <>0 alors maj
                   if (!conso_valide)
                   {
                        conso_valide = 1;  // debut de consommation
                        envoie_mesure_compteur_eau(0);  // 0:hexa  1:texte
                        timer_debut_conso = nouveau_tick;
                   }
                   else
                   {
                       if (conso_instant)
                       {
                           // ecart de plus de 5% avec le précédent
                           uint32_t ecart_pour100;
                           if (conso_instant > new_conso_instant)
                               ecart_pour100 = conso_instant - new_conso_instant;
                           else
                               ecart_pour100 = new_conso_instant - conso_instant;
                           if (((ecart_pour100*100) / conso_instant) > COMPTEUR_SEUIL)  // si ecart sup a 5% alors envoi mesure
                           {
                               conso_instant = new_conso_instant;
                               envoie_mesure_compteur_eau(0);  // 0:hexa  1:texte
                           }
                           // detection de fuite : verification que le debut de conso < 18 heures
                           if (((nouveau_tick - timer_debut_conso) > SEUIL_DETECT_FUITE*3600*100) && (!fuite))
                           {
                               uint8_t index;
                               fuite = 1;
                               strcpy((char *)message, "LCpt: Fuite:   ");
                               index = ajout_int16_message ( conso_instant, 12, 0x82);
                               strcpy((char *)message+index, "l/h");
                               envoi_message ();
                           }
                       }
                       else
                       {
                           conso_instant = new_conso_instant;
                           envoie_mesure_compteur_eau(0);  // 0:hexa  1:texte
                       }

                   }
                   time_dernier_tick_10ms = nouveau_tick;
                #endif

                #ifdef C10_Cent_Meteo   // tick -> increment compteur
                   //uint32_t nouveau_tick, ecart_tick;
                   cpt_somme_pluv++;  // compteur horaire
                   /*struct timeval tv;  // microsecondes
                   tv = NB_MICROSEC();  // seconds et microsecondes
                   nouveau_tick = (tv.tv_sec % (3600*24) )*10 + tv.tv_usec/100000;  // en 100ms, par jour
                   ecart_tick = nouveau_tick - time_dernier_tick_100ms;  // en 100ms
                   if (ecart_tick < time_conso_min)
                       time_conso_min = ecart_tick;
                   time_dernier_tick_100ms = nouveau_tick;*/
                #endif

               #ifdef RADAR_DET  // Anti-arrachage radar  - NON ENCORE TRAITE PAR LE STM32
                   strcpy((char *)message, "PDETxC");  // Envoi au STM32 d'un probleme radar (arrachage)
                   message[4] = NODE_Id ;  // Garches : 1:Radar entree, 2:Radar terasse
                     //   Ack / reenvoi 2fois / pas sup si echec / RX apres
                   Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                   #ifdef DEBUG_M
                      ACTIV_SORTIE(LED_cpu, 1, 6);  // activation Led N°0, mode:1, duree 0.6s
                      Mess_statut = 0x22;  // pas d'Ack/pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
                   #endif
                   envoi_message ();
                #endif

                #ifdef PIR_DET   // Detection PIR
                   strcpy((char *)message, "LS Spot:x");  // Envoi au STM32 de la detection PIR par le Spot
                   message[8] = NODE_Id+'0' ;
                     //   Ack / reenvoi 2fois / pas sup si echec / RX apres
                   Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                   #ifdef DEBUG_M
                      ACTIV_SORTIE(LED_cpu, 1, 20);  // activation Led N°0, mode:1, duree 0.6s
                      ACTIV_SORTIE(1, 1, 20);  // activation Led N°0, mode:1, duree 0.6s
                      Mess_statut = 0x22;  // pas d'Ack/pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
                   #endif
                   envoi_message ();

                   // activation spot si nuit
                   #ifdef SPOT_TOR_PWM
                       uint8_t ret;
                       uint16_t ent_anal;
                       ret = lecture_analog(0, &ent_anal);  // AL0:voltage panneau solaire
                       if ((!ret) && (ent_anal < (Spot_allumage_nuit_seuil*10)))
                           ACTIV_PWM ( 1,1, Spot_allumage_nuit*10, 5000, 255);  // duree:30 secondes, frequence 400Hz, duty:100%
                       else
                       {
                           if (Spot_allumage_jour)
                               ACTIV_PWM ( 1,1, Spot_allumage_jour, 250, 3);  // duree:0,5 secondes, frequence 4000Hz, duty:3%
                       }
                   #endif
                #endif


                if (Config_boutons)
                {
                    #ifdef DEBUG_M
                    #ifdef BROAD_BALISE_SYNCHRO
                        //ACTIV_SORTIE(LED_cpu, 4, 1);  // activation Led N°0, mode:1, duree 0.6s
                        //Demande_balise_synchro();
                    #endif
                    #endif

                    #ifdef CC1350_carte
                    #ifdef OPT_CAPTEUR
                    // changement de mode veille / activation periodique des capteurs
                    if (mode_periodique)
                    {
                        activation_mode_periodique(0);  // extinction
                        ACTIV_SORTIE(LED_cpu, 1, 6);  // activation Led N°0, mode:1, duree 0.6s
                        ACTIV_PWM ( 0,1,5,1700,2);    // activation buzzer, 1 bip long, duree 0.2s, periode 400, volume 5
                    }
                    else
                    {
                        activation_mode_periodique(1);  // mise en marche periodique
                        ACTIV_SORTIE(LED_cpu, 1, 2);  // activation Led N°0, mode:1, duree 0.1s
                        ACTIV_PWM ( 0,1,2,1700,2);    // activation buzzer, 1 bip court, duree 0.4s, periode 400, volume 5
                    }



                   /* if ((capt[0].activ & 0b11) == 0b11)   // capt lumi activé pour le bouton
                    {
                        lecture_lumi();
                        strcpy((char *)message, "LCAP:Lumi: xx.xx");
                        forcage_envoi_rf=2;  // débloque les nodes bloqués  hors portee
                        ajout_float_message(FloatValue, 2, 10, 0);
                        envoi_message ();
                    }
                    else if ((capt[1].activ & 0b11) == 0b11)  // capt température activé par bouton
                    {
                        lecture_temphdc1000();
                        strcpy((char *)message, "LCAP:temp:        hygro:  ");
                        forcage_envoi_rf=2;  // débloque les nodes bloqués  hors portee
                        ajout_float_message(FloatValue, 2, 10, 0);
                        ajout_float_message(Float2Value, 2, 24, 0);
                        envoi_message ();
                    }*/
                    #endif
                  #endif
                }
            }

            if (b0_appui == 0)        // *********************  BOUTON 0   LONG   ****************************
            {
                #ifdef C50_T_v4
                    strcpy((char *)message, "1B0long ");
                    message[4] = b0_appui+'0' ;
                    envoi_message ();
                 #endif

                #ifdef TELECOMMANDE   // Telecommande
                   strcpy((char *)message, "LS Telec 1 long");
                     //   Ack / reenvoi 2fois / pas sup si echec / RX apres
                   Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                   ACTIV_SORTIE(LED_cpu, 8, 8);  // activation Led N°0, mode:8, duree 8 => 3 flashs
                   #ifdef DEBUG_M
                      Mess_statut = 0x22;  // pas d'Ack/pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
                   #endif
                   envoi_message ();
                #endif

                // Réception message concentrateur
                //ACTIV_SORTIE(LED_cpu, 1, 20);  // activation Led N°0, mode:1, duree 2s
                //ACTIV_PWM ( 0,1,5,1500,2);        // Bip buzzer

                   // Test : envoi d'un mesage et attente de la réponse
                   strcpy((char *)message, "QT6");
                     //   Ack / reenvoi 2fois / pas sup si echec / RX apres
                   Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                   ACTIV_SORTIE(LED_cpu, 2, 2);  // activation Led N°0, mode:8, duree 8 => 3 flashs
                   #ifdef DEBUG_M
                      Mess_statut = 0x22;  // pas d'Ack/pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
                   #endif
                   envoi_message ();


                #ifdef VECTEUR_ORIENTATION
                    DELAY_MS(400);  // to avoid movement after button action
                    calibration_gyro();   //   Gyro : calibration (a plat -immobile)
                    // print : Ecart type  + 3 valeurs gyro (Divide by 4 to get 32.9 LSB per deg/s)

                #endif

            }

            if (b0_appui == 2)        // *********************  BOUTON 0   DOUBLE   ****************************
            {

                if (Config_boutons>=2)
                {
                    #ifdef TELECOMMANDE   // Telecommande
                        strcpy((char *)message, "Q-EC4");  // pas de réponse attendue
                        message[1] = 6;  // longueur payload
                        memcpy (message+5, &code_tournant_telec, 4);
                        envoi_message_def();
                        code_tournant_telec = code_tournant_suivant(code_tournant_telec);
                        ACTIV_SORTIE(LED_cpu, 8, 5);  // activation Led N°0, mode:8, duree 5 => 2 flashs
                    #endif

                   #ifdef OPT_CAPTEUR
                    // changement de mode  affichage serie -> envoi par RF -> enregistrement flash du capteur MPU
                    increment_mode_affich_enreg();
                    #endif
                }
            }
            if (b0_appui == 3)        // *********************  BOUTON 0   3 fois   ****************************
            {
            }
            b0_appui=0;
        }


        if (events & NODE_EVENT_BOUTTON_1)
        {

            #ifdef C50_Test
                strcpy((char *)message, "1B1: ");
                message[4] = b1_appui+'0' ;
                envoi_message ();
             #endif

            if (b1_appui == 1)                  // *********************  BOUTON 1  COURT   ****************************
            {
                #ifdef TELECOMMANDE    //  bouton du haut - Activation centrale - non sécurisé
                    strcpy((char *)message, "PMC1");
                    Mess_statut = 0x22;  // 22:RX apres,2 envois   30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
                    envoi_message();
                    ACTIV_SORTIE(LED_cpu, 1, 1);  // activation Led N°0, mode:1, duree 100ms
                #endif

                #ifdef RADAR_DET
                   //ACTIV_SORTIE(LED_cpu, 1, 6);  // Detection radar - activation Led N°0, mode:1, duree 0.6s
                    // Timer pour limiter 1 detection toutes 30 secondes
                    // conso veille 5uA  si detection:5mA pdt 10ms equivaut a 10 secondes de veille
                    if (!Clock_isActive(TimerDetectionClockHandle))
                    {
                        Clock_setPeriod(TimerDetectionClockHandle, 0); // Period=0 pour un one-shot
                        Clock_setTimeout(TimerDetectionClockHandle, 30000 *  (1000  / Clock_tickPeriod)) ;  // 30s
                        Clock_start(TimerDetectionClockHandle);
                        strcpy((char *)message, "PDEPxC");  // Envoi au STM32 d'une detection PIR Radar
                       //   Ack / reenvoi 2fois / pas sup si echec / RX apres
                       Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                       message[4] = My_Address;
                       envoi_message ();
                    }
                #endif

                #ifdef C10_Cent_Meteo   // tick -> increment compteur vent
                   cpt_somme_vent++;  // compteur horaire
                #endif

                //ACTIV_SORTIE(LED_cpu, 1, 3);  // activation Led N°0, mode:1, duree 0.3s

                if (Config_boutons)
                {

                    ACTIV_PWM ( 0,1,1,1300,2);        // Bip buzzer

                    RadioDebutBaliseTXdemande();
                    ACTIV_SORTIE(LED_cpu, 4, 1);  // activation Led N°0, mode:1, duree 0.6s
                    activ_led_reponse=1;


                    #ifdef CAPTEUR_MPU
                        if (mode_principal==1)  // mouvement
                        {
                            ACTIV_SORTIE(LED_cpu, 8, 4);
                            strcpy((char *)message, "LTest");
                            Mess_statut=0;  // bit0:dernier  bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)
                            // bit3:différé   bit4:pas d'ack  bit5:RX apres
                            envoi_message();  // Ack, pas de RX apres
                            chgt_acc_Z();

                            // Test fonction Emulation EEPROM
                            /*uint8_t buff_r[10], ret;
                            strcpy((char *)message, "RTest");
                            envoi_message();
                            uint8_t buffer[8];
                            ret = EEPROM_read(0, buffer, 2);
                            if (buffer[0]<'0')   buffer[0]='0';
                            if (buffer[1]<'0')   buffer[1]='0';
                            strcpy((char *)message, "RExxxx");
                            message[2]= buffer[0];
                            message[3]= buffer[1];
                            envoi_message();
                            buffer[0]= buffer[0]+1;
                            buffer[1]='0';
                            ret = EEPROM_write(0, buffer, 2, 0);
                            buffer[2]='1';*/
                        }
                        if (mode_principal==2)  // FFT
                        {
                            lancement_algo4();
                        }
                    #endif
                 }
            }
            if (b1_appui == 0)                  // *********************  BOUTON 1  LONG   ****************************
            {
                #ifdef TELECOMMANDE   // Telecommande
                   strcpy((char *)message, "LS Telec 2 long");
                   Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                   envoi_message ();
                   ACTIV_SORTIE(LED_cpu, 8, 8);  // activation Led N°0, mode:8, duree 8 => 3 flashs
                #endif


                if (Config_boutons>=2)
                {
                    if (mode_principal == 0)
                    {
                        //ACTIV_SORTIE(LED_cpu, 1, 2);  // activation Led N°0, 0.1s
                        S_Temp Temp;
                        float Ftemp, Fhygro;
                        mesure_temperature(&Temp);
                        strcpy((char *)message, "LTL Temp: xx.xx    Hygro: x:xx");
                        Ftemp = (float) Temp.temp / 100;
                        ajout_float_message(Ftemp, 2, 9, 0x80);
                        Fhygro = (float) Temp.hygro / 100;
                        ajout_float_message(Fhygro, 2, 25, 0);
                        Mess_statut = 0x0;  // 20:RX apres,  30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
                        envoi_message ();
                        ACTIV_SORTIE(LED_cpu, 4, 1);  // activation Led N°0, mode:1, duree 0.6s
                        ACTIV_PWM ( 0,1,1,1700,2);    // activation buzzer, 1 bip long, duree 0.1s, periode 400, volume 5
                    }

                    /*strcpy((char *)message, "PDEPxL");  // Envoi au STM32 d'un pb radar
                    message[4] = My_Address;
                    strcpy((char *)message, "QHLS");  // Envoi au STM32 de la detection radar
                    Mess_statut = 0x30;  // pas d'Ack/ RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
                    envoi_message ();*/

                    // Changement mode principal du capteur (Temp-> mouv-> FFT)

                    /*uint8_t new;
                    new = mode_principal + 1;
                    if ( new > 2 ) new=0;

                    activation_mode_principal(new);
                    ACTIV_SORTIE(LED_cpu, 8, new*2+4);  */// activation Led N°0, mode:8, duree 1(0.4s), 2(0.8s) ou 3(1.2s) eclats


                #ifdef CAPTEUR_MPU
                    // mode 2 : chgt de type de données par appui boutton :
                    // 0:quater  1:angles  2:gyro  3:accelero  4:magneto  5:val brutes  6:vibration  7:chocs
                    /*uint8_t mode_capt = config_mpu;
                    mode_capt ++;
                    if (mode_capt > 7) mode_capt=0;
                    config_type_mpu(mode_capt);  // application de la configuration sélectionnée*/

                    //chgt_mode_algo4();
                #endif
                }
            }
            if (b1_appui == 2)                  // *********************  BOUTON 1  DOUBLE   ****************************
            {
                if (Config_boutons>=2)
                {
                    #ifdef TELECOMMANDE   // Telecommande
                       strcpy((char *)message, "LS Telec 2 double");
                       Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                       envoi_message ();
                       ACTIV_SORTIE(LED_cpu, 8, 5);  // activation Led N°0, mode:8, duree 5 => 2 flashs
                    #endif

                    #ifdef CAPTEUR_MPU
                        // Mode mouvement
                        if (mode_principal == 1)
                        {
                            // 10:choc  11:choc+vib  12:Choc+vib+Accel  13:idem+quaternion  14:rien
                            uint8_t mode_capt = config_mpu;
                            if (mode_capt<10) mode_capt=10;
                            else
                            {
                                mode_capt++;
                                if (mode_capt > 13) mode_capt=10;
                            }
                            ACTIV_SORTIE(LED_cpu, mode_capt+1 , 5*(mode_capt-9));    // x flash court
                            config_type_mpu(mode_capt);  // application de la configuration sélectionnée
                        }


                        #ifdef VECTEUR_ORIENTATION
                        //DELAY_MS(300);  // to avoid movement after button action
                        calibration_accel();
                        #endif
                    #endif
                    /*strcpy((char *)message, "RB1 Double");
                    envoi_message ();*/

                }
            }
            b1_appui=0;
        }
        #if BOUTON_CC == 3  // 3e bouton
        if (events & NODE_EVENT_BOUTTON_2)
        {
            if ( b2_appui == 1 )          // *********************  BOUTON 2 Court ****************************
            {
                #ifdef TELECOMMANDE    //  bouton du bas - Désactivation centrale - code tournant sécurisé
                    /*strcpy((char *)message, "Q-EC1");
                    message[1] = 6;  // longueur payload
                    memcpy (message+5, &code_tournant_telec, 4);
                    code_tournant_telec = code_tournant_suivant(code_tournant_telec);
                    Mess_statut = 0x20;  // 20:RX apres,  30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
                    envoi_message_def();  */
                    //time_cpu[index_time++] = Clock_getTicks(); // 100.000 = 1 seconde
                    //if (index_time >18) index_time=0;

                    strcpy((char *)message, "PMC0");
                    Mess_statut = 0x20;  // 20:RX apres,  30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
                    envoi_message();

                    ACTIV_SORTIE(LED_cpu, 1, 1);  // activation Led N°0, mode:1, duree 100ms
                #else
                     ACTIV_SORTIE(LED_cpu, 1, 1);  // activation Led N°0, mode:1, duree 100ms
                 #endif
            }
            if ( b2_appui == 0 )          // *********************  BOUTON 2 Long ****************************
            {
                 strcpy((char *)message, "LB3Long");
                 envoi_message ();
                 ACTIV_SORTIE(LED_cpu, 8, 8);  // activation Led N°0, mode:8, duree 8 => 3 flashs
            }

            if ( b2_appui == 2 )          // *********************  BOUTON 2 Double ****************************
            {
                #ifdef TELECOMMANDE
                    strcpy((char *)message, "LS Telec 3 double");
                    Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
                    envoi_message ();
                    ACTIV_SORTIE(LED_cpu, 8, 5);  // activation Led N°0, mode:8, duree 5 => 2 flashs
                #endif
            }
            b2_appui = 0;
        }
        #endif

        if (events & NODE_EVENT_2BOUTTONS_APPUI)       // *********************  2 BOUTONS SIMULTANES ****************************
        {
            #ifdef TELECOMMANDE   // Telecommande
               strcpy((char *)message, "LS Telec 2 simu");
               Mess_statut = 0x22;  //bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
               envoi_message ();
               ACTIV_SORTIE(LED_cpu, 1, 3);  // activation Led N°0, mode:1, duree 0,3s
            #endif

            #ifdef  CAPTEUR_MPU
                if (Config_boutons>=2)
                {
                    // changement de mode d'envoi des infos du capteur MPU : hexa ou texte
                    increment_mode_envoi_mpu();

                    #ifdef RAM_TAMPON_SIZE
                    Flash_page_index = 0;
                    Ram_tampon_index = 0;
                    #endif
                    //index_flash_tail = 0;
                    //index_flash_head = 0;
                }
            #endif
        }

        if (events & NODE_EVENT_MAJ_HEURE)       // Demande pour mise à jour de l'heure
        {
            if (compteur_PHLS)  // initial
            {
                compteur_PHLS--;
                if (compteur_PHLS)   // periodique 10 secondes
                {
                    strcpy((char *)message, "QHLS");  // Demande date et heure au concentrateur
                    validite = Seconds_get();
                    Mess_statut = 0x30;  // 30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
                    envoi_message ();
                }
                else  // fin initial -> passage periodique 1h
                {
                    Clock_stop(MajHeureClockHandle);  // Démarrage de la maj periodique heure QHLS
                    uint32_t k = 60 * 60 * (1000 * (1000 / Clock_tickPeriod));   // j=1=>1s  j=2=>4s j=3=>16s j=10=>18h j=11=>3j
                    Clock_setPeriod(MajHeureClockHandle, k); // Period=0 pour un one-shot
                    Clock_setTimeout(MajHeureClockHandle, k );
                    if (compteur_maj_heure)
                        Clock_start(MajHeureClockHandle);  // Démarrage de la maj periodique heure QHLS
                }
            }
            else  // periodique
            {
                compteur_maj_heure--;
                if (!compteur_maj_heure)
                {
                    compteur_maj_heure = CPT_MAJ_HEURE;    // reinit cpt interrogation periodique

                    strcpy((char *)message, "QHLS");  // Demande date et heure au concentrateur
                    validite = Seconds_get();
                    Mess_statut = 0x30;  // 30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
                    envoi_message ();
                }
            }
        }

        if (events & EVENT_NODE_EXTINCT_ESP)       // Extinction ESP32
        {
            if ((OrCam)  && ((!camera_debug) || (camera_debug==3)))  // 0:normal 1:PIR 2:PIR+cam absent 3:PIR+CAM
                 err_esp_extinct(6);
            else
                extinction_esp32();
        }

        if (events & EVENT_KEEP_ALIVE)       // Envoi keep alive
        {
            if (Temps_interrog_maj_cpt)   // poursuite compteur 6 heures
            {
                Temps_interrog_maj_cpt--;
            }
            if (!Temps_interrog_maj_cpt)  // keep alive ou intero concentrateur, puis relance compteur initial
            // #define PERIOD_PASS_INTERO_CONC
            // en Nb d'interrogation passerelle  0:keepalive    1:interro uniq,  2:1fois sur 2
            {
                Temps_interrog_maj_cpt = Temps_interrog_maj;

                uint32_t secondes_mes;
                int8_t secondes_dec;
                NB_SECONDES(secondes_mes); //Time(ISeconds_Time *ts);  // seconds et nanosecondes
                secondes_dec = (secondes_mes+60) % 120 - 60;

                Clock_stop(Inter_ConcClockHandle);
                Clock_setPeriod (Inter_ConcClockHandle, (uint32_t) Temps_interrog_min * (1000  / Clock_tickPeriod * 1000));
                Clock_setTimeout (Inter_ConcClockHandle, (uint32_t) (Temps_interrog_min-secondes_dec) * (1000  / Clock_tickPeriod * 1000));
                Clock_start(Inter_ConcClockHandle);

                uint8_t keep_alive;
                keep_alive=1;
                if (cpt_pass_intero_conc)
                {
                    cpt_pass_intero_conc--;
                    if (!cpt_pass_intero_conc)    keep_alive=0; // interro
                }
                if (keep_alive)
                {
                    RadioDebutKeepAlive();
                }
                else   // Activ radio pour reception message en attente
                {
                    cpt_pass_intero_conc = PERIOD_PASS_INTERO_CONC;
                    RadioDebutBaliseTXdemande();
                }
            }
        }

        #ifdef OPT_CAPTEUR
        if (events & EVENT_VIDAGE_TAMPON)  // lorsque timeout de retention de donnees trop long
        {
            vidage_tampon();
        }
        #endif

        #ifdef FLASH_EXTERNE
        if (events & ENREG_EEPROM)  // apres timeout : enreg des index flash
        {
            sauve_eeprom_index_flash();
        }
        #endif

        if (events & EVENT_TIMER_S)
        {
            uint8_t i;
            for (i=0; i<NB_SORTIES; i++)
            {
                if (timer_inter & (1<<i))
                {
                    timer_inter &= ~((uint16_t)1<<i);  // clear bit
                    Clignot_sortie(i);
                }
            }
        }

        #ifdef C10_ElecVanne    // Fermeture EV apres temps d'ouverture, demarrage programme
        if (events & EVENT_TIMER_EV)
        {
            uint8_t i;
            for (i=0; i<NB_EVanne; i++)
            {
                if (timer_ev_inter & (1<<i))
                {
                    timer_ev_inter &= ~((uint16_t)1<<i);  // clear bit -  Extinction
                    Clock_stop(SortieEV[i].h_clock);
                    ACTIV_SORTIE( 2*i+1+1, 1, par16_var[0]);  // Extinction EV pendant 0,4 seconde (autonomie 4500s cycle avec pile 600mAh)
                }
            }
        }
        if (events & EVENT_PROG_EV)  // demarrage cycle programme
        {
            EV_prog_timer_rest = EV_prog_timer; // redemarrage cycle
            uint16_t duree_sec;
            duree_sec = transf_duree_ev(EV_prog_duree);
            strcpy((char *)message, "L Cycle EV0:      min");
            ajout_int8_message( duree_sec/60, 13,0x82);
            envoi_message ();

            activ_evanne (0, duree_sec);
        }

        #endif

        if (events & EVENT_TIMER_PWM0)
        {
            ETAT_PWM(0);
        }
        if (events & EVENT_TIMER_PWM1)
        {
            ETAT_PWM(1);
        }


        if (events & NODE_EVENT_ENVOI_VOLTAGE) // envoi voltage bas
        {
            envoi_voltage_bas();
        }

        if (events & NODE_MESS_LONG)  // Test : TML envoi message long  par RF
        {
            message[0]='L';
            message[2]='Y';
            message[4]= (send_long_num_trame >> 8);
            message[5]= (send_long_num_trame & 0xFF);

            if (send_long_num_trame)  // trame suivante
            {
                message[1] = 100 + 3;
                message[3] = 1;
                uint8_t i;
                for (i=0; i<100; i++)
                {
                    message[6+i] = send_long_num_trame + 'A';
                }
                message[105] = 'Z';
                if ((send_long_num_trame+1) == send_long_num_trame_tx)
                    message[3] =2; //dernier
            }
            else  // premiere trame ; nom du fichier
            {
                message[1]= 38 + 4;  // longueur+4
                message[3]= 0;       // type  0:premier  1:milieu  2:dernier
                strcpy ((char *)message+6, "C0a-137-T5-D00-F5-C0-210427-223700.jpg" );  // longueur du nom : 38 + un '0'
            }
            envoi_message_def();
            send_long_num_trame++;
            if (send_long_num_trame >= send_long_num_trame_tx)
                CLOCK_STOP(TestMessLongClockHandle);
        }

        if (events & NODE_EVENT_SEND_CAPTEUR)   // mesure periodique sonde
        {
            #ifdef C10_sonde_HUmSol     // timer : mesure periodique sonde
                 envoie_mesure_sonde_HumSol(0);  // 0:hexa  1:texte
            #endif

            /*#ifdef C10_Compteur   // envoi conso instantanée et index total
                 envoie_mesure_compteur_eau(0);  // 0:hexa  1:text
            #endif*/

            #ifdef C10_Cent_Meteo    // chaque heure : envoi pluviometrie
                    envoie_mesure_pluviometre(0);  // 0:hexa  1:texte
                    cpt_somme_pluv = 0;
                    cpt_somme_vent = 0;
                    //time_conso_min = 0;
            #endif
        }

        #ifdef C10_Compteur   // timeout tick  : plus de consommation
            if (events & NODE_Fin_timer_compteur)     // timeout en attente de tick compteur
            {
                if (nb_clock_cpt)
                    nb_clock_cpt--;
                else
                {
                    CLOCK_STOP (sensorCapteurTimeoutHandle);
                    conso_instant = 0;
                    conso_valide = 0;
                    envoie_mesure_compteur_eau(0);  // 0:hexa  1:texte
                    fuite = 0;
                }
            }
        #endif

        if (code_erreur)
            envoi_code_erreur();

        // verification que la stack de la tache est a moins de 70%
        Task_Stat statbuf;
        Task_stat(Task_self(), &statbuf);
        uint8_t stack_pourc = statbuf.used *10 / statbuf.stackSize;
        if ((stack_pourc > 7) && (stack_pourc > stack_over))
            {   code_erreur = erreur_stack_overfl;
                err_donnee1 = stack_pourc + '0';
                stack_over = stack_pourc;
                //System_printf("Over 70% of task's stack is in use.\n"); }
            }

       /* #ifdef DEBUG_reseau
            if ((events != EVENT_RECEP_MESSAGE_UART) && (events != EVENT_SEND_MESSAGE_UART))
            {
                strcpy((char *)message, "1 N_Event:xxxx-xxxx Statut:x suiv:x");
                ajout_word_message(events, 8);
                message[25]=Phase_radio()+'0';
                message[32]=Radio_suiv()+'0';
                envoi_message ();
            }
        #endif*/
    }
}



#ifdef C10_CAMERA_PIR  // detection PIR -> video/photo   allumage/extinction ESP


// return : action : // 0:rien  1:message   2:photo/video
// maj : OrCam_param1(format) OrCam_param2(duree)  Orcam_param3(envoi)
uint8_t calcul_pir_esp(void)
// WEV402  Video/Photo x:cam_format, y:duree, z:envoi
{
    uint8_t action;  // 0:rien  1:message   2:photo/video
    uint8_t niv_bat, valid_p, valid_v, i, duree;
    uint32_t tps;
    struct timeval tv;  // microsecondes

    //batt_soc1=69;
    for (niv_bat=0; niv_bat<5; niv_bat++)
    // Niv 0:<10  1:10-20  2:20-40  3:40-60  4:60-80  5:>80
     //   uint8_t esp_pho[6] = { 70, 85,  88, 91, 96, 100};   //
     //   uint8_t esp_vid[6] = { 70,  70,  81, 86, 96, 100};   //100:40s  90:6min  86:1h  81:1j  73:1an
    // uint8_t esp_bat[5] =    {10, 20, 40, 60, 80}; // rien, mess, photo stock, photo, video, HQ
    {
        if (batt_soc1 <= esp_bat[niv_bat] )
            break;
    }
    if (batt_soc1 > esp_bat[4]) niv_bat=5;

    tv = NB_MICROSEC();  // seconds et microsecondes
    tps = tv.tv_sec;

    // 12 dernieres videos des 24 dernieres heures
    // derniere video : date/heure (en sec)
    // les 11 autres sont décales en nb de 5min par rapport à la derniere

    // calcul du nb de videos prises depuis les 24 dernieres heures
    // temps entre la derniere video et maintenant
    uint32_t temps_dernier;
    uint8_t nb_videos, nb_photos;
    temps_dernier = (tps - date_derniere_video)/360;  // nb 6 minutes
    nb_videos=0;
    for (i=0; i<12; i++)
    {
        if (temps_dernier < 24*10)  // 24 heures
        {
            nb_videos++;
            temps_dernier += ecart_video[i];
        }
        else
            break;
    }
    // calcul du nb de photos prises depuis les 24 dernieres heures
    temps_dernier = (tps - date_derniere_photo)/360;  // nb 6 minutes
    nb_photos=0;
    for (i=0; i<30; i++)
    {
        if (temps_dernier < 24*10)  // 24 heures
        {
            nb_photos++;
            temps_dernier += ecart_photo[i];
        }
        else
            break;
    }

    // calcul du temps (en sec) entre la derniere video et la nouvelle video a prendre
    // depend du nb video des 24 dernieres heures et de la batterie
    // si batterie pleine => 1,4^nb*intervalle
    uint32_t attente_video, attente_photo;
    attente_video = VIDEO_INTERVALLE;
    for (i=0; i<nb_videos; i++)
        attente_video = VIDEO_MULTIPLI * attente_video;
        attente_video = (attente_video * tps_batt(niv_bat, 1));

    attente_photo = PHOTO_INTERVALLE;
    for (i=0; i<nb_photos; i++)
        attente_photo = PHOTO_MULTIPLI * attente_photo;
        attente_photo = (attente_photo * tps_batt(niv_bat, 0));

    //determination si 1 video ou photo doit être prise
    valid_p = 0;
    valid_v = 0;
    if ((tps - date_derniere_video > attente_video) || ( niv_bat == 5 ))
    {
        valid_v = 1;  // prise de video
        // decalage video
        for (i=11; i; i--)
            ecart_video[i] = ecart_video[i-1];
        if (tps - date_derniere_video > 3600*24)   // sup a 24h
            ecart_video[0] = 0xFF;
        else
            ecart_video[0] = (tps - date_derniere_video) / 360;
        date_derniere_video = tps;
    }
    else if (tps - date_derniere_photo > attente_photo)
    {
        valid_p = 1;  // prise de photo
        // decalage photo
        for (i=29; i; i--)
            ecart_photo[i] = ecart_photo[i-1];
        if (tps - date_derniere_photo > 3600*24)   // sup a 24h
            ecart_photo[0] = 0xFF;
        else
            ecart_photo[0] = (tps - date_derniere_photo) / 360;
        date_derniere_photo = tps;
    }


    if ((valid_p) || (valid_v))
    {
        OrCam = 3;  // prise de photo/video
        OrCam_essai = 3;

        // format
        OrCam_param1 = '5';  // VGA
        if (niv_bat < 4)
            OrCam_param1 = '4';  // <VGA

        // duree :
        duree=0;
        if (valid_v)   // prise de video
        {
            duree = 2;
            if (niv_bat > 4)
                duree = 3;
        }
        OrCam_param2 = duree + '0';  // duree

        // envoi ?
        OrCam_param3 = '3';  // SD + envoi
        if (niv_bat < 3)
            OrCam_param3 = '2'; // SD uniquement
        action = 2;
    }
    else  // pas de photo ni video
    {
        action = 1;  // message
    }
    return action;
}

uint16_t resteclockESP(void)
{
    return Clock_getTimeout(ESP32_sleepTimerClockHandle)/100/1000;  // en secondes
}

void allumage_esp32(uint16_t secondes)
{
    struct timeval tv;  // microsecondes

    CLOCK_RESTART_TIMEOUT(ESP32_sleepTimerClockHandle, (uint32_t)secondes*10*100);  // millisecondes
    Activ_Uart2();
    ACTIV_SORTIE(Enab_ESP, 1, 0);  // activation ESP32
    ESP32_etat=1;
    ESP32_statut=0;
    tv = NB_MICROSEC();  // seconds et microsecondes
    OrCam_debut = (tv.tv_sec % 654)*100 + tv.tv_usec/10000;  // 2530 = 25,30sec   max:65000=10min

    uint16_t voltage_batt;
    lecture_volt_batterie (&voltage_batt, &voltage_panneau_sol);

}

// renvoie le temps d'allumage, en secondes
uint16_t extinction_esp32(void)
{
    uint16_t nb_sec_esp, tps;
    struct timeval tv;  // microsecondes

    ACTIV_PWM ( 1, 0, 0, 0, 0); // extinction LED IR

    Clock_stop(ESP32_sleepTimerClockHandle);

    #if Uart2 == 1
    if (Uart_actif)
    {
        Uart_actif = 0;
        Uart2_close();
    }
    #endif

    ACTIV_SORTIE(Enab_ESP, 0, 0);  // Extinction  ESP32
    ESP32_etat = 0;
    ESP32_statut = 0;
    OrCam = 0;

    tv = NB_MICROSEC();  // seconds et microsecondes
    tps = (tv.tv_sec % 654)*100 + tv.tv_usec/10000;

    nb_sec_esp = tps - OrCam_debut; // nb de 10milli.secondes a 93mA

    // envoi du temps d'allumage
    //if (camera_debug <> 2)  // 0:normal 1:PIR 2:PIR+cam absent 3:PIR+CAM
    //{
        strcpy((char *)message, "L Tps ESP:      s");
        envoi_int_decimal (nb_sec_esp, 10);
        message[15] = message[14];
        message[14] = message[13];
        message[13] = ',';
        envoi_message();
    //}
    //Viellissement par cycle partiel
    // partiel : perte 50% en 600 cycles, à 25°C   8%-600cycles à 25°C
    // partiel :
    //   0°C
    //   20°C : 2,5% en 250cycles
    //   25°C : 4% en 250 cycles  (3 fois plus en cycle total) : plutot 8%
    //   35°C : 7% en 250 cycles
    //   45°C : perte 50% en 300 cycles, à 25°C    30%-620à900cycles à 50°

    // Profondeur de décharge : DoD (cycles totaux Soh) : perte de 30%
    // 100%:600cycles  80%:700cycles  60%:900cycles  40%:1200cycles 20%:1800cycles 10%:1500cycles
    // charge à 4,1V => 500 cycles, charge à 4,2V => 1000 cycles
    // mesure temperature interne et validation mesure

    // 1sec à 50mA = 50/3300/3600 cycle
    uint32_t degrad;
    if (temp_mesure >25) degrad = (temp_mesure-25)/2 +8;  // 25°C:8%  40°C:18%
    else if (temp_mesure >0) degrad = 2 + temp_mesure/3;
    else degrad = 2;   // 0°C:2%

    //degrad = degrad/100/250*nb_sec_esp*50/3600/(batt_soh/100000) * batt_soh;
    degrad = (uint32_t)(nb_sec_esp/10) * degrad /18;
    batt_soh -= degrad;
    batt_soh_cycle -= degrad;

    // calcul SOC par comptage
    batt_soc_int -= (uint32_t)nb_sec_esp * 1248; // 1 sec à 50mA => 150/3600*1000*100
    batt_soc2 = batt_soc_int / (batt_soh / 10000) / batt_soc100;

    return (uint16_t) nb_sec_esp;
}

#endif



void activation_mode_periodique(uint8_t new)
{
    #ifdef CAPTEUR_TEMP
        if (mode_principal == 0)   // capteur temperature / hygrometrie
        {
            if (new)
                Clock_start(sensorTempClockHandle);
            else  // désativation clock temp
            {
                Clock_stop(sensorTempClockHandle);
                vidage_tampon();  // si buffer pour envoi en groupe
            }
            mode_periodique = new;
        }
    #endif

    #ifdef CAPTEUR_MPU
        if (mode_principal == 1)   // Algo 1 à 3 : mouvement
        {
            if (capteur_mpu_init)
            {
                if (new)
                    Activ_sensor_mpu();
                else
                {
                    Desactiv_sensor_mpu();
                    if ((index_flash_tail != index_flash_head))
                            demarr_lect_flash();                }
            }
            mode_periodique = new;
        }

        if (mode_principal == 2)   // Algo 4 : FFT
        {
            if (new)      //  activation clock algo 4
                chgt_mode_algo4(1);
            else
                chgt_mode_algo4(0);
            mode_periodique = new;
        }
    #endif
}

// 0:temp  1:mouv  2:FFT
void activation_mode_principal(uint8_t new)
{
    #ifndef CAPTEUR_TEMP
        #ifndef CAPTEUR_PRESS
        if (new == 0) new = 1;
        #endif
    #endif
    #ifndef CAPTEUR_MPU
        if ((new == 1) || (new==2))
                new = 0;
    #endif

        if ((new != mode_principal) && (mode_periodique))
        {
            // desactivation ancien mode
            if (mode_principal == 0) // desactivation clock temp
            {
                #ifdef OPT_CAPTEUR
                #ifdef CAPTEUR_TEMP
                    Clock_stop(sensorTempClockHandle);
                #endif
                #ifdef CAPTEUR_PRESS
                    Clock_stop(sensorPressClockHandle);
                #endif
                    vidage_tampon();  // si buffer pour envoi en groupe
                #endif
            }
            #ifdef CAPTEUR_MPU
                if (mode_principal == 1)
                    Desactiv_sensor_mpu();

                if (mode_principal == 2)
                    chgt_mode_algo4(0);
                if (new == 1)
                    Activ_sensor_mpu();

                if (new == 2)
                    chgt_mode_algo4(1);
            #endif
            // activation nouveau mode
            if (new == 0) // activation clock temp
            {
                #ifdef CAPTEUR_TEMP
                    Clock_start(sensorTempClockHandle);
                #endif
                #ifdef CAPTEUR_PRESS
                    Clock_start(sensorPressClockHandle);
                #endif
            }
        }
        mode_principal = new;
}




void Envoi_data_capteur(void)
{
/*    uint8_t i;
    if (envoi_data_capteur==2)   // envoi syst de la temp
    {
       message[0]=dest_data_capteur;
       strcpy((char *)message+1, "CAP:");
       i=5;

       if (capteur_temp)
       {
           strcpy((char *)message+i, "Temp: xx.xx");
           ajout_float_message(TempValue, 2, i+5, 0);
           i = i+11;
       }
       if (capteur_hygro)
       {
           strcpy((char *)message+i, "Hygro: xx.xx");
           ajout_float_message(HrValue, 2, i+6, 0);
           i = i+12;
       }
       if (capteur_lumi)
       {
           strcpy((char *)message+1, "Lumi:   xx0");
           envoi_int_decimal(LumValue,  i+5);
           i = i+11;
       }
       message[i+1]= car_fin_trame;
       envoi_message();
    }*/
}

#ifdef PILE_RF_FLASH
    void demarr_lect_flash()
    {
        if (!Clock_isActive(TimerFlashClockHandle))
            Clock_start(TimerFlashClockHandle);
    }
#endif

void Envoi_une_valeur(uint16_t valeur)
{
    uint32_t time_val;

    strcpy((char *)message, "LxTT");
    time_val = Seconds_get()%(60*60*24);
    message[1] = 8;  // longueur : 1 valeur
    message[4] = time_val/3600;
    message[5] = (time_val - message[4]*3600 )/60;
    message[6] =  time_val - message[4]*3600 - message[5];
    message[7] = 0;
    message[8] = 0;
    message[9]   = (uint8_t)(valeur >> 8);
    message[10] = (uint8_t)(valeur & 0xFF);
    envoi_message_def();
}

#ifdef RAM_TAMPON_SIZE

void Envoi_regulier_data(void)
{
    uint16_t numero_envoi, value_b;
    uint32_t secondes_rtc, time_val, time_envoi;
    uint8_t num_seg;

    #define NB_VAL_MESSAGE  20   // Nombre de valeurs (de 2 octets) par message

    if (option_envoi_reg)
    {
        compteur_option_envoi++;
        if (compteur_option_envoi > (NB_VAL_MESSAGE - nb_capteurs ))
        {
            secondes_rtc = Seconds_get();
            time_envoi = secondes_rtc%(60*60*24);

            strcpy((char *)message, "LxTT");
            time_val = time_envoi - (compteur_option_envoi -1)*period_capteur;
            message[4] = time_val/3600;
            message[5] = (time_val - message[4]*3600 )/60;
            message[6] =  time_val - message[4]*3600 - message[5];
            message[7] = (uint8_t) (period_capteur >> 8);
            message[8] = (uint8_t) (period_capteur & 0xFF);
            numero_envoi= index_ram_tournante - compteur_option_envoi ;
            if (numero_envoi > MAX_RAM_TOURNANTE) numero_envoi = numero_envoi + MAX_RAM_TOURNANTE;
            for (num_seg=0; num_seg < compteur_option_envoi; num_seg++)
            {
                value_b = capt_ram_tournante[numero_envoi++];
                if (numero_envoi >= MAX_RAM_TOURNANTE) numero_envoi=0;
                message[9+num_seg*2]   = (uint8_t)(value_b >> 8);
                message[9+num_seg*2+1] = (uint8_t)(value_b & 0xFF);
            }
            message[1] = 6 + (num_seg)*2;
            envoi_message_def();
            compteur_option_envoi=0;
        }
    }
}

void ajout_ram_tournante (uint16_t Value_brute)
{
    capt_ram_tournante[index_ram_tournante++] = Value_brute;
    if (index_ram_tournante == MAX_RAM_TOURNANTE) index_ram_tournante=0;
}

void envoi_ram_valeur (uint8_t ca)
{
    uint16_t value_b;
    uint32_t secondes_rtc, time_val, time_envoi;
    uint8_t num_seg, index_mess;

    if (index_ram_tournante >= capt[ca].max_nb)
    {
        secondes_rtc = Seconds_get();
        time_envoi = secondes_rtc%(60*60*24);

        // QxDCPssssvvvv  ->
        strcpy((char *)message, "QxD");  //  puis C et P et ssss et vvvv
        time_val = time_envoi - (capt[ca].max_nb -1)*period_conversion(capt[ca].period)/(1000*1000/Clock_tickPeriod);
        message[3] = capt[ca].codage_trans ;
        message[4] = capt[ca].period;
        index_mess=5;
        if (message[3] & 0x80)  // ajout de la date
        {
            strncpy((char *)(message+5), (char *) &time_val, 4);
            index_mess = 9;
        }
        for (num_seg=0; num_seg < capt[ca].max_nb; num_seg++)
        {
            value_b = capt_ram_tournante[num_seg];
            message[index_mess++]   = (uint8_t)(value_b >> 8);
            message[index_mess++] = (uint8_t)(value_b & 0xFF);
        }
        message[1] = index_mess - 3;
        Mess_statut = 0;  // 0:ack, pas de RX  30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
        envoi_message_def();
        index_ram_tournante=0;
    }
}

void traitement_data_capteur(void)
{
    // Enreg dans une mémoire tournante
    if (capteur_temp)   ajout_ram_tournante(ValueBrut);
    if (capteur_hygro)  ajout_ram_tournante(Value2Brut);
    if (capteur_lumi)   ajout_ram_tournante(ValueBrut);

    if (Ram_tampon_index == 0)
    {
        Heure_debut = Seconds_get();
    }
    if (capteur_temp)  Ram_tampon_flash[Ram_tampon_index++] = (uint16_t) (ValueBrut );
    if (capteur_hygro) Ram_tampon_flash[Ram_tampon_index++] = (uint16_t) (Value2Brut);
    if (capteur_lumi)  Ram_tampon_flash[Ram_tampon_index++] = (uint16_t) (ValueBrut);
    if ((Ram_tampon_index) >= (FLASH_PAGE_SIZE-nb_capteurs-1))  // plus assez de place pour le prochaine enreg
    {
        ExtFlash_open();

        // effacement du secteur si la page est multiple de 16
        if (!(Flash_page_index & 0xF))  // 1 page sur 16 => effacement secteur suivant
                ExtFlash_erase( FLASH_ADDRESS (Flash_page_index,0), 4096);  // 256 secteurs de 4096 octets

        ExtFlash_write( FLASH_ADDRESS (Flash_page_index,0) , 4, (const uint8_t *) &Heure_debut, 0);
        ExtFlash_write( FLASH_ADDRESS (Flash_page_index,4),  Ram_tampon_index*2 , (const uint8_t *) Ram_tampon_flash, 0);

        Ram_tampon_index = 0;
        Flash_page_index++;
        if (Flash_page_index == 4096) Flash_page_index=0;

        ExtFlash_close();

    }
}
#endif


uint8_t retourne_last_rssi(void)
{
    return latestRxPacket.rssi ;
}

// Callback (en interruption) : ne pas envoyer de message dans l'interruption ! utiliser Event_post

static void packetReceivedCallback(struct Message_RF_Packet* packet, int8_t rssi)
{   // Task : Radio

    // If we received a message packet
    if (packet->RF_payload.header.packetType == RADIO_PACKET_TYPE_SIMPLE_MESSAGE_PACKET)
    {
        Event_post(nodeEventHandle, NODE_EVENT_SIMPLE_MESSAGE);

        if (activ_led_reponse)   // active la led si bouton 1 appuyé et message recu
        {
            ACTIV_SORTIE(LED_cpu, 1, 1);  // activation Led N°0, mode:1, duree 0.1s
            activ_led_reponse=0;
        }
    }
}

void MajHeureCallback(UArg arg0)
{
    // Post event pour effectuer une demande HLS de mise à jour de l'heure
    Event_post(nodeEventHandle, NODE_EVENT_MAJ_HEURE);
}


void sensorTempCallback(UArg arg0)
{
    Event_post(nodeEventHandle, NODE_EVENT_READ_SENSOR_TEMP);
}

void sensorLumiCallback(UArg arg0)
{
    Event_post(nodeEventHandle, NODE_EVENT_READ_SENSOR_LUMI);
}

void sensorPressCallback(UArg arg0)
{
    Event_post(nodeEventHandle, NODE_EVENT_READ_SENSOR_PRESS);
}

#ifdef PERIODE_CAPTEUR
void sensorCapteurCallback(UArg arg0)
{
    Event_post(nodeEventHandle, NODE_EVENT_SEND_CAPTEUR);
}
#endif


#ifdef C10_Compteur   // Callback apres Timeout sans tick
void sensorCapteurTimeoutCallback(UArg arg0)
{
    Event_post(nodeEventHandle, NODE_Fin_timer_compteur);
}
#endif


void Timer_EEPROM_Callback_task(void)
{
    // Post event pour enregistrer les index flash en EEPROM
    Event_post(nodeEventHandle, ENREG_EEPROM);
}


void Inter_ConcCallback(UArg arg0)
{
    // Post event toute les 6 heures
    Event_post(nodeEventHandle, EVENT_KEEP_ALIVE);
}

void RetenCallback(UArg arg0)
{
    // Post event
    Event_post(nodeEventHandle, EVENT_VIDAGE_TAMPON);
}

void SORTIE_S_Callback(UArg arg0)
{
    // Post event
    //uint8_t num_sor =  (uint8_t) arg0;
    timer_inter |= (1 << ((uint8_t) arg0));
    Event_post(nodeEventHandle, EVENT_TIMER_S);
}

#ifdef C10_ElecVanne    // Callback timer pour fermeture vanne et démarrage programme
    void SORTIE_EV_Callback(UArg arg0)
    {
        // Post event
        timer_ev_inter |= (1 << ((uint8_t) arg0));
        Event_post(nodeEventHandle, EVENT_TIMER_EV);
    }

    void EV_prog_Callback(UArg arg0)
    {
        // Post event
        if (EV_prog_timer_rest)
            EV_prog_timer_rest--;
        if (!EV_prog_timer_rest)
            Event_post(nodeEventHandle, EVENT_PROG_EV);
    }


#endif

void SORTIE_PWM_Callback(UArg arg0)
{
    uint8_t num_pwm =  (uint8_t) arg0;
    Event_post(nodeEventHandle, EVENT_TIMER_PWM0 << num_pwm);
}


#ifdef RAM_TAMPON_SIZE
void TimerRamCallback(UArg arg0)
{  // pour envoyer, toutes les 100ms un message Radio, pour décharger la RAM
    Event_post(nodeEventHandle, EVENT_RAM_A_ENVOYER);
}
#endif

void TimerFlashCallback(UArg arg0)
{   // pour envoyer, toutes les 30ms (à50kbps) un message Radio, pour décharger la Flash
    Event_post(nodeEventHandle, EVENT_FLASH_A_ENVOYER);
}

void Test_mess_Callback(UArg arg0)
{   // Teste periodiquement si la passerelle est dispo, pour décharger la Flash
    Event_post(nodeEventHandle, EVENT_TEST_PASSERELLE_DISPO);
}


void Hors_portee_timer_Callback (UArg arg0)    // fin du timer pour renvoyer TX suite hors portée
{
    Clock_stop(Hors_portee_timerHandle);  // inutile
    Event_post(nodeEventHandle, NODE_EVENT_HORS_PORTEE);
}

#ifdef C10_CAMERA_PIR  // Callback pour eteindre ESP32
void ESP32_sleepTimerClockCallback(UArg arg0)    // fin du timer pour eteindre l'uart vers l'ESP32, et l'ESP32
{
    Event_post(nodeEventHandle, EVENT_NODE_EXTINCT_ESP);
}
#endif

// Test
void  TestMessLongClockCallback(UArg arg0)
{
    Event_post(nodeEventHandle, NODE_MESS_LONG);   // Test envoi message long
}


void DetectionCallback (UArg arg0)
{

}

void TimerVoltageInitialCallback(UArg arg0)
{
  #ifndef DEBUG_M
     prec_tick_voltage = Clock_getTicks();  // evite le renvoi d'une valeur dans la seconde
     volt_mesure_bas = - mesure_volt_vref();  // mesure voltage initial apres 7 secondes
     Event_post(nodeEventHandle, NODE_EVENT_ENVOI_VOLTAGE);
  #endif
    }

void TimerTemp1mesureHandleCallback(UArg arg0)
{
    forcage_envoi_capteur=1;
    Event_post(nodeEventHandle, NODE_EVENT_READ_SENSOR_TEMP);
}

void TimerBattmesureHandleCallback(UArg arg0)
{
    Event_post(nodeEventHandle, NODE_EVENT_MESURE_CHARGE_BATT);
}

#ifdef TACHE_NODE_WDT
    void WdtNodeClockCallback(UArg arg0)
    {
        Event_post(nodeEventHandle, EVENT_TACHE_NODE_WDT);
    }
#endif

void event_envoi_voltage(void)
{
    Event_post(nodeEventHandle, NODE_EVENT_ENVOI_VOLTAGE);
}

void hors_portee_calcul_periode(void)
{
    char unsigned i,j;
    uint32_t k;

    j=30;
    for (i=0;i<lastAddedSensorNode;i++)   // Recherche le node avec etat le plus bas
    {
        if ((knownNodes[i].hors_portee_etat)  && (knownNodes[i].hors_portee_etat < j ))
            j =knownNodes[i].hors_portee_etat;
    }
    Clock_stop(Hors_portee_timerHandle);
    if (j == 30) // pas d'autre node hors portee
    {
        hors_portee_timing=0;   // plus d'autre envoi - conserve le temps de la clock hors_portee
    }
    else
    {
        //k = ((j*4) *1000 * (1000 / Clock_tickPeriod));
        k = ((1<<(2*(j-1))) *1000 * (1000 / Clock_tickPeriod));   // j=1=>1s  j=2=>4s j=3=>16s j=10=>18h j=11=>3j
        Clock_setPeriod(Hors_portee_timerHandle, k ); // Period=0 pour un one-shot
        Clock_setTimeout(Hors_portee_timerHandle, k );
        Clock_start(Hors_portee_timerHandle);
        hors_portee_timing=j;   // conserve le temps le plus petit de la clock hors_portee
    }
}

void hors_portee_augmente(uint8_t num_adres)
{
    if (knownNodes[num_adres].hors_portee_etat <= 10)
    {
        knownNodes[num_adres].hors_portee_etat++;
    }
    hors_portee_calcul_periode();
}

void hors_portee_desactive(uint8_t num_adres)
{
    knownNodes[num_adres].hors_portee_etat=0;
    hors_portee_calcul_periode();
}


#ifdef C10_CAMERA_PIR  // erreur ESP
void err_esp_extinct ( uint8_t code)
{
    code_erreur = erreur_esp;  // 0x76
    err_donnee1 = code + '0';
    err_donnee2 = OrCam + '0';
    extinction_esp32();
}
#endif

void traitement_message_appli (char unsigned longueur_m)  // CTA = longueur=5
{
    // reception message CH1 -> Alarme chien
    if ((message_in[2] == 'C') && (message_in[3] == 'H') && (longueur_m==5)) // CH1
    {
        DELAY_MS(2000);
        ACTIV_SORTIE(SORTIE_1, 1, 20);  // activation HP chien, mode:1, duree 13s
        ACTIV_SORTIE(LED_cpu, 1, 20);  // activation HP chien, mode:1, duree 13s
    }

    // ******************************** TTTTTTTTTTTTTTTTTTTTTT  ********************  Test


    if ((message_in[2] == 'T') && (message_in[3] == 'A') && (longueur_m==5)) // TAx
    {
        ACTIV_SORTIE(LED_cpu, 1, 6);  // activation Led N°0, mode:1, duree 0.6s
        if (message_in[4] == '1')   // TA1
        {
            Mess_statut = 0x10;  // pas d'Ack/pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
            strcpy((char *)message, "LTA1");
            envoi_message ();
        }
        if (message_in[4] == '2')   // TA2
        {
            Mess_statut = 0x10;  // pas d'Ack/pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
            strcpy((char *)message, "LTA2");
            envoi_message ();
            strcpy((char *)message, "LTA2b");
            envoi_message ();
        }
        if (message_in[4] == '3')   // TA3
        {
            Mess_statut = 0x20;  // Ack / RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
            strcpy((char *)message, "LTA3");
            envoi_message ();
            strcpy((char *)message, "LTA3b");
            envoi_message ();
        }
    }


    // ****************  BBBBBBBBB   BMS batterie ******************
#ifdef BATT_BMS
    if ((message_in[2] == 'B') && (message_in[3] == 'M'))  // BM
    {
        if (message_in[4] == 'L')
        {
            if (message_in[5]=='S')
            {
                if (message_in[6]=='O') // BMLSO : Lecture Soc et Soh
                {
                    envoi_soc_soh();
                }
                if (message_in[6]=='T')  // BMLST : Lecture Tmp, VolBatt, Volt panneau
                {
                    strcpy((char *)message+1, "BM:Temp:       Batt:       Sol:       ");
                    int16_t valeur = AONBatMonTemperatureGetDegC() ;
                    ajout_int8_message (valeur, 9, 0x82);

                    // mesure voltage batterie et panneau solaire
                    uint16_t voltage_batt;
                    uint16_t voltage_panneau;
                    uint8_t ret = lecture_volt_batterie (&voltage_batt, &voltage_panneau);

                    ajout_int16_message (voltage_batt, 21, 0x82);

                    //uint32_t volt_mesure;
                    //volt_mesure = AONBatMonBatteryVoltageGet();
                    //val_volt = volt_mesure * 125 / 32;  // en millivolts

                    ajout_int16_message (voltage_panneau, 32, 0x82);
                    envoi_message ();
                }
                if (message_in[6]=='H')  // BMLSH : Lecture SOH%, calend, cycle, SOH32
                {
                    strcpy((char *)message+1, "BM:Soh:   % calend:   % cycle:   % SOH:         ");
                    uint8_t Soh = batt_soh / (BATT_SOC0 / 100);
                    ajout_int8_message (Soh, 8, 0x82);

                    uint8_t calend = batt_soh_cal / ((batt_soh_cal + batt_soh_cycle) / 100);
                    ajout_int8_message (calend, 20, 0x82);

                    uint8_t cycle = batt_soh_cycle / ((batt_soh_cal + batt_soh_cycle) / 100);
                    ajout_int8_message (cycle, 31, 0x82);

                    ajout_word_message (batt_soh, 40);
                    envoi_message ();
                }
            }
        }
        if (message_in[4] == 'E')
        {
            if ((message_in[5]=='S') && (message_in[6]=='O') && (message_in[7]=='H'))  // BMESOH : Raz Soh  ou Soh à XX%
            {
                if (longueur_m==8)
                {
                    batt_soh = BATT_SOC0;
                    batt_soh_cal = 0;  // cumul du vieillismeent calendaire en 10nAh
                    batt_soh_cycle =0;   // cumul du viellissement utilisation en 10nAh
                }
                if (longueur_m==10)
                {
                    uint8_t soh_cent;
                    soh_cent = message_in[8]*10 + message_in[9];
                    batt_soh = (BATT_SOC0 / 100) * soh_cent;
                    batt_soh_cal = (BATT_SOC0 - batt_soh) / 2;  // cumul du vieillismeent calendaire en 10nAh
                    batt_soh_cycle = batt_soh_cal;   // cumul du viellissement utilisation en 10nAh

                }
            }
        }
    }
#endif

    // ******************** CCCCCCCCCCCC Capteurs *********************

    if (message_in[2] == 'C')
    {
        //   CEMPx   : activation mode periodique  0:inactif  1:periodique
        if ( (message_in[4] == 'M') && (message_in[5] == 'P'))
        {
            if ((message_in[3] == 'L') && (longueur_m==6))  // CLMP  Lecture mode periodique
            {
                strcpy((char *)message+1, "SLMP:x");
                message[6]=mode_periodique+'0';
                envoi_message ();
            }
            if ((message_in[3] == 'E') && (longueur_m==7))  // CEMPx Ecriture mode periodique
            {
                uint8_t i;
                i = (message_in[6]-'0')%2;
                activation_mode_periodique(i);  // extinction ou periodique
            }
        }


        if ((message_in[3] == 'E') && (message_in[4] == 'M') && (longueur_m==6)) //  CEMx: mode principal capteur
        {
                 uint8_t i;
                 i = message_in[5] - '0';
                 if ( i < 3 )
                 {
                     activation_mode_principal(i);
                 }
        }

        #ifdef C10_Compteur   // CLE : Lecture index  CEExxxxx:ecriture
            if ((message_in[3] == 'L') && (message_in[4] == 'E') && (longueur_m==5)) //  CLE : Lecture compteur eau
            {
                envoie_mesure_compteur_eau(1);  // 0:hexa  1:texte
            }
            if ((message_in[3] == 'E') && (message_in[4] == 'E') && (longueur_m==10)) // CEExxxxx : Ecriture compteur d'eau (apres reset) en m3
            {
                cpt_somme_cpt = (uint16_t) decodage16_message_in(5 , 5);  // 5 chiffres, 5°position
                envoie_mesure_compteur_eau(1);  // 0:hexa  1:texte
            }
        #endif

        #ifdef C10_Cent_Meteo   // CLM : Lecture conso centrale meteo
            if ((message_in[3] == 'L') && (message_in[4] == 'M') && (longueur_m==5)) //  CLM : Lecture centrale meteo
            {
                envoie_mesure_pluviometre(1);  // 0:hexa  1:texte
            }
        #endif


        #ifdef C10_sonde_HUmSol   // CLH lecture sonde
            if ((message_in[3] == 'L') && (message_in[4] == 'H') && (longueur_m==5)) //  CLH : Lecture sonde humidite
            {
                envoie_mesure_sonde_HumSol(1);  // 0:hexa  1:texte
            }
        #endif

        if ((message_in[3] == 'L') && (message_in[4] == 'U') && (message_in[5] == 'N') && (longueur_m==6)) //  CLUN : Lecture unique
        {
            #ifdef CAPTEUR_TEMP
                if (mode_principal == 0)
                {
                    S_Temp Temp;
                    float Ftemp;
                    mesure_temperature(&Temp);
                    Ftemp = (float) Temp.temp / 100;

                    strcpy((char *)message+1, "H Temp: xx.xx");
                    ajout_float_message(Ftemp, 2, 8, 0);
                    Mess_statut=0;  // bit0:dernier  bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)
                    // bit3:différé   bit4:pas d'ack  bit5:RX apres
                    envoi_message (); // Ack, pas Rx apres
                }
            #endif


            #ifdef CAPTEUR_MPU
                if (mode_principal==1)  // mouvement
                {
                }
                if (mode_principal==2)  // FFT
                {
                    lancement_algo4();
                }
            #endif
        }
    }

    #ifdef CAPTEUR_TEMP
    //  CLTT : lecture temperature et envoi des donnees en attente dans le buffer
    if ((message_in[2] == 'C') &&  (message_in[3] == 'L')  && (message_in[4] == 'T') &&  (message_in[5] == 'T') && (longueur_m==6))
    {
        Clock_stop(TimerTemp1mesureHandle);
        Clock_setPeriod(TimerTemp1mesureHandle, 0);
        Clock_setTimeout(TimerTemp1mesureHandle, 300 *  (1000  / Clock_tickPeriod)) ;  // 300 ms
        Clock_start(TimerTemp1mesureHandle);
    }


    if ((message_in[2] == 'C') &&  (message_in[3] == 'T')) //  CT capteur temperature/hygrometrie
    {
        /*if ((message_in[4] == 'A') && (longueur_m==6))  // CTAx : actif/inactif
        {
            if (message_in[5]=='0')
                Clock_stop(sensorTempClockHandle);

            if (message_in[5]=='1')
                Clock_start(sensorTempClockHandle);
        }*/
        if ((message_in[4] == 'P') && (message_in[5] == 'U') && (longueur_m==11))  // CTPU :Periode de mesure unitaire
        {
            Clock_stop(sensorTempClockHandle);
            Temp_period_U = (uint16_t) decodage16_message_in(5 , 6);  // 5 chiffres, 6°position
            Clock_setPeriod(sensorTempClockHandle, (Temp_period_U *  (1000  / Clock_tickPeriod) * 1000));
            Clock_setTimeout(sensorTempClockHandle, (Temp_period_U *  (1000  / Clock_tickPeriod) * 1000));
            Clock_start(sensorTempClockHandle);
            //ACTIV_SORTIE(LED_cpu, 1 , 2);     // 1 flash court
        }
        if ((message_in[4] == 'E') && (message_in[5] == 'E') && (message_in[6] == 'M') && (longueur_m==10))  // CTEEMxxx : Ecart minimum pour la prise de mesure
        {
            Temp_ecart_min = (uint8_t) decodage16_message_in( 3, 7);  // 3 chiffres, 7°position
        }
        if ((message_in[4] == 'L') && (message_in[5] == 'E') && (message_in[6] == 'M') && (longueur_m==7))  // CTLEM : Lecture Ecart minimum pour la prise de mesure
        {
            uint8_t posit;
            strcpy((char *)message+1, "CTLEM Ecart Min Temp:         ");
            posit = ajout_int16_message (Temp_ecart_min, 23,2);
            message[posit-1]=' ';
            ajout_float_message((float)Temp_ecart_min/400, 2, 27, 2);
            envoi_message ();
        }
        if ((message_in[4] == 'P') && (message_in[5] == 'M') && (longueur_m==8))  // CTPM : Periode de mesure minimum
        {
            Temp_period_min = (uint8_t) decodage16_message_in( 2, 6);  // 2 chiffres, 6°position
        }
        if ((message_in[4] == 'G') && (longueur_m==6))  // CTGx : groupe/individuel
        {
            if ((message_in[5]=='0') || (message_in[5]=='1'))
                Temp_groupes = message_in[5] - '0';
        }
        if ((message_in[4] == 'L') && (message_in[5] == 'D') && (longueur_m==6))    // CTLD : envoi la derniere Temp
        {
            S_Temp Temp;
            float Ftemp;
            mesure_temperature(&Temp);
            Ftemp = (float) Temp.temp / 100;
            strcpy((char *)message+1, "H Temp: xx.xx");
            ajout_float_message(Ftemp, 2, 8, 0);
            envoi_message ();
        }
    }
    #endif

    if ((message_in[2] == 'C') &&  (message_in[3] == 'G')) //  CG envoi en groupes
    {
        if ((message_in[4] == 'N') &&  (message_in[5]=='B') && (longueur_m==7))  // CGNBx : Nb max de mesures envoyees en meme temps
        {
            Groupes_nb  = message_in[6] - '0';
        }
        if ((message_in[4] == 'E') && (message_in[5] == 'T') && (longueur_m==9))  // CGETxxx : temps max de retention des donnees, en minutes, max 715
        {
            Groupes_temps = decodage16_message_in( 3, 6);  // 3 chiffres, 6°position
        }
        if ((message_in[4] == 'L') && (message_in[5] == 'T') && (longueur_m==6))  // CGLT : lecture temps max de retention des donnees, en minutes
        {
            strcpy((char *)message+1, "CGLT temps max(min):      ");
            ajout_int16_message (Groupes_temps, 22,0);
            envoi_message ();
        }

    }
    if (message_in[2] == 'C')  //  C  capteur
    {
        //uint8_t tempo1, tempo2, tempo3;

        if (message_in[3] == 'L')
        {

            #ifdef PILE_RF_FLASH
            if ((message_in[4] == 'F') && ((message_in[5] == '0') || (message_in[5] == '1')))
            {
                if (longueur_m == 6)                         // CLF0 ou CLF1 Lecture flash MPU
                {
                    demarr_lect_flash();
                }
            }
            #endif


            #ifdef OPT_CAPTEUR
            if (((message_in[4] == 'T') && (longueur_m==6) && ((message_in[5] == 'H') || (message_in[5] == 'D'))))         // CLTD CLTH  Lecture température
            {
                strcpy((char *)message+1, "TL Temp:          Hygro:    ");
                S_Temp Temp;
                float Ftemp, Fhygro;
                mesure_temperature(&Temp);
                Ftemp = (float) Temp.temp / 100;
                if (message_in[5]=='D')  // en décimal :  23,34
                {
                    ajout_float_message(Ftemp, 2, 9, 0x80);
                    Fhygro = (float) Temp.hygro / 100;
                    ajout_float_message(Fhygro, 2, 25, 0);
                    envoi_message ();
                }
                if (message_in[5]=='H')   // en hexa 16 bits : 611C
                {
                    ajout_message_16b(Temp.temp, 10);
                    message[14] = car_fin_trame;
                    envoi_message ();
                }
            }
            #endif
        }
    }

    #ifdef OPT_CAPTEUR
        if (message_in[4] == 'C')  // Ecriture capteur hygrométrie
        {
            if ( (longueur_m==9))
            {
                if (  (message_in[5] == 'R') && (message_in[6] == 'E') ) //  CECREGx Config Envoi regulier TX capteur 0:pas d'envoi, 1:envoi Data
                {
                    option_envoi_reg = message_in[8]  - '0';
                    dest_data_capteur = message_in[1];
                }
            }
            if ( (longueur_m==10))
            {                                        //  CECPY45s Config periodicité mesure capteur Y
                if  (message_in[5] == 'P')
                {
                    period_capteur = (((message_in[7]-'0')*10 + message_in[8]-'0')<<2) ;      // en 0,01 secondes
                    if (message_in[9]=='s')            period_capteur = period_capteur + 1;        // secondes
                    else if (message_in[9]=='h')       period_capteur = period_capteur + 3;        // heures
                    else if (message_in[9]=='m')       period_capteur = period_capteur +2;          // minutes

                    uint8_t ii = message_in[6]-'0';
                    capt[ii].period = period_capteur;
                }
            }
            if ( (longueur_m==8))
            {
                if (  (message_in[5] == 'T') && (message_in[6] == 'R') ) //  CECTRx Config Envoi TX capteur 0:pas d'envoi, 2:envoi Data
                {
                    envoi_data_capteur = message_in[7]  - '0';
                    dest_data_capteur = message_in[1];
                }
            }
        }
    #endif


    // ************************** KKKKKKKKKKKKKKK ***************************************

    #ifdef TEMPS_INTERROG_MIN
        if (message_in[2] == 'K')
        {
            if ((message_in[3] == 'P') && (message_in[4] == 'I'))  // KPI : modif keepalive
            {
                if ((message_in[5] == 'm') && (longueur_m==11))  // KPImxxxxx : modif periode keepalive (temp_min)
                {
                    uint16_t temps_interrog_conc;
                    temps_interrog_conc = decodage16_message_in( 5, 6);  // 5 chiffres, 6°position
                    if (temps_interrog_conc<42900)
                    {
                        Clock_stop(Inter_ConcClockHandle);
                        Temps_interrog_min = temps_interrog_conc;
                        Clock_setPeriod (Inter_ConcClockHandle, (uint32_t) Temps_interrog_min * (1000  / Clock_tickPeriod * 1000));
                        Clock_setTimeout (Inter_ConcClockHandle, (uint32_t) Temps_interrog_min * (1000  / Clock_tickPeriod * 1000));
                        Clock_start(Inter_ConcClockHandle);
                    }
                }
                if ((message_in[5] == 'M') && (longueur_m==9))  // KPIMxxx : modif periode keepalive (temp_maj)
                {
                    uint16_t temps_interrog_conc;
                    temps_interrog_conc = decodage16_message_in( 3, 6);  // 3 chiffres(max:255), 6°position
                    if (temps_interrog_conc<255)
                    {
                        Temps_interrog_maj = temps_interrog_conc;
                        Temps_interrog_maj_cpt = temps_interrog_conc;
                    }
                }
            }
        }
     #endif

    // ******************************** PPPPPPPPPPPPPPPP ****************************

    #ifdef SPOT_TOR_PWM
    if ((message_in[2] == 'P') && (message_in[3] == 'E') && (message_in[4] == 'S'))
    {
       // PESJ/N/S : Spot solaire duree jour(YYY:0,1s - PASJ5) / duree nuit (xxxx:1s ( PESN300) / seuil (YYY:10mv - PESS20)
        if (message_in[5] == 'J')
        {
            Spot_allumage_jour = (uint8_t) decodage16_message_in( 3, 6);  // 3 chiffres, 6°position
        }
        if (message_in[5] == 'N')
        {
            Spot_allumage_nuit = (uint16_t) decodage16_message_in( 4, 6);  // 4 chiffres, 6°position
        }
        if (message_in[5] == 'S')
        {
            Spot_allumage_nuit_seuil = (uint8_t) decodage16_message_in( 3, 6);  // 3 chiffres, 6°position
        }
    }
    #endif

    // ******************************  SSSSSSSSSSSSSSSSS ****************************


    //   SEBOx   : config boutons 0:boutons desactivés  1:B0C+B1C   2:tous actives
    if (message_in[2] == 'S')
    {
        if ((message_in[4] == 'B') && (message_in[5] == 'O'))
        {
            if ((message_in[3] == 'L') && (longueur_m==6))  // SLBO  Lecture activation boutons
            {
                strcpy((char *)message+1, "SLBO:x");
                message[6]=Config_boutons+'0';
                envoi_message ();
            }
            if ((message_in[3] == 'E') && (longueur_m==7))  // SEBOx Ecriture activation boutons
            {
                Config_boutons = message_in[6]-'0';
            }
        }

        #ifdef FLASH_EXTERNE
        if ((message_in[3] == 'E') && (message_in[4] == 'F') && (longueur_m==8))         // SEFraz  Raz index Flash et Ram et RAZ EEPROM
        {
            if ((message_in[5] == 'r') && (message_in[6] == 'a') && (message_in[7] == 'z'))
            {
                EEPROM_raz_appli();
                Pile_F[0].tail = 0;
                Pile_F[0].head = 0;

                sauve_eeprom_index_flash();
                #ifdef RAM_TAMPON_SIZE
                    Ram_tampon_index = 0;
                    Flash_page_index = 0;
                #endif
                ACTIV_PWM ( 0,4,20,1500,5);        // Raz Index Flash et Ram

            }
        }
        #endif
    }


    //  ******************  TTTTTTTTTTTTTTTTTTTTT *********************

    if ((message_in[2] == 'T') && (message_in[3] == 'V'))   // TVx
    {
        #ifdef BATT_BMS
        // mesure voltage batterie et panneau solaire
        uint16_t voltage_batt;
        uint16_t voltage_panneau;

        uint8_t val = message_in[4] - '0';
        uint8_t dest = message[0];

        lecture_analog(1, &voltage_batt);
        voltage_batt*=2;
        lecture_analog(0, &voltage_panneau);
        voltage_panneau*=2;

        strcpy((char *)message+1, "BM:init:       Batt:       Sol:       ");
        ajout_int16_message (voltage_batt, 21, 0x82);
        ajout_int16_message (voltage_panneau, 32, 0x82);
        envoi_message ();

        ACTIV_SORTIE( Charge_TP, 0, 0 );  // desactivation du circuit de charge

        DELAY_MS(val);

        lecture_analog(1, &voltage_batt);
        voltage_batt*=2;
        lecture_analog(0, &voltage_panneau);
        voltage_panneau*=2;

        strcpy((char *)message+1, "BM:Tempo:      Batt:       Sol:       ");
        message[0] = dest;
        message[11]= val+'0';
        ajout_int16_message (voltage_batt, 21, 0x82);
        ajout_int16_message (voltage_panneau, 32, 0x82);
        envoi_message ();

        ACTIV_SORTIE( Charge_TP, 1, 0 );  // reactivation du circuit de charge
        #endif
    }

    /*
     Fxx : film/photo
     Wi1 : Wifi se reliant au routeur
     Wi2 : Wifi se reliant a un PC (AP)
     BEx : baterie soc1
     DL : Download liste
     DF1xy : Download film/photo xy (1:photo, 2:petit film, 3:gd film)

     */

    if ((message_in[2] == 'T') && (message_in[3] == 'M') && (message_in[4] == 'L'))   // TMLx
    {
        send_long_num_trame = 0;
        send_long_num_trame_tx = message_in[5] - '0';
        CLOCK_START(TestMessLongClockHandle);
        //Event_post(nodeEventHandle, NODE_MESS_LONG);
    }

    /*if ((message_in[2] == 'W') && (message_in[3] == 'E'))
    {
        if ((message_in[4] == 'T') && (longueur_m == 6)) //--------- reponse WETx : Transfert fichier 0:ok dispo, 1:ok attente, 2:nok 3:dispo
        {
          if (message_in[5] == '2')  // nok
          {
            if (send_long_repeat < 3 )  // repete 3 fois
            {
              send_long_repeat++;  // TODO il faudrait garder le message, pour pouvoi le renvoyer
              Event_post(nodeEventHandle, NODE_MESS_LONG);
            }
            else
                send_long_actif = 0;
          }
          if (message_in[5] == '1')  // ok attente
          {
            if (send_long_actif)
            {
              send_long_num_trame++;
              send_long_repeat=0;
            }
          }
          if (message_in[5] == '0')  // ok dispo
          {
            if (send_long_actif)   // envoi trame suivante
            {
              send_long_num_trame++;
              send_long_repeat=0;
              //Event_post(nodeEventHandle, NODE_MESS_LONG);
            }
          }
          if (message_in[5] == '3')  //  dispo
          {
              //Event_post(nodeEventHandle, NODE_MESS_LONG);
          }
        }
    }*/


    // ----------------- WWWWWWWWWWWW -------------------------

    #ifdef C10_CAMERA_PIR  // messages reçus de l'ESP (WTP, WK) et commandes (WLF, WEV, WEP,
    if (message_in[2] == 'W')
    {
      // Processus
      //  CC1350 :                  ESP32 :
      //  allumage
      //                        envoie WLSN
      //  envoie HES(et lumi)
      //                        init cam-eeprom puis envoie WK1
      //  envoie WEV502
      //                  prise de photo  envoie WK3
      //  extinction

      if ((message_in[1] == '1') && (message_in[3] == 'T') && (message_in[4] == 'P'))  // 11WTP
      {
          // TEST pour mesure du temps photo
          uint32_t tps;
          uint16_t nb_sec_esp;
          struct timeval tv;  // microsecondes
          tv = NB_MICROSEC();  // seconds et microsecondes
          tps = (tv.tv_sec & 0xFFF)*100 + tv.tv_usec/10000;

          nb_sec_esp = tps - OrCam_debut; // nb de 10milli.secondes a 93mA
          strcpy((char *)message, "L Photo:         s");
          envoi_int_decimal (nb_sec_esp, 10);
          message[15] = message[14];
          message[14] = message[13];
          message[13] = ',';
          message[16] = 's';
          message[17] = car_fin_trame;
          envoi_message();

      }
      if ((message_in[1] == '1')  && (message_in[3] == 'K'))  // 11WKx.. 1:fin init  2:wlf 3:photo
      {
          if ((message_in[4] == '1') && (longueur_m==11)) // 11WK1yertdL init
           // 1WK1yertdL // x:type=1(init) y:statut(0:ok 7:tout rate) e:nb essai  r:reset  t:tag d:duree L:lumi
          {
              // envoie du message au PC
              memcpy (message, message_in+1, 11);  // 11WK1yertdL
              message[0] = 'L';  // LWK1yertdL
              envoi_message();

              // TEST pour mesure du temps photo
              /*uint32_t tps;
              uint16_t nb_sec_esp;
              struct timeval tv;  // microsecondes
              tv = NB_MICROSEC();  // seconds et microsecondes
              tps = (tv.tv_sec & 0xFFF)*100 + tv.tv_usec/10000;

              nb_sec_esp = tps - OrCam_debut; // nb de 10milli.secondes a 93mA

              message[8]=' ';
              message[9]=' ';
              message[10]=' ';
              message[11]=' ';
              message[12]=' ';
              envoi_int_decimal (nb_sec_esp, 10);
              message[15] = message[14];
              message[14] = message[13];
              message[13] = ',';
              message[16] = 's';
              message[17] = ' ';
              message[18] = message_in[9]; // d:duree
              message[19] = voltage_panneau_sol/256 + 'A';  // 0 à 22 -> A à Z
              message[20] = car_fin_trame;
              envoi_message(); */

          }
          // : message ok de l'ESP  x:1:init 2:wlf 3:photo  y:0(ok) 1(pb SD) 2(pb cam) 4(pb heure)
          // message_in[5] : bit 0:camera   1:carte SD   2:heure
          // OrCam = 1(extinction) 2(WLF) 3(prise photo) 4(wifi)

            if (OrCam == 1)  // extinction  WKxy (x:1:init  2:WLF  3:photo)  y:0:ok  1:nok
            {
              //uint16_t tps_esp;
              //tps_esp =
                extinction_esp32();
              if (message_in[5] != '0')  // retour pas ok
                  { code_erreur = erreur_esp; err_donnee1 = '7'; err_donnee2 = message_in[4]; }
              /*else // ok : renvoie le temps d'allumage
              {
                  strcpy((char *)message, "LTps ESP:      s");
                  envoi_int_decimal (tps_esp, 9);
                  message[14] = message[13];
                  message[13] = ',';
                  envoi_message();
              }*/
            }

            if (OrCam == 2)  // WLF lecture de fichier
            {
                if (message_in[4] == '1')  // init fait
                {
                    if (!((message_in[5]-'0') & 2))  // bit 1 : pas de pb SD (11WK10 recu)
                    {
                        OrCam = 1; // extinction

                        strcpy((char *)message, "1T1180");  // debit : chaque 150ms
                        message[3] = m_long_periode_e/100 + '0';
                        message[4] = m_long_periode_e/10 - (m_long_periode_e/100)*10  +'0';
                        message[5] = m_long_periode_e%10 +'0';
                        envoi_message();

                        strcpy((char *)message, "1WLF    ");
                        message[4] = (OrCam_param1 / 100) + '0';
                        message[5] = ((OrCam_param1 / 10) % 10) + '0';
                        message[6] = (OrCam_param1 % 10) + '0';
                        message[7] = OrCam_param2 + '0';
                        envoi_message();

                        // attente max
                        uint8_t temps_cam=15;  // nb de secondes - image compressee
                        if ( OrCam_param2 == 6 ) temps_cam = 20; // image peu compresse
                        else if ( OrCam_param2 == 5 ) temps_cam = 25; // image non compressee
                        else if ( OrCam_param2 == 4 ) temps_cam = 50; // D2:20s D3:40s
                        else if ( OrCam_param2 == 3 ) temps_cam = 80;
                        else if ( OrCam_param2 == 2 ) temps_cam = 120;
                        else if ( OrCam_param2 == 1 ) temps_cam = 150;
                        else if ( OrCam_param2 == 0 ) temps_cam = 200;  // avi non compresse
                        CLOCK_RESTART_TIMEOUT(ESP32_sleepTimerClockHandle, (uint32_t) temps_cam*1000);
                    }
                    else
                    {
                        if (OrCam_essai) OrCam_essai--;
                        if (OrCam_essai)
                            ACTIV_SORTIE(Enab_ESP, 0, 4);  // desactivation ESP32 pendant 0,2sec
                        else
                        {
                            extinction_esp32();
                            code_erreur = erreur_esp_wlf;
                            err_donnee1  = message_in[5];
                        }
                    }
                }
                else
                    err_esp_extinct(1);
            }
            if (OrCam == 3)  // prise de photo/film
            {
                // WEV402  Video/Photo yyy:cam_format, duree, envoi  Video:412 ou 413  Photo:402
                if (message_in[4] == '1')  // init fait
                {
                    if (!((message_in[5]-'0') & 3))  // bit 0-1 : pas de pb camera ni sd
                    {
                        strcpy((char *)message, "1WEVxyz");
                        message[4] = OrCam_param1;  // cam_format : 4-HVGA  5-VGA  6-SVGA
                        message[5] = OrCam_param2;  // duree :  0:1 photo  1:5 photo  2:8 photo 3:15  4:30
                        message[6] = OrCam_param3;  // envoi : 1:envoi  2:SD  3:envoi+SD
                        envoi_message ();
                        OrCam = 1;  // extinction apres

                        // attente max 5 sec(jpeg) ou 20sec(avi)
                        uint8_t temps_cam=0;
                        if (OrCam_param3 & 1)  // envoi
                            temps_cam=25;  // 25 secondes pour envoi

                        temps_cam += (OrCam_param2-'0')*6 + 7;  // Ajout temps prise : 0:7sec 1:13s 2:19s 3:25s 5:31s                        if ( OrCam_param2 == 6 ) temps_cam = 15; // image peu compresse

                        CLOCK_RESTART_TIMEOUT(ESP32_sleepTimerClockHandle, (uint32_t) temps_cam*1000);
                    }
                    else
                    {
                        if (OrCam_essai) OrCam_essai--;
                        if (OrCam_essai)
                            ACTIV_SORTIE(Enab_ESP, 0, 4);  // desactivation ESP32 pendant 0,2sec
                        else
                            err_esp_extinct(4);
                    }
                }
                else   err_esp_extinct(5);
            }

            if (OrCam == 4) // allumge wifi
            {
                if (message_in[4] == '1')  // init fait
                {
                    if (!((message_in[5]-'0') & 2))  // bit 1 : pas de pb SD
                    {
                        strcpy((char *)message, "1WEWx");  // WEW0(stop) WEW1(AP) WEW2(Station)
                        message[4] = OrCam_param2;  // 0:stop, 1:AP, 2:STA
                        envoi_message ();
                        OrCam = 0;  // extinction a la fin timeout 10 min, sans erreur
                        CLOCK_RESTART_TIMEOUT(ESP32_sleepTimerClockHandle, (uint32_t) OrCam_param1*1000);  // 1O min
                         if (OrCam_param2=='0') // stop
                         {
                             CLOCK_RESTART_TIMEOUT(ESP32_sleepTimerClockHandle, (uint32_t) 5*1000); // 5 secondes
                         }
                    }
                    else
                    {
                        if (OrCam_essai) OrCam_essai--;
                        if (OrCam_essai)
                            ACTIV_SORTIE(Enab_ESP, 0, 4);  // desactivation ESP32 pendant 0,2sec
                        else
                            err_esp_extinct(2);
                    }
                }
                else   err_esp_extinct(3);
            }

            if (OrCam == 5)  // prise 1 photo
            {
                // WEP5  Photo x:cam_format
                if (message_in[4] == '1')  // init fait
                {
                    if (!((message_in[5]-'0') & 1))  // bit 0 : pas de pb camera
                    {
                        strcpy((char *)message, "1WEPx");
                        message[4] = OrCam_param1;  // cam_format : 4-HVGA  5-VGA  6-SVGA
                        envoi_message ();
                        OrCam = 1;  // extinction apres
                        CLOCK_RESTART_TIMEOUT(ESP32_sleepTimerClockHandle, 7000);  // 7 secondes
                    }
                    else
                    {
                        if (OrCam_essai) OrCam_essai--;
                        if (OrCam_essai)
                            ACTIV_SORTIE(Enab_ESP, 0, 4);  // desactivation ESP32 pendant 0,2sec
                        else
                            err_esp_extinct(8);
                    }
                }
                else   err_esp_extinct(9);
            }
      }
      if ((message_in[3] == 'L') && (message_in[4] == 'S') && (message_in[5] == 'N'))  // WLSN : demande init du esp32
      {
          // envoi heure/date
          strcpy((char *)message, "1xHES");  // HEShhhh et lumi
          message[1] = 12; // longueur
          uint32_t secondes_mes;  // secondes
          NB_SECONDES(secondes_mes); //Time(ISeconds_Time *ts);  // seconds et nanosecondes
          memcpy(message+5,  &secondes_mes,4);
          uint16_t reste_avant_fen = timeout_prochaine_fenetre_node();
          message[9]=0;
          message[10] = ((reste_avant_fen >> 8) & 0xFF);  // en millisecondes
          message[11] = (reste_avant_fen  & 0xFF);
          message[12] = periode_activ_RX; // periode en secondes : 5, 10
          message[13] = 0x80;// luminosite
          #ifdef BATT_BMS
              message[13] = voltage_panneau_sol/24;   // 0 à 5,6V -> 0 a 230
          #endif
              message[14] = '5';// format camera
          envoi_message_def ();
      }


      if (message_in[3] == 'Z')  // TODO
      {
          strcpy((char *)message, "LWWxx xx");  // HEShhhh
          message[3] = deci ( LOW_BYTE(message_test[0]) >> 4);
          message[4] = deci (LOW_BYTE(message_test[0]) & 0x0F);
          message[6] = deci ( LOW_BYTE(message_test[2]) >> 4);
          message[7] = deci (LOW_BYTE(message_test[2]) & 0x0F);
          message_test[0] = 0;
          message_test[1] = 0;
          message_test[2] = 0;
          message_test[3] = 0;
          envoi_message();
      }

      // permet de tester le capteur PIR en recevant un message sur L
      if ((message_in[4] == 'D') && (message_in[5] == 'B') )  // WL/EDBx lecture/ecriture mode debug PIR
      {
          if ((message_in[3] == 'L') && (longueur_m==6))  // lecture  WLDB 0:normal 1:PIR 2:PIR+cam
          {
              strcpy((char *)message+1, " Debug Pir: ");
              message[12]=camera_debug+'0';
              envoi_message();

          }
          if ((message_in[3] == 'E') && (longueur_m==7))  // Ecriture WEDBx
          {
              camera_debug=message_in[6]-'0';
          }
      }

      /* ----------------- ORDRES ESP32 --------------------------

          WEPx   : Prise 1 photo simple format x
          WEV402 : Video/Photo x:cam_format, y:duree, z:envoi(1:envoi, 2:SD, 3:SD+envoi)
          WEW16  : Allumage wifi  x:stop/AP/Sta  y:dure (min)
          WLFxxxy lecture fichier  x:numero   y:0-3:avi 5-8:jpeg
          WEC1x  : allumage ESP32 x minutes  (WEC0 extinction - WLC:lecture temps restant)

       */
      if ((message_in[3] == 'E') && (message_in[4] == 'C') )
      {
          if ((message_in[5]=='0') && (longueur_m==6))   // WEC0 : extinction
              extinction_esp32();
          else  if (message_in[5]=='1')                  // WEC1x : x:duree en min
          {
              OrCam = 0;
              allumage_esp32(((uint16_t)message_in[6]-'0')*60);
              // Extinction au bout du temps defini
          }
      }
      if ((message_in[3] == 'L') && (message_in[4] == 'C') && (longueur_m==5))  // WLC lecture temps restant ESP
      {
         uint16_t tps_restant;
         tps_restant = (Clock_getTimeout (ESP32_sleepTimerClockHandle)) / ((1000 / Clock_tickPeriod) * 1000);
         strcpy((char *)message+1, "Tps restant ESP:     s  ");
         ajout_int16_message ( tps_restant, 17,0x82);

      }

      if (message_in[3] == 'E')
      {
          if (message_in[4] == 'T')  // WET150
          {
              m_long_periode_e = (message_in[5]-'0')*100 + (message_in[6]-'0')*10 + message_in[7]-'0' ;
          }

        if ((message_in[4] == 'V') && (!OrCam) && (longueur_m==8))   // WEVxyz   Prise film/photo -> WEVxyz
        {  // WEV402  Video/Photo x:cam_format, y:duree, z:envoi
            if (!ESP32_etat)
            {
                // allumage ESP 5 secondes
                allumage_esp32(10);
                allumage_led_pwm();  // allumage LED IR si nuit

                // attente message OK de l'ESP
                OrCam = 3;  // prise de photo/film
                OrCam_essai = 3;
                OrCam_param1 = message_in[5];
                OrCam_param2 = message_in[6];
                OrCam_param3 = message_in[7];
            }
            else  // deja allume : transfert message
            {
                memcpy((char *)message, message_in+1, longueur_m);
                message[0] = '1';  // vers ESP
                envoi_message();
            }
        }

        if ((message_in[4] == 'P') && (!OrCam) && (longueur_m==8))   // WEPx   Prise 1 photo simple format x
        {  // WEP5  Photo x:cam_format
            if (!ESP32_etat)
            {
                // allumage ESP 5 secondes
                allumage_esp32(10);
                allumage_led_pwm();  // allumage LED IR si nuit

                // attente message OK de l'ESP
                OrCam = 5;  // prise 1 photo
                OrCam_essai = 3;
                OrCam_param1 = message_in[5];
            }
            else  // deja allume : transfert message
            {
                memcpy((char *)message, message_in+1, longueur_m);
                message[0] = '1';  // vers ESP
                envoi_message();
            }
        }

        if (message_in[4] == 'W')
        {
            if (longueur_m == 7)    // WEW16 WEWxy   Allumage wifi  (x:stop/AP:Sta) y:dure (min)
            {
                if (!ESP32_etat)
                {
                    // allumage ESP 5 secondes
                    allumage_esp32(10);

                    // attente message OK de l'ESP
                    OrCam = 4;  // allumage wifi
                    OrCam_essai = 3;
                    OrCam_param1 = 60*((uint16_t)message_in[6] - '0'); // duree en secondes
                    OrCam_param2 = message_in[5]; // 0:stop  1:Station  2:AP
                }
                else  // deja allume : transfert message
                {
                    memcpy((char *)message, message_in+1, longueur_m);
                    message[0] = '1';  // vers ESP    1WEW1
                    message[5] = car_fin_trame;
                   envoi_message();
                }
            }
            if ((longueur_m == 6) && (message_in[5] == '0'))   // WEW0    Extinction wifi
            {
                extinction_esp32();
            }
        }
      }
      if (message_in[3] == 'L')
      {
        if ((message_in[4] == 'F') && (longueur_m == 9) && (!OrCam)) // WLFxxxy lecture fichier   0-3:avi  y:5-8:jpeg
        {
            if (!ESP32_etat)  // ESP32 eteint
            {
                // allumage ESP 5 secondes
                allumage_esp32(10);

                // attente message OK de l'ESP
                OrCam = 2;  // lecture fichier par WLF
                OrCam_essai = 3;
                OrCam_param1 = ((uint16_t)message_in[5]-'0')*100 + (message_in[6]-'0')*10 + (message_in[7]-'0');
                OrCam_param2 = message_in[8]-'0';
            }
            else  // deja allume : transfert message
            {
                memcpy((char *)message, message_in+1, longueur_m);  // aLWLFxxxyc -> LWLFxxxyc
                message[0] = '1';  // vers ESP                      // -> 1WLFxxxyc
                envoi_message();
            }
        }
        if (message_in[4] == 'A')     // WLA  lecture statut webcam
        {
            if ((message_in[5] == '0') && (longueur_m == 6))    // WLA0  lecture statut videos
            {
                uint8_t emett = message[0];
                strcpy((char *)message+1, " Vid:");
                uint8_t posit = 6;
                ajout_message_32b (date_derniere_video, posit);
                posit += 9;
                message[posit++] = '-';
                uint8_t i;
                for (i=0; i<12; i++)
                {
                    ajout_message_8b( ecart_video[i] , posit);
                    posit += 2;
                    message[posit++] = ' ';
                }
                message[posit] = car_fin_trame;
                envoi_message();

                // detemination nb_videos sur 24h
                uint8_t niv_bat;
                for (niv_bat=0; niv_bat<5; niv_bat++)
                // Niv 0:<10  1:10-20  2:20-40  3:40-60  4:60-80  5:>80
                // uint8_t esp_bat[5] =    {10, 20, 40, 60, 80}; // rien, mess, photo stock, photo, video, HQ
                {
                    if (batt_soc1 <= esp_bat[niv_bat] )
                        break;
                }
                if (batt_soc1 > esp_bat[4]) niv_bat=5;

                uint32_t tps;
                struct timeval tv;  // microsecondes
                tv = NB_MICROSEC();  // seconds et microsecondes
                tps = tv.tv_sec;

                // 12 dernieres videos des 24 dernieres heures
                // derniere video : date/heure (en sec)

                // calcul du nb de videos prises depuis les 24 dernieres heures
                // temps entre la derniere video et maintenant
                uint32_t temps_dernier;
                uint8_t nb_videos;
                temps_dernier = (tps - date_derniere_video)/360;  // nb 6 minutes
                nb_videos=0;
                for (i=0; i<12; i++)
                {
                    if (temps_dernier < 24*10)  // 24 heures
                    {
                        nb_videos++;
                        temps_dernier += ecart_video[i];
                    }
                    else
                        break;
                }

                // calcul du temps (en sec) entre la derniere video et la nouvelle video a prendre
                // depend du nb video des 24 dernieres heures et de la batterie
                // si batterie pleine => 1,4^nb*intervalle
                uint32_t attente_video;
                attente_video = VIDEO_INTERVALLE;
                for (i=0; i<nb_videos; i++)
                    attente_video = VIDEO_MULTIPLI * attente_video;
                attente_video = (attente_video * tps_batt(niv_bat, 1));

                uint16_t ecart;
                ecart = tps - date_derniere_video ;

                message[0] = emett;
                strcpy((char *)message+1, " Nb Vid 24h:      niv_bat:       attente:      derniere:   ");
                ajout_int8_message ( nb_videos, 13, 0x82);
                ajout_int8_message ( niv_bat, 27, 0x82);
                ajout_int8_message ( attente_video, 42, 0x82);
                ajout_int16_message ( ecart, 57, 2);
                envoi_message();
            }

            if ((message_in[5] == '1') && (longueur_m == 6))    // WLA1  lecture statut photos
            {
                uint8_t emett = message[0];
                strcpy((char *)message+1, " Pho:");
                uint8_t posit = 6;
                ajout_message_32b (date_derniere_photo, posit);
                posit += 9;
                message[posit++] = '-';
                uint8_t i;
                for (i=0; i<12; i++)
                {
                    ajout_message_8b( ecart_photo[i] , posit);
                    posit += 2;
                    message[posit++] = ' ';
                }
                message[posit] = car_fin_trame;
                envoi_message();

                //batt_soc1=69;
                uint8_t niv_bat;
                for (niv_bat=0; niv_bat<5; niv_bat++)
                // Niv 0:<10  1:10-20  2:20-40  3:40-60  4:60-80  5:>80
                // uint8_t esp_bat[5] =    {10, 20, 40, 60, 80}; // rien, mess, photo stock, photo, video, HQ
                {
                    if (batt_soc1 <= esp_bat[niv_bat] )
                        break;
                }
                if (batt_soc1 > esp_bat[4]) niv_bat=5;

                uint32_t tps;
                struct timeval tv;  // microsecondes
                tv = NB_MICROSEC();  // seconds et microsecondes
                tps = tv.tv_sec;

                // 12 dernieres videos des 24 dernieres heures
                // derniere video : date/heure (en sec)
                // les 11 autres sont décales en nb de 5min par rapport à la derniere


                // calcul du nb de photos prises depuis les 24 dernieres heures
                // temps entre la derniere video et maintenant
                uint32_t temps_dernier;
                uint8_t nb_photos;
                temps_dernier = (tps - date_derniere_photo)/360;  // nb 6 minutes
                nb_photos=0;
                for (i=0; i<30; i++)
                {
                    if (temps_dernier < 24*10)  // 24 heures
                    {
                        nb_photos++;
                        temps_dernier += ecart_photo[i];
                    }
                    else
                        break;
                }

                // calcul du temps (en sec) entre la derniere video et la nouvelle video a prendre
                // depend du nb video des 24 dernieres heures et de la batterie
                // si batterie pleine => 1,4^nb*intervalle
                uint32_t attente_photo;

                attente_photo = PHOTO_INTERVALLE;
                for (i=0; i<nb_photos; i++)
                    attente_photo = PHOTO_MULTIPLI * attente_photo;
                attente_photo = (attente_photo * esp_pho[niv_bat]) / 100;

                uint16_t ecart;
                ecart = tps - date_derniere_photo ;

                message[0] = emett;
                strcpy((char *)message+1, " Nb Pho 24h:      niv_bat:       attente:      derniere:   ");
                ajout_int8_message ( nb_photos, 13, 0x82);
                ajout_int8_message ( niv_bat, 27, 0x82);
                ajout_int8_message ( attente_photo, 42, 0x82);
                ajout_int16_message ( ecart, 57, 2);
                envoi_message();

            }
            if ((message_in[5] == '2') && (longueur_m == 6))    // WLA2  lecture video ou photos
            {
                uint8_t action;
                action = calcul_pir_esp();
                if (action == 2)  // photo/video
                {
                    // WEV402  Video/Photo x:cam_format, y:duree, z:envoi
                    // allumage ESP 10 secondes puis attente message WK
                    allumage_esp32(10);
                    allumage_led_pwm();  // allumage LED IR si nuit
                }
            }
        }
      }
    }
#endif  // end CC13xx_ESP2

    #ifdef C10_ElecVanne   // Commande Electrovannes et lecture etat
        // commande EV : XVnd  d:0:arret 1:30s 2:15min 3:30min 4:1h 5:1h30 6:2h 7:3h 8:4h 9:5h
        if (message_in[2] == 'X')  // X   SORTIE  X      XV0L => consigne - duree
        {
            uint8_t b;
            b = message_in[4] - '0';
            if ((message_in[3] == 'V') && (b < NB_EVanne) && (longueur_m == 6))  // XVnL(lecture) ou XVnd(activ)
            {
                if (message_in[5] == 'L')  // lecture statut electrovanne  XVnL ->XVn0- ou XVn1-
                {
                    strcpy ( (char *)message+1, "XVn: ");
                    message[5]='0';
                    if (Clock_isActive(SortieEV[b].h_clock))    ///100/1000;  // en secondes
                    {
                        message[5]='1';
                        message[6] = '-';
                        uint8_t dur_min, index;
                        dur_min = (Clock_getTimeout (SortieEV[b].h_clock)) / ((1000 / Clock_tickPeriod) * 1000) / 60;
                        index = ajout_int8_message (dur_min, 7, 0x82);
                        strcpy ( (char *)message+index, "min");
                    }
                    message[3] = b+'0';
                    envoi_message();
                }
                else   // Activation/arret Electrovanne XVnd
                {
                    uint16_t duree_sec;
                    duree_sec = transf_duree_ev(message_in[5]-'0');
                    activ_evanne (b, duree_sec);
                }
            }
        }

        // programmation électrovanne  XVPEpdD
                // duree:d:0:arret 1:30s 2:15min 3:30min 4:1h 5:1h30 6:2h 7:3h 8:4h 9:5h
                // periode:0(arret) 1à7:1à7jours 8:10j  9:15j
                // Decalage
        // XVP00 arret  XVP43 1 h/4jours
        if (message_in[2] == 'X')
        {
            if ((message_in[3] == 'V') && (message_in[4] == 'P'))
            {
                if ((message_in[5] == 'E'))   // XVPEdp programme ev
                {
                    uint8_t EV_decalage;
                    EV_decalage=0;
                    if  (longueur_m == 9)   // XVPEdpD  D:Decalage max:10heures
                    {
                        EV_decalage=  message_in[8]-'0';  // Decalage en heures
                    }
                    if  ((longueur_m >=7) && (longueur_m <=9))
                    {
                        if (message_in[6]=='0')   // XVPE0 : arret programme
                        {
                            Clock_stop(EV_prog_ClockHandle);
                            activ_evanne (0, 0);  // fermeture vanne 0
                        }
                        else   // démarrage programme C10_ElecVanne
                        {
                        EV_prog_periode = message_in[7]-'0';
                        EV_prog_duree = message_in[6]-'0';
                        EV_prog_timer = transf_periode_ev(EV_prog_periode);
                        EV_prog_timer_rest = 1;
                        Clock_stop(EV_prog_ClockHandle);
                        Clock_setPeriod(EV_prog_ClockHandle, (uint32_t)  6*3600 * (1000 / Clock_tickPeriod) * 1000); // 6 heures
                        Clock_setTimeout(EV_prog_ClockHandle, (uint32_t)  (EV_decalage*3600+5) * (1000 / Clock_tickPeriod) * 1000); // X heures + 5 secondes
                        Clock_start(EV_prog_ClockHandle);
                        //uint16_t duree_sec;
                        //duree_sec = transf_duree_ev(EV_prog_duree);
                        //activ_evanne (0, duree_sec);
                        }
                    }
                }
                if (message_in[5] == 'L')  // XVPL lecture programme
                {
                    strcpy ( (char *)message+1, "XVProg:   ");
                    message[8] = EV_prog_duree +'0';
                    message[9] = EV_prog_periode +'0';
                    uint8_t dur_restante, index;
                    if (Clock_isActive(EV_prog_ClockHandle))    ///100/1000;  // en secondes
                    {
                        if (EV_prog_timer_rest)
                            dur_restante = (EV_prog_timer_rest-1)*6 + (Clock_getTimeout (EV_prog_ClockHandle)) / ((1000 / Clock_tickPeriod) * 1000) / 60/60;
                        else
                            dur_restante = (Clock_getTimeout (EV_prog_ClockHandle)) / ((1000 / Clock_tickPeriod) * 1000) / 60/60;

                        index = ajout_int16_message (dur_restante, 11, 0x82);
                        strcpy ( (char *)message+index, "h");
                    }
                    envoi_message();

                }
            }
        }

    #endif

    longueur_message_in = longueur_m;
    if ((message_in[2] == 'C') &&  (message_in[3] == 'M'))
    {
        #ifdef CAPTEUR_MPU
            traitement_message_capteur_mpu();  // transfert a la tache sensor_mpu
        #endif
    }
}

#ifdef BATT_BMS
void allumage_led_pwm(void)
{
    // periode : 2700=> 750hz   900=>2200Hz   500=>4000Hz   100->20kHz
    uint16_t periode=100;

    // duty :  20=8%  31=12%  41=16%  64=25%  128=50%  255=100%
    uint8_t duty = 64;

    if (voltage_panneau_sol < 20*24)
        ACTIV_PWM ( 1, 1, 300, periode, duty); // duree 300=30sec

}
#endif

#ifdef RAM_TAMPON_SIZE
void envoi_ram_message_date ( uint16_t * adresse_ram , unsigned char lg_ram, unsigned char type)
{
    char unsigned heure_ram, minute_ram;

    flash_decodage = type & 0x1;  // 0:décodage  1:hexa
    date_imp = type & 0x2;  // 0:pas de date   1:date imprimée
    minute_ram = ( Heure_debut / 60) % 60;
    heure_ram =  ( Heure_debut / 3600 ) % 24;

    if (date_imp)
    {
        strcpy((char *)message+1, "HH Capteur Hygro - Heure: x:xx");
        if (flash_decodage)
        {
            message[1]=27;
            message[27]=heure_ram;
            message[29]=minute_ram;
            envoi_message_def();
        }
        else
        {
            message[26] = heure_ram / 10 +'0';
            message[27] = heure_ram % 10 +'0';
            message[29] = minute_ram / 10 +'0';
            message[30] = minute_ram % 10 +'0';
            envoi_message();
        }
    }

    index_r=0;
    index_lg = lg_ram;
    adresse_ram_index = adresse_ram;
    dest_data_capteur = message_in[1];

    if (lg_ram)
    {
        Clock_start(TimerRamClockHandle);
    }
}

void envoi_ram_message (void)
{
    char unsigned i_position;
    bool continu;
    uint16_t value_brut;
    double value;

    message[0]=dest_data_capteur;
    message[1]='H';
    message[2]='H';
    message[3]=' ';

    if (flash_decodage)   // hexa
    {
        i_position = 3;
        continu=1;
        while (continu)
        {
            message[i_position++] = *(adresse_ram_index+index_r);
            if (index_r >= index_lg)
            {
                Clock_stop(TimerFlashClockHandle);
                continu=0;
            }
            if (i_position >= 39)    continu = 0;
        }
        message[1] = i_position;
        envoi_message_def();
    }
    else    // textuel
    {
        i_position = 3;
        continu=1;
        while (continu)
        {
            value_brut = (uint16_t) *(adresse_ram_index + index_r);
            if (!(index_r & 1))
            {
                  value = ((double)(int16_t)value_brut / 65536)*165 - 40;   // temp
                  message[i_position+6] = ' ';
            }
            else
            {
                  value  = ((double)value_brut / 65536)*100;               //-- calculate relative humidity [%RH]
                  message[i_position+6] = ' ';
            }

            ajout_float_message(value, 2, i_position, 0);
            message[i_position+7] = ' ';
            i_position = i_position+7;
            index_r++;
            if (index_r >= index_lg)
            {
                Clock_stop(TimerRamClockHandle);
                continu=0;
            }
            if (i_position >= 39)    continu = 0;
        }

        message[i_position] = car_fin_trame;
        envoi_message();
    }
}


void envoi_flash_message_date( uint16_t adresse_flash, uint16_t adresse_flash_fin, char unsigned type)
{
    // 256 secteurs de 4096 octets (1 secteur = 16 pages de 256 octets) ou 4096 pages de 256 octets
    flash_decodage = type & 0x1;  // 0:hexa  1:textuel
    date_imp = type & 0x2;  // 0:pas de date   1:date imprimée

    index_flash_tail = adresse_flash ;
    index_r=0;
    index_lg = 0x80;
    adresse_page_index     = adresse_flash >> 8;
    adresse_page_index_fin = adresse_flash_fin;
    dest_data_capteur = message_in[1];

    if ((adresse_page_index < 0x1000) && (index_flash_tail < index_flash_head))
    {
        Clock_start(TimerFlashClockHandle);
    }

}
#endif


#ifdef PILE_RF_FLASH


// envoi 1 trame de Flash au destinataire
// codage : Trame envoyee : Lg+mode_rx + Dest,Lg, M, Emetteur, code, Type, timestamp, 4 valeurs
void envoi_flash_message_mpu (void)
{
    uint8_t retc, num;
    uint8_t message_f[TAILLE_MAX_MESS_RF];

    retc = lecture_message_flash(message_f, 0, 0, TAILLE_MAX_MESS_RF);  // Nota : n'envoie qu'à partir de la première section => à compléter

    num = numero_device_rf( passerelle_routage(message_f[2] & 0x7F));
    if ((num) && (!retc))
    {
        Envoie_RF(num-1, message_f);
    }
}
#endif


#ifdef RAM_TAMPON_SIZE
void envoi_flash_message (void)
{
    char unsigned i_position;
    bool continu;
    uint8_t nb_valeur_envoi;
    uint16_t value_brut;
    double value;
    uint32_t secondes_value;

    if (!flash_decodage)   // mode texte
    {
        if (!index_r)
        {
            if (date_imp)  // affichage date en première ligne
            {
                ExtFlash_open();
                ExtFlash_read( FLASH_ADDRESS (adresse_page_index,0) , 4, (uint8_t *) &secondes_mem, 0) ; //, (const uint8_t *) &Heure_debut);
                ExtFlash_close();

                strcpy((char *)message+1, "HH Capteur Hygro - xx-xx-xx  xx:xx:xx");
                conversion_sec_date  (secondes_mem, 20);
                conversion_sec_heure (secondes_mem, 30,0);
                message[18]=9;
                message[28]=9;
                message[38]=car_fin_trame;
                envoi_message();
            }
            index_r=2;
        }
        else
        {
            message[0]=dest_data_capteur;
            message[1]='H';
            message[2]=9;

            i_position = 3;
            continu=1;
            while (continu)
            {
                //value_brut = (uint16_t) *(adresse_ram_index + index_r);
                ExtFlash_open();
                ExtFlash_read( FLASH_ADDRESS (adresse_page_index,index_r*2) , 2, (uint8_t *) &value_brut, 0) ; //, (const uint8_t *) &Heure_debut);
                ExtFlash_close();
                if (!(index_r & 1))
                {
                      value = ((double)(int16_t)value_brut / 65536)*165 - 40;   // temp
                }
                else
                {
                      value  = ((double)value_brut / 65536)*100;               //-- calculate relative humidity [%RH]
                }

                message[i_position]=' ';
                ajout_float_message(value, 2, i_position, 0);
                message[i_position+6] = 9;
                i_position = i_position+7;
                index_r++;
                if (index_r >= index_lg)
                {
                    index_r=0;
                    adresse_page_index++;
                    if (adresse_page_index > adresse_page_index_fin)
                    {
                        Clock_stop(TimerFlashClockHandle);
                    }
                    continu=0;
                }
                if (i_position >= 39)    continu = 0;
            }

            message[i_position] = car_fin_trame;
            envoi_message();
        }
    }
    else   // mode hexa
    {
        if (!index_r)
        {
            ExtFlash_open();
            ExtFlash_read( FLASH_ADDRESS (adresse_page_index,0) , 4, (uint8_t *) &secondes_mem, 0) ; //, (const uint8_t *) &Heure_debut);
            ExtFlash_close();
            index_r=2;
        }
        secondes_value = secondes_mem + ((index_r-2)/nb_capteurs)*period_capteur;
        message[0]=dest_data_capteur;
        message[2] = 'T';
        message[3] = 'H';
        message[4] = (secondes_value >> 24) & 0xFF;
        message[5] = (secondes_value >> 16) & 0xFF;
        message[6] = (secondes_value >> 8)  & 0xFF;
        message[7] = secondes_value & 0xFF;

        nb_valeur_envoi = index_lg - index_r;
        if (nb_valeur_envoi >  20) nb_valeur_envoi = 20;
        ExtFlash_open();
        ExtFlash_read( FLASH_ADDRESS (adresse_page_index,index_r*2) , nb_valeur_envoi*2, (uint8_t *) (message+8), 0 );
        ExtFlash_close();

        index_r = index_r + nb_valeur_envoi;

        if (index_r >= index_lg)
        {
            index_r=0;
            adresse_page_index++;
            if (adresse_page_index > adresse_page_index_fin)
            {
                Clock_stop(TimerFlashClockHandle);
            }
        }

        message[1] = nb_valeur_envoi*2 + 5;
        envoi_message_def();
    }
}
#endif


#ifdef OPT_CAPTEUR

// A (Acceleration) C(Choc) E(Energie) L(lumi) P(Pression) Q(Quaternion) T(Temp) V(Vibration) Z(Acc Z)
// envoi les infos résultat à la liaison série ou par RF. Enregistre sur Flash
// mode_affich_enreg :  0:affichage port série 'R'  1:affichage radio 'L'    2:enregis Flash
// Params : Code : type de donnée     chaine:trame
void envoi_enreg_resultat (uint8_t code, uint8_t * chaine)
{
    Seconds_Time ts;  // nanosecondes
    //uint32_t secondes_rtc;

    // trame : lg+MSTLttttt
    chaine[0] += ( chaine[0] % 2 );  // arrondi au chiffre pair superieur
    chaine[1] = 'M';  // 0x4D
    chaine[2] = My_Address; // 0x53=S
    chaine[3] = code;
    chaine[4] = 0;  // sans timestamp (Q) sinon avec Timestamp

    #ifdef DEBUG_M
        chaine[3] = code | 0x80;
        //ACTIV_PWM ( 0,1, 1, 1700, 6);  // 1,8,1700,6 = bip 0,2s 1100Hz, volume 6
    #endif

    if (code != 'Q')   // Hors quaternion : Choc/vib/acc
    {
        if (! SortieTor[0].duree)
        {
            if (code != 'T')
            {
                //Turn_on_LED(0);   //
                ACTIV_SORTIE(LED_cpu,4,1);
                /*if (code == 'Z')
                    DELAY_US(100);
                else
                    DELAY_MS(1);
                Turn_off_LED(0);*/
            }
        }
        //secondes_rtc = Seconds_get();  // seconds
        Seconds_getTime(&ts);  // seconds et nanosecondes

        /*if (code=='Z')
        {
            if ((!annul_suivant) && (time_prec_sec) && (ts.secs - time_prec_sec <4))
            {
                uint32_t milli_sec = (ts.secs-time_prec_sec)*1000 + ts.nsecs/1000000 - time_prec_milli;
                if (milli_sec > 300)
                {
                    System_printf("toto\n");
                    annul_suivant=1;
                }
            }
            else
                annul_suivant=0;
            time_prec_sec = ts.secs;
            time_prec_milli = ts.nsecs/1000000;
        }*/

        chaine[4] = 0x80;  // avec timestamp
        memcpy (chaine+5, &ts.secs, 4);
        chaine[9]= ts.nsecs / 4000000;  // nb de 4 milli-secondes
    }

    if (!mode_affich_enreg)  // envoi par liaison serie
    {
        message[0]=PC_NODE_SERIE_ADDRESS;
        if (mode_hexa_texte) // 1:mode texte
        {
            Affiche_texte_resultat(chaine);
        }
        else  // 0:mode hexa
        {
            message[1] = chaine[0]-2;  // longueur payload
           if ((chaine[0]) && (chaine[0]<MESSAGE_SIZE))
                   memcpy(message+2, chaine+1, chaine[0]-1);  // M, Emetteur, code, Type, timestamp(5),  valeurs
           envoi_message_def ();  // envoi message format Hexa
        }
    }
    if (mode_affich_enreg)  // envoi a la passerelle par Radio, puis par liaison serie   OU enreg Flash
    {
        uint8_t groupe_envoi=0,  i;

        if (code == 'P')   // Pression : envoi uniquement de la pression
            if (chaine[0]>2) chaine[0] -= 2;   // suppression de la temperature

        if ( ((code != 'T' ) && (code != 'P' )) || (!Temp_groupes))
        {  // envoi individuel
            // decalage de 1 pour ajouter l'emetteur
            if (chaine[0]>1)  // si chaine[0]=3   0:  1:  2:1  3:2
            {
                for (i=chaine[0]-1; i; i--)
                    chaine[i+1] = chaine[i];   // Lg, M, Emetteur, code, Type, timestamp, 4 valeurs
                    //memcpy(message+2, chaine+1, chaine[0]-1);
                chaine[1] = chaine[0]-2;  // longueur payload
                if (mode_hexa_texte) // 1:mode texte (hexa en rf, puis texte ensuite)
                {
                    chaine[0] = RADIO_CONCENTRATOR_ADDRESS;  // vers passerelle
                    chaine[5] = chaine [5] + PC_ADDRESS;
                }
                else  // 0:mode hexa
                {
                    chaine[0] = PC_ADDRESS;
                }
                envoi_capt (code, chaine[1]+3, chaine);  // envoi premier message format Hexa
            }
        }
        else   // envoi en groupes (T ou P)
        {
            // (ecart), code, mesure1-x
            ram_tampon[index_ram_car++] = code;
            if (!index_ram_nb)
            {
                timestamp_init = ts.secs ;
                MS_init = ts.nsecs / 4000000;  // Nb de 4 ms
            }
            else
            {
                uint16_t ecart;
                ecart = ts.secs - timestamp_init;
                ram_tampon[index_ram_car++] = (uint8_t) (ecart >>8);
                ram_tampon[index_ram_car++] = (uint8_t) (ecart);
            }
            if ((chaine[0]>=10) && ((chaine[0]-10) < (GROUPES_CAR_MAX-index_ram_car) ) && (index_ram_car<GROUPES_CAR_MAX) )
                memcpy (ram_tampon+index_ram_car, chaine+10, chaine[0]-10); // mesure1 et 2  chaine[0]=14
            index_ram_car += chaine[0]-10;
            index_ram_nb++;

            if ((index_ram_nb >= Groupes_nb) || (index_ram_car >= (GROUPES_CAR_MAX-8-9)) || (forcage_envoi_capteur))
                groupe_envoi=1;
            else
            {
                if (index_ram_nb==1)
                {
                    Clock_setTimeout(RetenClockHandle, Groupes_temps * 60 * (1000  / Clock_tickPeriod) * 1000);
                    if (Groupes_temps)
                        Clock_start(RetenClockHandle);  // clock de vidage tampon si trop long
                }
            }
        }

        if (groupe_envoi)
        {
            vidage_tampon();
        }
    }
}
 // Dest lg-3 M emett code dest-final, 5*TS, capt
void envoi_capt(char code, char longu, uint8_t * chaine)
{
    uint8_t valide;

    valide=0;
    if ((longu>1) && (longu < MESSAGE_SIZE))
    {
        memcpy(message, chaine, longu);

        // mode_affich_enreg = 0:envoi par port série 'R'  1:envoi par radio 'L'   2:enregis Flash
        if (mode_affich_enreg == 1)  // envoi a la passerelle par Radio, puis par liaison serie
        {
            valide=1;
            Mess_statut = 0x00;  // Ack, pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
        }
        else if (mode_affich_enreg == 2) // Flash = Differé
        {
            if ((ENREG_QUAT) || (code != 'Q'))  // n'enregistre pas les quaternions si ENREG_QUAT=0
            {
                valide=1;
                Mess_statut = 0x80;  // Ack, pas RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:différé   bit4:pas d'ack  bit5:RX apres
            }
        }
        if (valide)
        {
            envoi_message_def();  // todo :message peut etre corrompu par tache de priorite superieure
        }
    }
}

void vidage_tampon(void)
{
    // Trame envoyee : Lg, M, Emetteur, code, Type, timestamp, 4 valeurs
    Clock_stop(RetenClockHandle);  // plus besoin de continuer le timer
    if (index_ram_car)
    {
        message[0] = PC_NODE_SERIE_ADDRESS;
        message[1] = 7 + index_ram_car;  // longueur payload
        message[2] = 'M';
        message[3] = My_Address;
        message[4] = ram_tampon[0];
        #ifdef DEBUG_M
            message[4] |= 0x80;
        #endif

        message[5] = 0x80;  // avec timestamp
        memcpy (message+6, &timestamp_init, 4);
        message[10]= MS_init;  // nb de 4 milli-secondes
        if ((index_ram_car) && (index_ram_car < MESSAGE_SIZE-11))
                memcpy (message+11, ram_tampon+1, index_ram_car-1);
        if (mode_hexa_texte) // 1:mode texte (hexa en rf, puis texte ensuite)
        {
            message[0] = RADIO_CONCENTRATOR_ADDRESS;  // vers passerelle
            message[5] = message [5] + PC_ADDRESS;
        }
        else  // 0:mode hexa
            message[0] = PC_ADDRESS;
        envoi_capt (ram_tampon[0], message[1]+2, message);  // envoi premier message format Hexa
        index_ram_car=0;
        index_ram_nb=0;
    }
}

// A (Acceleration) C(Choc) E(Energie) L(lumi) P(Pression) Q(Quaternion) T(Temp) V(Vibration) Z(Acc Z)
// Affiche en clair le resultat, en le decodant
// valeur : nb d'octets, M, code, type, x valeurs
// nb d'octets : inclus cet octet
// code : V:vib, Q:quat, ...
// type : 0:sans timestamp, 1:avec timestamp
// Cas type 0 : affiche en temps reel sur port série, sans timestamp
// Cas type 1 : lecture de la flash en differe, sur le port serie  : avec timestamp
void Affiche_texte_resultat(uint8_t * chaine)
{
    uint8_t emetteur, code, type, index, ind, i;
    uint32_t secondes_rtc;
    uint8_t ms4;
    float flo_valeur[4];
    uint16_t u16_valeur[4];

    emetteur = chaine[2];
    code = chaine[3];
    type = chaine[4];
    memcpy (&secondes_rtc, chaine+5, 4);
    ms4 = chaine[9];
    ind = 10;    // chaine
    index = 10;  // message
    message[1] = 'M';
    message[2] = emetteur; //My_Address;

    if ((type) && (code != 'V'))
    {
        conversion_sec_date  (secondes_rtc, index);
        conversion_sec_heure (secondes_rtc, index+9,0);
        message[index+17] = '-';
        message[index+18] = ((ms4*4/10) /10) +'0';
        message[index+19] = ((ms4*4/10) %10) +'0';
        message[index+20] = ' ';
        index += 21;
    }

    if (code=='E')  // Energie vibratoire
    {
        strcpy((char *)message+3, "E.Vib:");
        message[9]=' ';
        memcpy (flo_valeur, chaine+ind, 12);
        ajout_float_multi_valeurs (flo_valeur, 2, index, 3, 0);
        envoi_message();
    }
    if (code=='C')  // Chocs
    {
        strcpy((char *)message+3, "Choc: ");
        message[9]=' ';
        message[index++] = 'X' + chaine[ind++];
        message[index++] = ' ';

        memcpy (flo_valeur, chaine+ind, 4);
        ind += 4;
        index = ajout_float_multi_valeurs (flo_valeur, 0, index, 1, 0);

        memcpy (u16_valeur, chaine+ind, 2);
        index = ajout_int16_multi_valeurs ((int16_t *) u16_valeur, index, 1, 0);
        message[index++]='m';
        message[index++]='s';
        message[index++]=car_fin_trame;
        envoi_message ();
    }
    if (code=='A')  // Acceleration/freinage
    {
        strcpy((char *)message+3, "Accel:");
        message[9]=' ';
        memcpy (flo_valeur, chaine+ind, 12);
        index = ajout_float_multi_valeurs (&flo_valeur[0], 0, index, 1, 0);
        message[index++]='m';
        message[index++]='g';
        index = ajout_float_multi_valeurs (&flo_valeur[1], 0, index, 1, 0);
        index = ajout_float_multi_valeurs (&flo_valeur[2], 0, index, 1, 0);
        message[index++]='m';
        message[index++]='s';
        message[index++]=car_fin_trame;
        envoi_message ();
    }
    if (code=='Z')  // Accelerometre axe Z
    {
        strcpy((char *)message+3, "AcceZ:");
        message[9]=' ';
        //memcpy (message+index, (int16_t *) (chaine+ind), 7);
        ind++;
        for (i=0; i<3; i++)
        {
            int16_t val_acc;
            val_acc = (int16_t) chaine[ind++]<<8;
            val_acc +=  chaine[ind++];
            index = ajout_int16_message(val_acc, index, 0);
            index--;
        }
        envoi_message();
    }

    if (code=='Q')  // Quaternion
    {
        strcpy((char *)message+3, "Q0  ");
        memcpy (flo_valeur, chaine+5, 16);
        ajout_float_multi_valeurs (flo_valeur, 2, 4, 4, 1);
        envoi_message ();
    }
    if (code=='V')  // Vibration-FFT
    {
        //Taille : Statut(1), Axe(1), efficace(float), ratio(float), Ratio_kurt(float)
        //   5 freq(int), 5 val(float)
        // Statut(1), axe(1), efficace(4), ac_ratio(4), ac_kurt(4), ecart_type_kurt(4), 4 * fft_freq(4), 4 * fft_max(2)

        strcpy((char *)message+3, "Vibra:");
        message[9]=' ';
        message[index++] = chaine[ind++] + '0';   // status
        message[index++]=' ';
        message[index++] = chaine[ind++] + '0';   // Axe
        message[index++]=' ';
        memcpy (flo_valeur, chaine+ind, 4);   // Efficace
        ind += 4;
        index = ajout_float_multi_valeurs (flo_valeur, 0, index, 1, 2);
        strcpy((char *)message+index, " Rt: ");
        memcpy (flo_valeur, chaine+ind, 4);   // Ratio
        ind += 4;
        index = ajout_float_multi_valeurs (flo_valeur, 2, index+4, 1, 2);
        memcpy (flo_valeur, chaine+ind, 4);   // Ratio Kurtosis
        ind += 4;
        message[index++]=' ';
        index = ajout_float_multi_valeurs (flo_valeur, 2, index, 1, 2);
        memcpy (flo_valeur, chaine+ind, 4);   // Ecart type Kurtosis
        ind += 4;
        message[index++]=' ';
        index = ajout_float_multi_valeurs (flo_valeur, 0, index, 1, 2);
        index--;
        strcpy((char *)message+index, "%   Fq: ");
        memcpy (u16_valeur, chaine+ind, 2*Nb_valeur_max);   // Frequence Hz
        index = ajout_int16_multi_valeurs ((int16_t *)u16_valeur, index+6, Nb_valeur_max, 0);  // Freq Hz
        memcpy (flo_valeur, chaine+ind+2*Nb_valeur_max, 4*Nb_valeur_max);   // Frequence Hz
        index = ajout_float_multi_valeurs ((float *)flo_valeur, 1, index, Nb_valeur_max, 2);   // milli-g
        envoi_message ();
    }
    if (code=='T')  // Temperature hygrometrie
    {
        strcpy((char *)message+3, "Tempe:");
        message[9]=' ';
        int16_t rawN[2];
        //float rawF[2];
        memcpy (rawN, chaine+ind, 4);
        flo_valeur[0] = (float) rawN[0]/100;
        flo_valeur[1] = (float) rawN[1]/100;
        ajout_float_multi_valeurs (flo_valeur, 2, index, 2, 0);  // Temp et hygro
        envoi_message();
    }
    if (code=='L')  // Luminosite
    {
        strcpy((char *)message+3, "Lumi: ");
        message[9]=' ';
        //uint16_t rawV[0];
        memcpy (flo_valeur, chaine+ind, 4);
        //flo_valeur[0] = SensorOpt3001_convert  ( rawV[0]);
        ajout_float_multi_valeurs (flo_valeur, 2, index, 1, 0);  // Lumi
        envoi_message();
    }
    if (code=='P')  // Pression
    {
        strcpy((char *)message+3, "Press:");
        message[9]=' ';
        int16_t TemperN;
        uint16_t PressN;
        memcpy (&PressN, chaine+ind, 2);
        memcpy (&TemperN, chaine+ind+2, 2);
        flo_valeur[0] = (float) (PressN + 50000 ) / 100;
        flo_valeur[1] = (float) TemperN / 100;
        ajout_float_multi_valeurs (flo_valeur, 2, index, 2, 0);  // Pression
        envoi_message();
    }
}


void Calibration_T_hygro(void)
{
    // Chaque device calibré est numerote de 0 à xx

   // COEFF CALIBRATION
   int16_t const OFFSET_TEMP[NB_MODULES_CAL_TH]={0,0,0,0,0,0,0};
   int16_t const OFFSET_HYGRO[NB_MODULES_CAL_TH]={0,0,0,0,0,0,0};


   uint8_t  part_id = Part_Identif();

     G_OFFSET_T = OFFSET_TEMP[part_id];
     G_OFFSET_H = OFFSET_HYGRO[part_id];
}
#endif


#ifdef BATT_BMS
void envoi_soc_soh()
{
    strcpy((char *)message+1, "BM:Soc=   % Soc2=   % Soh=   %");
    uint8_t Soc = batt_soc_int / (batt_soh / 100);
    uint8_t Soh = batt_soh / (BATT_SOC0 / 100);
    ajout_int8_message (batt_soc1, 8, 0x82);
    ajout_int8_message (Soc, 18, 0x82);
    ajout_int8_message (Soh, 27, 0x82);
    envoi_message ();
}

uint8_t mesure_temper_val (int16_t * temp_mes)
{
    *temp_mes = AONBatMonTemperatureGetDegC() ;
    uint8_t temp_valide=1;
    if ((*temp_mes < -30) || (*temp_mes > 50))  temp_valide=0;
    int16_t temp_ecart;
    temp_ecart = *temp_mes - temp_precedent;
    if (temp_ecart < 0) temp_ecart = -temp_ecart;
    if (temp_ecart > 30) temp_valide = 0;
    #ifdef DEBUG_M
        if (!temp_valide)
        {
            strcpy((char *)message, "LErrTemp:");
            ajout_int16_message(*temp_mes, 9, 0);
            ajout_int16_message(temp_precedent, 15, 0);
            envoi_message();
         }
    #endif
    if (temp_valide) temp_precedent = *temp_mes;
    return temp_valide;
}
#endif


#ifdef C10_CAMERA_PIR   // calcul temps d'attente si batterie niveau faible

//  uint8_t esp_pho[6] = { 70, 85,  88, 91, 96, 100};   //
//  uint8_t esp_vid[6] = { 70,  70,  81, 86, 96, 100};
  //100:40s  90:7min  88:15min  86:1h  81:1j  73:1an
uint32_t tps_batt(uint8_t batterie_niv, uint8_t type)
{
    uint32_t res=1;
    uint8_t init,i;
    if (type)
        init = esp_vid[batterie_niv];
    else
        init = esp_pho[batterie_niv];

    if (init > 89)
    {
        if (init <= 100)
            res = 101 - init;
    }
    else if (init == 89)
        res = 15;
    else if (init > 60)
    {
        res = 20;
        for (i=0; i< (88 - init); i++)
            res = res * 2;
    }
    return res;
}
#endif

#ifdef C10_ElecVanne    // activation Vanne et Conversion duree

// Conversion duree electrovanne
// commande EV : XVnd  d:0:arret 1:20s 2:15min 3:30min 4:1h 5:1h30 6:2h 7:3h 8:4h 9:5h
uint16_t transf_duree_ev(uint8_t duree)
{
    if (!duree) return 0;
    else if (duree==1) return 20;     // 20sec
    else if (duree <3) return 60*15*(duree-1);   // 2
    else if (duree <7) return 60*30*(duree-2);   //  3, 4, 5, 6
    else if (duree <10) return 3600*(duree-4);   // 7, 8, 9
    else return 0;
}

// conversion periode programme ev : en nb de 6heures
// periode:0(arret) 1:0,5j 2à7:1à6jours 8:10j  9:15j
uint16_t transf_periode_ev(uint8_t duree)
{
    if (!duree) return 0;
    else if (duree ==1) return 2;  //12h
    else if (duree < 8) return 4*(duree-1);     // 24h
    else if (duree == 8) return 40;   // 10 jours
    else if (duree == 9) return 60;   // 15 jours
    else return 0;
}

void activ_evanne(uint8_t num, uint16_t duree)
{
    if (num < NB_EVanne)
    {
        if (!duree) // arret immediat
        {
            Clock_stop(SortieEV[num].h_clock);
            ACTIV_SORTIE( 2*num+1+1, 1, par16_var[0]);  // extinction EV pendant 0,4 seconde
        }
        else
        {
            //impulsion immediate d'activation
            ACTIV_SORTIE( 2*num+1, 1, par16_var[0]);  // allumage EV pendant 0,4 seconde
            // impulsion arret differee
            Clock_stop(SortieEV[num].h_clock);
            Clock_setTimeout(SortieEV[num].h_clock, (uint32_t)  duree * (1000 / Clock_tickPeriod) * 1000);
            Clock_start(SortieEV[num].h_clock);
        }
    }
}

// demarrage programme a 20 heures(m-verger) ou 21heures(k-plantes)
void demar_prog_21h(time_t secondes_mes)
{
    if ((EV_prog_periode) && (EV_prog_duree))
    {
        activ_evanne (0, 0);  // fermeture vanne si elle etait ouverte
        Clock_stop(EV_prog_ClockHandle);
        EV_prog_timer = transf_periode_ev(EV_prog_periode);
        // calcul du nb de 10secondes restant avant 21heures : max 8640
        uint16_t nb_10secondes_restant;
        nb_10secondes_restant = 24*360 - (((secondes_mes+4*3600) % (24*3600)) / 10);  // 20 heures
        if (NODE == 'k')
            nb_10secondes_restant = 24*360 - (((secondes_mes+3*3600) % (24*3600)) / 10);  // 21 heures
        EV_prog_timer_rest = 1 + nb_10secondes_restant/2160;
        Clock_setPeriod(EV_prog_ClockHandle, (uint32_t)  6*3600 * (1000 / Clock_tickPeriod) * 1000); // 6 heures
        Clock_setTimeout(EV_prog_ClockHandle, (uint32_t)  (nb_10secondes_restant - (EV_prog_timer_rest-1)*2160)*10 * (1000 / Clock_tickPeriod) * 1000); // 6 heures
        Clock_start(EV_prog_ClockHandle);
    }
}
#endif   // fin du ifdef C10ElecVanne


#ifdef C10_sonde_HUmSol    // mesure et envoi valeur sonde
void envoie_mesure_sonde_HumSol(uint8_t type)  // 0:hexa  1:texte
{
    // mesure sonde
    uint16_t humid;
    int16_t temp_mesure_tt;
    Seconds_Time ts;  // nanosecondes

    ACTIV_SORTIE( Power_sonde, 1, 0 );  // Power on de la sonde
    DELAY_MS(par16_var[0]);  // attente stabilisation valeur sonde

    lecture_analog(0, &humid);   // très humide:900mV(100%)   très sec:2100mV(0%)

    ACTIV_SORTIE( Power_sonde, 0, 0 );  // power off de la sonde

    temp_mesure_tt = (AONBatMonTemperatureGetDegC()) +20 ;
    //temp_mesure_tt = (AONBatMonTemperatureGetDegDec()>>7) +20 ;

#if NB_SONDE_SOL == 2
    uint16_t humid2;

    DELAY_MS(par16_var[0]);  // attente
    ACTIV_SORTIE( Power_sonde2, 1, 0 );  // Power on de la sonde
    DELAY_MS(par16_var[0]);  // attente stabilisation valeur sonde

    lecture_analog(1, &humid2);   // très humide:900mV(100%)   très sec:2100mV(0%)
    ACTIV_SORTIE( Power_sonde2, 0, 0 );  // power off de la sonde
#endif

    // envoie valeur
    if (type) // texte
    {
        uint8_t index;
        strcpy((char *)message+1, "CLH: ");
        index = ajout_int16_message ( humid, 5, 0x82);
        message[index++] = ' ';
        index = ajout_int16_message ( temp_mesure_tt-20, index, 2);
        #if NB_SONDE_SOL == 2
            strcpy((char *)message+index-1, "  Sonde2:  ");
            ajout_int16_message ( humid2, index+9, 2);
        #endif
        envoi_message ();
    }
    else  // message hexa
    {
        strcpy((char *)message, "Lxc");   // c:donnee capteur hexa
        message[1] = 8;  // longueur : 2 valeur
        Seconds_getTime(&ts);  // seconds et nanosecondes
        memcpy (message+3, &ts.secs, 4);
        message[7]   = 'H';  // humidite et temp
        message[8]   = (uint8_t)(humid >> 8);
        message[9] = (uint8_t)(humid & 0xFF);
        //message[8]   = (uint8_t)(temp_mesure_tt >> 8);
        message[10] = (uint8_t)(temp_mesure_tt & 0xFF);
        #if NB_SONDE_SOL == 2
            message[11]   = (uint8_t)(humid2 >> 8);
            message[12] = (uint8_t)(humid2 & 0xFF);
            message[1] = 10;  // longueur : 3 valeurs
        #endif
        envoi_message_def();
    }
}
#endif




#ifdef C10_Compteur    // envoie mesure
    void envoie_mesure_compteur_eau ( uint8_t type)  // 0:hexa  1:texte
    {
        uint8_t index;
        if (type) // texte
        {
            strcpy((char *)message+1, "Cpt: Conso inst:   ");
            index = ajout_int16_message ( conso_instant, 17, 0x82);
            strcpy((char *)message+index, "l/h   index: ");
            index = ajout_int16_message ( cpt_somme_cpt/100, index+12, 0x82);  // cpt_somme_cpt en 10l : 34,12m3
            strcpy((char *)message+index, ",00m3");
            message[index+1] = ((cpt_somme_cpt/10) % 10) + '0';
            message[index+2] = (cpt_somme_cpt % 10) + '0';
            envoi_message ();
        }
        else
        {
            strcpy((char *)message, "LxCLE");
            message[1] = 6;  // longueur : 2 valeurs
            message[5]   = (uint8_t)((conso_instant) >> 8);
            message[6] = (uint8_t)((conso_instant) & 0xFF);
            message[7]   = (uint8_t)((cpt_somme_cpt/100) >> 8);  // max 650000m3  (2000 ans)
            message[8] = (uint8_t)((cpt_somme_cpt/100) & 0xFF);
            envoi_message_def();
        }
    }
#endif  // fin C10 compteur

#ifdef C10_Cent_Meteo    // envoie mesure
    void envoie_mesure_pluviometre ( uint8_t type)  // 0:hexa  1:texte
    {
        Seconds_Time ts;  // nanosecondes
        uint8_t index;
        if (type) // texte
        {
            strcpy((char *)message+1, "CLMeteo:Pluv: ");
            index = ajout_int16_message ( cpt_somme_pluv, 14, 0x82);
            strcpy((char *)message+index-1, "  Vent:  ");
            index = ajout_int16_message ( cpt_somme_vent, index+9, 2);
            envoi_message ();

            /*uint16_t conso_ins;
            conso_ins=0;
            if (time_conso_min)  // exemple 100 pour 10,0s => 3600*10/sec*FIXE
                conso_ins = 360000/time_conso_min*2;
            strcpy((char *)message+1, "Cpt: Conso instant:   ");
            index = ajout_int16_message ( conso_ins/10, 20, 0x82);
            message[index++] = ',';
            message[index++] = (conso_ins%10) + '0';
            strcpy((char *)message+index, "mm/h"); */
        }
        else
        {
            strcpy((char *)message, "LxCLM");
            message[1] = 8;  // longueur : 2 valeur
            Seconds_getTime(&ts);  // seconds et nanosecondes
            memcpy (message+3, &ts.secs, 4);
            message[7]   = 'M';  // Meteo:pluvio, force vent,
            message[8]   = (uint8_t)((cpt_somme_pluv) >> 8);  // en 0,1mm/h
            message[9] = (uint8_t)((cpt_somme_pluv) & 0xFF);
            message[10]   = (uint8_t)((cpt_somme_vent) >> 8);
            message[11] = (uint8_t)(cpt_somme_vent & 0xFF);
            envoi_message_def();
        }
    }
#endif  // fin C10_Pluviometre


#endif  // Fin du ifdef NODE

