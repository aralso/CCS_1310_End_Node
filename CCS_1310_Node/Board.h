
#ifndef __BOARD_H
#define __BOARD_H

#define Board_CC1310_LAUNCHXL

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/Board.h>

#define Board_initGeneral()     Board_init()  /* deprecated */



/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/drivers/PIN.h>

//#include <ti/devices/cc13x0/driverlib/aon_batmon.h>

#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>


/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Seconds.h>

#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>
#include <ti/posix/ccs/sys/time.h>

#include <string.h>


#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/prcm.h)



#include <ti/devices/CC13x0/driverlib/AON_BATMON.h>

#include <ti/devices/cc13x0/inc/hw_aux_wuc.h>

#define CC13xx
//#define ESP32

#include "extflash/ExtFlash.h"
#include "extflash/EEPROM_emu.h"


// -------------   Configuration Hard et adresses module  -----------------


// Config C10_Radar : module CC1310 pour Radar
// Config C50_T_v4  : module CC1350 hard v4 avec capteur température toutes les 6 heures
// Config C50_T_v5  : module CC1350 hard v5 avec capteur température toutes les 6 heures
// Config C50_Mpu   : module CC1350 hard v5 avec capteur acceleromètre
// Config C10_Telec : module CC1310 Telecommande avec 3 boutons

#define C10_CAMERA_PIR  //C10_Radar  // C10_Chien  // C10_Compteur // C10_CAMERA_PIR //  C50_T_v5
#define  DEBUG_M  // Paramètres de débug (plus rapide, sans watchdog)
//#define DEBUG_reseau
//#define  Reseau_test
//#define  DEBUG_TIME 0b1000 // bit0-1:01:radio  10:sensor   Bit2:led tx-rx radio  bit3:Synchro Taille:0x500

#define CODE_VERSION  0x01

#ifndef DEBUG_M
    #define WATCHDOG
#endif

/* Node La Catalane
    a à f : camera
    g : sonde unique
    k : EV plantes (3EV)
    m : EV verger (1EV)
    n : compteur
    o : centrale meteo
    p : reserve (centrale meteo)

 Node Garches :
     X:Radar terrasse
     Y:Radar entree
     Z:HP Chien
     c(EV), g, n, S, V, i, b(C10 test)

Test :
    h : sonde unique
    i : sonde double/unique
    j : réserve (sonde double)
    k : test EV

CODE_APPLI
    a : C50_Mpu accelero
    b : C50_T_V5 sensortag
    c : C10_Test
    d : C50_Test
    e : C50_T_V4 CC1350 sensortag
    f : C10_telecommande
    g : C10_Camera_PIR
    h : C10_Spot solaire
    i : C10_Radar
    k : C10_Compteur
    m : C10_Sonde_humidité sol
    n : C10_Electrovanne
    o : C10_centrale meteo
    p : C10_Chien
*/

#ifdef C10_Chien  // -----------   Module CC1310 pour HP Alarme Chien
// Reception du message ZCH1 -> sortie 1  (DIO_9)
    #define CC1310_carte
    #define CODE_APPLI  'p'
    #define NB_SORTIES  2    // Sorties TOR = 1(Led) + HP Chien
    #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define CPT_MAJ_HEURE   96   // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
    #define MODE_ARRET  0     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
  #ifdef DEBUG_M
    #define NODE 'Z'           // exemple : S:temp, T:temp  U et V :telecommande
    #define TEMPS_INTERROG_MAJ   0      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN   0   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIODE_ACTIV_RX          5      // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
  #else
    #define NODE 'Z'           // exemple : S:temp, T:temp  U et V :telecommande
    #define TEMPS_INTERROG_MAJ   4      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN  21600   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIODE_ACTIV_RX      3      // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
  #endif
#endif

#ifdef C10_ElecVanne
// commande EV : XVnd  d:0:arret 1:30sec 2:15min 3:30min 4:1h 5:1h30 6:2h 7:3h 8:4h 9:5h
// Lecture etat EV : XVnL
// activation du bouton pour activation/désactiv Electrovanne
// XVPEdpD: programme vanne  (XVPL:lecture)  d:duree  p:periode   D(decalage en heures/max10)
// mesure des 2 piles tous les jours et envoi au concentrateur + Alarme si < seuil
// A faire : watchdog, prgm:démarrage clock, surv pile 9V, batterie faible, décalage début prgm, condo sur pile 3V, condo sur pile 9V, protect jack
    #define CC1310_carte
    #define CODE_APPLI  'n'
    #define NB_ENTREES  1    // Entrees TOR : 0:bouton (chgt EV)
    #define CONFIG_BOUTONS     2
    #define NB_EVanne   1    // Sorties Electrovannes
    #define NB_SORTIES  3    // Sorties TOR = 1(Led) + 2 * Nb_EV
    #define NB_PWM      0    // pas de Sortie PWM
    #define MODE_RX   5      // 1:RX continu,       4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define NO_LED0
    #define TEMPS_INTERROG_MAJ   1      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN  21600       // 0:ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define KEEP_AL_Type  3         // type 3: volt_cpu, volt_pile, temp
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define WATCHDOG_SLEEP   300   // en nb de 20 secondes : 18=>6minutes  180:60min  5000=>25heures
    #define TACHE_NODE_WDT  60  // En minutes : pour eviter watchdog
    #define WATCHDOG_ACTIVE  100  // nb d'activation en 20 secondes (max mesure:18-50)
    #define TPS_ALLUM   3    // Temps (en 0,1 secondes) de sollicitation de l'electrovanne
    #define DUREE_VANNE_BP 60  // Duree (minutes) d'ouverture vanne si appui sur BP
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3  4:arret total
    #ifdef DEBUG_M
        #define NODE 'm'           // exemple : S:temp, T:temp  U et V :telecommande
        #define PRGM_VANNE      0x21   // Duree-Periode   0 :pas de prog, 12:30sec-1j   24:15min-3 jours    35:30min-4jours
        //#define Uart2           0     // 0:activé tout le temps  1:pour communiquer avec l'ESP32
        #define PERIODE_ACTIV_RX          5    // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #else
        #define NODE 'k'           // exemple : S:temp, T:temp  U et V :telecommande
        #define PRGM_VANNE  0x43   // 0 :pas de prog, 12:30sec-1j   24:15min-3 jours    35:30min-4jours  43:1h-2j
        #define PERIODE_ACTIV_RX          15    // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
        #define WATCHDOG
    #endif
#endif

#ifdef C10_sonde_HUmSol
// lecture de la sonde (CLH) toutes les heures et envoi au concentrateur :
// très humide:700mV   très sec:2100mV
// mesure de la pile tous les 3 jours et envoi au concentrateur + Alarme si < seuil
// consommation : sleep(0,7uA)+recharge(0,5uA)+watchdog(0,25uA)+interro3jour(0,01uA)=1,5uA
// consommation : 1 envoi sonde par RF chaque heure (13mA pdt 30ms + 6mApdt300ms)   : 0,6uA
// consommation mode=5 (balise synchro) 30 secondes, fenetre=4ms=>9ms=>11,3ms à 4mA => 1,5uA
// TOTAL : 3,6uA =  30 ans avec CR2470

    #define CC1310_carte
    #define CODE_APPLI  'm'
    #define NB_SONDE_SOL  1     // Sonde 1 ou 2 (Attention si 2 sondes => NODE suivant aussi utilise)
    #define NB_SORTIES  1+NB_SONDE_SOL    // Sortie : 0:LED  1:power sonde1 2:power_sonde 2
    #define NB_ANALOG NB_SONDE_SOL   // Entrees Ana 1 ou 2 : 0:IO30(sonde1)  1:IO27(sonde2)
    #define NB_PWM      0    // pas de Sortie PWM
    #define NB_ENTREES  1       // Entrees TOR : 0:bouton
    #define MODE_RX   5      // 1:RX continu,       4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define BATT_MES_PERIODE   1440       // Periode de mesure charge batterie, en minutes
    #define TEMPS_INTERROG_MAJ   1      // KA 4 * MIN  1:1 fois,....  4&21600=24heures  12:3 jours
    #define TEMPS_INTERROG_MIN  21600       // 0:ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define TPS_ALLUM   250    // Temps (en milli secondes) d'allumage de la sonde pour mesure
    #define WATCHDOG_SLEEP   300   // en nb de 20 secondes : 18=>6minutes  180:60min  5000=>25heures
    #define TACHE_NODE_WDT  60  // En minutes : pour eviter watchdog
    #define WATCHDOG_ACTIVE  100  // nb d'activation en 20 secondes (max mesure:18)
    #ifdef DEBUG_M
        #define NODE 'i'           // exemple : S:temp, T:temp  U et V :telecommande
        #define PERIODE_CAPTEUR   30  //  en secondes
        //#define Uart2    0     // 0:activé tout le temps  1:pour communiquer avec l'ESP32
        #define PERIODE_ACTIV_RX          5    // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #else
        #define NODE 'h'           // exemple : S:temp, T:temp  U et V :telecommande
        #define PERIODE_CAPTEUR   3600  //  en secondes
        #define PERIODE_ACTIV_RX          30    // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
        #define WATCHDOG
    #endif
#endif


#ifdef C10_Compteur  // -----------   Module CC1310 pour compteur eau
// envoi index+conso instantanee si ecart avec precedente conso de plus de 5%
// si plus de conso => envoi conso=0 apres trois fois le temps precedent
// Detection de fuite si pas d'arret en 18heures.
    #define CC1310_carte
    #define CODE_APPLI  'k'
    #define NB_ENTREES 1    // Entrees TOR : 0:contact reed
    #define NB_SORTIES 1    // Sortie : 0:LED
    #define NB_PWM 0        // Sorties PWM
    #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define TEMPS_INTERROG_MAJ   28      // KA 4 * MIN  1:1 fois,....  4&21600=24heures  28:1sem
    #define TEMPS_INTERROG_MIN  21600   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define SEUIL_DETECT_FUITE    18   // nb d'heures sans arret => detection de fuite
    #ifdef DEBUG_M
      #define NODE 'n'            // exemple : S:temp, T:temp  U et V :telecommande
      #define Uart2       0       // 0:activé tout le temps  1:pour communiquer avec l'ESP32
      #define COMPTEUR_SEUIL  5   // seuil en % d'evolution de la conso instantanee pour envoi de la mesure
      #define PERIODE_ACTIV_RX   5    // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #else
      #define NODE 'n'               // exemple : S:temp, T:temp  U et V :telecommande
      #define COMPTEUR_SEUIL  5      //  seuil en % d'evolution de la conso instantanee pour envoi de la mesure
      #define PERIODE_ACTIV_RX  30   // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #endif
#endif

#ifdef C10_Cent_Meteo  // -----------   Module CC1310 pour Centrale_Meteo
    #define CC1310_carte
    #define CODE_APPLI  'o'
    #define NB_ENTREES 3    // Entrees TOR : 0:pluvio 1:girouette  2:vent
    #define NB_SORTIES 1    // Sortie : 0:LED
    #define NB_PWM 0        // Sorties PWM
    #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define TEMPS_INTERROG_MAJ   12      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN  21600   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define COMPTEUR_SEUIL  5   // seuil en % d'evolution de la conso instantanee pour envoi de la mesure
    #ifdef DEBUG_M
      #define NODE 'o'           // exemple : S:temp, T:temp  U et V :telecommande
      #define Uart2    0     // 0:activé tout le temps  1:pour communiquer avec l'ESP32
      #define PERIODE_CAPTEUR       60      // periode d'envoi info meteo (secondes)
      #define PERIODE_ACTIV_RX      5   // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #else
      #define NODE 'o'           // exemple : S:temp, T:temp  U et V :telecommande
      #define PERIODE_CAPTEUR       3600      // periode d'envoi info meteo (secondes)
      #define WATCHDOG
      #define PERIODE_ACTIV_RX  30   // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #endif
#endif


#ifdef C10_Radar  // -----------   Module CC1310 pour Radar
// Entree_0 pour Radar PIR sur IOID_15
    #define CC1310_carte
    #define CODE_APPLI  'i'
    #define RADAR_DET         // Detecteur Radar
    //#define PIR_DET          // 1 entree PIR : Detection PIR
    #define NODE_Id      2   // Identifiant radar
    #define Pull_B0  PIN_GEN | (2 << 13)           // 2<<13:pulldown (1:appuyé)
    #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define CPT_MAJ_HEURE   96   // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
    #define MODE_ARRET  0     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
  #ifdef DEBUG_M
    #define NODE 'S'           // exemple : S:temp, T:temp  U et V :telecommande
    #define TEMPS_INTERROG_MAJ   0      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN   0   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIODE_ACTIV_RX          20      // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
  #else
    #define NODE 'X'           // exemple : S:temp, T:temp  U et V :telecommande
    #define PERIODE_ACTIV_RX          20      // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #define TEMPS_INTERROG_MAJ   8      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN  21600   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
  #endif
#endif



#ifdef C10_Spot_solaire  // -----------   Module CC1310 pour Spot solaire avec Radiocontrolli RC868
    #define CC1310_carte
    #define CODE_APPLI  'h'
    #define NB_ENTREES 1    // Entrees TOR : 0:PIR
    #define NB_ANALOG 2     // Entrees Ana : 0:IO30(Volt solaire)  1:IO27(Volt batt)
    #define NB_PWM 2        // Sorties PWM : 0:buzzer  1:Light
    #define Uart2   0  // 0:activé tout le temps  1:pour communiquer avec l'ESP32
    #define NB_SORTIES 2    // Sorties TOR : 0:Led0  1:Led1
    #define PIR_DET          // 1 entree PIR : Detection PIR Spot
    #define SPOT_TOR_PWM     // 1 sortie mixte PWM/TOR : Allumage spot
    #define NODE_Id      2   // Identifiant Spot
    #define CPT_MAJ_HEURE   96   // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
    #define KEEP_AL_Type 1  //  Envoi le voltage batterie avec le keepalive
    #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
  #ifdef DEBUG_M
    #define NODE 'a'           // exemple : S:temp, T:temp  U et V :telecommande
    #define TEMPS_INTERROG_MAJ   4      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN  21600   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define PERIODE_ACTIV_RX          20                // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
  #else
    #define NODE 'T'           // exemple : S:temp, T:temp  U et V :telecommande
    #define TEMPS_INTERROG_MAJ   8      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN  21600   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define PERIODE_ACTIV_RX          10                // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
  #endif
#endif


#ifdef C10_CAMERA_PIR   // Camera solaire avec Detection PIR
// LED IR (PWM)  SEObn Bit0:activ PIR  Bit1:activ PIR+Cam
// WEDB1(test PIR)
    #define CC1310_carte
    #define CODE_APPLI  'g'
    #define OPT_CAPTEUR
    #define Board_HDC1000_ADDR      (0x40)   // HDC1080
    #define BATT_BMS
    #define FLASH_EXTERNE
    #define PILE_RF_FLASH         // Si défini : pile d'envoi en Flash externe (max 1Moctet, 50ms erase), sinon en RAM (max 250octets)
    #define CC13xx_ESP32   // CC connecte a ESP32 : reoriente les messages ESP32 vers le PC, 115kbauds
    #define NB_ENTREES 1    // Entrees TOR : 0:PIR
    #define Pull_B0  PIN_GEN | (2 << 13)           // 2<<13:pulldown (1:appuyé)
    #define NB_ANALOG 2     // Entrees Ana : 0:IO30(Volt solaire)  1:IO27(Volt batt)
    #define NB_PWM 2        // Sorties PWM : 0:buzzer   1:LED IR Flash
    #define NB_SORTIES 3    // Sorties TOR : 0:Led0  2:Enable TP4056   3:ESP32 Power
    #define KEEP_AL_Type 1  //  0:volt_cpu+temp_cpu 1:volt_batt+temp_cpu 2:volt_batt+temp_capt  Envoi le voltage batterie avec le keepalive
    #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define VIDEO_INTERVALLE  40  // intervalle entre 2 videos (en secondes)
    #define VIDEO_MULTIPLI    1.7 // coeff multiplicateur à chaque nouvelle video
    #define PHOTO_INTERVALLE  20  // intervalle entre 2 photos (en secondes)
    #define PHOTO_MULTIPLI    1.25 // coeff multiplicateur à chaque nouvelle photo
    #ifdef DEBUG_M
        #define NODE 'b'                    // exemple : S:temp, T:temp  U et V :telecommande
        #define Uart2   1                   // 0:activé tout le temps  1:pour communiquer avec l'ESP32
        #define MODE_PERIODIQUE      0      //CAPTEUR TEMP   0:arret  1:periodique
        #define CPT_MAJ_HEURE   96          // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
        #define STATUT_OPS  0b01            // Bit0:PIR  Bit1:PIR+Cam
        #define BATT_MES_PERIODE   60       // Periode de mesure charge batterie, en minutes
        #define TEMPS_INTERROG_MAJ   4      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
        #define TEMPS_INTERROG_MIN  21600       // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
        #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
        #define PERIODE_ACTIV_RX          5    // 1à255 sec:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    #else
        #define NODE 'b'                        // exemple : S:temp, T:temp  U et V :telecommande
        #define Uart2   1                       // 0:activé tout le temps  1:pour communiquer avec l'ESP32
        #define MODE_PERIODIQUE      0         //CAPTEUR TEMP   0:arret  1:periodique
        #define CPT_MAJ_HEURE   96              // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
        #define STATUT_OPS  0b11            // Bit0:PIR  Bit1:PIR+Cam
        #define BATT_MES_PERIODE   60           // Periode de mesure charge batterie, en minutes
        #define TEMPS_INTERROG_MAJ   4      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
        #define TEMPS_INTERROG_MIN  21600       // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
        #define PERIOD_PASS_INTERO_CONC    0    // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
        #define PERIODE_ACTIV_RX          10    // 5a90:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
        #define WATCHDOG
        #endif
#endif
// Ordres : aWLFxxxy(lecture 1 fichier)


#ifdef C10_Telec  // -----------   Module CC1310 pour telecommande
    #define CC1310_carte
    #define CODE_APPLI  'f'
    #define TELECOMMANDE
    #define CONFIG_BOUTONS     2
    #define BOUTON_CC     3    // 3 boutons sur la carte
    #define NB_ENTREES 3    // Entrees TOR : 0:PIR
    #define NB_PWM 1        // Sorties PWM : 0:buzzer
    #define MODE_ARRET  0     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #ifdef DEBUG_M
      #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
      #define NODE 'Z'           // exemple : S:temp, T:temp  U et V :telecommande
      #define TEMPS_INTERROG_MAJ   0      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
      #define TEMPS_INTERROG_MIN  22   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
      #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #else
      #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
      #define NODE 'Z'           // exemple : S:temp, T:temp  U et V :telecommande
      #define TEMPS_INTERROG_MAJ   28      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
      #define TEMPS_INTERROG_MIN  21600   // 0/ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
      #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #endif
#endif

#ifdef C50_T_v4   //  ------------  Module CC1350 Hard v4 avec capteur température toutes les 6 heures
    #define CC1350_carte
    #define CODE_APPLI  'e'
    #define CONFIG_BOUTONS     1
    #define dest_erreur_reset  '1'
    #define VERS_HARD    4 // v4:infra  v5:hall
    #define OPT_CAPTEUR  // Permet d'activer les capteurs temp/hygro/lumi
    #define CPT_MAJ_HEURE   96   // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
    #define Uart2  0               // 0:actif
    #define NODE 'c'           // exemple : S:temp, T:temp  U et V :telecommande
    #define MODE_RX   5    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define FLASH_EXTERNE
    #define PILE_RF_FLASH  // Si défini : pile d'envoi en Flash externe (max 1Moctet), sinon en RAM (max 250octets)
    #define MODE_PERIODIQUE      1   //CAPTEUR 1   0:arret  1:periodique
    #define TEMPS_INTERROG_MAJ   4      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN   21600 // 21600    // 0/ndef:pas de keep-alive/interro conce  x:keep-alive/interro conc max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define PERIODE_ACTIV_RX          5                // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
#endif


#ifdef C50_Test   //  ------------  Module CC1350 Hard v5 avec test capteur température toutes les 20 secondes
    #define CC1350_carte
    #define dest_erreur_reset  '1'
    #define CODE_APPLI  'd'
    #define VERS_HARD    4        // v4:infra  v5:hall
    #define MODE_RX      1   // 1:RX continu,         4:Demande TX (periodique)    5:RX periodique broadcast  6:pas de RX
    #define CPT_MAJ_HEURE   96   // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
    #define NODE 'a'               // exemple : S-Z a-z
    #define NB_ANALOG 0     // Entrees Ana : 0:IO27(Volt batt) 1:IO30(Volt solaire)
    #define Uart2  0               // 0:actif
    //#define Uart2_Add              // permet d'adresser l'uart avec l'adresse S
    //#define OPT_CAPTEUR            // Permet d'activer les capteurs temp/hygro/lumi
    #define MODE_ARRET  0     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define FLASH_EXTERNE
    #define PILE_RF_FLASH         // Si défini : pile d'envoi en Flash externe (max 1Moctet), sinon en RAM (max 250octets)
    #define MODE_PERIODIQUE      0   //CAPTEUR 1   0:arret  1:periodique
    #define TEMPS_INTERROG_MAJ   4      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN   21600   // 0/Ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define PERIODE_ACTIV_RX          10                // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
    //#define WATCHDOG_SLEEP 2   //    6=>120s=2 min
#endif

#ifdef C10_Test   //  ------------  Module CC1350 Hard v5 avec test capteur température toutes les 20 secondes
    #define CC1310_carte
    #define CODE_APPLI  'c'
//    #define NB_ENTREES 1    // Entrees TOR : 0:PIR
    #define NB_ANALOG 3     // Entrees Ana : 0:IO27(Volt batt) 1:IO30(Volt solaire)
    #define NB_PWM 1        // Sorties PWM : 0:buzzer  1:Light
    #define NB_SORTIES 3    // Sorties TOR : 0:Led0  1:Led1  2:Enable_charge
/*    #define PIR_DET          // 1 entree PIR : Detection PIR Spot
    #define SPOT_TOR_PWM     // 1 sortie mixte PWM/TOR : Allumage spot
    #define NODE_Id      2   // Identifiant Spot*/

    #define MODE_RX     5   // 1:RX continu,         4:Demande TX (periodique)    5:RX periodique broadcast  6:pas de RX
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define CPT_MAJ_HEURE   96   // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
    #define NODE 'Z'               // exemple : S-Z a-z
    //#define Uart2  0               // 0:actif
    //#define Uart2_Add              // permet d'adresser l'uart avec l'adresse S
    //#define OPT_CAPTEUR            // Permet d'activer les capteurs temp/hygro/lumi
    //#define FLASH_EXTERNE
    //#define PILE_RF_FLASH         // Si défini : pile d'envoi en Flash externe (max 1Moctet), sinon en RAM (max 250octets)
    #define MODE_PERIODIQUE      0   //CAPTEUR 1   0:arret  1:periodique
    #define TEMPS_INTERROG_MAJ   0      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN   2000   // 0/Ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define PERIODE_ACTIV_RX          10                // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
#endif

// Periode de lecture capteur : voir en ligne 512
#ifdef C50_T_v5   //  ------------  Module CC1350 Hard v5 avec capteur température toutes les 6 heures
    #define CC1350_carte
    #define CODE_APPLI  'b'
    #define VERS_HARD    5        // v4:infra  v5:hall
    #define MODE_RX   6    // 1:RX continu,         4:Demande TX (periodique)    5:RX periodique broadcast  6:pas de RX
    #define CPT_MAJ_HEURE   168   // Périodicite pour mise à jour de l'heure 168=1 semaine  /  en heures  max:65000.  0 pas d'interrogation periodique
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
  #ifdef DEBUG_M
    #define NODE 'S'               // exemple : S-Z a-z
    //#define Uart2  0
    #define OPT_CAPTEUR           // Permet d'activer les capteurs temp/hygro/lumi
    #define FLASH_EXTERNE
    #define PILE_RF_FLASH         // Si défini : pile d'envoi en Flash externe (max 1Moctet), sinon en RAM (max 250octets)
    #define MODE_PERIODIQUE      1   //CAPTEUR 1   0:arret  1:periodique
    #define TEMPS_INTERROG_MAJ   0      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN   2000   // 0/Ndef:pas de keep-alive/interro conce  x:durée en secondes max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define PERIODE_ACTIV_RX          10                // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
  #else
    #define NODE 'U'               // exemple : S: T:Entree U:  V:Exterieur
    #define OPT_CAPTEUR           // Permet d'activer les capteurs temp/hygro/lumi
    #define FLASH_EXTERNE
    #define PILE_RF_FLASH             // Si défini : pile d'envoi en Flash externe (max 1Moctet), sinon en RAM (max 250octets)
    #define MODE_PERIODIQUE      1   //CAPTEUR 1   0:arret  1:periodique
    #define TEMPS_INTERROG_MAJ   12      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
    #define TEMPS_INTERROG_MIN   21600 // 21600=6h    // 0/Ndef:pas de keep-alive/interro conce  x:keep-alive/interro conc max:42000
    #define PERIOD_PASS_INTERO_CONC    0   // KA 0:ka+int    1:interro uniq,  2:1fois sur 2
    #define PERIODE_ACTIV_RX 60     // ecoute toutes les 30 secondes
    #define WATCHDOG
    #define WATCHDOG_ACTIVE  100  // nb d'activation en 20 secondes
  #endif
#endif


#ifdef C50_Mpu   //  ------------  Module CC1350 Hard v5 avec capteur accéléromètre
    #define CC1350_carte
    #define CODE_APPLI  'a'
    #define VERS_HARD    5 // v4:infra  v5:hall
    #define NODE 'S'           // exemple : S:temp, T:temp  U et V :telecommande
    #define MODE_RX   6    // 1:RX continu,         4:Balise TX (peridique)    5:RX periodique  6:pas de RX
    #define MODE_ARRET  3     // 0:pas d'arret  1:arret int1  2:arret int2  3:arret int3   4:arret total
    #define FLASH_EXTERNE
    #define PILE_RF_FLASH  // Si défini : pile d'envoi en Flash externe (max 1Moctet), sinon en RAM (max 250octets)
    #define OPT_CAPTEUR  // Permet d'activer les capteurs temp/hygro/lumi
    #define CAPTEUR_MPU  // Permet d'activer les fonctions liées au capteur MPU acceleromtre  ROM=40ko
#endif



// Valeurs par défaut des defines

// #define BALISE_FINIES  // ne demande que 3 balises puis s'arrete

#ifdef OPT_CAPTEUR
    #define NB_CAPTEURS 3
    #define CAPTEUR_TEMP  // ou pas pour désactiver
    //#define CAPTEUR_LUMI
    //#define CAPTEUR_PRESS  // ou pas pour désactiver

    #define ACTIV_CAPTEUR  1    // 0 (inactif au démarrage)   ou 1 (actif au démarrage)
    #define NB_MODULES_CAL_TH  7   // Nb de modules calibres en temp/hygro dans le code
#endif


#ifdef CAPTEUR_TEMP
    #ifdef DEBUG_M   // mode debug
        #define TEMP_ACTIF       1   // 0:inactif  1:actif
        #define TEMP_PERIOD      30   // 10 periode de mesure en secondes      (max:39000)
        #define TEMP_PERIOD_MIN   1   // 18(3h) periode minimum d'enreg de valeur (chaque x mesures) (max:255)
        #define TEMP_ECART_MIN   30   // 50 ecart en 1/400 degres  (max:255) int:50, ext:70, cave:20
        #define TEMP_GROUPES     1     // 1:groupe   0:immediat
    #else    // mode normal
        #define TEMP_ACTIF       1   // 0:inactif  1:actif
        #define TEMP_PERIOD      600   // 600 periode de mesure en secondes      (max:39000)
        #define TEMP_PERIOD_MIN  8   // 18(3h) periode minimum d'enreg de valeur (chaque x mesures) (max:255)
        #define TEMP_ECART_MIN   40   // 50 ecart en 1/400 degres  (max:255) int:50, ext:70, cave:20
        #define TEMP_GROUPES     1     // 1:groupe   0:immediat
    #endif
#endif
// Nota : voir aussi TEMP_GROUPES dans comm_messages.h


#ifndef CONFIG_BOUTONS
    #define CONFIG_BOUTONS     1  // 0:boutons desactivés  1:B0C+B1C   2:tous actifs  (message SEBOx)
#endif

#ifndef Pull_B0  // defaut:1<<13:pullup (0:appuyé)    2<<13:pulldown (1:appuyé)
    #define Pull_B0  PIN_GEN | (1 << 13)
#endif

#ifndef Pull_B1  // defaut:1<<13:pullup (0:appuyé)    2<<13:pulldown (1:appuyé)
    #define Pull_B1 PIN_GEN | (1 << 13)
#endif

#ifndef Pull_B2  // defaut:1<<13:pullup (0:appuyé)    2<<13:pulldown (1:appuyé)
    #define Pull_B2 PIN_GEN | (1 << 13)
#endif

#ifndef dest_erreur_reset
      #define dest_erreur_reset 'L'
#endif

#ifndef NB_TACHES_WDT  // nb de taches surveillees par le watcdog
    #define NB_TACHES_WDT 2
#endif


#ifndef WATCHDOG_SLEEP   // declenchement watchdog si sleep>25heures (5000) 0 pour desactivation
    #define WATCHDOG_SLEEP 5000
#endif

#ifndef WATCHDOG_ACTIVE   // declenchement watchdog si activ 3000 fois en 20 sec   0 pour desactivation
    #define WATCHDOG_ACTIVE 3000
#endif

#ifndef  PERIODE_ACTIV_RX
    #define PERIODE_ACTIV_RX          30      // 1à255:Periode d'interrogation (mode 4) ou d'activ fenetre RX (mode 5), en secondes
#endif

#ifndef NB_ENTREES
    #define NB_ENTREES 2    // 0:BP0   1:BP1
#endif
#ifndef NB_ANALOG
    #define NB_ANALOG 2     // 0:IO 27   1:IO 30    +1ANAL => +20(RAM) +19(Flash)
#endif
#ifndef NB_PWM
    #define NB_PWM 1        // 0:buzzer             +1PWM => +95(RAM) +103(Flash)
#endif
#ifndef NB_SORTIES
    #define NB_SORTIES 1    // 0:Led ou radar/PIR
#endif

#ifndef CPT_MAJ_HEURE
    #define CPT_MAJ_HEURE 0
#endif

#ifndef BROAD_ADD   // Adresse broadcast pour l'envoi de la balise (utile pour le concentrateur)
    #define BROAD_ADD  0xFF
#endif

#ifndef BOUTONS_CC
    #define BOUTONS_CC    2   // 2 boutons par défaut
#endif

#define BATT_SOC0  330000000  // en 10nAh  3300mAh

#ifdef Uart2
    #define NB_UART 1   // 1 Uart max sur CC13xx
#endif

#ifndef PERIOD_PASS_INTERO_CONC
    #define PERIOD_PASS_INTERO_CONC    1   // KA 0:tout ka    1:interro uniq,  2:1fois sur 2
#endif

#ifndef TEMPS_INTERROG_MAJ
    #define TEMPS_INTERROG_MAJ   0      // KA 4 * MIN  1:1 fois,....  4&21600=24heures
#endif

#if (MODE_RX == 5)
    #define BROAD_BALISE_SYNCHRO
#endif

// Config FLASH
#define OFFSET_FLASH 0 //  1ère section de la flash pour la pile_RF (1er destinataire)

#define NB_DESTINATAIRES_NODE           1  // Nb de destinaires qui doivent avoir une pile d'envoi

#include "RadioProtocol.h"

#ifdef CONCENTRATOR
   #include "DmConcentratorTask.h"
#endif

#ifdef NODE
   #include "DmNodeTask.h"
#endif


#include <DmRadioTask.h>
#include "comm_message.h"
#include "utilitaires.h"
#include "periph.h"

//  ----- Mode Debug -------------

/*Reception :
10 : reception message normal
12 : send Ack (debut)
15 :keepalive recu

Envoi :
20 :début envoi message  (0x22 :resend)
24 : Ack receveid (fin)
28: fin envoi message (avec result)

0 : Interr radio
30 :phase radio suivante*/



// modifier aussi :
//   comm_message.h : lignes 20 : confif uart, addresse concentrateur...
//   comm_message.c : lignes 160-200 : adresse uart, table routage
//                    lignes 420 : ancien et nouveau hard
//   RadioProtocol.h ligne  20 a 50 (donnees du node)





#define Board_shutDownExtFlash() CC1310_LAUNCHXL_shutDownExtFlash()
#define Board_wakeUpExtFlash() CC1310_LAUNCHXL_wakeUpExtFlash()

/* These #defines allow us to reuse TI-RTOS across other device families */

#define Board_MPU_INT           CC1350STK_MPU_INT
#define Board_MPU_POWER         CC1350STK_MPU_POWER
#define Board_MPU_POWER_OFF     CC1350STK_MPU_POWER_OFF
#define Board_MPU_POWER_ON      CC1350STK_MPU_POWER_ON

#define Board_ADC0              CC1350STK_ADC0

#define Board_BUZZER            CC1350STK_BUZZER
#define Board_BUZZER_ON         CC1350STK_LED_ON
#define Board_BUZZER_OFF        CC1350STK_LED_OFF

#define Board_GPIO_LED_ON       1
#define Board_GPIO_LED_OFF      0

#define Board_GPTIMER0A         CC1350STK_GPTIMER0A
#define Board_GPTIMER0B         CC1350STK_GPTIMER0B

#define Board_I2C0              CC1350STK_I2C0
#define Board_I2C0_SDA1         CC1350STK_I2C0_SDA1
#define Board_I2C0_SCL1         CC1350STK_I2C0_SCL1
#define Board_I2C_TMP           CC1350STK_I2C0

#define Board_NVSINTERNAL       CC1350STK_NVSCC26XX0
#define Board_NVSEXTERNAL       CC1350STK_NVSSPI25X0

#define Board_PIN_BUTTON0       CC1350STK_KEY_LEFT
#define Board_PIN_BUTTON1       CC1350STK_KEY_RIGHT
#define Board_PIN_BUTTON2       CC1350STK_RELAY
#define Board_PIN_LED0          Board_GPIO_S0
#define Board_PIN_LED1          Board_GPIO_S1

#define Board_PWM0              CC1350STK_PWM0
#define Board_PWM1              CC1350STK_PWM1

#define Board_SPI0              CC1350STK_SPI0
#define Board_SPI1              CC1350STK_SPI1
#define Board_SPI_FLASH_CS      CC1350STK_SPI_FLASH_CS
#define Board_FLASH_CS_ON       0
#define Board_FLASH_CS_OFF      1


#define Board_UART0             CC1350STK_UART0

#define Board_WATCHDOG0         CC1350STK_WATCHDOG0

/* Board specific I2C addresses */
#define Board_BMP280_ADDR       (0x77)
#ifndef Board_HDC1000_ADDR
    #define Board_HDC1000_ADDR      (0x43)
#endif
#define Board_MPU9250_ADDR      (0x68)
#define Board_MPU9250_MAG_ADDR  (0x0C)
#define Board_OPT3001_ADDR      (0x45)
#define Board_TMP_ADDR          (0x44)

#ifdef __cplusplus
}
#endif


#include "CC1350STK.h"

#endif /* __BOARD_H */
