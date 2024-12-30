

 /*  ======== rfWsnMain.c ========

 Capteurs programmés :

GARCHES :
    -Q 145EU 39F6 : conc_2.2.2 Passerelle
    -Q Test  0F60 : conc test
     R
     S 151EU 5B04 (2,6uA) 2.1.7 le 16/4/2020  Ch nous
    -T 151EU 0A66 (4uA)  2.1.7 le 16/4/2020   Entree
    -U 145EU 03F0 (4,3uA) 2.1.7 le 16/4/2020  Ch Arthur
     V 151EU 0F60 (2,8uA) 2.1.7 le 16/4/2020  Exterieur
    -W 151EU 7992 (2,9uA) 2.1.7 le 16/4/2020  Cave
     X 151EU 628C (87uA 2,8uA) 2.1.6 le 15/2/2020  Ref
    -Y Radar ext : CC1310RC   : 2.19.v2 le 20/12/19 Radar ext
    -Z Télécommande : CC1310RC   : Telecommande
     0 boucle interne
     1 port série rattache
     2 Camera ESP 32
     h Sonde humidite
KERNERS :

CATALANE :

     - 151EU 52A4 avec capteur hall (76uA 3,2uA)
       151EU 3C78 (170uA-HS ex U)
       145US 1230 :
       145EU 7E30 : passerelle test
       145US 2E62 (4,2uA) calib
       145fils 72A8 (4uA)

TODO : au "power on" la memoire Flash du CC13xx ne s'initilise pas toujours correctement
        verifier la reception du port serie : si plein de caracteres non formates
        verifier capteur de temperature
        BMLST : mettre temperature univ


points à tester :
        investiguer pourquoi conso de 1mA résiduelle apres CLTT
        investiguer pourquoi temp=15°C aleatoirement
        corriger BEPxx
        telecommande : allumage led en AR / code tournant non fonctionnel / vérifier RX apres
        spot solaire : verif pir, verif temp
        cam solaire : pcb, E70, emission uart, modif concentrateur

  v3.0.2  Sept 2023
        Ajout : debug_reseau
        Fix : BLSY non renvoye
        Ajout : BLST : nb de demande resynchro et temps pour la prochaine
        Ajout : type
        Ajout : possibilité d'inversion des boutons/entrées : pullup -> pulldown
        Ajout Camera_Pir:C
        Modif Camera_Pir:activation Flash externe

  v3.0.1  Juin 2023
        Ajout : Lecture temps restant ESP (WLC)

  v3.0.0   Juin 2023 : identique au 2.3.14 mais avec Simplelink 4.20.02.07
        Fonctionnel sur Capteur temperature SensorTag

  v2.3.14  Janvier 2023
        Ajout numéro de version (par SLV dans board.h)

  v2.3.13  Mai 2022
        Ajout Commande Electrovanne(XVnd, XVnL, XVPEdpD)
        Ajout Sonde humidite(CLH), mode_arret(PEM3),
        Ajout Compteur(CLE),
        Ajout Camera:nb_videos/photos
        Modif boutons plus lent
        Modif demande balise avec temps d'attente random

  v2.3.12  Février 2022 (avec ESP 3_3_15, STM 3_2_8 et Con_2_2_10)
        modif HES : ajout format
        fix bug sur LastaddedsensorNode (pb TAILLE_PILE_ENVOI_RADIO)

  v2.3.10  Mai 2021 (avec ESP 3_3_14, STM 3_2_8 et Con_2_2_10)
         Operationnel
         envoi temps avant balise
         Ajout luminosité

  v2.3.9  Mai 2021
          Ordres pour camera, allumage ESP, 11WK10, 11WK20
          Opérationnel mail bug sur transfert de fichiers trop longs

  v2.3.8  Avril 2021
          reception et transmission Messages long
          fix bug sur reception uart de plusieurs messages

  v2.3.7  Mars 2021 - non fonctionnel - messages longs

  v2.3.6  Fevrier 2021
          ajout keepalive envoyé a 20h pile, et envoi d'un seul voltage
          adaptation HDC1080 et Camera_PIR_solaire

  v2.3.5  Dec 2020
          test BiasMode et frontEndMode
          ajout QRLRZ pour test multiples reception
          fix Watchdog et 2 warning
          fix No_pullup sur entrée PIR
          fix pb entrees analogiques (0:30, 1:27, 2:26)

  v2.3.4  Dec 2020
          amélioration mode 5 balise
          réduction fuite 80uA avec DP3

  v2.3.3  Dec 2020
          renomme RLT, RLU

  v2.3.2  Nov 2020 : fichiers comm, util, periph commun avec Concentrator
          Watchdog, test approfondi sur modes 1,5,6

  v2.3.1  Sept 2020 : fichier comm_message.c et utilitaires.c generiques
          callback clock identiques
          Ajout module pluviometrie

  v2.3.0  Sept 2020 (pour interface ESP32_camera)
          Ajout gestion energie batterie (BATT_BMS)
          Ajout loop(0) et interface_serie(1)

  v2.2.3  Juillet 2020   (avec STM 3_2_8 et Con_2_2_4)
          Appli Camera solaire
          Uart activable en run

  v2.2.2  Mai 2020   (avec STM 3_2_8 et Con_2_2_4)
          Applis Spot solaire et  Telecommande
          Suppression REBH
          BEP : amelioration modif periode d'ecoute balise synchro
          Voltage batterie : envoie 1 fois par seconde max
          XS147 (ajout) : allumage spot PWM pour CC1310
          AL0/AL1/AL2 : lecture analogique IO:27/30/24
          BLZ (ajout) : réactivation balise synchro
          Plusieurs PWM(2), Entrees Analogiques(3), Sorties TOR(3)
          Appli Telecommande

  v2.2.1  Mai 2020   (avec STM 3_2_8 et Con_2_2_4)
          Balise : pas de raz ecart quand nouvelle demande
          Extinction : modif seuil de non detection rssi
          BLD (ajout) lecture infos balise synchro
          SLPx/SEPx (ajout) ecriture/lecture parametre
          BEP/BEF (ajout) : ecriture duree periode et fenetre des balises synchro


  v2.2.0  Avril 2020   (avec STM 3_2_8 et Con_2_2_3)
          Ajout fonctionnalité balise synchronisee pour Node en écoute (mode_rx=5)
          CLTT : Ajout : Lecture temperature + envoie precedentes mesures du buffer

  v2.1.8  Avril 2020    (avec STM 3_2_7 et Con_2_2_2)
          modif envoi Voltage_batterie du keepalive (apres premier envoi)
          Correction bug sur modif periode keepalive (KPIm)

  v2.1.7  Février 2020  (avec STM 3_2_7 et Con_2_2_2)
          HET : Ajout : demande mise à jour de l'heure
          REBH/M : Ajout : Activation balise demande RX periodique courte
          SEVB : Ajout : Raz valeur voltage sauvegardé
          RLR : Modif : envoi niveau réception à Q puis L
          SDEB1 : Ajout : envoi debug_time à L
          flash LED si bouton 1 appuyé (flash long si message recu)
          Réduction puissance émission à 13mA (au lieu de 26mA)
          correction bug sur parenthese (1000*Clock_tickPeriod)
          correction bug sur eeprom/flash
          correction bug sur RX trop tard (pb flash 2ms)

  v2.1.6  Février 2020   (avec STM 3_2_7 et Con_2_2_1)
          Réduction conso 50uA->1uA avec CC1310  et 4uA avec CC1350
          BHE/BP : Ajout : Balise temporelle - mesure synchro

  v2.1.5  Janvier 2020   (avec STM 3_2_7 et Con_2_2_1)
          Fonctionnel
          Debug fonction Pile_RF_flash
          Debuggage pin led et buzzer, capteur temp (avec pression)
          Envoi radio : lg message

  v2.1.4  Décembre 2019
          non fonctionnel
          Nouveau protocole d'échange radio (depuis 2.0.18)
          reenvoi si echec
          message radar : PDETxx

  v2.0.20(3) cc1350(non validé), keepalive

  v2.0.19(2) Dec 2019 : transfert vers CC1310 : v1:simplement TX  V2:ops sur cc1310(avec conc2.2.0)

* v2.0.18(1)  Sept 2019
*          si echec envoi, reessaye plusieurs fois
*          Refonte cycle radio
*          Modif structure des paquets recus-envoyés (ajout statut)
*
*
* v2.0.17  Sept 2019
              Passage au sdk simplelink 3.20.0.23
              Ajout lecture analogique

* v2.0.15  Sept 2019
           CC1310 radio controlli
           Ajout balise keepalive avec 2 volt et 1 temp

* v2.0.14  Mai 2019
           Correction bug hygrometrie
           Activation lecture pression

* v2.0.13  Mai 2019   OPS-REF
          ajout fonctions gestion seuils de temperature (ecart minimim CTLEM et temps max CGLT)
          correction bug dans lecture Flash section EEPROM

* v2.0.11 Avril 2019
          Identification version hard (4 ou 5) - correction bug sensor_init
          simplification des fonctions hexa_deci
          ajout protection sur les memcpy (évite dépassement tableaux)


* v2.0.7  Avril 2019
          ajout fonction lecture-ecriture index_flash (CLFI, CLFDxx, CEFTxx)
          Correction bug dans ecriture flash à la fin de la Flash FDFFF+

* v2.0.6  Avril 2019
          Correction corruption variable message utilisée dans 2 taches
          Correction bug dans algo choc (init_algo2)

* v2.0.5  Avril 2019
          Complet comme 7.10.4

* v2.0.4  Avril 2019
          simplification de  7.10.4 mais avec Simplelink 3.10.0.11 et RTOS 2.21.0.06

* v7.10.4 Avril 2019
          Flash(1DEEC) Ram(0x3F4D-2636)
          Utilisation d'une clock (RTOS) pour LED et buzzer

* v7.10.3 Mars 2019
          Désactivation paramétrable des boutons (SEBOx)
          Paramétrage activation mode periodique (SEMPx)
          Paramétrage lecture unique (CLUN)
          Paramétrage enreg données brutes (CMEAZx)
          Interrogation flash à distance (CLF0)
          Clock écoute/envoi passerelle toutes les 10 s (mode_rx=4)
          Test d'envoi des donnees toutes les 10s
          bugs corrigés : rajout wait dans Flash_close pour passage correct en power_down
          bugs non encore corriges : parfois le module ne s'éteignait pas, amélioration déchargement flash


* v7.10.2 Mars 2019
          Emulation EEPROM avec Flash externe
          Flash : ajout de sections (0:flash normale, 1:EEPROM)
          Gestion de index_flash en EEPROM
          bugs corriges : read flash circulaire, effacement flash

* v7.10.1 Mars 2019
          Mémoire tampon en mémoire Flash
          Re-test d'envoi à la prochaine dispo de données
          Possibilités d'interrogation de la mémoire flash pour déchargement

* v7.9.3 Janv/2019
          correction bug dechagement flash (CLF)
          Finalisation Vibration-FFT
*
* v7.9.2  Dates RTC corrigé
*         Ram tampon dans node : Envoi mesures par groupes de 6
*         Flash tampon dans concentrateur

* v7.9.1  Capt temp-hygro
*         millisecondes
*
* v7.8.14 define capteur_mpu, mode_rx sans ack ni rx, envoi balise sur mouvement, init capteur_mpu
*         reduction temps init capteur mpu
*         protocole radio 50kbps
*
* v7.8.13 modification trame radio : ajout code sans ack ni ecoute apres envoi
*         radio fonctionnelle, ajout FFT machine tournante
*
* v7.8.12 amélioration radio : correction bug si envoi avec echec
*
* v7.8.11 diminution code, lowpower, réduction heapsize
*         RAM 14892  CODE 111231
*         Radio  fonctionnelle
*
* v7.8.10 modif cc1350.h : diminution RAM : limitation a 1 seul pwm (au lieu de 8)
          fonctionnel : modes 0 à 13, messagerie, 3 algorithmes, calibration
          RAM 19386  CODE 113975
*
* v7.8.9 tache sensor_mpu séparée
*        rassemblement des 4 algorithmes ensemble
*        gestion interruption sur accelero
*        réveil sur mouvement accelero
*
* v7.8.8 mode 0 à 8 operationnels : avec chocs et val brutes
*        calibration accelero par min & max
*
* v7.8.6 Analyse vibratoire et FFT : ok
         correction bug sur appui bouton anti-rebond double/court

* v7.8.5 autonomie capteur accelero
*        analyse vibratoire et FFT : ok
*       RAM node  :19069
*
* v7.8.4 ajout capteur hygro, fonctionnement capteur accelero
*
* v7.8.3 : non fonctionnel
*
* V7.8.2 Test buzzer et resend. Ajout accelero- fonctionnement ok sauf magnetometre. Node ok, mais pas concentrator
*
* V7.8  22/06/2018
     - refonte du séquencement Radio Node-concentrateur en modes 4-5-6
     - bug residuel : Radio_error_rx aléatoire

* V7.7  22/05/2018
*    - table avec choix capteurs
*    - decodage (concentrateur) des messages transmis par les capteurs
*    - QHLS demande periodique de l'heure (mise à jour)
*    - QLTAx demande etat capteur S
*
* V7.6  22/05/2018
     - opérationnel - fonctionne sans bootloader - S envoie temp à L si on appuie su BP0

* V7.5  21/05/2018
     - opérationnel - fonctionne avec bootloader

* V7.3  20/05/2018
     - passage au sdk 2.10.00.36 (Avril 2018) et TI-RTOS 2.21.00.06 (sept 2016) non fonctionnel

* V7.2  19/04/2018
    - passage au sdk 1.60.0.21 et TI-RTOS

 * v5  31/01/2018 (node:Flash:16BA3 ram:43B3 )
    - config avec plusieurs capteurs activables
    - possiblité d'activer un message périodique (température, RSSI),
    - possibilité de passer en Long Range (SL-LR2) dans radioprotocol.h
    - gestion des node hors portée : réémission temps*4 puis 1 fois/3jour
    - possibilité de forcer l'envoi vers les node hors portee

 * v4  28/01/2018 (node: Flash:161F6 ram:432C)
    - config avec OPT_CAPTEUR
    - option d'envoi toutes les x mesures
    - option : uniquement temp, sans hygro
    - à l'init. le Node demande de l'heure au concentrator
    - maj de l'heure du concentrateur => maj de l'heure des nodes
    - affichage sur L du niveau de réception

 * v3  25/10/2017 (conc: Flash:13679  Ram:4883)
 *   Concentrateur (Q) : pb quand send en meme temps que reception balise TX -> pas de réémission
 *   Effacement des message RF a envoyer si trop long/court -> ok
 *   Effacement des message Rf si pas envoyé au bout de 5 fois
 *   Modif reception si mode_rx=4/5/6 : activ_rx peut s'indiquer avant l'envoi d'un message si une réponse est attendue
 *   Les réponses sont envoyées après la réception du dernier message
 *   Telecommande : sécurisation envoi (code tournant et délai)
 *   Table d'Enregistrement des nouveaux nodes (avec mode_rx, last_rssi,...)
 *   si mode_rx du node = 4, 5, 6, pas d'émission par le concentrateur lors de l'élaboration d'un message
 *   Node : pb qd reception message REM6 -> ok
            S=capteur temperature
            T=capteur luminosité
 *          U,V=Node telecommande (3 bouttons) -> ok
 *          pb i2c et uart simultané

 * v2  22/10/2017 (conc: Flash:132CD  Ram:4897)
 *   Concentrateur (F) : RX continu -> donné à MM
 *   Node : appui 2 fois sur boutons
 *
 * v1  15/10/2017 (conc: Flash:12987  Ram:47B5)
 *   Concentrateur (F) : RX continu
 *   Node (G-I-J) : Uart sans semaphore, + ZR Reset, +luminosité
 *             Multi-nodes, RX configurable (1,4,6), Uart non activé,
 *             SCS non activé (conso > 250uA), capteur Temp Cpu : avec tache/Event : envoie toutes les 5min la temp
 *             message hexa, Voltage batterie, Flash, Watchdog non activé,
 *     Pb : non-compatibilité Uart/I2C
 *     ajouts futurs : concentrateur RX periodique




Compil pour le Node :
    - board.h : 23:adresse du node
    - Radio protocole : 25:Range(10<->1)  et  20:Adresse reseau(27-28)   28:mode_rx
    - Dnnode : 120-123 : définir mode de fonctionnement par defaut
               TimerEEPROMHandle : voir periode de maj EEPROM
    - ancien / nouveau hard : comm_message.c 500
    - comm_message.h :  51: GROUPES_NB_MAX=6


Pour le concentrateur :
   - Radio protocole : 23:Range(10<->1)  et  19:Adresse reseau(27-28)   19:Flash tampon
   - bootloader (ci-dessous)
   - comm_message.c : 131 UART_AJOUT_EMETTEUR -> commenter pour prod
   - port serie activé (comm_message.h)


Pour téléchager avec bootloader :
Avec bootloader :
    Format hex : dans properties/Build/ARM hex utility : cocher Enable ARM hex utility
        Dans output format options : Intel txt format
    Dans linker (cmd file) :
      #define FLASH_BASE 0x2000
      FLASH (RX) : origin = 0x1000, length = 0x1E000
      FLASH_VEC (RX) : origin = 0x1F000, length = 0x1000
      Sections :    .resetVecs : > FLASH_VEC
                    .ccfg      : > FLASH_VEC (HIGH)
    Dans TI_RTOS release.cfg :
        Ligne 136 : m3Hwi.resetVectorAddress = 0x0001F000;


 */
/* XDCtools Header files

Si chgt de version :
dans cc1350stk.h : ligne 130 : CC1350STK_PWMPIN0   CC1350STK_BUZZER
dans cc1350stk.c : ligne 158 : SPI Flash CSN  : pin_14
dans board.h        : node & concentrator
dans easylink.c : addrsize=3 à 2 endroits (ligne 628 et ligne 113)

Alignement :
typedef struct __attribute__ ((aligned (4))) rfc_command_s rfc_command_t;
*/
/*
 *  ======== rfEasyLinkRx.c ========
 */

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/knl/Task.h>
//#include <ti/sysbios/knl/Semaphore.h>
//#include <ti/sysbios/knl/Clock.h>


/* TI-RTOS Header files */
#include <ti/display/Display.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/ADC.h>

/* Board Header files */
#include "Board.h"
#include "CC1350STK.h"

#include <DmRadioTask.h>
#include "DmNodeTask.h"

#include "comm_message.h"
#include "utilitaires.h"

char unsigned val_reset, rrt;

/*
 * Délais-delai
 *     #define DELAY_MS(i)      (Task_sleep((i) * (1000 / Clock_tickPeriod)))
 *     DELAY_MS (100);           100 = 100ms (ne pas utiliser en interruption)
 *     CPUdelay(8000*50);    300.000 = 30ms  (ne fait pas de sleep)
 *     usleep(time);  // en micro-secondes, max 1.000.000
 *    Task_sleep(23); // en tick (de 30us)
 *
 *Clock_tickPeriod = 10
 *RF_getCurrentTime();  // 1 tick = 0,25 micro-second (si radio est allumée)
 *Clock_getTicks();     // 1 tick = 10 micro seconde
 *
 *Node : de Envoi balise à fin réception message = 9ms (4ms pour l'envoi balise, 4ms pour réception message)
 *
 *Consommation :
     micro utilisé a 100% : 6,6mA    clock20ms:300uA   clock100ms:20uA
 *   RX continu : 6mA
 *   Flash off, capteurs off, micro sleep = 4uA
 *   capteurs pas éteints : 260uA
 *   buzzer : freq 1000: 19mA (1000-9) 10mA (8) 2,2mA (5)  1,1mA (2)  actif max:32mA
 *   LED : 1,7mA

 *
 *  ======== main ========
 */
int main(void)
{
    val_reset =    ( HWREG(AON_SYSCTL_BASE + 4) >>1) & 7 ;  // AON_SYSCTL_O_RESETCTL=4  field RESET_SRC
    // 0:POR 1= Resetpin  2=Brownout VDDS  3=Brownout VDD 4=Brownout VDDR 5=Clockloss
    // 6=Softwarereset SYSRESET  7= Softwarereset/Watchdog PRCMwarmresetrequest
    //val_reset = SysCtrlSystemReset();

    raz_erreur ();

    // Call driver init functions.
    CC1350STK_initGeneral(); //  Power-Initialisation des IO
    //Board_initGeneral();

    // Initialise the UART and SPI, buttons

    #ifdef FLASH_EXTERNE
        SPI_init();   // initialisation du driver SPI
    #endif

    #ifdef Uart2
        Uart_actif=0;
        UART_init();                // Initialisation driver UART
    #endif

    Periph_init();              // Config led, boutons
    ADC_init();                 // Initialisation lecture analogique

    ES_init();  // init utilitaires (buzzer, batterie..)

    tache_comm_init();  // Création tache et sémaphore : Réception UART

    //tache_test_init();

    // Initialize  tasks
    NodeTask_init();   // init task (lecture index eeprom,..)

    #ifdef CAPTEUR_MPU
         SensorTask_init();   // ne pas commenter
    #endif

   RadioTask_init();


    //ACTIV_PWM ( 0,6, 200, 4000, 2);  // 1, 5, 1000, 9 = bip 0,5s 1000Hz, volume max
    //ACTIV_SORTIE(LED_cpu, 2, 3000);


    /* Start BIOS TI-RTOS */
    BIOS_start();

    return (0);
}

