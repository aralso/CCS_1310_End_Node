/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 *
 * Node-Tx : avec infos dem ack / dernier mess / rx apres
 * Node-RX 100ms pour attendre Ack (si demandé) avec info : mess rx en attente
 * Node-TX pour 2° message (si le cas)
 * Node RX attente ack - infos message rx en attente
 * Node RX : attente rx (si le cas ou message en attente) infos:ack / dernier
 * Node TX ack si demandé
 * Node RX : attente 2° messageavec infos:ack/dernier
 * Node TX ack si demandé
 *
 *
 * Envoi message avec Ack_tx=0, dernier_tx=1, rx_apres=0
 *     TX 50ms (10 car)
 *     si rx_total_apres=0 alors terminé
 *
 * Envoi message avec Ack_tx=1, dernier_tx=1, rx_apres=0
 *     TX 50ms (10 car)
 *     RX : Attente ack  100ms
        si ok :
        is timeout :
 *     si rx_total_apres=0 alors terminé
 *
 * Envoi un Ack, que si un packet valide a été reçu (RADIO_EVENT_VALID_PACKET_RECEIVED), hors balise_tx
 * Mode 6 :
 *  Tache Node : envoie_message(XXX)   -> envoi_buffer_RF_9(++,phase_radio->1) -> EVENT_MESSAGE_RF_A_ENVOYER)
 *  Tache Node : EVENT_MESSAGE_RF_A_ENVOYER -> envoi_pile_rf -> transmission_RF(--) -> envoi_mess_RF_10(phase->2) -> RADIO_EVENT_SEND_SIMPLE_MESSAGE (attend resultat)-> 14-17-> RADIO_EVENT_ALLUMAGE_PERIOD
 *  Tache Radio: RADIO_EVENT_SEND_SIMPLE_MESSAGE -> sendDmPacket_13 --(attend)--  rxDoneCallback_20 Ack -> RADIO_EVENT_DATA_ACK_RECEIVED -> semaphore result
 *  Tache Radio: RADIO_EVENT_ALLUMAGE_PERIOD-32 (phase->2, Clock_extinction)
 *  Tache Radio: RADIO_EVENT_EXTINCTION_RX-38 (phase->0/1)
 *  Activ_radio_rx_31 : définit la clock allumage périodique et la balise TX

 * Envoi_balise_TX-41/42
 *
 *  si forcage=0 => n'envoie que les messages non différés
 *
 *
 * Envoi des messages RF en attente dans la pile d'envoi (mess_RF_dans_bufferTX) :
 *     - dès l'élaboration du message a envoyer (envoi_buffer_RF), si pile pleine et forcage
 *     - dès l'élaboration du message (envoi_buffer_RF), si mode_dest=1 et phase_radio != 2
 *     - à l'extinction RX (pour le mode_emetteur 4,5,6) si phase_radio_apres=1
 *     - si reception balise "demande_message"

 Code pour Debug_Time : etat_cpu
 Reception :
10 : reception message normal (fin)
12 : send Ack (debut) et 13(fin)
15 :keepalive recu
16 :balise demande reçue
Envoi :
         22 : debut cycle_TX
         23 :trouve 1er mess a envoyer
         24 :message trouvé
         26 :Event_tx_suite
28 :début envoi message  (21 :fin)
29 :resend
2A : Ack receveid (fin)
2B: fin envoi message (avec result)

       0 : Interr radio
30 :phase radio suivante
FF : fin phase

 */

#define NB_REPET_TX_RF_non_reçu  5

/***** Includes *****/
#include <DmRadioTask.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>


#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

#include "easylink/EasyLink_config.h"
#include "easylink/EasyLink.h"

#include "utilitaires.h"
#include "comm_message.h"
#include "RadioProtocol.h"

#include <ti/devices/CC13x0/driverlib/AON_BATMON.h>
#include <ti/devices/cc13x0/inc/hw_aux_wuc.h>


/*  ***  timing *****
Envoi sans reception (failed) : 490ms car 3 envois avec 3*160ms de timeout
Envoi reussi : 5ms à 8ms  (25 ms pour 125 car)

Vitesse de transmission a 50kbps  ex : QLHLSc=6 car  Rajouter +16 : 2(sync)+2(sync)+3(add)+6(rssi..)+3(payload)
4ms (6 car) 5,7ms(13car) 7,11ms (18car) 7,3ms(23car) 8,9ms(33car) 10,5ms(43car) 13,5(60c)
16,8(80c) 20ms(100c) 21,6(110c) 23,2ms(120c) = 5600car/s = 50kbps

vitesse 5kpbs : 140ms (80 car)
*/

// Balise Broadcast
#ifdef DEBUG_M
    //#define BROAD_LED           // Debug - Affiche la led pendant que le module ecoute
    //#define BROAD_ENVOI_MESURE  // Debug - Envoi mesure du temps restant, a chaque reception de balise
    //#define BROAD_DUREE_FIXE    // Debug - Départ avec 10 fois la duree, baisse progressivement
#endif

/***** Defines *****/

#define TEMPO_2_MESSAGES 10000*RADIO_PROTOCOLE   // 100000=1s pour 5kbps SL-LR  10000=100ms pour 50kbps Custom


#define RADIO_TASK_STACK_SIZE 860  // max constaté : 648-768
#define RADIO_TASK_PRIORITY   4   // 3 pour concentrateur ?

#define RADIO_EVENT_ALL                  0xFFFFFFFF
#define RADIO_EVENT_DEBUT_CYCLE_TX         (uint32_t)(1 << 0)  // 1
#define RADIO_EVENT_DEBUT_TX_KEEPALIVE     (uint32_t)(1 << 1)
#define RADIO_EVENT_DEBUT_TX_Balise_Demande (uint32_t)(1 << 2)
#define RADIO_EVENT_DEBUT_ECOUTE_RX        (uint32_t)(1 << 3)
#define RADIO_EVENT_DEBUT_TX_BALISE_SYNCHRO (uint32_t)(1 << 4) // 10
#define RADIO_EVENT_TX_SUITE               (uint32_t)(1 << 5)
#define RADIO_EVENT_TX_KEEPALIVE           (uint32_t)(1 << 6)
#define RADIO_EVENT_TX_BALISE_DEMANDE      (uint32_t)(1 << 7)
#define RADIO_EVENT_ECOUTE_RX              (uint32_t)(1 << 8)  // 100
#define RADIO_EVENT_TX_BALISE_SYNCHRO      (uint32_t)(1 << 9)
#define RADIO_EVENT_VALID_PACKET_RECEIVED  (uint32_t)(1 << 10)
#define RADIO_EVENT_INVALID_PACKET_RECEIVED (uint32_t)(1 << 11)
#define RADIO_EVENT_DATA_ACK_RECEIVED      (uint32_t)(1 << 12)   //1000
#define RADIO_EVENT_ACK_TIMEOUT            (uint32_t)(1 << 13)
//#define RADIO_EVENT_SEND_FAIL              (uint32_t)(1 << 5)
#define RADIO_EVENT_SEND_SIMPLE_MESSAGE    (uint32_t)(1 << 14)
#define RADIO_EVENT_ALLUMAGE_PERIOD        (uint32_t)(1 << 15)   // 8000
#define RADIO_EVENT_EXTINCTION_RX          (uint32_t)(1 << 16)   // 1.0000
#define RADIO_EVENT_RX_ABORTED             (uint32_t)(1 << 17)   // 2.000
#define RADIO_EVENT_RX_BALISE_DEMANDE      (uint32_t)(1 << 18)
#define RADIO_EVENT_KEEPALIVE_RECEIVED     (uint32_t)(1 << 19)
#define RADIO_EVENT_CYCLE_BLOQUE           (uint32_t)(1 << 20)   // 10.0000
#define RADIO_EVENT_TIMEOUT_RX             (uint32_t)(1 << 21)
#define RADIO_EVENT_RX_BALISE_SYNCHRO      (uint32_t)(1 << 22)   // 40.000
#define RADIO_EVENT_FIN_CYCLE_RADIO        (uint32_t)(1 << 23)   // 80.000
#define RADIO_EVENT_DEMANDE_BALISE_SYNCHRO (uint32_t)(1 << 24)

#define RADIO_MAX_RETRIES 1   // 0 : pas de retry

#define RADIO_ACK_TIMEOUT_COURT_TIME_MS (15*RADIO_PROTOCOLE)  //(160) Ack=25ms à 5kbps
#define RADIO_ACK_TIMEOUT_LONG_TIME_MS (50*RADIO_PROTOCOLE)  //(160) Ack=25ms à 5kbps
#define TEMPS_ENTRE_RESEND  500         // 500 ms


// Codes erreur
    // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
    // Erreurs 20 a 7F : 4 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
    // Erreurs 80 a FF : tout le temps (Appli > 0xD0)


#define EasyLink_init_failed                0x51
#define EasyLink_receiveAsync_failed        0x52
#define EasyLink_transmit_failed            0x53
#define erreur_pile_envoi_RF_full           0x54
#define EasyLink_transmit_balise_failed     0x55
#define Nb_Node_max_atteint                 0x56
#define EasyLink_error_RX                   0x57
#define erreur_pile_RF_TX                   0x58
//#define Timout_Semaphore_Radio_acess        0x39
#define erreur_easylink_bloque              0x5A
#define erreur_longueur                     0x5B
#define erreur_mess_rf_encours_bloque       0x5C
#define erreur_mess                         0x5D

/***** Type declarations *****/


struct Mess_RF_a_env {
     uint8_t ret;    //0:pas de message dispo  1:message dispo
     uint8_t id_dest;
     uint32_t tail;
     uint8_t longueur;
     uint8_t long_complet;
     uint8_t statut;  // bit0:dernier  bit1-2:reenvoi(00:non, 01:2 fois, 10:5 fois)   bit3:différé   bit4:pas d'ack  bit5:RX apres  bit6:sup si pas envoyé
};

struct Radio_S {
      uint8_t statut;
      uint8_t nb_mess_envoyes;
      uint8_t RX_apres;
      uint8_t phase_suivante;  // bit0:TX balise dem, bit1:TX keepalive , bit2:ecoute RX, bit3:TX,  bit4:Tx_Synchro bit5:TX(apres balise demande)
      uint8_t cpt_packet; // cpt pour renvoyer l'ack avec le meme numéro
      uint8_t dernier_mess_recu; // le node distant a envoye son dernier message
      uint8_t destinat_rx_apres; // le node distant se met en rx apres l'envoi
};


extern struct RemoteNode knownNodes[NB_DESTINATAIRES_NODE];
extern    UART_Handle hUart2;


/***** Variable declarations *****/
static Task_Params RadioTaskParams;
Task_Struct RadioTask; /* not static so you can see in ROV */
static uint8_t RadioTaskStack[RADIO_TASK_STACK_SIZE];
Event_Struct radioOperationEvent;  /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;

struct Message_RF_Packet Mess_RF_Packet;

// définition des messages radio
struct RF_synchro_Payload* RxPacket_syn;

static Radio_PacketReceivedCallback packetReceivedCallback;
struct Message_RF_Packet latestRxPacket;
static EasyLink_TxPacket txPacket;
static uint8_t My_RadioAddress;
//static uint8_t My_Tot_RadioAddress[4];
static int8_t latestRssi;
uint32_t latestAbsT;
uint8_t verif_radio_run_cpt, verif_radio_bal_cpt;  // surveille la radio RX

int8_t recep[50];
int8_t paramA;

uint8_t message_temp_rf[TAILLE_MAX_MESS_RF];

// bit3:différé   bit4:pas d'ack  bit5:RX apres

uint8_t timeout_restart;
uint8_t timer_mess_Rf_en_cours;

uint8_t lastAddedSensorNode;

uint8_t rxrf_emetteur;

uint16_t comptage_radio_CB;
uint16_t  comptage_radio_CB_last;

uint8_t premier_mess_envoye;  // numero du message envoye, dans la meme phase TX

uint8_t destinataire_pass_RF;
uint8_t mode_rx;  // bit0-3:mode rx  bit4:sans ack   bit5:sans rx apres trans
uint8_t duree_fenetre_RX;  // (mode 4,5), en milliseconde
uint8_t periode_activ_RX;  // (mode 4,5), en seconde : periode d'activation en ecoute du module
uint8_t ajustement_periode_rx; // ajustement periode RX, en 30 microsecondes
uint8_t periode_balise_synchro; // mode 5 : periode en minutes : 3 minutes
uint8_t cpt_perte_balise_synchro;
uint8_t cpt_fenetres, cpt_balise;

uint32_t prec_tick_voltage;

uint8_t decrement_nb_repet_activ_rx;
uint8_t envoi_balise;  // 1 pour envoi de la balise lors de la première sollicitation

int8_t rssi_bruit;
uint8_t rssi_bruit_premier;

#if (DEBUG_TIME & 0b100)
uint8_t type_allumage_led;      // Led allumée bit0:send  bit1:rx
#endif

//Semaphore_Struct radioAccessSem;  /* not static so you can see in ROV */
//static Semaphore_Handle radioAccessSemHandle;
//Semaphore_Struct radioResultSem;  /* not static so you can see in ROV */
//static Semaphore_Handle radioResultSemHandle;
struct RadioOperation currentRadioOperation;
struct Mess_RF_a_env  mess_a_envoyer;


/* Clock for RX Radio allumage periodique - Fenetre d'ecoute */
Clock_Struct RadioRxPeriodTimerClock;     /* not static so you can see in ROV */
static Clock_Handle RadioRxPeriodTimerClockHandle;

/* Timer pour demander la balise de sunchro initial au bout de 7 secondes*/
Clock_Struct RadioTimerBalInitial;     /* not static so you can see in ROV */
static Clock_Handle RadioTimerBalInitialHandle;

/* Timout si la balise synchro n'est pas reçue 3 fois de suite */
Clock_Struct RadioBalTimeout;     /* not static so you can see in ROV */
static Clock_Handle RadioBalTimeoutHandle;


/* Timer pour extinction */
Clock_Struct RadioRxTimerClock;     /* not static so you can see in ROV */
static Clock_Handle RadioRxExtinctTimerHandle;

/* Timer pour cycle radio bloqué*/
Clock_Struct RadioCycleTimer;     /* not static so you can see in ROV */
static Clock_Handle RadioCycleTimerHandle;

/* Timer pour fin cycle radio*/
Clock_Struct RadioFinCycle;     /* not static so you can see in ROV */
static Clock_Handle RadioFinCycleHandle;



char unsigned index_test, num_test;

struct Radio_S Radio;
struct Synchro_S Synchro;

int16_t decal_conc;

uint8_t phase_radio_dest;       // noeud en communication active
uint8_t radio_dest_TX;          // noeud a transmettre
uint8_t radio_dest_tx_bal;      // noeud a trnsmettre, apres reception balise demande

// parametre enregistres pour activation radio
uint8_t activ_premiere_balise, activ_nb_activation;
uint16_t activ_duree_activation_ms, activ_delai_1ere_activ_ms;


/********************* Extern variables ******************/

extern uint8_t Mess_statut;  // bit0:dernier  bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)
extern char unsigned  code_erreur;
extern  char unsigned err_donnee1, err_donnee2;
extern uint8_t hors_portee_timing;   // conserve le temps de la clock hors_portee
extern Clock_Handle TimerFlashClockHandle;
extern uint16_t volt_mesure_dernier;
extern uint8_t  rxrf_numero_device;  // 0:pas de device  1:device 0 etc...


#ifdef FLASH_EXTERNE
    //uint8_t timeout_envoi;
    extern struct Pile_Fl_S Pile_F[NB_PILES_F];
#endif

/**************** Prototypes internes **********************/

static void RadioTaskFunction(UArg arg0, UArg arg1);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void notifyPacketReceived(struct Message_RF_Packet* latestRxPacket);
static void sendAck(uint8_t latestSourceAddress);

static void returnRadioTX(enum RFRadioOperationStatus status);
static void sendDmPacket(struct Message_RF_Packet RFPacket, uint8_t maxNumberOfRetries);
static void resendPacket();
static uint8_t envoi_balise_TX(uint8_t type);
void mess_RF_dans_bufferTX(void);
void application_mode_rx(void);
void hors_portee_calcul_periode();
uint8_t analyse_pile_tx(uint8_t id_dest, uint8_t tail, uint8_t* mess_stat, uint8_t* long_complet);
void Send_mess_radio (void);  // paramètre : forcage:0:immediat uniquement  1:tous
void Phase_radio_suivante(uint8_t mess_val, uint8_t ret);
uint8_t phase_reception_courte(uint8_t);
void phase_fin(void);
uint8_t cherche_message (uint8_t id_dest, uint8_t forcage);
void envoi_balise_synchro(uint8_t type);

void event_envoi_voltage(void);
void reactiv_timeout_balise_synchro(void);


/***** Function prototypes *****/

void RadioRxPeriodTimerCallback (UArg arg0);     // fin clock de Radio RX pour  allumage périodique
void RadioTimerBalInitialCallback (UArg arg0);        // fin au bout de 7s pour demande de balise synchro initiale
void RadioBalTimeoutCallback (UArg arg0);             // fin si 3 balises synchro de suite ne sont pas reçues
//void RadioRxTimerRapidAllumCallback (UArg arg0);    // fin du timer pour allumage rapide sur eve
void RadioRxTimerExtinctionCallback (UArg arg0);    // fin du timer de Radio RX pour extinction
void RadioCycleTimerCallback (UArg arg0);             // fin du timout cycle radio bloqué
void RadioFinCycleCallback (UArg arg0);               // fin du timeout de fin de cycle radio
//void RadioCycleSynchroCallback (UArg arg0);         // fin du timout cycle synchro
uint8_t enreg_message_ram(char unsigned * messageRF, uint8_t numero_dest);

Void errHook(Error_Block *eb);

uint8_t lecture_volt_batterie(uint16_t * voltage_bat, uint16_t * voltage_sol);


/***** Function definitions *****/

// Timeout si le cycle radio ne se termine jamais
void timeout_cycle_restart(uint8_t num)
{
    timeout_restart=num;
    Clock_stop(RadioCycleTimerHandle);
    Clock_setTimeout(RadioCycleTimerHandle, 1000 * (1000 / Clock_tickPeriod));     // Timeout de 1 seconde
    Clock_start(RadioCycleTimerHandle);
}

void RadioTask_init(void) {
    char unsigned i;

    /* Create semaphore used for exclusive radio access Transmit */
    /*Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&radioAccessSem, 1, &semParam);
    radioAccessSemHandle = Semaphore_handle(&radioAccessSem);*/


    /* Create semaphore used for callers to wait for result - Transmit*/
    //Semaphore_construct(&radioResultSem, 0, &semParam);
    //radioResultSemHandle = Semaphore_handle(&radioResultSem);

    //Semaphore_Handle_label(radioAccessSemHandle, (xdc_runtime_Types_Label *) "rr");

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the radio protocol task */
    Task_Params_init(&RadioTaskParams);
    RadioTaskParams.stackSize = RADIO_TASK_STACK_SIZE;
    RadioTaskParams.priority = RADIO_TASK_PRIORITY;
    RadioTaskParams.stack = &RadioTaskStack;
    RadioTaskParams.instance->name = "Radio";
    Task_construct(&RadioTask, RadioTaskFunction, &RadioTaskParams, NULL);

    mode_rx = MODE_RX;
    phase_radio_dest=0;
    radio_dest_TX=0;
    radio_dest_tx_bal=0;


    periode_activ_RX = PERIODE_ACTIV_RX;  // (Mode=4 et 5) en seconde (30 secondes)
    Synchro.periode_fenetre = PERIODE_ACTIV_RX * 1000 * 100;  // Mode=5
    ajustement_periode_rx = 0;
    duree_fenetre_RX  = DUREE_FENETRE_RX;  // (Mode=5) en milliseconde
    Synchro.duree_fenetre = DUREE_FENETRE_RX * 100;  // Mode=5   duree en 10us multiplié par 50 initialement
    periode_balise_synchro = 3;  // Mode 5 : en minutes
    cpt_perte_balise_synchro = 3;  // 3:ok    2:perte   1:perte    0:arret recherche


    // initialisation des parametres par defaut pour l'allumage temporaire radio
    activ_premiere_balise = 0;
    activ_nb_activation =1;
    activ_delai_1ere_activ_ms = 0;

    Clock_Params clkParams;
    Clock_Params_init (&clkParams);

    /* setup  clock pour allumage periodique  mod 4 ou 5 */
    clkParams.period = PERIODE_ACTIV_RX *1000 * (1000 / Clock_tickPeriod);
    clkParams.startFlag = FALSE;
    Clock_construct(&RadioRxPeriodTimerClock, RadioRxPeriodTimerCallback, 1, &clkParams);
    RadioRxPeriodTimerClockHandle = Clock_handle(&RadioRxPeriodTimerClock);

    /* setup timeout pour balise synchro initiale */
    clkParams.period = 0;  // 0:non periodique
    clkParams.startFlag = FALSE;
    Clock_construct(&RadioTimerBalInitial, RadioTimerBalInitialCallback, 4*1000*100, &clkParams);  // 4 secondes
    RadioTimerBalInitialHandle = Clock_handle(&RadioTimerBalInitial);

    /* setup timeout pour perte de balise au bout de 3 non-reception */
    clkParams.period = 3 * 3 * 61 *1000 * (1000 / Clock_tickPeriod);  // 9 minutes
    clkParams.startFlag = FALSE;
    Clock_construct(&RadioBalTimeout, RadioBalTimeoutCallback, clkParams.period, &clkParams);  //
    RadioBalTimeoutHandle = Clock_handle(&RadioBalTimeout);


    // setup timer extinction
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&RadioRxTimerClock, RadioRxTimerExtinctionCallback, 1, &clkParams);
    RadioRxExtinctTimerHandle = Clock_handle(&RadioRxTimerClock);

    /* setup timeout cycle radio bloqué */
    clkParams.period = 0;  // 0:non periodique
    clkParams.startFlag = FALSE;
    Clock_construct(&RadioCycleTimer, RadioCycleTimerCallback, 1, &clkParams);
    RadioCycleTimerHandle = Clock_handle(&RadioCycleTimer);

    /* setup clock fin cycle radio */
    clkParams.period = 0;  // 0:non periodique
    clkParams.startFlag = FALSE;
    Clock_construct(&RadioFinCycle, RadioFinCycleCallback, 1, &clkParams);
    RadioFinCycleHandle = Clock_handle(&RadioFinCycle);



    index_test = 0;
    num_test=0;

    rxrf_emetteur=0;
    #if (DEBUG_TIME & 0b100)
    type_allumage_led=0;
    #endif

    lastAddedSensorNode = 0;

    for (i=0; i<NB_DESTINATAIRES_NODE; i++)  // inutile car lastAddedSensorNode = 0
    {
        knownNodes[i].adresse=0;
        knownNodes[i].tail=0;
        knownNodes[i].head=0;
    }
    comptage_radio_CB = 0;
    comptage_radio_CB_last = 0;

    Radio.statut=0;
    Radio.RX_apres=0;
    Radio.phase_suivante=0;
    Radio.cpt_packet = 0;
    Radio.dernier_mess_recu = 0;
    Radio.destinat_rx_apres = 0;  // RAZ RX apres chez le node distant

    verif_radio_run_cpt=0;
    verif_radio_bal_cpt = 0;
}


// vient de envoi_message()
// Ajoute dans la pile d'envoi RF du device destinataire (knownNodes) le message rentré en paramètre
// Tente d'envoyer le message par RF (si mode destinataire = 1)

// retourne le numero de device associé à l'adresse entrée en paramétre (0:inconnu, 1:premier device,...)
uint8_t numero_device_rf(uint8_t adresse_dest)
{
    uint8_t i,j;
    // identification de la pile correspondant au destinataire
    j=0;
    for (i=0; i<lastAddedSensorNode; i++)
    {
        if (knownNodes[i].adresse == adresse_dest)
        {
            j = i+1;
            break;
        }
    }
    return j;
}

// return 0:ok 1:erreur
// enregsitre le message a envoyer dans le buffert RF_TX
uint8_t envoi_buffer_RF(char unsigned * messageRF, unsigned char longueur_mess, uint8_t destinataire_RF)
{
// Si pile_flash -> stockage en flash  puis envoie .  plus de clock(flash) puis envoi_RF_flash
// Si pile_RAM -> stockage en RAM puis envoie

   uint8_t retc;
   retc=1;

   #if (NB_DESTINATAIRES_NODE > 0)
   char unsigned  numero_dest, j;
   bool reta;

   // identification de la pile correspondant au destinataire (1 de plus)
   numero_dest = numero_device_rf(destinataire_RF);

   j=1;
   if (!numero_dest)  // Ajoute le noeud s'il n'existe pas
   {
       reta = addNewNode(destinataire_RF);
       if (!reta)
       {
           numero_dest = lastAddedSensorNode;
       }
       else j=0;  // Plus de place pour un nouveau node
   }
   numero_dest--;

   if ((j) && (longueur_mess > 1) && (longueur_mess < (EASYLINK_MAX_DATA_LENGTH-HEADER_LENGTH-2)))    // et Vérifie la longueur
       // 5 < 127 - 5
   {
       // décalage de 2 pour rajouter la longueur totale et le statut
       char unsigned  c;
       for (c=longueur_mess+1; c; c--)  // PRZZf ou PR1ZZ  lg=4
       {
           messageRF[c+1] = messageRF[c-1];
       }
       messageRF[0] = longueur_mess+3;  // longueur 4=>7 qui donnera 8 au moment de la parité si Flash
       messageRF[1] = Mess_statut & 0xFE;  // non prise en compte du dernier bit (rajoute ensuite)

       #ifdef PILE_RF_FLASH  // Pile d'envoi en mémoire Flash externe

           // ajoute les données en flash externe (retc=0=> ok)

           retc = enreg_message_flash (messageRF, numero_dest+OFFSET_FLASH);  // 2,9ms
           if (!retc)    // if enreg flash ok
               if (!(Mess_statut & 0x08))  // if pas différé
                #ifdef ENVOI_FLASH_RF_CLOCK
                      demarr_lect_flash();
                #else
                      Envoie_RF(numero_dest, messageRF);
                #endif

        #else
           retc = enreg_message_ram (messageRF,numero_dest);
           if ((!retc) && (!(Mess_statut & 0x08)))  // ok et pas différé
               Envoie_RF(numero_dest, messageRF);
        #endif
    }
   #endif
   return retc;
}


uint16_t calcul_espace_dispo_node(uint8_t numero_dest)
{
    uint16_t free;
    // calcul places libres dans buffer
    if (knownNodes[numero_dest].head < knownNodes[numero_dest].tail)
       free =  knownNodes[numero_dest].tail - knownNodes[numero_dest].head ;
    else
       free = (TAILLE_PILE_ENVOI_RADIO - knownNodes[numero_dest].head) + knownNodes[numero_dest].tail;
    return free;
}

// return : 0:OK
uint8_t enreg_message_ram(char unsigned * messageRF, uint8_t numero_dest)
{
    uint8_t free, longueur_mess, c, retc, full;
    retc=1;

// mise en pile d'envoi pour empilement des messages (destinataire en sleep)
    free = calcul_espace_dispo_node(numero_dest);

    longueur_mess = messageRF[0];

    full=0;
    if (messageRF[4]=='E')  // message erreur
    {
        if ( (longueur_mess+1) > free )
            full=1;
    }
    else   // message normal  - laisse 13 places pour un message d'erreur eventuel futur
    {
        if ( ((longueur_mess+1) > free ) || ((longueur_mess+13) > free ))
            full=1;
    }


    if ( full )   // Tx_buffer plein => n'envoie pas
    {
        code_erreur = erreur_pile_envoi_RF_full;   // Tx_buffer full
        err_donnee1 = messageRF[4];
        err_donnee2 = longueur_mess+'0';
        #ifdef DEBUG_reseau
            message_test[0] = numero_dest+'0';
            message_test[1] = free+'0';
            message_test[2] = knownNodes[numero_dest].head;
            message_test[3] = knownNodes[numero_dest].tail;
            UART_write(hUart2, (char *)message_test, 4);
            //strcpy((char *)message, "1 recept balise synchro");
            //envoi_message ();
        #endif

        //knownNodes[numero_dest].tail = knownNodes[numero_dest].head; // vide buffer
    }
    else
    {
        uint8_t m_head;
        m_head = knownNodes[numero_dest].head;

       for (c=0; c<longueur_mess; c++)  // PRZZf ou PR1ZZ  lg=7
       {
           knownNodes[numero_dest].buffer_tx[m_head++] = messageRF[c]; // copy over this byte of data
          if( m_head == TAILLE_PILE_ENVOI_RADIO ) m_head = 0;  // reset pointer  if wrapping around
       }
       knownNodes[numero_dest].head = m_head;
       retc = 0;
    }
   return retc;
}


void Envoie_RF(uint8_t numero_dest, char unsigned * messageRF)
{
   if (!(messageRF[1] & 8))  // non differe
       knownNodes[numero_dest].forcage=1;

   // TRANSMISSION immédiate
   // Vers destinataire toujours actif RX (vers concentrateur) ou inconnu ET si message immédiat
       radio_dest_TX = numero_dest;
       if (Radio.statut<2)
       {
           mess_RF_dans_bufferTX();     // envoi message normal, radio en veille
       }
       else   // radio active
       {
           Radio.phase_suivante |= 8;   // bit0:TX balise dem, bit1:TX keepalive , bit2:ecoute RX, bit3:TX  bit4:synchro bit5:TX(apres balise demande)
       }
       //mess_RF_dans_bufferTX();
}

void suppression_message(void)
{
    uint8_t num_ad;

    num_ad = mess_a_envoyer.id_dest;
    knownNodes[num_ad].hors_portee_etat=0;
    hors_portee_timing=0;   // remet à zéro la clock
    knownNodes[num_ad].nb_messages_effaces++;
    #ifdef PILE_RF_FLASH
        Clock_stop(TimerFlashClockHandle);
        supp_message_flash (mess_a_envoyer.long_complet, num_ad+OFFSET_FLASH);
        enreg_index_flash();
    #else
        uint8_t index;
        index = mess_a_envoyer.tail + mess_a_envoyer.long_complet;
        if( index >= TAILLE_PILE_ENVOI_RADIO )   index = index - TAILLE_PILE_ENVOI_RADIO;   /* if wrapping around  reset pointer */
        knownNodes[num_ad].tail = index;
    #endif
}

// renseigne la structure mess_a_envoyer, et renvoi 1 si ok.
// param : dest, forcage=0(immediat), 1:tous    0x1x:tous destinataire
// retc : 0 : pas de message à envoyer, 1 : 1 message
uint8_t trouve_mess_RF_envoyer(uint8_t dest, uint8_t forcage)
{
    uint8_t retc;

    retc=0;
    if (forcage & 0x10)
    {
        // recherche d'abord sur le destinataire designé
        retc = cherche_message (dest, forcage & 0x0F);

        // recherche ensuite sur les autres destinataires
        if (!retc)
        {
            dest=0;
            while ((dest < lastAddedSensorNode) && (!retc))
            {
                retc = cherche_message (dest, forcage & 0xF);
                dest++;
            }
        }
    }
    else
    {
        // recherche uniquement sur le destinataire designé
        retc = cherche_message (dest, forcage);
    }
    return retc;
}

// return : 0:pas de message  1:message
uint8_t cherche_message (uint8_t id_dest, uint8_t forcage)
{
    // Envoie le message si forcage ou si node pas bloqué
    //if ( (!(knownNodes[id_dest].hors_portee_etat)) || (forcage))
    //{
    // analyse le premier message de la pile de message en attente d'envoi pour ce destinataire

    mess_a_envoyer.ret=0;

    #ifdef PILE_RF_FLASH  // Pile RF en Flash externe

        uint8_t retc, mes_der, valide, section;
        uint32_t index;

        mes_der=1;
        valide=0;
        section = id_dest + OFFSET_FLASH;

        index = Pile_F[section].tail;
        if (index != Pile_F[section].head)
        {
            retc = lecture_message_flash(message_temp_rf, section, 0, TAILLE_MAX_MESS_RF); // 800us
            if (retc)
            {
                valide=1;
                mess_a_envoyer.ret = 1;
                mess_a_envoyer.id_dest = id_dest;
                mess_a_envoyer.long_complet = message_temp_rf[0] +1;
                mess_a_envoyer.long_complet += mess_a_envoyer.long_complet & 1;
                mess_a_envoyer.tail = index;
                if (message_temp_rf[2] & 0x80)  // message hexa    xxPR1ZZ lg=5
                {
                    mess_a_envoyer.longueur = message_temp_rf[4] + 4;
                }
                else   // message texte    xxPRZZf   lg=5
                {
                    uint8_t c, longueur_tx;

                    c = 0;
                    for (longueur_tx=2; ((c != car_fin_trame) && (longueur_tx < (TAILLE_PILE_ENVOI_RADIO - 1))); longueur_tx++)
                    {
                        c = message_temp_rf[longueur_tx];
                    }
                    mess_a_envoyer.longueur = longueur_tx - 2;
                    if (longueur_tx == TAILLE_PILE_ENVOI_RADIO-1)
                        valide=0;
                }
                if (retc==1)  mes_der = 0; // 2e message (valide ou non valide) suit le premier
                mess_a_envoyer.statut = message_temp_rf[1] | mes_der;

                // test de validité
                uint8_t num;
                num = numero_device_rf( passerelle_routage(message_temp_rf[2] & 0x7F));
                if (num != (id_dest+1))
                    valide=0;
                num = mess_a_envoyer.longueur + 3;
                num += num & 1;
                if (num != mess_a_envoyer.long_complet)  // verif que le longueur calculee colle
                    valide = 0;
                if ((retc) && (!valide))
                {
                    Pile_F[section].head -= Pile_F[section].head&1;  // pair
                    Pile_F[section].tail = Pile_F[section].head;  // raz pile flash
                    sauve_eeprom_index_flash();
                    mess_a_envoyer.ret = 0;
                }
            }
        }

    #else    // Pile RF en RAM

        uint8_t longu, long_complet;
        uint16_t index;

        index = knownNodes[id_dest].tail;
        if (index != knownNodes[id_dest].head)
        {
            uint8_t mes_stat, mes_der;
            longu = analyse_pile_tx(id_dest, index, &mes_stat, &long_complet);
            if ((longu) && ((forcage) || (knownNodes[id_dest].forcage) || (!(mes_stat&0xF))))  // 1er message valide
            {
                mess_a_envoyer.ret = 1;
                mess_a_envoyer.id_dest = id_dest;
                mess_a_envoyer.long_complet = long_complet;
                mess_a_envoyer.tail = index;
                mess_a_envoyer.longueur = longu ;
                index = index + long_complet;
                if( index >= TAILLE_PILE_ENVOI_RADIO )   index = index - TAILLE_PILE_ENVOI_RADIO;   /* if wrapping around  reset pointer */
                longu = analyse_pile_tx(id_dest, index, &mes_der, &long_complet);
                mes_der = 1;
                if (longu)  mes_der = 0; // 2e message valide suit le premier
                mess_a_envoyer.statut = mes_stat | mes_der;
            }
        }
    #endif
return mess_a_envoyer.ret;
}


// Analyse la pile d'envoi à partir de tail/index et donne la longueur du prochain message valide
uint8_t analyse_pile_tx(uint8_t id_dest, uint8_t tail, uint8_t* mes_stat, uint8_t* long_comp)
{
    uint8_t tx_rejoint, longueur_tx, type, mess_err, c;
    uint16_t index;

    mess_err=0;
    index = tail;
    if (index == knownNodes[id_dest].head)  // pas de message en pile
    {
        longueur_tx=0;
    }
    else
    {
        *long_comp = knownNodes[id_dest].buffer_tx[index++];
        if( index == TAILLE_PILE_ENVOI_RADIO ) index = 0;  /* reset pointer  if wrapping around */
        *mes_stat = knownNodes[id_dest].buffer_tx[index++];
        if( index == TAILLE_PILE_ENVOI_RADIO ) index = 0;  /* reset pointer  if wrapping around */
        type = knownNodes[id_dest].buffer_tx[index] & 0x80;
        tx_rejoint=0;

        if (type)  // message hexa    PR1ZZ lg=5
        {
            index++;
            index++;
            if( index >= TAILLE_PILE_ENVOI_RADIO )   index = index - TAILLE_PILE_ENVOI_RADIO;   /* if wrapping around  reset pointer */
            longueur_tx = (knownNodes[id_dest].buffer_tx[index] & 0x7F) + 4;

            index = tail;
            for (c=0; c<(longueur_tx); c++)  // PRZZ ou PR1ZZ  lg=5
            {
               index++;
               if( index == TAILLE_PILE_ENVOI_RADIO ) index = 0;  /* reset pointer  if wrapping around */
               if (index == knownNodes[id_dest].head)
               {
                   tx_rejoint=1;
                   if (c != (longueur_tx-1)) mess_err=3;
               }
            }
        }
        else  // message texte
        {
            c = 0;
            for (longueur_tx=0; ((c != car_fin_trame) && (longueur_tx < (TAILLE_PILE_ENVOI_RADIO - 1))); longueur_tx++)  // PRZZ   lg=5
            {
                c = knownNodes[id_dest].buffer_tx[index];
                index++;
                if( index == TAILLE_PILE_ENVOI_RADIO ) index = 0;  /* reset pointer  if wrapping around */
                if (index == knownNodes[id_dest].head) tx_rejoint=1;
            }

            //if ((mess_a_envoyer.longueur>1) && ( mess_a_envoyer.longueur < TAILLE_MAX_MESS_RF) && (mess_a_envoyer.longueur < EASYLINK_MAX_DATA_LENGTH-HEADER_LENGTH))

            if (longueur_tx == (TAILLE_PILE_ENVOI_RADIO - 1))          mess_err=2;
            if ((tx_rejoint) && (index != knownNodes[id_dest].head))   mess_err=4;
            if (longueur_tx<2)                                         mess_err=5;
            if (longueur_tx>=TAILLE_MAX_MESS_RF-2)                     mess_err=6;
            if (longueur_tx>=EASYLINK_MAX_DATA_LENGTH-HEADER_LENGTH-2) mess_err=7;
        }

        // Vérifie la longueur  -  inutile car déja dans envoi_buffer_RF
        //if ((longueur_tx < 3) || (longueur_tx > (TAILLE_MAX_MESS_RF-3)))  mess_err=1; // 128-3

        if (mess_err)  // message non valide
        {
            if (tail == knownNodes[id_dest].tail)  // si ce message est le premier de la pile => suppression
            {
                // Suppression de toute la pile d'envoi
                knownNodes[id_dest].tail = knownNodes[id_dest].head;
            }
            longueur_tx=0;
            code_erreur = erreur_pile_RF_TX;
            err_donnee1 = mess_err+'0';
            knownNodes[id_dest].nb_messages_effaces++;  // normalement pas possible
        }
    }
    return longueur_tx;
}






void RadioTask_registerPacketReceivedCallback(Radio_PacketReceivedCallback callback) {
    packetReceivedCallback = callback;        // => packetReceivedCallback
}

uint8_t RadioTask_getMyAddr(void) {
    return My_RadioAddress;
}


Void errorHook(Error_Block *eb)
{
    System_printf(Error_getMsg(eb));
    //System_printf(Error_getData(eb)->arg[0], Error_getData(eb)->arg[1]);
    System_printf("toto");
}

static void RadioTaskFunction(UArg arg0, UArg arg1)
{
    uint8_t retc;

    DELAY_MS(100);  // attente de 100 ms

    /*    Error_Block eb;

    Error_init(&eb);
    DELAY_MS(30);  // attente de 1 s

    System_printf("Running timerFxn\n");
    Error_raise(&eb, Error_E_generic, "Value is not a multiple of 2", 0);
    if (Error_check(&eb)) {
        System_printf(Error_getMsg(&eb), Error_getData(&eb)->arg[0]);
        System_printf(Error_getData(&eb)->arg[1]);
        System_printf("ErrorFxn\n");
    }*/
//    Memory_alloc(NULL, 0xffffff, 0, NULL);

    /* Set multiclient mode for Prop Sub1G  */
    //EasyLink_setCtrl(EasyLink_Ctrl_MultiClient_Mode, 1);

    /* Initia   lize EasyLink */
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);

    easyLink_params.ui32ModType = (EasyLink_PhyType)  (RADIO_EASYLINK_MODULATION);
     // (EasyLink_PHY_50KBPS2GFSK);
     //  EasyLink_PHY_50KBPS2GFSK

    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        //System_abort("EasyLink_init failed");
        code_erreur = EasyLink_init_failed;
    }

    //EasyLink_setCtrl(EasyLink_Ctrl_AddSize,4);  // 4 byte d'adresses

    //EasyLink_setRfPwr(0);  // Limitation de la puissance à 0=-10dbm
    //change_power_radio(9);

    /* If you wich to use a frequency other than the default use the below API
     * EasyLink_setFrequency(868000000);
     */

    /* Set My_adress source address */;
    My_RadioAddress = RADIO_MY_ADDRESS;
    //My_Tot_RadioAddress[0] = RADIO_MY_ADDRESS;
    //My_Tot_RadioAddress[1] = AddReseau1; //0x28
    //My_Tot_RadioAddress[2] = AddReseau2; //0x45
    //My_Tot_RadioAddress[3] = AddReseau3; //0x11
    //EasyLink_enableRxAddrFilter(My_Tot_RadioAddress, 4, 1);  inutile car déja par defaut

    currentRadioOperation.easyLinkTxPacket.dstAddr[1] = AddReseau1; //0x28;
    currentRadioOperation.easyLinkTxPacket.dstAddr[2] = AddReseau2; //0x45;
    currentRadioOperation.easyLinkTxPacket.dstAddr[3] = AddReseau3; //0x11;

    // pour envoi Ack et balise
    txPacket.dstAddr[1] = AddReseau1; //0x28 ou 0x27(test)
    txPacket.dstAddr[2] = AddReseau2; //0x45;
    txPacket.dstAddr[3] = AddReseau3; //0x11;

    /* Prépare receive continu */
    EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(0));


    application_mode_rx();  //  initialisation mode_rx et demande balise synchro
    Synchro.statut = 0;
     //ACTIV_SORTIE(LED_cpu,8,10);
    timer_mess_Rf_en_cours = 0;

    #ifdef BROAD_BALISE_SYNCHRO
        paramA=0;
        Synchro.ecart=0;
    #endif


    while (1)
    {
        #ifdef WATCHDOG
            watchdog_tache[1]=0; // tache asleep
            Watchdog_clear(watchdogHandle);
        #endif


        // Wait for event
        // plusieurs events peuvent être activés en meme temps (pas ordonnés, le meme evenement n'est actif qu'une fois)
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        #ifdef WATCHDOG
            watchdog_tache[1]=1; // tache active
            watchdog_cpt_activ[1]++;   // compteur d'activation
            watchdog_cpt_sleep[1]=0;   // compteur de sleep
        #endif


        // test si plusieurs events peuvent être activés en meme temps
        /*uint8_t cpte=0, k;
        for (k=0; k<32; k++)  if (events & (1<<k)) cpte++;
        if (cpte!=1)
            k=28;  // breakpoint ici*/

       /* #if (DEBUG_TIME&0b1000)
        if ((trigger_time) && (index_time <MAX_Index))
        {
            etat_cpu[index_time] = 0x01;
            etat_detail[index_time] = events +   (phase_radio<<24) + (phase_radio_apres<<28);
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
        }
        #endif*/

        /* ------------------  Envoi message RF   ----------------------------------*/

        if (events & RADIO_EVENT_DEBUT_CYCLE_TX)  // envoi du premier message radio TX
        {
            uint8_t ret;

            // trouve 1er message à envoyer
            // forcage:0:immediats 1:imm+differe
            ret = trouve_mess_RF_envoyer(radio_dest_TX, 0);  // immediat,  destinataire designé

            #if DEBUG_TIME & 0b1000
              if ((trigger_time) && (index_time <MAX_Index))
              {
                  etat_cpu[index_time] = 0x22;  // TX début cycle
                  etat_detail[index_time] = ret ;
                  time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                  if (index_time >MAX_Index) index_time=0;
              }
            #endif

            // transmission vers les nodes reveillés

            if (ret)
            {
                // si message dispo, envoie ce message sauf si differe, ou dest en veille
                if ((Radio.statut<2)  && (knownNodes[mess_a_envoyer.id_dest].mode_rx<2))
                // Radio.statut :  0:arret 1:RX continu 2:RX court 3:RX(apres balise demande)
                // 4:RX apres TX  5:RX Ack (apres TX)  6:fenetre ecoute
                // 0x10:TX 0x11:TX dernier 0x12:TX Keepalive 0x13:TX Balise demande 0x14:TX Balise synchro
                // 0x15/16 : TX apres RX balise demande
                {
                    timeout_cycle_restart(1);   // Timeout si le cycle ne se termine jamais
                    Radio.phase_suivante &= ~0x08;
                    Radio.statut = 0x10;
                    phase_radio_dest = radio_dest_TX;
                    Event_post(radioOperationEventHandle, RADIO_EVENT_TX_SUITE);
                    premier_mess_envoye=0;
                }
                else
                    Radio.phase_suivante |= 8;   // bit0:TX balise dem, bit1:TX keepalive , bit2:ecoute RX, bit3:TX  bit4:synchro bit5:TX(apres balise demande)
            }
        }


        /* -----------  (Concentrateur) Reception Balise de demande de messages --------------------------*/

       if (events & RADIO_EVENT_RX_BALISE_DEMANDE)
        {
            uint8_t ret;
            // Demande de messages en attente
            // Le concentrateur envoie les messages qu'il a en pile, qd la balise est reçue
            update_node(0);
            radio_dest_tx_bal = rxrf_numero_device;
            // cherche messages immediats+differé vers ce node uniquement
            ret = trouve_mess_RF_envoyer(rxrf_numero_device, 1);

            if (ret)
            {
                if (Radio.statut < 2)
                {
                    Radio.statut = 0x15;
                    phase_radio_dest = radio_dest_tx_bal;
                    Event_post(radioOperationEventHandle, RADIO_EVENT_TX_SUITE);
                }
                else
                    Radio.phase_suivante |= 0x20;
            }
        }


        if (events & RADIO_EVENT_TX_SUITE)  // envoi des messages TX
        {
            #if (DEBUG_TIME&0b1000)
              if ((trigger_time) && (index_time <MAX_Index))
              {
                  etat_cpu[index_time] = 0x26;   // TX suite
                  etat_detail[index_time] = Radio.statut ;
                  time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                  if (index_time >MAX_Index) index_time=0;
              }
            #endif

              timeout_cycle_restart(3);   // Timeout si le cycle ne se termine jamais
            if ((Radio.statut==0x10) || (Radio.statut==0x11)) // TX
            {
                if (mess_a_envoyer.statut & 1) Radio.statut=0x11;  // dernier
                Radio.phase_suivante &= ~0x08;
            }
            else  // 0x15 TX apres reception balise demande (concentrateur)
            {
                if (mess_a_envoyer.statut & 1) Radio.statut=0x16;  // dernier
                Radio.phase_suivante &= ~0x20;
            }
            if (mess_a_envoyer.ret)
                Send_mess_radio();
            else
                Phase_radio_suivante(0, 0);
        }


        /* -----------   Envoi Balise keepalive --------------------------*/

        if (events & RADIO_EVENT_DEBUT_TX_KEEPALIVE)
        {
            timeout_cycle_restart(4);   // Timeout si le cycle ne se termine jamais
            if (Radio.statut < 2)
            {
                Radio.statut = 0x12;
                Event_post(radioOperationEventHandle, RADIO_EVENT_TX_KEEPALIVE);
            }
            else
                Radio.phase_suivante |= 2;
        }

        if (events & RADIO_EVENT_TX_KEEPALIVE)  // envoi balise keepalive
        {
            Radio.statut = 0x12;
            Radio.phase_suivante &= ~0x02;
            Radio.phase_suivante |= 4; // ecoute RX ensuite
            envoi_keepalive();
            Phase_radio_suivante(0, 0);
        }


        /* -----------   Envoi Balise Demande message en attente --------------------------*/

        if (events & RADIO_EVENT_DEBUT_TX_Balise_Demande)
        {
            timeout_cycle_restart(5);   // Timeout si le cycle ne se termine jamais
            if (Radio.statut < 2)
            {
                Radio.statut = 0x13;
                Event_post(radioOperationEventHandle, RADIO_EVENT_TX_BALISE_DEMANDE);
            }
            else
                Radio.phase_suivante |= 1;
        }

        if (events & RADIO_EVENT_TX_BALISE_DEMANDE)  // envoi balise demande message en attente
        {
            Radio.phase_suivante &= ~0x01;
            timeout_cycle_restart(6);
            retc = envoi_balise_TX(RADIO_PACKET_TYPE_DEMANDE_MESS_PACKET);  // protocole=50kb => 3.5ms

            if (!retc)   // 0:ok
                 retc = phase_reception_courte(3);  // O:ok => puis lance la phase suivante
            Phase_radio_suivante(0, 0);
        }


        /* -----------   Phase ecoute RX courte --------------------------*/

        if (events & RADIO_EVENT_DEBUT_ECOUTE_RX)
        {
            timeout_cycle_restart(7);   // Timeout si le cycle ne se termine jamais
            if (Radio.statut < 2)
            {
                Radio.statut = 0x2;
                Event_post(radioOperationEventHandle, RADIO_EVENT_ECOUTE_RX);
            }
            else
                Radio.phase_suivante |= 4;
        }
        if (events & RADIO_EVENT_ECOUTE_RX)  // passage  en RX (sans balise demande)
        {
            // Passe en mode reception RX court
            Radio.phase_suivante &= ~0x04;
            timeout_cycle_restart(8);
            retc = phase_reception_courte(2);
            if ((retc) || (mode_rx<4))  // erreur ou rx continu
                Phase_radio_suivante(0, 0);
        }


        #ifdef CONCENTRATOR
        /* -----------   Phase TX Broadcast Balise synchro --------------------------*/
            if (events & RADIO_EVENT_TX_BALISE_SYNCHRO)  // passage cyclique en RX (sans balise demande)
            {
                // Envoi balise synchro
                Radio.phase_suivante &= ~0x10;
                timeout_cycle_restart(10);
                envoi_balise_synchro(0);
                Phase_radio_suivante(0, 0);
            }
        #endif


           #ifdef BROAD_BALISE_SYNCHRO

            /* -----------   Phase RX Reception Balise broadcast synchro --------------------------*/

            if (events & RADIO_EVENT_RX_BALISE_SYNCHRO)
            {

                if (( !(RxPacket_syn->type)) && (latestRxPacket.length==3))  // Type 0 : normal CONC-> NODE balise synchro
                {
                    // impact température -14°C => -40ticks/minute = -6ppm
                    // Fenetre 5ms avec balise 1 minute : 41ppm max    si chaque 5s : conso 5uA + 1uA(balise)
                    // Fenetre 1ms avec balise 1 minute : 8ppm max     si chaque 5s : conso 1uA + 1uA(balise)
                    // Fenetre 5ms avec balise 10 minutes : 4ppm max   si chaque 5s : conso 5uA + 0,1uA(balise)

                    // Nb de ticks apres allumage : 435 + fenetre/2
                    // pour 5ms (500) : 1ere balise recue : 685 ticks apres allumage (fin de reception)
                    // pour 1ms (100) : 1ere balise recue : 485 ticks apres allumage (fin de reception)
                    // 2ms : +80:non  +40:ok
                    // Decal balise 2ms : ok:    nok:45

                    // calcul nouveau décalage
                    uint32_t periode_succ;
                    uint8_t  nb_minutes;
                    uint16_t volatilite;
                    int16_t ecart;
                    uint32_t depuis_rx;
                    decal_conc=0;
                    cpt_balise++;

                    // duree reception balise : 339 ticks (3,4ms)
                    Clock_setTimeout(RadioRxPeriodTimerClockHandle, Synchro.periode_fenetre);
                    depuis_rx = Clock_getTimeout(RadioRxPeriodTimerClockHandle);
                    depuis_rx = Synchro.periode_fenetre - depuis_rx;
                    //  temps depuis l'allumage RX : normalement 339 + duree/2
                    #ifdef BROAD_ENVOI_MESURE  // mode debug:Envoi mesure du temps restant, a chaque reception de balise
                        int16_t ecart_m;
                        ecart_m = depuis_rx - (339 + Synchro.duree_fenetre/2);  //  positif => rx trop tot  negatif=>trop tard
                    #endif
                    num_test++;
                    if (num_test == 1)
                    {
                        num_test=0;
                    }

                    if (Synchro.ticks_prec)
                    {
                        periode_succ = Synchro.ticks_nouv - Synchro.ticks_prec;  // nb de 10 us en 3 minutes
                        nb_minutes = (periode_succ + 30*1000*100) / (60*1000*100);  // arrondi du nb de minutes : ajout 30 secondes
                        decal_conc = ((periode_succ + 30*1000*100) % (60*1000*100)) - (30*1000*100);  // differentiel de tick sur concentrateur
                        decal_conc = decal_conc  / nb_minutes;  // ecart (nb de 10us) par minute  entre 2 balises broadcast
                        //Synchro.periode_fenetre -= ecart;

                        Synchro.nb_balises_ratees += (nb_minutes-1)/periode_balise_synchro;

                        if (cpt_balise==2)
                        {
                            Synchro.ecart = decal_conc;  // Ecart (en tick) par minute : moyenne des 3 derniers
                            Synchro.duree_fenetre = duree_fenetre_RX * 100 * 3;   // premiere reduction duree
                        }
                        else
                        {
                            Synchro.ecart = (Synchro.ecart*2 + decal_conc )/3;  // Ecart (en tick) par minute : moyenne des 3 derniers
                            if (Synchro.ecart < decal_conc)
                                volatilite = (decal_conc - Synchro.ecart);  // entre 0 et 30
                            else
                                volatilite = (Synchro.ecart - decal_conc);
                            if (volatilite < Synchro.volatilite)   // volatilite baisse
                            {
                                Synchro.volatilite = ((Synchro.volatilite*2) + volatilite)/3;
                            }
                            else  // volatilite augmente
                            {
                                Synchro.volatilite = volatilite;
                            }

                            // Adaptation de la duree de la fenetre d'ecoute a la volatilite de l'ecart

                            #ifndef BROAD_DUREE_FIXE  // Départ avec 5 fois la duree, baisse progressivement
                                uint16_t dur_fen, ratio_fen;
                                dur_fen = Synchro.duree_fenetre / periode_balise_synchro;
                                ratio_fen=101;
                                if (Synchro.volatilite)
                                    ratio_fen = dur_fen / Synchro.volatilite;
                                if (ratio_fen > 100)    // volatilite de 4 <=> duree_fenetre de 200/minute => faible volat =50
                                    Synchro.duree_fenetre /= 2;
                                else if (ratio_fen > 50)  // faible   /1,5
                                {
                                    Synchro.duree_fenetre *= 2;
                                    Synchro.duree_fenetre /= 3;
                                }
                                else if (ratio_fen < 20)
                                {
                                    Synchro.duree_fenetre *= 2;  // ratio=20 tres forte volat   *2
                                }
                                else if (ratio_fen < 40)
                                {
                                    Synchro.duree_fenetre *= 3;  // ratio=30  volat   *1,5
                                    Synchro.duree_fenetre /= 2;
                                }

                                if (Synchro.duree_fenetre < duree_fenetre_RX*100)   // valeur min
                                    Synchro.duree_fenetre = duree_fenetre_RX*100;
                            #endif
                        }
                    }



                    ecart = (Synchro.ecart * periode_activ_RX + 30) / 60;  // synchro.ecart : Ecart par minute
                    Synchro.periode_fenetre = periode_activ_RX *1000 * 100 + ecart;  // Mode=5   duree en 10us  - normalement 30 secondes
                    Synchro.ticks_prec = Synchro.ticks_nouv;

                    // periode pour les prochaines fenetres de reception RX
                    Clock_stop(RadioRxPeriodTimerClockHandle);
                    Clock_setPeriod(RadioRxPeriodTimerClockHandle, Synchro.periode_fenetre ); // Period=0 pour un one-shot
                    Clock_setTimeout(RadioRxPeriodTimerClockHandle, Synchro.periode_fenetre - Synchro.duree_fenetre/2 - (339 + 10 )); //+ par_var[3] - par_var[4]) );//- ecart_m/2); // Timeout pour le premier
                    Clock_start(RadioRxPeriodTimerClockHandle);

                    Synchro.statut = 1;
                    cpt_fenetres=0;

                    reactiv_timeout_balise_synchro(); // raz du timeout synchro perdue

                    #ifdef BROAD_ENVOI_MESURE  // mode debug:Envoi mesure du temps restant, a chaque reception de balise
                        strcpy((char *)message, "LSynchro:                              ");
                        ajout_int8_message(decal_conc, 9, 0); //modif de la period_rx/minute:si 22 et 5s=>22/12 =>5s+20us
                        ajout_int8_message(ecart_m,14,0); // ecart par rapport a l'activation rx:si 5=>
                        ajout_int8_message(Synchro.duree_fenetre/100, 19,0);
                        ajout_int8_message(Synchro.volatilite, 24,0);
                        message[13]=' ';
                        message[18]=' ';
                        message[23]=' ';
                        envoi_message();
                    #endif
/*
  Conc:             A:envoi balise                                                 B:Env balise
  Node:          C   F:rx bal                       D    2                     E   xx:rx bal 2

  A->B: nb_minutes-PERIODE_BALISE_SYNCHRO=3
  F->D: Synchro.periode_fenetre = periode_activ_RX + ecart - duree_fenetre/2
  D->E: Synchro.periode_fenetre = periode_activ_RX + ecart
  D->2 : initial : 20ms  objectif:duree_fenetre_RX:4ms   courant:Synchro.duree_fenetre:6ms=600
  E->xx : depuis_rx   E->x:ecart_m

  22°->64à70  36°->80a85

  */
                }
                if (( RxPacket_syn->type==1) && (latestRxPacket.length==13))  // Type 1 : balise resynchro CONC-> NODE
                {
                    // sauvegarde period
                    Synchro.period_RX = RxPacket_syn->period_RX;
                    Synchro.period_bal = RxPacket_syn->period_balise;  // en 10 secondes
                    Synchro.derniere_balise = RxPacket_syn->time;
                    Synchro.statut = 0;
                    Synchro.cpt_period_RX = Synchro.period_RX;
                    // utilisation de l'info : temps jusqu'à la prochaine balise
                }
                if (( RxPacket_syn->type==2) && (latestRxPacket.length==3))  // Type 2 : demande NODE balise resynchro
                {

                }
            #ifdef DEBUG_reseau
                message[0]= 'B';
                UART_write(hUart2, (char *)message, 1);
                //strcpy((char *)message, "1 recept balise synchro");
                //envoi_message ();
            #endif
            /* 08-6
             * 1-6 puis 7
             * 2-0
             * 80-0
             * 1-@
             * 2-A
             */
            }
           #endif

        /* ------------------  If valid packet received --------------------------------------*/

        if (events & RADIO_EVENT_VALID_PACKET_RECEIVED)
        {
            //if (Radio.statut != 1)
            timeout_cycle_restart(11);
            Radio.statut = 6 ; // reception active

            /* Activity LED */
           // ACTIV_SORTIE(LED_cpu, 1,2);

            /* Send ack packet */
            if (!(latestRxPacket.RF_payload.header.statut & 0x10))  // si le destinataire demande un Ack
            {
                sendAck(latestRxPacket.RF_payload.header.sourceAddress);    // => packetReceivedCallback
                if ((mode_rx < 4) || (!(latestRxPacket.RF_payload.header.statut & 1))) // RX continu ou pas dernier
                {
                    /* Re-Enter receive après envoi Ack */
                    if (mode_rx < 4)
                        EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(0));
                    else
                        EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(DUREE_MAX_ECOUTE_RX));

                    if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                                code_erreur = EasyLink_receiveAsync_failed; // 0
                                err_donnee1 = '0';
                    }
                }
            }
            if (latestRxPacket.RF_payload.header.statut & 1)  // si dernier message recu
                Phase_radio_suivante(0,0);  // phase suivante apres reception de tous les messages

            // mise à jour des informations du node remote (et création éventuelle)
            update_node(1);

            // statut recu : b0:dernier  bit4:pas d'ack  bit5:RX apres
            //latestRxPacket.RF_payload.header.statut;

            // Call packet received callback -> Concentrator/NodeTask (traitement du message recu)
            notifyPacketReceived(&latestRxPacket);

        }


        /* If invalid packet received ---------------------------------*/
        if (events & RADIO_EVENT_INVALID_PACKET_RECEIVED)
        {
        }


        // ---------------- NODE: ALLUMAGE PERIODIQUE RADIO  RX ----------------------------------

        if (events & RADIO_EVENT_ALLUMAGE_PERIOD) // mode 4
        {
            #ifdef CONCENTRATOR
                uint8_t ecoute_bal;
                ecoute_bal=0;
                if (Synchro.cpt_period_RX)
                    Synchro.cpt_period_RX--;
                else
                {
                    Synchro.cpt_period_RX = Synchro.period_RX;
                    ecoute_bal=1;
                }

                if (ecoute_bal)  // Ecoute pour attente balise de synchro
                {
                    Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_ECOUTE_RX);
                }
                else  // Ecoute/TX pour messages entrant/sortants
                {
                    if (Radio.phase_suivante & 0b1011)  // messages en attente : bits:0,1,3
                        Phase_radio_suivante(0, 0);
                    else
                        Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_ECOUTE_RX);
                }
            #endif

            #ifdef NODE
                if (mode_rx == 4)
                {
                    uint8_t ret;
                    ret = trouve_mess_RF_envoyer(0, 0);  // -> Concentrateur, immediats,
                    if (ret)
                        Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_CYCLE_TX);  // envoie messages
                    else
                        Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_TX_Balise_Demande);  // envoie balise demande
                }

                if (mode_rx == 5)  // Fenetre d'ecoute
                {
                    if (Radio.statut < 2)
                    {
                        Radio.statut = 0x06;
                        // passage en RX
                        EasyLink_abort();  // stop RX
                        EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(0));  // sans timeout
                        if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                                code_erreur = EasyLink_transmit_balise_failed;
                                err_donnee1 = '2';
                        }
                        //time_cpu[0]=Clock_getTicks();

                        // preparation extinction
                        Clock_stop(RadioRxExtinctTimerHandle);  // stoppe l'extinction , si elle tournait déja
                        // Clock_setPeriod(RadioRxExtinctTimerHandle, 0 ); // Period=0 pour un one-shot  inutile
                        Clock_setTimeout(RadioRxExtinctTimerHandle, Synchro.duree_fenetre);
                        Clock_start(RadioRxExtinctTimerHandle);

                        #ifdef BROAD_LED
                           ACTIV_SORTIE(LED_cpu, 1, 0);  // activation Led N°0
                        #endif

                        //DELAY_US(700);
                        //time_cpu[1] = Clock_getTicks(); // 1 tick = 10 micro-second

                        /*uint8_t i;
                        int8_t rssi_live;
                        //time_cpu[0] = Clock_getTicks(); // 1 tick = 10 micro-second
                        for (i=0; i<15; i++)
                        {
                            //time_cpu[i+1] = Clock_getTicks(); // 1 tick = 10 micro-second
                            EasyLink_getRssi(recep+i);  //RF_getRssi(); //CMD_GET_RSSI();
                            CPUdelay(1000);  // 1000=141us           // 10.000=1ms   10=1us
                            //DELAY_US(20);   // 400:580us  100:290us  50:235us  0:180us
                            if ((recep[i] > -90) && (recep[i]))
                            {
                                time_cpu[2] = Clock_getTicks(); // 1 tick = 10 micro-second
                                break;
                            }
                        }
                        //time_cpu[2] = Clock_getTicks(); // 1 tick = 10 micro-second
                        time_cpu[2]++;*/

                    }
                    cpt_fenetres++;
                }
            #endif
        }


        // -------------------  Extinction RX ------------------------------------------------

        if (events & RADIO_EVENT_EXTINCTION_RX)
        {
            uint8_t rx_arret, retc, retc1, retc2;
            int8_t rssi_live;
            rssi_live=0;


             retc2= Radio.statut;

             #ifdef BROAD_LED
                ACTIV_SORTIE(LED_cpu, 0, 0);  // désactivation Led N°0
             #endif

             if (Radio.statut == 0x06)
             {
                 rx_arret=0;

                 //clear_Watchdog();
                 retc1 = EasyLink_get_running();    // 0:eteint   0x0A:RX activé

                 // lecture rssi . Si -128 : c'est que la radio est off, ou que le rssi n'est pas encore dispo (attendre 1200us)
                 // aléatoirement, la valeur peut être 0 : a ignorer/relire valeur
                 uint8_t cpt_rssi;
                 cpt_rssi=0;
                 while ((!rssi_live) && (cpt_rssi<4))
                 {
                     retc = EasyLink_getRssi(&rssi_live);  //RF_getRssi(); //CMD_GET_RSSI();
                     cpt_rssi++;
                     if (!rssi_live)
                         DELAY_US(100);
                 }

                 // test pour savoir si on n'est pas bloqué dans cet état de reception
                 timer_mess_Rf_en_cours++;

                 if (timer_mess_Rf_en_cours > 9)  // pb bloqué plus de 7 fois
                 {
                     code_erreur = erreur_mess_rf_encours_bloque;
                     rx_arret=1;
                 }
                #ifdef C10_CAMERA_PIR   // blocage interferences
                 if (timer_mess_Rf_en_cours > 3)  // pb bloqué plus de 3 fois car interferences
                 {
                     code_erreur = erreur_mess_rf_encours_bloque;
                     rx_arret=1;
                 }
                #endif

                 if (!rssi_bruit_premier)
                 {
                     rssi_bruit = rssi_live;
                     rssi_bruit_premier=1;
                 }
                 if (rssi_live < -109)   // Seuil de reception en cours, arret : entre 109 et 128
                 {
                     rx_arret=1;
                 }
                /*#ifdef DEBUG_reseau
                     uint8_t rssi_l = (uint8_t) rssi_live;
                     message_test[0]= deci(rssi_l>>4);
                     message_test[1]= deci(rssi_l&15);
                     message_test[2]= rx_arret+'0';
                     message_test[3]= ' ';
                     UART_write(hUart2, (char *)message_test, 4);
                #endif*/
                 if (rx_arret)  // arret
                 {
                     Radio.statut = 0x07;  // Statut RX en arret
                     EasyLink_abort();  // stop RX
                     timer_mess_Rf_en_cours = 0;
                     rssi_bruit_premier=0;
                 }
                 else  // pas d'arret : report
                 {
                     // pour 50kbps : 16 ms de plus (message de 80 caractères)
                     Clock_setTimeout(RadioRxExtinctTimerHandle, (DUREE_FENETRE_RX *4 * 1000 / Clock_tickPeriod));
                     Clock_start(RadioRxExtinctTimerHandle);
                 }
             }
            #if (DEBUG_TIME & 0b1000)
            if ((trigger_time) && (index_time <MAX_Index))
            {
                etat_cpu[index_time] = 0x2E;  // RX fenetre d'ecoute
                etat_detail[index_time] = retc + (retc1<<4) + (retc2<<8) ;
                time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second
                if (index_time >MAX_Index) index_time=0;
            }
            #endif
            rx_arret = retc+retc1+retc2;  //evite l'erreur à la compil
        }

        /* ----------   If we get an ACK  -------------------------------*/

        if (events & RADIO_EVENT_DATA_ACK_RECEIVED)
        {
            //update node ou enregistrement du nouveau Node
            #if (DEBUG_TIME&0b1000)
            if ((trigger_time) && (index_time <MAX_Index))
            {
                etat_cpu[index_time] = 0x2A;    // TX  Ack received
                time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                if (index_time >MAX_Index) index_time=0;
            }
            #endif

            update_node(0);
            returnRadioTX(RFRadioStatus_Success);
        }

        /* -----------  If we get a Keepalive from a node --------------------------*/

        if (events & RADIO_EVENT_KEEPALIVE_RECEIVED)
        {
        #ifdef CONCENTRATOR
            uint8_t  k_type;
            index_test=14;
            //update node ou enregistrement du nouveau Node
            update_node(0);
            k_type = latestRxPacket.RF_payload.Message[0] & 0xF;
            if  (!(k_type  & 0x7))  // Type 0 ou 1 = avec Temp et 1 ou 2 voltages
            {
                float volt_float, volt_modu;
                int8_t temp_modu;
                uint8_t index, k_etat;
                k_etat = latestRxPacket.RF_payload.Message[0] >> 4;

                volt_float =  (float) (latestRxPacket.RF_payload.Message[1] + (float)latestRxPacket.RF_payload.Message[2] / 256);
                volt_modu = ((int16_t) latestRxPacket.RF_payload.Message[4] <<8) + latestRxPacket.RF_payload.Message[5];
                volt_modu = (float) volt_modu/1000;
                strcpy((char *)message, "LKx Volt:  ");
                message[2] = latestRxPacket.RF_payload.header.sourceAddress;
                index = ajout_float_message(volt_float, 2, 9, 2);
                if (k_type)  // type=1
                {
                    message[index-1] = '-';
                    index = ajout_float_message(volt_modu, 2, index, 2);
                }
                temp_modu =  (int8_t) (latestRxPacket.RF_payload.Message[3]) -30;
                strcpy((char *)message+index-1, " Temp:  ");
                index += 6;
                index = ajout_int16_message(temp_modu, index, 2);

                //message[index] = car_fin_trame;
                envoi_message ();

                if (k_etat)  // Etat non nul => Probleme
                {
                    strcpy((char *)message, "LRx-prob");
                    message[2] = latestRxPacket.RF_payload.header.sourceAddress;
                    envoi_message ();
                }

            }
        #endif
        }


        /* ------------    If we get an ACK timeout - message non recu par dest -----------------------------------*/

        if (events & RADIO_EVENT_ACK_TIMEOUT)
        {

            /* If we haven't resent it the maximum number of times yet, then resend packet */
            if (currentRadioOperation.retriesDone < currentRadioOperation.maxNumberOfRetries)
            {
                knownNodes[mess_a_envoyer.id_dest].nb_trans_renvoye++;
                //DELAY_MS (TEMPS_ENTRE_RESEND);  // 500ms entre chaque envoi
                resendPacket();
            }
            else
            {
                returnRadioTX(RFRadioStatus_Timeout);
            }
        }

        /* --------------  If RX timeout ( apres reception courte) ------------------------------------------*/

        if (events & RADIO_EVENT_TIMEOUT_RX)
        {
            Phase_radio_suivante(0, 0);
        }

        /* --------------  If RX aborted ------------------------------------------*/


        if (events & RADIO_EVENT_RX_ABORTED)
        {
            if (Radio.statut == 0x07)   // Statut radio en arret apres la fenetre d'écoute periodique
            {
                Radio.statut = 0;
                Phase_radio_suivante(0, 0);
            }
        }

        /*  --------------Cycle radio bloqué ------------------------*/

        if (events & RADIO_EVENT_CYCLE_BLOQUE)
        {
            // debloquage cycle radio : fin timer RadioCycleTimerHandle
            code_erreur = erreur_easylink_bloque;  // Erreur : redémarrage  0x3A
            err_donnee1 = '1';
            err_donnee2 = deci(timeout_restart);
            EasyLink_abort();  // stop RX
            EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(0));
            Radio.statut=0;
            if  ( mode_rx < 4)
            {
                Radio.statut=1;
                if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                    code_erreur = EasyLink_receiveAsync_failed;  // 1-2
                    err_donnee1 = '1';
                    err_donnee2 = '2';
                    Radio.statut=0;
                }
            }
        }

        /*  -------------- Fin de cycle Radio ------------------*/

        if (events & RADIO_EVENT_FIN_CYCLE_RADIO)
        {
            phase_fin();
        }

        #ifdef BROAD_BALISE_SYNCHRO
        if (events & RADIO_EVENT_DEMANDE_BALISE_SYNCHRO)  // Demande initiale au bout de 4 secondes de switchon ou apres perte
        {
            Clock_stop(RadioRxPeriodTimerClockHandle);
            Demande_balise_synchro();
        }
        #endif

       /* #ifdef DEBUG_reseau
            strcpy((char *)message, "1 R_Event:xxxx-xxxx Statut:x suiv:x");
            ajout_word_message(events, 10);
            message[27]=Radio.statut+'0';
            message[34]=Radio.phase_suivante+'0';
            envoi_message ();
        #endif */

    }
}

void arret_ecoute_radio(void)
{
    Clock_stop(RadioBalTimeoutHandle);            // clock pour la relance de balise toutes les 9 minutes
    Clock_stop(RadioRxPeriodTimerClockHandle);             // Clock pour l'allumage périodique ou balise temporelle
}

void reprise_ecoute_radio(void)
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_DEMANDE_BALISE_SYNCHRO);  // Demande nouvelle balise, toutes les 3 minutes
}


uint8_t Phase_radio(void)
{
    return Radio.statut;
}

uint8_t Radio_suiv(void)
{
    return Radio.phase_suivante;
}

void RadioRxPeriodTimerCallback (UArg arg0)    // fin du timer/clock de Radio RX pour allumage periodique lent de la Radio
{
        Event_post(radioOperationEventHandle, RADIO_EVENT_ALLUMAGE_PERIOD);
}

/*void RadioRxTimerRapidAllumCallback (UArg arg0)    // fin du timer de Radio RX pour allumage rapide sur evenement
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_ALLUMAGE_PERIOD);
}*/

void RadioRxTimerExtinctionCallback (UArg arg0)    // fin de la fenetre d'ecoute rx (en mode 5)
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_EXTINCTION_RX);
}

void RadioTimerBalInitialCallback (UArg arg0)  // au bout de 4 secondes : balise synchro initiale et raz voltage_bas
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_DEMANDE_BALISE_SYNCHRO);
}

void RadioBalTimeoutCallback (UArg arg0)  // si 3 balises synchro de suite ne sont pas reçues
{
    #ifdef BALISE_FINIES
        cpt_perte_balise_synchro--;  // pas de décompte si balise_infinie
    #endif
    if (cpt_perte_balise_synchro)
    {
        #ifdef DEBUG_reseau
            //strcpy((char *)message, "cpt_pert_balise_synchro");
            message[0]= cpt_perte_balise_synchro+'0';
            UART_write(hUart2, (char *)message, 1);
        #endif
        knownNodes[0].resynchro++;
        Event_post(radioOperationEventHandle, RADIO_EVENT_DEMANDE_BALISE_SYNCHRO);  // Demande nouvelle balise, toutes les 3 minutes
    }
    else
        Clock_stop(RadioBalTimeoutHandle);  // au bout de 3 fois : plus de demande

}

void RadioCycleTimerCallback (UArg arg0)    // fin du timer de Radio RX pour extinction
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_CYCLE_BLOQUE);
}

void RadioFinCycleCallback (UArg arg0)    // fin du timer de fin de cycle radio
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_FIN_CYCLE_RADIO);
}


void Radio_debut_tx (void)    // Debut phase TX
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_CYCLE_TX);
}

void RadioDebutKeepAlive (void)    // demande d'envoi keep alive
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_TX_KEEPALIVE);
}

void RadioDebutBaliseTXdemande (void)    // demande balise TX de demande messages en attente
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_TX_Balise_Demande);
}

void RadioDebutEcouteRX (void)    // demande d'écoute RX
{
    Event_post(radioOperationEventHandle, RADIO_EVENT_DEBUT_ECOUTE_RX);
}

// prochaine demande de resynchro (si pas de reception de balise)
uint16_t timeout_prochaine_resynchro(void)
{
    return Clock_getTimeout(RadioBalTimeoutHandle)/100000;
}



// return apres la transmission de message (avec Ack eventuel)
static void returnRadioTX(enum RFRadioOperationStatus result)
{
    uint8_t ret, retc, mess_valid;
    uint8_t num_ad;
    uint8_t reenvoi;
    uint8_t action_ok;

    #if (DEBUG_TIME&0b1000)
    if ((trigger_time) && (index_time <MAX_Index))
    {
        etat_cpu[index_time] = 0x2B;  // TX  fin total envoi du message
        etat_detail[index_time] = result ;
        time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
        if (index_time >MAX_Index) index_time=0;
    }
    #endif

    action_ok=0;
    mess_valid = 0;

    num_ad = mess_a_envoyer.id_dest;

    /* Save result */
    currentRadioOperation.result = result;

    /* Post result semaphore */
    //Semaphore_post(radioResultSemHandle);

    #if (DEBUG_TIME & 0b100)
    if (! SortieTor[0].duree)    // si LED deja active => on ne fait rien
    {
        type_allumage_led = type_allumage_led & 0b10;  // extinction bit 0

        if (!type_allumage_led)  // led reste allumée si RX activé  - A supprimer par sécurité
                ACTIV_SORTIE(LED_cpu,0,0);  // éteint la led
    }
    #endif


    if (currentRadioOperation.result)   //  message non envoyé (pas d'ack, erreur, etc...)
    {
        reenvoi = mess_a_envoyer.statut & 0x6; // bit1&2
        knownNodes[num_ad].nb_trans_ratees++;

        if (reenvoi)  // different de 0 => reenvoi futur 2:1 renvoi 4:5 renvois 6:x renvois
        {
            if (((reenvoi<3) && (knownNodes[num_ad].hors_portee_etat)) \
               || ((reenvoi>2) && (knownNodes[num_ad].hors_portee_etat > 4)))
                    // 1 => Apres 3 essais (1+2re-essais)
                    // 2 => Apres 5 essais (1+4re-essais) => suppression du 1er message
            {
                suppression_message();
            }
            else  // Sinon réenvoi futur
            {
                #ifdef PILE_RF_FLASH
                    Clock_stop(TimerFlashClockHandle);
                #else
                #endif

                // Attend 2 secondes avec clock Hors_portee_timerHandle
                // puis ré-envoi dans callback
                if (knownNodes[num_ad].mode_rx < 2) // Concentrateur en mode écoute
                {
                    knownNodes[num_ad].hors_portee_etat++;
                    hors_portee_calcul_periode(); // simplement : remet la clock au début
                }
            }
        }
        else // pas de réenvoi : suppression ou garde en mémoire
        {
            if (mess_a_envoyer.statut & 0x40)  // bit 6 : sup si pas envoyé
                suppression_message();
        }
        if (mode_rx < 4)
        {
            retc = phase_reception_courte(9);
            Phase_radio_suivante(0, 0);
        }
    }
    else  // success
    {
        #ifdef PILE_RF_FLASH
            uint8_t mess_supp;
            mess_supp=0;
        #endif

        premier_mess_envoye++;

        Radio.RX_apres |= mess_a_envoyer.statut & 0x20;

        knownNodes[num_ad].hors_portee_etat=0;
        knownNodes[num_ad].nb_transmission++;
        hors_portee_desactive(num_ad);

        // avance au message suivant si pas Flash
        #ifndef PILE_RF_FLASH
            uint16_t index;
            index = mess_a_envoyer.tail + mess_a_envoyer.long_complet;
            if( index >= TAILLE_PILE_ENVOI_RADIO )   index = index - TAILLE_PILE_ENVOI_RADIO;   /* if wrapping around  reset pointer */
            knownNodes[num_ad].tail = index;
        #endif

            uint8_t actif;
            actif = EasyLink_get_running();  // 0:pas actif  A:actif
            err_donnee1 = actif;

        // si c'était le dernier envoyé => passage en phase RX ?
            if ((Radio.statut != 0x10) && (Radio.statut != 0x15))  // Dernier
            //if (mess_a_envoyer.statut & 1)  // TODO dernier : mettre plutot cette ligne
            {
                if (Radio.RX_apres)  //  RX apres
                {
                    knownNodes[num_ad].forcage=0;
                    action_ok=1;
                    Radio.RX_apres = 0;
                    retc = phase_reception_courte(4);
                    if ((retc) || (mode_rx <4))  // erreur ou pas d'action
                        action_ok=0;  // RX en erreur => action=0
                }
                else  // Dernier mais pas de RXaprès
                {
                    if (mode_rx < 4)
                    {
                        retc = phase_reception_courte(4);
                        action_ok=0;  // RX en erreur => action=0
                    }
                }
            }
            else  // pas le dernier
            {
                // avance au message suivant
                #ifdef PILE_RF_FLASH
                    // message suivant
                    supp_message_flash(mess_a_envoyer.long_complet,0);
                    mess_supp=1;
                #endif

                ret = trouve_mess_RF_envoyer(phase_radio_dest, 1);
                mess_valid=1;
                if (((Radio.statut == 0x10) || (Radio.statut == 0x15)) && (ret))  // TX mais il reste des messages a envoyer
                {
                    action_ok=1;
                    Event_post(radioOperationEventHandle, RADIO_EVENT_TX_SUITE);
                }
                else if (mode_rx <4)
                {
                    phase_reception_courte(4);  // mode reception continue
                    action_ok = 0;
                }
            }

        if (premier_mess_envoye==1)
        {
            if ( (Clock_getTicks() - prec_tick_voltage) > 1000*100) // > 1 seconde => envoie 1 voltage par seconde
            {
                prec_tick_voltage = Clock_getTicks();
                if (mesure_volt_vref() > 12)  // mesure voltage apres emission radio du premier message
                    event_envoi_voltage();
            }
        }
        //if ((Radio.statut != 0x10) && (Radio.statut != 0x15)) // dernier message envoye
          //  premier_mess_envoye=0;



        // avance au message suivant
        #ifdef PILE_RF_FLASH
            // message suivant
            if (!mess_supp)
                supp_message_flash(mess_a_envoyer.long_complet,0);
            enreg_index_flash();
        #endif


        /*ret = trouve_mess_RF_envoyer(phase_radio_dest, 1);
        mess_valid=1;

        if (((Radio.statut == 0x10) || (Radio.statut == 0x15)) && (ret))  // TX mais il reste des messages a envoyer
        {
            action_ok=1;
            Event_post(radioOperationEventHandle, RADIO_EVENT_TX_SUITE);
        }
        else if (Radio.RX_apres) // c'était le dernier message a envoyer  : RX apres ?
        {
            knownNodes[num_ad].forcage=0;
            action_ok=1;
            Radio.RX_apres = 0;
            retc = phase_reception_courte(4);
            if (retc) action_ok=0;  // RX en erreur => action=0
        }*/

    }
    // Si message long actif : apres envoi message : verif si il y a de la place dans le buffer
    if ((send_long_actif) && (currentRadioOperation.easyLinkTxPacket.payload[5]==('L'|0x80))
            && (currentRadioOperation.easyLinkTxPacket.payload[8]=='Y')
            && (!(currentRadioOperation.easyLinkTxPacket.payload[3] & 0x10)))  // demande d'ack
    {
        if (!currentRadioOperation.result)  // success
        {
            if (latestRxPacket.RF_payload.header.statut)
            {
                strcpy((char *)message, "1WETx");
                message[4] = latestRxPacket.RF_payload.header.statut + '0';
                envoi_message ();
            }
        }
        else  // echec
        {
            /*strcpy((char *)message+1, "WET4xx");
            message[5] = send_long_num_trame_tx >> 8;
            message[6] = send_long_num_trame_tx & 0xFF;
            message[0]=send_long_emett;
            envoi_message ();*/
        }
       /* if ((send_long_buffer_suiv & 0x10) && )  // si success d'un message long
        {
            uint8_t ar = send_long_buffer_suiv & 7;  // 0, 1, 2 (ou 3)
            if ((!ar) && (send_long_buffer_tx)) // message ok , et place dispo pour un autre
            {
                send_long_buffer_tx=0;
            }
              uint16_t free;
              if (send_long_pile)
                  send_long_pile--;
              free = calcul_free_space(6);
              if (free > (TAILLE_FICHIER_TRAME+8))
              {  // place dispo pour un nouveau message
              }
        }
        else   // plusieurs places dispo ou echec : on recupere quand meme une place
        {
            if (send_long_pile)
                send_long_pile--;  // todo : il faudrait verifier que le message parti etait bien un message long
        }*/
    }

    send_long_buffer_suiv=0;

    if (!action_ok)
        Phase_radio_suivante(mess_valid, ret);  // erreur ou pas d'action TX ni RX
}

uint8_t action_suivante (void)
{
    uint8_t action_ok;

    action_ok=0;
    timeout_cycle_restart(12);

    if (Radio.phase_suivante)
    {
        // Radio.statut :  0:arret 1:RX continu 2:RX court 3:RX(apres balise demande)
        // 4:RX apres TX  5:RX Ack (apres TX)
        // 0x10:TX 0x11:TX dernier 0x12:TX Keepalive 0x13:TX Balise demande 0x14:TX Balise synchro
        // 0x15/16 : TX apres RX balise demande
        if (Radio.phase_suivante & 1)  // TX balise Demande messages en attente
        {
            Radio.statut = 0x13;
            Event_post(radioOperationEventHandle, RADIO_EVENT_TX_BALISE_DEMANDE);
            action_ok=1;
        }
        else if (Radio.phase_suivante & 2)  // TX balise keepalive
        {
            Radio.statut = 0x12;
            Event_post(radioOperationEventHandle, RADIO_EVENT_TX_KEEPALIVE);
            action_ok=1;
        }
        else if (Radio.phase_suivante & 4)  // écoute RX court
        {
            Radio.statut = 2;
            Event_post(radioOperationEventHandle, RADIO_EVENT_ECOUTE_RX);
            action_ok=1;
        }
        else if ((Radio.phase_suivante & 8))  // TX nouvelle transmission
        {
            Radio.statut = 0x10;
            phase_radio_dest = radio_dest_TX;
            trouve_mess_RF_envoyer(phase_radio_dest, 0);
            Event_post(radioOperationEventHandle, RADIO_EVENT_TX_SUITE);
            action_ok=1;
        }
        if (Radio.phase_suivante & 0x10)  // TX balise synchro
        {
            Radio.statut = 0x14;
            Event_post(radioOperationEventHandle, RADIO_EVENT_TX_BALISE_SYNCHRO);
            action_ok=1;
        }
        if (Radio.phase_suivante & 0x20)  // TX apres balise demande
        {
            phase_radio_dest = radio_dest_tx_bal;
            trouve_mess_RF_envoyer(phase_radio_dest, 1);
            Radio.statut = 0x15;
            Event_post(radioOperationEventHandle, RADIO_EVENT_TX_SUITE);
            action_ok=1;
        }
    }
    return action_ok;
}

void Phase_radio_suivante(uint8_t mess_val, uint8_t ret)
{
    uint8_t action_ok;

    action_ok = action_suivante();

    #if (DEBUG_TIME&0b1000)
      if ((trigger_time) && (index_time <MAX_Index))
      {
          etat_cpu[index_time] = 0x30;  // phase radio suivante
          etat_detail[index_time] = action_ok  + (Radio.RX_apres<<5) + (Radio.statut<<8) + (Radio.phase_suivante<<12) ;
          time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
          if (index_time >MAX_Index) index_time=0;
      }
    #endif

    if (!action_ok)  // fin du cycle
    {
        //Radio.statut = 0;
        Clock_stop(RadioFinCycleHandle);
        Clock_setTimeout(RadioFinCycleHandle,  2000 / Clock_tickPeriod);  // 2 milliseconde
        Clock_start(RadioFinCycleHandle);
    }
}

// fin de phase suite aux 2ms d'attente sans action
void phase_fin(void)
{
    uint8_t retc, retc2, retc3;

    retc = action_suivante();
    retc2=0;
    retc3=0;

    if (!retc)  // fin du cycle
    {
        Clock_stop(RadioCycleTimerHandle); // stop timeout
        Radio.statut = 0;
        /* Re-Enter receive    */
        if  ( mode_rx < 4)
        {
            Radio.statut = 1;
            retc3 = EasyLink_get_running();  // 0:pas actif  A:actif
            if (retc3) retc3=1;
            if (!retc3) // pas actif
            {
                code_erreur = erreur_easylink_bloque;  // Erreur : redémarrage
                err_donnee1 = '2';

                EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(0));
                retc2 = EasyLink_receiveAsync(rxDoneCallback, 0);
                if (retc2 != EasyLink_Status_Success)
                {
                    code_erreur = EasyLink_receiveAsync_failed;  // 5 - retc
                    err_donnee1 = '5';
                    err_donnee2 = '0' + retc2;
                    Radio.statut = 0;
                }
            }
        }
    }
    #if (DEBUG_TIME&0b1000)
    if ((trigger_time) && (index_time <MAX_Index))
    {
        uint8_t retc4;
        retc4 = EasyLink_get_running();  // 0:pas actif  A:actif
        etat_cpu[index_time] = 0xFF;  // fin cycle apres 2ms
        etat_detail[index_time] = retc + (retc2<<4) +(retc4<<6) +(retc3<<8);
        time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
        if (index_time >MAX_Index) index_time=0;
    }
    #endif

}


// return :0:ok   1:erreur
// Passe en mode reception RX courte
// orig : 2:RX court periodique 3:RX(apres balise deman)
    // 4:RX apres TX  5:RX Ack (apres TX)
uint8_t phase_reception_courte(uint8_t orig)
{
    uint8_t retc=1, actif;

    if (mode_rx < 4)  // mode reception continue
    {
        // Passe en mode reception RX
        actif = EasyLink_get_running();  // 0:pas actif  A:actif
        if (!actif)  // pas actif
        {
            EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(0));
            retc = EasyLink_receiveAsync(rxDoneCallback, 0);  // activation radio
            if (retc != EasyLink_Status_Success)
            {
                code_erreur = EasyLink_receiveAsync_failed;  // 5 + statut - retc
                err_donnee1 = '7';
                err_donnee2 = '0' + retc;
            }
        }
        else  // radio deja active
        {
            code_erreur = EasyLink_receiveAsync_failed;  // Radio deja en mode RX
            err_donnee1 = '6' ;
            err_donnee2 = '1' ;
            retc = 0;  // ok
        }
        Radio.statut = 1;
    }
    else  // mode 4, 5 ou 6
    {
        // Passe en mode reception RX
        EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(DUREE_MAX_ECOUTE_RX));
        retc = EasyLink_receiveAsync(rxDoneCallback, 0);
        if (retc != EasyLink_Status_Success)
        {
            code_erreur = EasyLink_receiveAsync_failed;  // 5 + statut - retc
            err_donnee1 = '5' + orig;
            err_donnee2 = '0' + retc;
        }
        Radio.statut = orig;
    }
    #if (DEBUG_TIME&0b1000)
      if ((trigger_time) && (index_time <MAX_Index))
      {
          etat_cpu[index_time] = 0x31;  // phase reception courte
          etat_detail[index_time] = orig + (Radio.statut<<4) + (Radio.phase_suivante<<8) + (Radio.RX_apres<<12);
          time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
          if (index_time >MAX_Index) index_time=0;
      }
    #endif

    return retc;
}

// Tache Radio : envoi le message dans mess_a_envoyer
// erreurs : 0:ok 1à10:pb Easylink 11:pb longueur
void Send_mess_radio (void)
{
    uint8_t c, num_ad;
    uint8_t index;

    // Get radio access semaphore - Normalement pas bloquant
    //retc = Semaphore_pend(radioAccessSemHandle, 3000*100); // BIOS_WAIT_FOREVER   3 secondes

    index = mess_a_envoyer.tail;
    index++;
    if( index == TAILLE_PILE_ENVOI_RADIO ) index = 0;  /* reset pointer  if wrapping around */
    index++;
    if( index == TAILLE_PILE_ENVOI_RADIO ) index = 0;  /* reset pointer  if wrapping around */

    #ifdef PILE_RF_FLASH
        index=2;
    #endif

    num_ad = mess_a_envoyer.id_dest;

    if ((mess_a_envoyer.longueur>1) && ( mess_a_envoyer.longueur < TAILLE_MAX_MESS_RF) && (mess_a_envoyer.longueur < EASYLINK_MAX_DATA_LENGTH-HEADER_LENGTH))
    {
        for (c=0; c<(mess_a_envoyer.longueur); c++)  // PRZZf(texte) ou PR1ZZ(hexa)  lg=5
        {
            #ifdef PILE_RF_FLASH
                Mess_RF_Packet.RF_payload.Message[c] = message_temp_rf[index++]; /* copy over this byte of data */
            #else
                Mess_RF_Packet.RF_payload.Message[c] = knownNodes[num_ad].buffer_tx[index++]; /* copy over this byte of data */
                if( index == TAILLE_PILE_ENVOI_RADIO ) index = 0;  /* reset pointer  if wrapping around */
            #endif
        }
        Mess_RF_Packet.length = mess_a_envoyer.longueur;
        Mess_RF_Packet.RF_payload.header.packetType = RADIO_PACKET_TYPE_SIMPLE_MESSAGE_PACKET;
        Mess_RF_Packet.RF_payload.header.mode_rx = mode_rx;
        Mess_RF_Packet.RF_payload.header.statut = mess_a_envoyer.statut;
        Mess_RF_Packet.RF_payload.Message [mess_a_envoyer.longueur] = 0;  // Utile ?

        #if (DEBUG_TIME & 0b100)
        if (! SortieTor[0].duree)    // si LED deja active => on ne fait rien
        {
            ACTIV_SORTIE(LED_cpu,1,0);  // active la led
            type_allumage_led = type_allumage_led | 0b01;  // allumage bit 0
        }
        #endif


        if ((send_long_actif) && (Mess_RF_Packet.RF_payload.Message[3]=='Y'))  // message long
        {
            send_long_num_trame_tx =  (Mess_RF_Packet.RF_payload.Message[6] <<8) +  Mess_RF_Packet.RF_payload.Message[7];
        }
        destinataire_pass_RF = knownNodes[num_ad].adresse;

       /* if (message_test[1])  // TODO
        {
            message_test[0]++;
            if ((send_long_actif) && (Mess_RF_Packet.RF_payload.Message[3]=='Y')
            && (Mess_RF_Packet.RF_payload.Message[0]==0xCC)
            && (Mess_RF_Packet.RF_payload.Message[1]==0xE1) )
            {
                if ( (Mess_RF_Packet.RF_payload.Message[4]==1))
                {
                    if (message_in[2]!=0x67)
                        test_err(4);
                }
            }
            else
                test_err(5);
        }*/

        sendDmPacket(Mess_RF_Packet, RADIO_MAX_RETRIES);
    }
    else
    {
        code_erreur = erreur_mess;  // 8 + err_2
        err_donnee1 = '8';
        err_donnee2 = '0'+ mess_a_envoyer.longueur;
        returnRadioTX(RFRadioStatus_FailedNotConnected);
    }
}


static void sendDmPacket(struct Message_RF_Packet RFPacket, uint8_t maxNumberOfRetries)
{
    char unsigned  retc;

    retc=0;

    /* Set destination address in EasyLink API */
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = destinataire_pass_RF;
    currentRadioOperation.easyLinkTxPacket.len = RFPacket.length + HEADER_LENGTH;

    // TRAME : DestAdd(4), AbsTime(4), Len(1), Payload(128) : SourceAdd(1), PacketType(1), mode_rx, statut, message(124)
    /* Copy simple message packet to payload
     * Note that the EasyLink API will implicitily both add the length byte and the destination address byte. */
    currentRadioOperation.easyLinkTxPacket.payload[0] = My_RadioAddress; // .header.sourceAddress;
    currentRadioOperation.easyLinkTxPacket.payload[1] = RFPacket.RF_payload.header.packetType;
    currentRadioOperation.easyLinkTxPacket.payload[2] = mode_rx;
    currentRadioOperation.easyLinkTxPacket.payload[3] = RFPacket.RF_payload.header.statut;
    Radio.cpt_packet++;
    currentRadioOperation.easyLinkTxPacket.payload[4] = Radio.cpt_packet;
    currentRadioOperation.statut = RFPacket.RF_payload.header.statut;

    memcpy (currentRadioOperation.easyLinkTxPacket.payload + HEADER_LENGTH, RFPacket.RF_payload.Message, RFPacket.length);
    message_test[5]++;


    #if (DEBUG_TIME&0b1000)
    // envoi QHLS => debut debug_time
    if ((RFPacket.RF_payload.Message[3]=='L') && (RFPacket.RF_payload.Message[4]=='S'))
         trigger_time=1;

    if ((trigger_time) && (index_time <MAX_Index))
    {
        etat_cpu[index_time] = 0x27;  // TX envoi senddm
        etat_detail[index_time] = currentRadioOperation.statut;
        time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
        if (index_time >MAX_Index) index_time=0;
    }
    #endif

    /* Send packet  */
    EasyLink_abort();  // stop RX

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = maxNumberOfRetries;
    currentRadioOperation.ackTimeoutMs = RADIO_ACK_TIMEOUT_COURT_TIME_MS;
    currentRadioOperation.retriesDone = 0;

    // Durée à 50kbps : 5,16ms pour 6 caractères (HLS)
    // Durée à 5kbps : 18car ->31octets =54ms     51car(110ms)      100car(180ms)
    // pendant la transmission, les autres taches continuent de s'exécuter
    //if ((!send_long_actif) || (RFPacket.RF_payload.Message[3] != 'Y'))  // TODO
      //      || (RFPacket.RF_payload.Message[4] == 0) )  // message long
    //{
        retc = EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket);
    //}



    #if (DEBUG_TIME&0b1000)
    if ((trigger_time) && (index_time <MAX_Index))
    {
        etat_cpu[index_time] = 0x28;  // TX fin envoi senddm
        etat_detail[index_time] = retc;
        time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
        if (index_time >MAX_Index) index_time=0;
    }
    #endif

    if (retc != EasyLink_Status_Success)
    {
        //System_abort("EasyLink_transmit failed");
        code_erreur = EasyLink_transmit_failed;  // 1 + easylink
        err_donnee1 = '1';
        err_donnee2 = '0' + retc;
        returnRadioTX(RFRadioStatus_Failed);
    }
    else
    {
        if (!(RFPacket.RF_payload.header.statut & 0x10))  // si on demande un Ack
        {
            // Timeout pour la reception Ack (uniquement si un ack est demandé)
            EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(RADIO_ACK_TIMEOUT_COURT_TIME_MS));
            /* Enter RX and wait for ACK with timeout (avec resend) */
            retc = EasyLink_receiveAsync(rxDoneCallback, 0);
            if (retc != EasyLink_Status_Success)
            {
                code_erreur = EasyLink_receiveAsync_failed;  // 2 - retc
                err_donnee1 = '2';
                err_donnee2 = '0' + retc;
                returnRadioTX(RFRadioStatus_TXok_RXFailed);
            }
        }
        else   // 1 : si on n'attend pas d'ack , succes immediat
        {
            returnRadioTX(RFRadioStatus_Success);
        }
    }
}


static void resendPacket()
{
    #if (DEBUG_TIME&0b1000)
    if ((trigger_time) && (index_time <MAX_Index))
    {
        etat_cpu[index_time] = 0x29;
        time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
        if (index_time >MAX_Index) index_time=0;
    }
    #endif

    message_test[6]++;
    EasyLink_abort();  // stop RX

    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        code_erreur = EasyLink_transmit_failed;  // 2
        err_donnee1 = '2';
        returnRadioTX(RFRadioStatus_Failed);
    }
    else
    {
        EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(RADIO_ACK_TIMEOUT_LONG_TIME_MS));
        /* Enter RX and wait for ACK with timeout */
        if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
        {
            code_erreur = EasyLink_receiveAsync_failed;  // 4
            err_donnee1 = '4';
            returnRadioTX(RFRadioStatus_TXok_RXFailed);
        }
    }

    /* Increase retries by one */
    currentRadioOperation.retriesDone++;
}

// durée à 50kbps : 3,3ms, equivalent à 20 car (8 bits)
static void sendAck(uint8_t latestSourceAddress) {

    char unsigned retc;

    #if (DEBUG_TIME&0b1000)
      if ((trigger_time) && (index_time <MAX_Index))
      {
          etat_cpu[index_time] = 0x12;   // RX  début send Ack
          etat_detail[index_time] = 0 ;
          time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
          if (index_time >MAX_Index) index_time=0;
      }
    #endif

    /* Set destinationAdress, but use EasyLink layers destination adress capability */
    txPacket.dstAddr[0] = latestSourceAddress;

    /* Copy ACK packet to payload, skipping the destination adress byte.
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    /* Set up Ack packet */
    //ackPacket.header.sourceAddress = My_RadioAddress;
    //ackPacket.header.packetType = RADIO_PACKET_TYPE_SIMPLE_MESSAGE_PACKET; //RADIO_PACKET_TYPE_ACK_PACKET;
    //memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
    txPacket.payload[0] = My_RadioAddress;
    txPacket.payload[1] = RADIO_PACKET_TYPE_ACK_PACKET;
    txPacket.payload[2] = mode_rx;
    txPacket.payload[3] = latestRxPacket.RF_payload.header.compteur;
    uint8_t ar=0;
    if (send_long_actif) ar = (send_long_buffer_tx | 0x10);
    txPacket.payload[4] = ar;
    txPacket.len = 5; //sizeof(ackPacket);

    EasyLink_abort();

    /* Send packet  */
    retc = EasyLink_transmit(&txPacket);
    if (retc != EasyLink_Status_Success)
    {
        //System_abort("EasyLink_transmit failed");
        code_erreur = EasyLink_transmit_failed;
        err_donnee1 = '3';
        err_donnee2 = retc;
    }
    #if (DEBUG_TIME&0b1000)
      if ((trigger_time) && (index_time <MAX_Index))
      {
          etat_cpu[index_time] = 0x13;   // RX  fin send Ack
          etat_detail[index_time] = 0 ;
          time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
          if (index_time >MAX_Index) index_time=0;
      }
    #endif
}



// Envoi balise par le Node au concentrateur pour demande si des messages sont en attente pour le node
static uint8_t envoi_balise_TX(uint8_t type)
{
    char unsigned retc;
    retc=0;

#ifdef NODE

    /*#if (DEBUG_TIME&0b1000)
    if ((trigger_time) && (index_time <MAX_Index))
    {
        etat_cpu[index_time] = 0x40;
        etat_detail[index_time] =  (phase_radio<<24) + (phase_radio_apres<<28);
        time_cpu[index_time] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
        if (index_time >MAX_Index) index_time=0;
        index_time++;
    }
    #endif*/

    /* Set destinationAdress, but use EasyLink layers destination adress capability */
    txPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;
    txPacket.len = 3;
    txPacket.payload[0] = My_RadioAddress;
    txPacket.payload[1] = type;
    txPacket.payload[2] = mode_rx;

    EasyLink_abort();

    /* Send packet  */
    retc = EasyLink_transmit(&txPacket);
    if (retc != EasyLink_Status_Success)
    {
        //System_abort("EasyLink_transmit failed");
        code_erreur = EasyLink_transmit_failed;
        err_donnee1 = '3';
        err_donnee2 = retc;
    }
#endif
    return retc;
}

// Envoi keepalive Radar par le Node au concentrateur
void envoi_keepalive(void)
{
#ifdef NODE
    char unsigned retc;
    uint16_t volt_batt,  voltage_panneau, voltage_pile;
    int32_t temp_cpu;
    uint8_t ret;

    #ifndef KEEP_AL_Type
    #define KEEP_AL_Type 0
    #endif
        //volt_batt = AONBatMonBatteryVoltageGet();
        volt_batt = volt_mesure_dernier;
        temp_cpu = AONBatMonTemperatureGetDegC();

        txPacket.len = 8;

        txPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;
        txPacket.payload[0] = My_RadioAddress;                      // Header
        txPacket.payload[1] = RADIO_PACKET_TYPE_KEEPALIVE_PACKET;   // Header
        txPacket.payload[2] = mode_rx;                              // Header
        txPacket.payload[4] = KEEP_AL_Type;  // 4 bits fort : etat (0:0K) 4 bits faible : type
        txPacket.payload[5] = volt_batt >> 8;  // 1er voltage
        txPacket.payload[6] = volt_batt & 0xFF;
        txPacket.payload[7] = temp_cpu+30;

        // Si type=0 : volt_cpu+temp    si type=1 : volt_batt+temp  si type=2:volt_batt + temp_capteur
        // si type=3 : volt_cpu+volt_pile+temp
        if (KEEP_AL_Type == 3)  // C10_ElecVanne  Keepalive 2 piles
        {
            ret = lecture_analog(0, &voltage_pile);  // mesure 0-3V pour 0-12V
            voltage_pile *= 4;
            if (ret) voltage_pile= 900;  // erreur -> 9V
            txPacket.payload[8] = voltage_pile >> 8;  // 2° voltage pile 9V
            txPacket.payload[9] = voltage_pile & 0xFF;
            txPacket.len = 10;

        }
        else if (KEEP_AL_Type)  // 1 ou 2
        {
            //lecture analogique voltage module (millivolts)
            //txPacket.len = 10;

            // mesure voltage batterie et panneau solaire
            ret = lecture_volt_batterie (&volt_batt, &voltage_panneau);  // en milli-volts
            volt_batt = volt_batt/3.9; // pour passer en centi-volts, base 256 : *0,256
            if (ret)  volt_batt = 100;  // si erreur-> 1 volt
            txPacket.payload[5] = volt_batt >> 8;  // 1er voltage
            txPacket.payload[6] = volt_batt & 0xFF;
        }
        //txPacket.payload[8] = volt_batt2 >> 8;  // 2° voltage
        //txPacket.payload[9] = volt_batt2 & 0xFF;

        #if (DEBUG_TIME&0b1000)
        if ((trigger_time) && (index_time <MAX_Index))
        {
            etat_cpu[index_time] = 0x41;  // envoi keepalive
            etat_detail[index_time] = 0 ;
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
        }
        #endif

        EasyLink_abort();

        /* Send packet  */
        retc = EasyLink_transmit(&txPacket);
        if (retc != EasyLink_Status_Success)
        {
            //System_abort("EasyLink_transmit failed");
            code_erreur = EasyLink_transmit_failed;  // 4
            err_donnee1 = '4';
            err_donnee2 = retc;
        }

        #if (DEBUG_TIME&0b1000)
        if ((trigger_time) && (index_time <MAX_Index))
        {
            etat_cpu[index_time] = 0x42;
            etat_detail[index_time] = 0 ;
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
        }
        #endif

#endif
}

// Envoi balise synchro  par  concentrateur
void envoi_balise_synchro(uint8_t type)
{

}

static void notifyPacketReceived(struct Message_RF_Packet* latestRxPacket)
{
    if (packetReceivedCallback)
    {
        packetReceivedCallback(latestRxPacket, latestRssi);
    }
}

// interruption
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    //union RadioPacket* tmpRxPacket;
    //struct PacketHeader* packetHeader;
    struct RF_Mess_Payload* RxPacket_PL;
    uint8_t passage_rx, err_rx;
    uint8_t comptage_packet_type=0;
    #ifdef DEBUG_reseau
        uint8_t debug_val1=0;
        uint8_t debug_radio_statut_avant = Radio.statut;
        uint8_t debug_radio_suiv_avant = Radio.phase_suivante;
    #endif

    #if (DEBUG_TIME&0b1000)

      /*if ((rxPacket->payload[9]=='T') && (rxPacket->payload[1]==2) && (!status))
          // reception VLHET => activ Debug_time
      {
         // trigger_time=1;
          index_time=0;
      }*/

      if ((trigger_time) && (index_time <MAX_Index))
      {
          etat_cpu[index_time] = 0x00;  // RX Radio interrupt
          etat_detail[index_time] = status + (rxPacket->len <<8) ;
          time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
          if (index_time >MAX_Index) index_time=0;

          /*if (status == EasyLink_Status_Rx_Timeout)
          {
            //  trigger_time=1;
          }*/
      }
      /*if ((rxPacket->payload[9]=='E') && (rxPacket->payload[1]==2)  \
              && (!status) && (rxPacket->payload[10]=='S'))
                           // Reception VLxHES => fin debut_time
      {
        //trigger_time=0;
      }*/
    #endif

    err_rx=1;
    passage_rx=1; // repasse en RX à la fin de cette routine
    comptage_radio_CB++;   // pour debug : compteur d'interruption
    comptage_radio_CB_last = (status<<8);  // pour debug : status + adresse

    rxrf_emetteur = rxPacket->payload[0];
    rxrf_numero_device  = numero_device_rf(rxrf_emetteur);

    if (status == EasyLink_Status_Rx_Error)
    {
        code_erreur = EasyLink_error_RX;  // 0x37:4
        err_donnee1 = '4';
    }

    // Mode_rx : 1:continu 2:RX court periodique 3:RX(apres balise deman)
    // 4:RX apres TX  5:RX Ack (apres TX)


    /* If we received a packet successfully */
    if ((status == EasyLink_Status_Success) && (rxPacket->dstAddr[1] == AddReseau1) && (rxPacket->dstAddr[2]==AddReseau2)
            && (rxPacket->dstAddr[3] == AddReseau3) && ( rxPacket->payload[0] >= RF_address_debut) && ( rxPacket->payload[0]  <= RF_address_fin))
    {
      comptage_radio_CB_last += rxPacket->payload[0];  // pour debug : status + adresse
        #ifdef DEBUG_reseau
              debug_val1=1;
        #endif

      if (rxPacket->dstAddr[0] == BROAD_ADD)  // Diffusion broadcast : reception balise de synchro
      {
          comptage_packet_type=7;
          // check si c'est une balise synchro temporelle
          if (rxPacket->payload[1] == RADIO_PACKET_TYPE_SYNCHRO)
          {
              /* Pointeur vers la Payload */
              latestRxPacket.length = rxPacket->len ;
              RxPacket_syn = (struct RF_synchro_Payload*)(rxPacket->payload);
              Synchro.ticks_nouv = Clock_getTicks(); // 1 tick = 10 micro-second
              err_rx=0;

              Event_post(radioOperationEventHandle, RADIO_EVENT_RX_BALISE_SYNCHRO);
          }
      }
      else  // diffusion normale
      {
        /* Save packet */
        /* Save the latest RSSI, which is later sent to the receive callback */
        latestRxPacket.dest = (int8_t)rxPacket->dstAddr[0];
        latestRxPacket.rssi = (int8_t)rxPacket->rssi;
        latestRxPacket.absTime = (uint32_t)rxPacket->absTime;
        latestRxPacket.rx_TimeOut = (uint32_t)rxPacket->rxTimeout;
        latestRxPacket.tick_rx = Clock_getTicks();
        latestRxPacket.length = rxPacket->len ;

        /* Pointeur vers la Payload */
        RxPacket_PL = (struct RF_Mess_Payload*)(rxPacket->payload);

        latestRxPacket.RF_payload.header.sourceAddress = RxPacket_PL->header.sourceAddress;
        latestRxPacket.RF_payload.header.packetType = RxPacket_PL->header.packetType;
        latestRxPacket.RF_payload.header.mode_rx = RxPacket_PL->header.mode_rx;
        // statut recu : b0:dernier  bit4:pas d'ack  bit5:RX apres
        latestRxPacket.RF_payload.header.statut = RxPacket_PL->header.statut;


        // Message bien reçu d'un émetteur => déblocage éventuel de celui-ci
        if (rxrf_numero_device)
            {
            hors_portee_desactive (rxrf_numero_device-1);
            phase_radio_dest = rxrf_numero_device-1;
            #ifdef DEBUG_reseau
                 debug_val1=2;
            #endif
            }


        /*Enchainement des Events :
        ACK_Packet            -> Radio_event
        KeepAlive_Packet      -> Radio_event_keepalive
        Demande_mess_Packet   -> Concentrator_event (EVENT_MESSAGE_RF_A_ENVOYER)
        Simple_message_packet -> RADIO_EVENT_VALID_PACKET_RECEIVED
        Balise_synchro        -> Radio_event_synchro

        Check if this is an ACK packet */
        if (latestRxPacket.RF_payload.header.packetType == RADIO_PACKET_TYPE_ACK_PACKET)
        {
            err_rx=0;
            passage_rx=0;
            comptage_packet_type=1;
            // Signal ACK packet received   4,1ms après la fin du message envoyé
            send_long_buffer_suiv = latestRxPacket.RF_payload.Message[4];  // Ack du message long
            Event_post(radioOperationEventHandle, RADIO_EVENT_DATA_ACK_RECEIVED);
        }
        // check si c'est un message d'info keepalive
        else if (latestRxPacket.RF_payload.header.packetType == RADIO_PACKET_TYPE_KEEPALIVE_PACKET)
        {
            comptage_packet_type=2;
            if ((latestRxPacket.length ==7) || (latestRxPacket.length==9))
            {
                memcpy (latestRxPacket.RF_payload.Message, RxPacket_PL->Message, latestRxPacket.length-6); // Longueur keepalive=4
                err_rx=0;
                Event_post(radioOperationEventHandle, RADIO_EVENT_KEEPALIVE_RECEIVED);
            }
        }
        // Vérifie si c'est une balise de demande de message en attente
        else  if (latestRxPacket.RF_payload.header.packetType == RADIO_PACKET_TYPE_DEMANDE_MESS_PACKET)
            // concentrateur reçoit ce message : il renvoie les message dans la pile
        {
            comptage_packet_type=3;
            err_rx=0;
            Event_post(radioOperationEventHandle, RADIO_EVENT_TX_BALISE_DEMANDE);
        }
        //  -------------- messages normal recu ---------------------
        else if (latestRxPacket.RF_payload.header.packetType == RADIO_PACKET_TYPE_SIMPLE_MESSAGE_PACKET)
        {
                #if (DEBUG_TIME&0b1000)
                if ((trigger_time) && (index_time <MAX_Index))
                {
                    etat_cpu[index_time] = 0x10;  // RX message normal recu
                    etat_detail[index_time] = RxPacket_PL->header.statut ;
                    time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                    if (index_time >MAX_Index) index_time=0;
                }
                #endif


            comptage_packet_type=5;
            // If this is a known packet, puis envoie Ack
                // vitesse 50kbps : 4ms (6 car) 5,7ms(13car) 7,11ms (18car) 7,3ms(23car) 8,9ms(33car) 10,5ms(43car) après réception de l'ack précédent
                // vitesse 5kpbs : 140ms (80 car)

            if (latestRxPacket.length > 127) latestRxPacket.length = 127;
            if (latestRxPacket.length <= HEADER_LENGTH + 1)
                    latestRxPacket.length = 0;
            else
                latestRxPacket.length -= (HEADER_LENGTH + 1);


            //latestRxPacket.RF_payload.header.statut = RxPacket_PL->header.statut ;

            // si le précédent était le dernier alors init du flag rx_destinataire
            if (Radio.dernier_mess_recu)
            {
                Radio.dernier_mess_recu = 0;
                Radio.destinat_rx_apres = 0;  // RAZ RX apres chez le node distant
            }
            Radio.destinat_rx_apres |= (RxPacket_PL->header.statut & 0x20);  // RX apres chez le node distant

            // si dernier, alors flag
            if (RxPacket_PL->header.statut & 1)
                Radio.dernier_mess_recu=1;

            //Radio.RX_apres |= mess_a_envoyer.statut & 0x20;  // RX apres

            latestRxPacket.RF_payload.header.compteur = RxPacket_PL->header.compteur ;

            // Save RX_packet message
            memcpy (message_in, RxPacket_PL->Message, latestRxPacket.length+1);

            if (latestRxPacket.length)  // longueur valide
            {
                /* Signal packet received , puis envoie Ack */
                // pas d'ack => repasse en RX
                passage_rx=0;  // Comme on doit envoyer l'Ack, pas besoin de passer en RX
                if ((latestRxPacket.RF_payload.header.statut & 0x10) ) // && (Radio.statut==1))
                    passage_rx=1; // si le destinataire ne demande pas d'ack, alors passage en rx
                err_rx=0;
                Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);

            }
        }
        else // type inconnu
        {
            comptage_packet_type=6;
            code_erreur = EasyLink_error_RX;  // 5
            err_donnee1 = '5';
            /* Signal invalid packet received */
            //Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
        }
      }
      comptage_radio_CB_last += (comptage_packet_type<<12);  // pour debug : packet_type 0:pas ok, 1:Ack 2:keepalive 3:demande 4:synchro 5:ok, 6:inconnu 7:broadcast
    }
    /* did the Rx timeout (Ack non reçu  ou fin de reception RX) */
    else if(status == EasyLink_Status_Rx_Timeout)
    {
        err_rx=0;
        passage_rx=0;
        if (Radio.statut >= 0x10)   // Ack apres transmission TX
        {
            /* Post a RADIO_EVENT_ACK_TIMEOUT event */
            Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
        }
        else  // réception courte
        {
            /* Post a RADIO_EVENT_TIMEOUT_RX event */
            Event_post(radioOperationEventHandle, RADIO_EVENT_TIMEOUT_RX);
        }
    }
    /* did the Rx aborted : action utilisateur*/
    else if (status == EasyLink_Status_Aborted)
    {
        err_rx=0;
        passage_rx=0;
        Event_post(radioOperationEventHandle, RADIO_EVENT_RX_ABORTED);
    }
    else
    {
         /* Rx Error : Post a RADIO_EVENT_INVALID_PACKET_RECEIVED  event */
        code_erreur = EasyLink_error_RX;  // 8
        err_donnee1 = '8';
        err_donnee2 = status+'0';
        if (err_donnee2==(10+'0')) err_donnee2='1';
        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }

    if (err_rx)
    {
        uint8_t num;
        num = 0;
        if (rxrf_numero_device) num = rxrf_numero_device - 1;
        knownNodes[num].nb_rx_err++;
    }


    /* Re-Enter receive    */
    if ( passage_rx)  // erreur ou message recu
    {
        if (mode_rx < 4)
            EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(0));
        if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
            //System_abort("EasyLink_receiveAsync failed");
                    code_erreur = EasyLink_receiveAsync_failed;  // 3
                    err_donnee1 = '3';
                    Radio.statut = 0;
        }
    }
    #ifdef DEBUG_reseau
        strcpy((char *)message, "1 status:x val1:x long:x type:x R:x-x suiv:x-x");
        message[9]=status+'0';
        message[16]=debug_val1+'0';
        message[23]=rxPacket->len+'A';
        message[30]=comptage_packet_type+'0';
        message[34]=debug_radio_statut_avant+'0';
        message[36]=Radio.statut+'0';
        message[43]=debug_radio_suiv_avant+'0';
        message[45]=Radio.phase_suivante+'0';
        envoi_message();
    #endif
}

// mode : 1(lecture) 2(ecriture)
void chgt_mode_radio(char unsigned action, char unsigned mode, uint8_t periode)
{
    if (action==1)   // lecture
    {
        message[12] =  mode_rx  +'0';
        conversion_deci_text_message(periode_activ_RX, 22);
        message[31] = (Synchro.duree_fenetre/1000) + '0';
        message[32] = (Synchro.duree_fenetre%100) + '0';
        message[43] = periode_balise_synchro + '0';
    }
    if (action==2)   // ecriture
    {
        if (mode_rx != MODE_RX)     // chgt mode_rx
        {
            mode_rx = mode ;
            periode_activ_RX = periode;  // periode en secondes (30 secondes)
            application_mode_rx();
        }
        else       // chgt periode d'ecoute RX (mode 4 et 5)
        {
            if ((MODE_RX==4) || (MODE_RX == 5))
            {
                periode_activ_RX = periode;  // periode en secondes (30 secondes)
                if (mode_rx == 4)
                {
                    Clock_stop(RadioRxPeriodTimerClockHandle);
                    Clock_setPeriod(RadioRxPeriodTimerClockHandle, periode_activ_RX *1000 * (1000 / Clock_tickPeriod)); // Period=0 pour un one-shot
                    Clock_start(RadioRxPeriodTimerClockHandle);
                }
                if (mode_rx == 5)
                {
                    #ifdef BROAD_BALISE_SYNCHRO
                    Clock_stop(RadioRxPeriodTimerClockHandle);
                    Demande_balise_synchro();  // demande immediate de balise synchro
                    #endif
                }
            }
        }
    }
}

// Nouv_Deman_balise_synchro();  le node redemande dans 4 sec une balise synchro au concentrateur (par BLSY)

// modifPerioFeneBal( PERIODE_ACTIV_RX, 0 );


void change_power_radio (int8_t as)
{
    EasyLink_abort();
    EasyLink_setRfPower(as);
    if (mode_rx == 1)   // Radio RX : Allumage continu
    {
        //phase_radio=2;
        if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
            code_erreur = EasyLink_receiveAsync_failed; // 2
            err_donnee1 = '2';
        }
    }
}

void application_mode_rx(void)  // Activation du nouveau mode RX demandé
{

    Clock_stop(RadioBalTimeoutHandle);            // clock pour la relance de balise toutes les 9 minutes
    Clock_stop(RadioRxPeriodTimerClockHandle);             // Clock pour l'allumage périodique ou balise temporelle
    EasyLink_abort();
    #if (DEBUG_TIME & 0b100)
    ACTIV_SORTIE(LED_cpu,0,0);  // eteint la led
    type_allumage_led = 0;
    #endif

    if ( mode_rx ==1)   // Radio RX : Allumage continu
    {
        if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                code_erreur = EasyLink_receiveAsync_failed;  // 1
                err_donnee1 = '1';
        }
        else
            Radio.statut=1;
    }
    if (mode_rx == 4)  // Radio RX : Allumage périodique lent (4:avec balise TX)
    {
        Clock_setPeriod(RadioRxPeriodTimerClockHandle, periode_activ_RX *1000 * (1000 / Clock_tickPeriod)); // Period=0 pour un one-shot
        Clock_start(RadioRxPeriodTimerClockHandle);
    }
    #ifdef BROAD_BALISE_SYNCHRO
        if (mode_rx == 5)  // Radio RX : Allumage périodique RX (balise temporelle de synchro)
        {
            // Demande balise synchro dans 4 secondes
            Nouv_Deman_balise_synchro();
            // Activation du timeout si balise non recue (au bout de 9 minutes)
            reactiv_timeout_balise_synchro();
        }
    #endif
    if (mode_rx == 6)   //  pas de RX periodique
    {
    }
}



static void updateNode(uint8_t active_node, uint8_t aug_nb_recep)
{
    knownNodes[active_node].latestRssi = latestRxPacket.rssi;
    knownNodes[active_node].mode_rx = latestRxPacket.RF_payload.header.mode_rx;
    if (aug_nb_recep) knownNodes[active_node].nb_reception ++;
    knownNodes[active_node].latestLength = latestRxPacket.length+1;
}


void update_node (uint8_t aug_nb_recep)
{
    uint8_t active_node, node_add;
    bool retc;

    node_add = latestRxPacket.RF_payload.header.sourceAddress;
    /* If we knew this node from before, update the value */
    active_node = numero_device_rf(node_add);
    if (active_node)
    {
        updateNode(active_node-1, aug_nb_recep);
    }
    else
    {
        /* Else add it */
        retc = addNewNode(node_add);
        if (!retc)
        {
            rxrf_numero_device = lastAddedSensorNode;

            updateNode(lastAddedSensorNode-1, aug_nb_recep);
            #ifdef CONCENTRATOR  // TODO a déplacer dans la tache concentrator
            strcpy((char *)message,"LNouveau capteur : ");
            message[19]= node_add ;
            message[20]= 13;
            envoi_message();
            #endif
        }
    }
}


 bool addNewNode(uint8_t adresse) {

    /* Increment  */
    if (lastAddedSensorNode < NB_DESTINATAIRES_NODE)
    {
        knownNodes[lastAddedSensorNode].adresse = adresse;
        knownNodes[lastAddedSensorNode].type_node = 1;
        knownNodes[lastAddedSensorNode].puissance = 0;
        knownNodes[lastAddedSensorNode].nb_reception= 0;
        knownNodes[lastAddedSensorNode].nb_transmission= 0;
        knownNodes[lastAddedSensorNode].tail=0;
        knownNodes[lastAddedSensorNode].head=0;
        knownNodes[lastAddedSensorNode].mode_rx=0;
        knownNodes[lastAddedSensorNode].nb_trans_renvoye = 0;
        knownNodes[lastAddedSensorNode].nb_trans_ratees = 0;
        knownNodes[lastAddedSensorNode].nb_messages_effaces = 0;
        knownNodes[lastAddedSensorNode].nb_rx_err = 0;
        knownNodes[lastAddedSensorNode].hors_portee_etat = 0;
        knownNodes[lastAddedSensorNode].forcage=1;
        knownNodes[lastAddedSensorNode].synchro=0;
        knownNodes[lastAddedSensorNode].code_tournant = 0xD4A6A59B + (adresse << 16) + adresse;  // code tournant initial

        lastAddedSensorNode++;
        return 0;
    }
    else
    {
        code_erreur = Nb_Node_max_atteint;
        return 1;
    }
}

 void traitement_message_radio (char unsigned longueur_m)
 {
     if (message_in[3] == 'L')
     {
         if ((message_in[4] == 'N') && (longueur_m==5))        //   RLN    : Lecture Nb et Identification des Nodes enregistres
         {
             uint8_t index, i, a;
             strcpy((char *)message+1, "Radio Node   /       ");  // S0 X1 b2...
             message[12] = ((lastAddedSensorNode)/10)+'0';
             message[13] = ((lastAddedSensorNode)%10)+'0';
             message[15] = (NB_DESTINATAIRES_NODE/10)+'0';
             message[16] = (NB_DESTINATAIRES_NODE%10)+'0';
             index = 20;
             for (i=0; i<lastAddedSensorNode;i++)
             {
                 a = knownNodes[i].adresse;
                 if ((a>='0') && (a<='z'))
                     message[index++] = a;
                 else
                     message[index++] = '-';

                 message[index++] = i+'0';
                 message[index++] = ' ';
             }
             message[index] = car_fin_trame;
             envoi_message ();
         }
     }
     if (message_in[3] == 'E')
     {

         if ((message_in[4] == 'N')  && (message_in[5] == 'S') && (longueur_m==7))   // RENSx  : Suppression Node x dans la pile des Node
         {
             uint8_t node_sup ;
             uint16_t lg_node;
             //uint16_t in_dest, in_sour, i;
             node_sup = message_in[6] - '1';  // 3 noeuds(0-1-2)  mess=2 => node_sup=1:2->1
             if ( (node_sup) < lastAddedSensorNode )
             {
                 if ( (node_sup+1) < lastAddedSensorNode )
                 {
                     lg_node = sizeof(struct RemoteNode);
                     memcpy ( ((uint8_t*) &knownNodes) + node_sup*lg_node, ((uint8_t*) &knownNodes) +(lastAddedSensorNode-1)*lg_node, 1);

                     /*in_sour = (lastAddedSensorNode-1)*lg_node;
                     in_dest = node_sup*lg_node;
                     uint8_t * addK;
                     addK = ((uint8_t*) &knownNodes) ;
                     for (i=0; i<lg_node; i++)
                     {
                         *(addK+in_dest+i) = *(addK+in_sour+i);
                     }*/
                 }
                 lastAddedSensorNode--;
             }
         }
     }
 }

 void envoi_cpt_radio_CB(void)
 {
    strcpy((char *)message+1, "Radio cpt:     last: -   ");
    message_1int (comptage_radio_CB, 11);
    message_1hexa (comptage_radio_CB_last >> 8, 21);
    message[24] = comptage_radio_CB_last & 0xFF;
    envoi_message ();
 }

 void envoi_cpt_index_test(void)
 {
    strcpy((char *)message+1, "Radio ind:   ");
    message_1int (index_test, 11);
    message[15] = car_fin_trame;
    envoi_message ();
 }

#ifdef BROAD_BALISE_SYNCHRO

// Demande au concentrateur une nouvelle balise synchro
// le node redemande dans 4 sec une balise synchro au concentrateur (par BLSY)
void Nouv_Deman_balise_synchro(void)
{
    Clock_setTimeout(RadioTimerBalInitialHandle, 4*1000*100);  // 4 secondes
    Clock_start(RadioTimerBalInitialHandle);
    reactiv_timeout_balise_synchro();
}

// nb de millisecondes avant prochaine fenetre
uint16_t timeout_prochaine_fenetre_node(void)
{
    uint32_t reste = Clock_getTimeout(RadioRxPeriodTimerClockHandle);
    reste /= 100;  // en millisecondes : 0 à 5000 (si 5s)
    return (uint16_t) reste;
}

 // demande de balise pour se resynchroniser avec une passerelle en mode écoute continue (mode_rx=1)
 void Demande_balise_synchro(void)
 {
     message_test[4]++;
     strcpy((char *)message+1, "BLSY");  // Envoi de la demande de resynchro à la passerelle/repeteur
     message[0] = PASSERELLE_ADDRESS;
     message[5] = deci (periode_activ_RX >> 4);
     message[6] = deci (periode_activ_RX & 0x0F);
     message[7] = car_fin_trame;
     Mess_statut = 0x30; // reenvoi - RX apres // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)  bit3:différé   bit4:pas d'ack  bit5:RX apres
     envoi_message ();
 }

void recept_balise_resynchro (uint32_t temp_restant)  // retour de BESY
{
// Consommation pendant un cycle de balise de fentre 4ms : moyenne 11mA
    //  2ms de préparation à 3mA
    // 2,4 ms de réception à 7mA  + ajout de 15ms chaque minute
    // 5ms de fin à 4mA (extinction)

   Synchro.ecart=0;
   #ifdef BROAD_DUREE_FIXE  // mode debug
    Synchro.duree_fenetre = duree_fenetre_RX * 100 * 4;  // Mode=5   duree en 10us
    Synchro.ecart=-86;
   #else
    Synchro.duree_fenetre = duree_fenetre_RX * 100 * 10;  // Mode=5   duree en 10us multiplié par 5 initialement
   #endif

    Synchro.periode_fenetre = periode_activ_RX *1000 * 100 +  Synchro.ecart*periode_activ_RX/60;  // Mode=5   duree en secondes
    Synchro.decalage = 0;
    Synchro.volatilite =  0; //Synchro.duree_fenetre/50;   // pour premier calcul
    periode_balise_synchro = message_in[10];
    cpt_fenetres = 0;
    cpt_balise = 0;

    int32_t ratio = Synchro.ecart*periode_activ_RX/60;
    ratio = 43;
    temp_restant = temp_restant - (temp_restant*ratio)/Synchro.periode_fenetre;
     // periode pour les prochaines fenetre de reception RX
     Clock_stop(RadioRxPeriodTimerClockHandle);
     // 66/minute => 11/10sec
     Clock_setPeriod(RadioRxPeriodTimerClockHandle, Synchro.periode_fenetre );  // Ecart par periode de fenetre rx
     // Pour une fenetre de 5ms(500) : Min:440(debut de fenetre)   Max:860(fin de fenetre)
     // Pour une fenetre de 1ms(100) : Min:635   Max:715  => 675
     // 1ms :  635->445   645->456  660->475    670>482  675->485  700->511   710->522   715->529(limite)   720:non
     // 2ms : +72:non +70:ok -50:ok  -60:ok  -65:non  => fenetre de 1,3ms (conso:29ma.ms)
     // 3ms => fenetre de 2,3ms (36ma.ms)
     // 4ms => fenetre de 3,3ms (43ma.ms)
     Clock_setTimeout(RadioRxPeriodTimerClockHandle, temp_restant - Synchro.duree_fenetre/2 - 671 + 88); // 2ms=>671  5ms->650
     Clock_start(RadioRxPeriodTimerClockHandle);

     Synchro.statut = 1;

     reactiv_timeout_balise_synchro();  // Activation timeout synchro perdue
     Synchro.ticks_prec = 0;
}

void reactiv_timeout_balise_synchro(void)
{
     // Timer de timeout si pas de réception de 4 balises de suite (12 min pour 3min de periode balise)
     cpt_perte_balise_synchro = 3;
     Clock_stop(RadioBalTimeoutHandle);
     uint32_t hasard;
     hasard =  ((uint32_t)rand()<<7);  // 0xFFFFFF, 10us a 84s
     Clock_setPeriod(RadioBalTimeoutHandle, 1 * periode_balise_synchro * 61 *1000 * (1000 / Clock_tickPeriod));  // Clock_tickPeriod=10
     Clock_setTimeout(RadioBalTimeoutHandle, hasard  +  1 * periode_balise_synchro * 61 *1000 * (1000 / Clock_tickPeriod)); // Timeout
     Clock_start(RadioBalTimeoutHandle);
}

// modifie la periode de la fenetre d'ecoute de la balise periodique de synchro
// perio : en secondes
// mode : 0: définitif   1:temporaire
void modifPerioFeneBal( uint8_t perio, uint8_t mode)
{
    uint32_t ecoule, resteF, temp_restant;

    if ( !((60*(uint16_t)periode_balise_synchro) %  perio ) )  // la periode totale doit être un multiple de la periode demandee
    {

        // si avant 10s et apres 15s => timeout 5s
        // si avant 10s et apres 30s => timeout 10s ou 20s
        //uint32_t resteC,  resteF;
        Clock_setTimeout(RadioRxPeriodTimerClockHandle, Synchro.periode_fenetre);
        ecoule = Clock_getTimeout(RadioRxPeriodTimerClockHandle);
        Clock_stop(RadioRxPeriodTimerClockHandle);
        ecoule = Synchro.periode_fenetre - ecoule + periode_activ_RX * cpt_fenetres *1000*100 ;        //  temps depuis derniere balise recue

        resteF = (perio*1000*100) - (ecoule % (perio*1000*100));  // reste avant la prochaine fenetre d'ecoute

        // si resteF est trop petit (<200ms), rajout d'une periode
        if (resteF < 20000 )
            resteF += perio*1000*100;

        periode_activ_RX = perio;
        temp_restant = resteF + resteF*(Synchro.ecart*perio/60)/Synchro.periode_fenetre;  // ajout de l'ecart au prorata du temps restant

        // periode pour les prochaines fenetres de reception RX
        Clock_setPeriod(RadioRxPeriodTimerClockHandle, Synchro.periode_fenetre ); // Period=0 pour un one-shot
        Clock_setTimeout(RadioRxPeriodTimerClockHandle, temp_restant - Synchro.duree_fenetre/2 - (100 )); // + par_var[1] - par_var[2]) ); // Timeout pour le premier
        Clock_start(RadioRxPeriodTimerClockHandle);

    }
}

void envoi_info_balise()
{
    strcpy((char *)message, "LBalise: ");
    ajout_int8_message(decal_conc, 9, 0); //ecart entre node et concentrateur
    message[13]=' ';
    ajout_int8_message(periode_activ_RX, 14, 0);
    message[18]=' ';
    ajout_int8_message(duree_fenetre_RX,19,0);
    message[23]=' ';
    ajout_int8_message(Synchro.duree_fenetre/100, 24,0);
    message[28]=' ';
    ajout_int8_message(Synchro.nb_balises_ratees, 29,0);
    message[33]=' ';
    ajout_int8_message(Synchro.volatilite, 34,0);
    envoi_message();
}

#endif // #ifdef BROAD_BALISE_SYNCHRO


// en mode RX continu, test si la radio est on , sinon message + mode ecoute
void verif_radio_rx(void)
{
    uint8_t radio_run = EasyLink_get_running();  // 0:eteint   0x0A:RX activé
    #ifdef CONCENTRATOR
    if (mode_rx == 1) // normalement radio on
    {
        if (!radio_run)
        {
            verif_radio_run_cpt++;
            if (verif_radio_run_cpt>3)   // 4° fois que la radio est eteinte
            {
                verif_radio_run_cpt=0;
                phase_reception_courte(9);
                strcpy ( (char *)message, "LWatch: RX inactif");
                envoi_message();
            }
        }
        else
            verif_radio_run_cpt=0;

        if (envoi_f_balise)
        {
            verif_radio_bal_cpt++;
            if (verif_radio_bal_cpt>120)   // 120 fois (20 min) que la bal est active
            {
                envoi_f_balise=0;
                verif_radio_bal_cpt=0;
                Clock_stop(RadioTimerFenetreHandle);
                strcpy ( (char *)message, "LWatch: TX bal_actif");
                envoi_message();
            }
        }
        else
            verif_radio_bal_cpt=0;
    }
    #endif
    if (mode_rx > 1) // normalement radio off
    {
        if (radio_run)
        {
            verif_radio_run_cpt++;
            if (verif_radio_run_cpt>3)   // 4° fois que la radio est active
            {
                verif_radio_run_cpt=0;
                strcpy ( (char *)message, "LWatch: RX actif");
                envoi_message();
                Radio.statut = 0;
                EasyLink_abort();  // stop RX
            }
        }
        else
            verif_radio_run_cpt=0;
    }
}

