/* Comm_message.c
 *
 *  3.1.8 18 Nov 2020 commun avec concentrator
 *  3.1.7 13 Oct 2020
 *  3.1.6  9 Oct 2020 : transfert message cc13xx_esp32
 *  3.1.5 Sept 2020 : cc13xx clock
 *  3.1.4 Sept 2020 : callback clock identique
 *  3.1.1 Sept 2020 : generique
 *  1.0.0 Avril 2017 : creation CC13xx
 *

Codes utilises : B, C, E, H, K, M, P, R, S, V, X, Z

messages recus :
 A : Analogique
 B : Batterie BMS & Balise
 C : Capteur (activation, config, lecture)
 E : Entrees & Erreurs
 H : Heure/date
 K : Keepalive
 P : Periodiqe
 R : Radio (mode, periodicité,...)
 S : Statut & Parametres
 T : Test
 V : Valeur Ram (RVLRxx RVLXxxx RVERxxzz RVEXxxxzz)
 W : camera
 X : Sorties (LED & buzzer) XExyz (Activation sortie X, mode y pdt z sec)  XLx (Lecture Etat Sortie)
 Y : messages long : fichiers
 Z : Reset ou Raz




A Analogique
           ALV : Lecture voltage micro
           AL0 : lecture voltage panneau solaire en millivolts (faire *2)
           AL1 : lecture voltage batterie en millivolts (faire *2)
           ALT : Lecture temperature micro

Balise     BLD   : L->N : decal_Node/conc, Lecture periode, fenetre, fenetre actuelle, perdues, volatilite
           BLZ   : L->N : Reactivation balise synchro (si desynchro : appuyer sur Bout1 apres message)
           BESZ0/1: L->C : 0:Arret stable balise   1:Arret balise avec redemarrage sur demande
           BLSBx  : L->C : Statut du QBLSBa Node X :Temps avant la prochaine synchro, synchro?, nb resynchro
           BLST(R): L->C : tps avant proch bal, Nb de resynchro par node et raz
           BLSA   : L->C : Temps en secondes avant la prochaine balise de resynchro
           BLRS   : L->N : Rssi du bruit (a l'arret de la periode d'�coute)
           BEPxx : L->N : chgt balise periode (10s..)
           BEFxx : L->N : chgt balise fenetre (2ms..)
           BLSY  : N->C : Demande de balise resynchro temporelle
           BESY  : C->N : ecriture Balise synchro avec temps restant

Batterie BMS
           BMESOH : Raz Soh
           BMESOHxx : Soh  xx %
           BMLSO : Lecture Soc et Soh, Tmp, VolBatt, Volt pann
           BMLST : Lecture Tmp, VolBatt, Volt pann
           BMLSH : Lecture SOH%, calend, cycle, SOH32

Capteur
           CEMPx  : activation/desactivation mode periodique (0 ou 1)
           CLMP   : lecture mode periodique
           CEMx   : mode principal capteur (0:arret, 1:temp, 2:E.Vib, 3:FFT)
           CLF0    :  Demarrage ecriture flash
           CTEEMxxx : Ecriture ecart minimum temperature (1/400 degres => 50=0,12�) max 255
           CTLEM  : Lecture ecart minimum temperature
           CECTRx : Ecriture Config Envoi TX capteur 0:pas d'envoi, 2:envoi mesure
           CECPYxxU : Ecriture Periode de mesure du capteur Y (ex 03m : 3 minutes) U=c,s,m ou h
           CLTD CLTH    : Lecture valeur de temperature(D), renvoie en float ou en int16(H)
           CTPUxxxxx : Periode de mesure unitaire (en sec)
           CLTT   : Lecture temperature et envoi en hexa avec le buffer

           CECAxy  : x:0(lumi) ou 1(temp) 2(acc) y: activation(1)/desactivation(0) du capteur
           CECTRx : Ecriture Config Envoi TX capteur 0:pas d'envoi, 2:envoi mesure
           CECPYxxU : Ecriture Periode de mesure du capteur Y (ex 03m : 3 minutes) U=c,s,m ou h
           CEram  : Ecriture de la Ram dans la flash

           CEMC : Magneto : calibration (figure en 8)
           CLMx : MAgneto : lecture valeur chaque x seconde

           CLE  : Lecture mesure compteur d'eau
           CLM  : Lecture centrale meteo
           CEExxxxx : Ecriture compteur d'eau (apres reset) en m3
           CLH  : Lecture mesure sonde Humidite sol

D : Decode un message capteur hexa en texte  QxDssssCPvvvv
        -> LQDSPjjhmsCPvv.vv vv.vv

Capteur MPU :
           M...

Entrees    EC1 : Desactivation centrale
           EC2 : Envoi au concentrateur du code tournant du node (pour synchro code)

Heure      HLH/D  : Lecture Heure/Date
           HLS    : Lecture secondes en mode hexa
           HEHhhmmss : Ecriture heure
           HEDjjmmaa : Ecriture date
           HESxxxx: Ecriture secondes en mode hexa  (si HLS moins de 10sec avant)
           HEXxxxx: Ecriture secondes en mode hexa
           HLX    : Lecture vitesse RTC (tuning quartz)
           HEXxxxxxx  modification vitesse RTC (tuning quartz)
           HET  :  Demande de mise a l'heure

Keepalive
           KPImxxxxx : periode principale du KeepA en sec (0:desactive) (max:42900)
           KPIMxxx  : multiple de la periode princ + 1 (0->1, 3->4)  (max:255)

Periodique-mode Arret
           PEMx / PLM: Ecriture/lecture mode arret (0:marche  1�3:reduit  4:arret)
           PEA1/0 : Active envoi periodique au PC (L)
           PEPxx  : Periode en seconde du message
           PETx   : Type de message : 1:temperature, 2:message RSSI (recu par Q)
           PESJ/N/S : Spot solaire duree jour(xx:0,1s - PASJ5) / duree nuit (xxxx:1s ( PESN300) / seuil (yyy:10mv - PESS20)

Radio      RLA    : lecture compteur radio (nb d'interr Radio_callback)
           RLP    : Lecture Power Tx
           RLB    : lecture index_test
           RLN    : Lecture Nb et Identification des Nodes enregistres (REN0 : raz)
           RLC    : Lecture mode RX et periode allumage Radio. Reponse : R mode=x Periode=xxU NB_repet:x temps_entre_repet:x
           RLS    : lecture statut radio
           RLTx   : Lecture statut des message supprimes et RX/TX echec
           RLUx   : Lecture mode_rx, nb mess recu, nb mess envoyes
           REMx   : Ecriture mode RX (1:continu, 4:periodique-balise,  5:periodique,    6:pas de RX
           REPxxU : Ecriture periode allumage RX Radio (en s-min-h), pour le mode=4   Ex: REP10m
           REAx   : Ecriture affiche Rssi (0 ou 1)
           RERxy  : Ecriture mode rapide : x(nb repet) y(duree en secondes entre chaque repet)
           RLR    : Envoi du dernier niv reception radio a Q
           RLRx   : ->Q : Envoi multiple de niveau de reception � x  ->x:envoi niveau reception Z
           REPxx  : Ecriture Power TX : 0 pour -10db, 1 pour 0 db, 12 pour 11db
           RENSx  : Suppression Node x dans la pile des Node
           REPAxxx : Ecriture periode activ RX Radio (en s), pour le mode=4&5   Ex: REP003

Valeur     VLRxxxx : lecture 16 valeurs RAM

Statut     SEBOx  : activation boutons  0:boutons desactives  1:B0C+B1C   2:tous actives
           SLBO   : lecture activation boutons
           SLIF/E : Lecture Index Flash/EEPROM
           SEFraz : Raz des index Ram et Flash et raz EEPROM
           SEVB   : Raz valeur voltage batterie sauvegard�
           SDEB1  : Debug1:recupe debug_time
           SEPxyy : Ecriture parametre x :  0 a FF
           SLP8x   : Lecture parametre x 8 bit
           SLPHx   : Lecture parametre 16 bit
           SLE    : Lecture erreurs
           SLV    : Lecture type appli et code version
           SLO/SEO: Lecture/ecriture statut SEObx (bit b:x)

Test       THL   : demande de balise temporelle (avec V et T)
           TVx   : Demande voltage batterie et solaire, avec d�lai x*4ms

           VLV Lecture Part number-serial-numero serie
           VLR Lecture RAM : VLRxxxx  0x0 a� 0x5000 en full word (32bits)

Webcam     WEV503  : Video/Photo x:cam_format(4-HVGA, 5-VGA, 6-SVGA), y:duree(0a4), z:envoi(1:envoi  2:SD  3:SD+envoi)
           WECx    : 0:extintion  1y:allumage y min  (WLC:lecture temps ESP restant)
           WEWxy   : Allumage wifi  x:0(stop), 1(STA), 2(AP)   y:duree (min)
           WLFxxxy lecture fichier   0-3:avi  y:5-8:jpeg
           WLA0    : Lecture statut 0:videos 1:photos
           WLSN    : Demande Init webcam
           WLDB / WEDBx : Debug PIR : 0:non 1:CC1310

Watchdog   WL      : Lecture watchdog supervision taches 1-2 (WLZ:efface)

Sorties    XE0d   : Activation LED pdt d sec
           XE0md  : Activation LED mode m pdt d sec    XE021 mode2 1 sec
Buzzer     XEBmdpd  XEB1257   mode 1  duree:0,2s /periode 3 /volume 3
PWM-spot   XSabc    XEB1233   mode 1  duree:0,2s /periode 3 /volume 3
           XLx   : Lecture consigne et duree de la sortie x
           XVnd  : Commande EVanne : d:0:arret 1:15min 2:30min 3:1h 4:1h30 5:2h 6:2h30 7:3h 8:4h 9:5h
           XVnL  : lecture statut EVanne : XVn:0 ou XVn:1
           XVPEdpD: programme vanne  (XVPL:lecture) p:periode  d:duree  D(decalage en heures/max10)

Messages longs : LYTP : T(Type:0:AR, 1:premier 2:courant 3:dernier) P:passerelle

RAS        ZR : Reset
           ZZ : Raz erreur

Surveillance du concentrateur :
    QRLN : pour voir les nodes connectes
    QBLST : pour voir le temps pour la prochaine balise et le nb de resynchro pour les modes 5
    QBLSBx : pour voir le detail par node synchronise : temps avant prochaine fenetre+nb resyncrho
    QSLIF : lecture index Flash
    QRLTx : lecture etat node (mess supp,pile,...)
    QRLUn : lecture mode, nb messages recus & envoyes
    QRLC : mode concentrator
    QRLA : compteur d'interruption et dernier message recu : last:Y-50

Surveillance du node :
    xRLR : niveau reception node + concentrateur
    xBMLST : temp�rature et niveau batterie
    xBLD : pour voir info de d�tail balise

Tests a effectuer :
message en provenance du port serie :
    texte : vers micro (rep vers port ou PC), vers port s�rie (X11HLY), vers RF(L1HLH)
    hexa : vers micro (rep vers port ou PC), vers port s�rie (X131HLY), vers RF(L1HLH)
    + A/R
message du micro interne :
    idem 8 tests
message de la liaison RF
    idem 8 tests

Envoi message long :
Uart 112kbauds 100car = 8ms
transfert 1ms
RF 50kb 100 car = 16ms
Uart_Inter(RX_buff) -> message_in(size) -> message(size) -> Envoi_message(AR) = knownNode(TXbuffer)

Taille max des messages : 123 car (en comptant le car de fin, mais hors expediteur)

 *
 */

/*#include <stdlib.h>
#include <string.h>
#include <stdarg.h>*/


#include "Board.h"
#include "comm_message.h"

#ifdef NB_UART
  UartStruct UartSt[NB_UART];
#endif

#ifdef Uart2
       void TIMEOUT_RX_Callback(TIMER_ARG pxTimer );
       void TIMEOUT_TX_Callback(TIMER_ARG pxTimer );
#endif

#ifdef ESP32
        #define nb_ligne_routage 1
        uint8_t table_routage[nb_ligne_routage][6] = {{'1','Z',3,0,0,0}};  // End device (S) -> tout vers Uart
        static const int RX_BUF_SIZE = 256;
        static QueueHandle_t uart0_queue;
        uint8_t Uart_actif;
#endif


#ifdef CC13xx
   #ifdef Uart2
      Clock_Struct TimeoutRX2;     /* not static so you can see in ROV */
      Clock_Struct TimeoutTX2;     /* not static so you can see in ROV */

   #endif
   #ifdef CONCENTRATOR // Q
      // Table de routage : {de, a, N� liaison (3-6-7), Adresse passerelle, nb reemission, puissance emission)
        #define nb_ligne_routage 4
        uint8_t table_routage[nb_ligne_routage][6] = {{'z','z',7,'Z',0,0}, {'a','z',6,0,0}, {'R','Z',6,0}, {'A','P',3,0,0,0} };  // Passerelle RF (Q)
      #define PERIOD_ACTIV    0   // Desactive l'envoi periodique regulier d'un message
      #define PERIOD_PERIODE  5   // Periode de l'envoi periodique en secondes
      #define PERIOD_TYPE     2   // Type d'action periodique : 1:temperature, 2:envoi mess RSSI
      uint8_t Uart_actif;

    #else  // NODE
        #ifdef Uart2
            #ifdef Uart2_Add  // permet de tester l'uart du node avec l'adresse S
                #define nb_ligne_routage 3  //  0->Loop  1ouS->Uart   Add+1->Uart
                uint8_t table_routage[nb_ligne_routage][6] = {{'1','1',3,0,0,0}, {'z','z',3,0,0,0}, {'2','Z',7,'Q',0,4}};  // -> tout vers passerelle RF (Q) sauf 1etS (Uart)
            #else
                //#define UART_AJOUT_EMETTEUR  '1'   // Ajoute code lors de la reception message par uart
                #define nb_ligne_routage 2  //  0->Loop  1->Uart   Add+1->Uart
                uint8_t table_routage[nb_ligne_routage][6] = {{'1','1',3,0,0,0}, {'2','z',7,'Q',0,0}};  // -> tout vers passerelle RF (Q) sauf 1 (Uart)
            #endif
            uint8_t Uart_actif;

        #else
            #define nb_ligne_routage 1  // S
            uint8_t table_routage[nb_ligne_routage][6] = {{'2','Z',7,'Q',0,4}};  // End device (S) -> tout vers passerelle RF (Q)
        #endif
        #define PERIOD_ACTIV    0   // Active l'envoi periodique r�gulier d'un message
        #define PERIOD_PERIODE  2   // Periode de l'envoi periodique en secondes
        #define PERIOD_TYPE     2   // Type d'action periodique : 1:temp�rature, 2:envoi mess RSSI

    #endif
#else
#ifdef STM32
   #define Uart0
   #define Uart1
   #define Uart2
#endif
#endif


// -------------------  CONFIG  -------------------------



// ------------------------------------ Fonctions externes ---------------------------------------------------


// raz_init_communication(void)
// raz des liaisons de comm

// void reception_transmission_message(void)
// fonction appelée toutes les 2ms (ou en continu) pour envoyer un message et traiter les messages reçus

// void envoi_message (void)   // rajoute l'emetteur et CRC à message[] -> message_temp[], calcule longueur, insère nouveau message dans pile d'envoi
// Envoi message texte contenu dans message[] (terminé par car_fin_trame ou 00)

// void envoi_message_def (void) // rajoute l'emetteur, 0x80 et CRC à message[] -> message_temp[], augmente longueur, P1ZZ
// Envoi message hexa (longueur définie au 2° octet)

//  void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
// Fonction appelée en Interruption, à la fin de la transmission du message


//  ---------------   CONFIGURATION UTILISATEUR ------------------------*/



     /* modif adresse node => modifier aussi :
     Comm_message.h  ligne  22 (adresse node)
     RadioProtocol.h ligne  20�50 (données du node)*/


     // -----------------  CONFIGURATION DIVERSES -----------------------------




#define TIMEOUT_rx_ct  10000     // timeout au bout de 10 secondes (uart, spi, i2c) si dernier car pas reçu
#define TIMEOUT_tx_ct  5000     // timeout au bout de 5 secondes (uart, spi, i2c) si envoi bloqué

#define Dump_ram_max 4   // Nb de mots de 4 octets par ligne,   envoyés avec la fonction VLP (Lecture RAM partielle)


 // Nb de secondes pour enregistrement index flash en eeprom
 // 0 pour enregistrement imm�diat
 #define ENREG_INDEX_FLASH_W   0   // apres enreg flash TODO a d�placer
 #define ENREG_INDEX_FLASH_R   0   // suppression flash

//  -------------------------------------   Codes erreur  ----------------------------------------code_erreurs


    // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
    // Erreurs 20 a 7F : 4 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
    // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

     // Envoye 4 fois :

     #define erreur_RX_full         0x20
     #define erreur_RX_buff_vide    0x21
     #define erreur_RX_pas_fin      0x22
     #define erreur_RX_framing      0x23
     #define timeout_RX             0x24
     #define erreur_TX_empty        0x25
     #define erreur_TX_full         0x26
     #define erreur_TX_pas_fin      0x27
     #define erreur_TX_DMA          0x28
     #define erreur_mess_tab        0x29 // +2
     #define erreur_mess_decod_hexa 0x2C
     #define timeout_TX             0x2D
     #define erreur_rtos            0x2E
     #define erreur_mess            0x2F
     #define erreur_uart            0x30
     #define erreur_messageIn       0x31




//------------------------------------Define------------------------------------------------------


// -----------------------------------Global Constants--------------------------------------------



// ------------------------------------Global Variables--------------------------------------------------

uint8_t message_test[10];

// message longs
uint8_t send_long_actif, send_long_repeat, send_long_buff_suiv, send_long_buffer_tx, send_long_port_emett, send_long_dest;
uint8_t send_long_attente, send_long_emett, send_long_port_dest, send_long_emett, send_long_buffer_suiv, send_long_pile;
uint16_t send_long_num_trame, send_long_num_trame_tx, send_long_autre;

uint8_t message_in[MESSAGE_SIZE];
uint8_t longueur_message_in;
uint8_t message[MESSAGE_SIZE];
uint8_t flag_comm_transfert;
uint8_t dump_ram_fin, dump_ram_timer;

struct RemoteNode knownNodes[NB_DESTINATAIRES_NODE];

uint8_t mess_ok;

uint32_t code_tournant_prec;

// Variable compteur
   uint8_t par8_var[PAR_VAR_MAX];
   uint16_t par16_var[PAR_VAR_MAX];


uint8_t StatutOps;  // statut operationnel par bit

// ------------------------------------Local Variables--------------------------------------------------


/* static Task_Params CommTaskParams;
 Task_Struct CommTask;    // not static so you can see in ROV
 static uint8_t CommTaskStack[stack_size_comm];*/

uint8_t Mess_statut;  // bit0:dernier  bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)
uint8_t  rxrf_numero_device;  // 0:pas de device  1:device 0 etc...


uint8_t  code_tournant_timer;  // random
uint32_t code_tournant_time1;  // premi�re datation + random

uint8_t period_activ, period_periode, period_type;   // variables pour envoi regulier

#ifdef CC13xx
// Clock pour execution d'action periodiques
Clock_Struct PeriodiqueClock;     /* not static so you can see in ROV */
static Clock_Handle PeriodiqueClockHandle;

/* Timer pour envoi message long */
Clock_Struct Timer_Envoi_Long;     /* not static so you can see in ROV */
static Clock_Handle xTimer_envoi;
#endif

#ifdef CC13xx
Error_Block eb;
Task_Handle Uart2_handle;

Semaphore_Struct MessageInSem;  /* not static so you can see in ROV */
Semaphore_Handle MessageInSemHandle;
#else
SemaphoreHandle_t MessageInSemHandle; 

#endif


#ifdef FLASH_EXTERNE
    struct Pile_Fl_S Pile_F[NB_PILES_F];
    extern uint16_t index_EEPROM;

    /* Timer pour enregistrement EEPROM */
    Clock_Struct Timer_EEPROM_Clock;     /* not static so you can see in ROV */
    static Clock_Handle TimerEEPROMHandle;

    // Semaphore
    Semaphore_Struct FlashAccessSem;  /* not static so you can see in ROV */
    Semaphore_Handle FlashAccessSemHandle;

#endif


 // ------------------------------ Extern -------------------------------------------------------------

 extern uint32_t  validite;

#ifdef CC13xx
 extern struct RemoteNode knownNodes[];
 extern uint8_t duree_fenetre_RX;  // (mode 4,5), en milliseconde
// extern uint8_t duree_fenetre_RX;  // (mode 4,5), en milliseconde
 extern uint8_t periode_activ_RX;  // (mode 4,5), en seconde
 extern uint8_t periode_balise_synchro; // mode 5 : periode en minutes
 extern struct Synchro_S Synchro;
#endif


 extern  uint8_t  comptage_erreur, enr_erreur[];


 extern uint16_t volt_mesure_bas;


#ifdef BROAD_BALISE_SYNCHRO
    extern uint8_t synchro_actif;
#endif

// ------------------------------------Local Prototypes---------------------------------------------------

 void dump_xdata_1_ligne(uint32_t * pregistre_int_xdata);
 void reset (void);
 uint8_t transf_buff (uint8_t);

 void modif_affiche_Rssi(uint8_t valeur);
 void TimerButtonCallback(UArg arg0);
 void PeriodiqueCallback(UArg arg0);
 void Uart_sleepTimerCallBack(UArg arg0);

 void TIMEOUT_M_long_Callback (UArg arg0);

 void Timer_EEPROM_Callback(UArg arg0);
 void Timer_EEPROM_Callback_task(void);



// UART 2
#ifdef Uart2


#ifdef CC13xx
    Task_Struct RxUart2Task;    /* not static so you can see in ROV */
    #if Uart2 == 0
       void Recep_Uart2 (UArg arg0, UArg arg1);
    #else
       static Void RxUart2Task_Ptr(UArg arg0);
    #endif
    static uint8_t RxUart2TaskStack[512];
#endif
#ifdef ESP32
    void Recep_Uart2(void *pvParameter);
    static TaskHandle_t Uart2Task;
#endif

  uint8_t envoi_Uart2 (uint8_t);
  void raz_init_Uart2(void);
  void Uart2_RX (uint16_t);
  void transmission_Uart2 (void);
  uint8_t Uart2_receive (uint8_t* data, bool type);

  uint8_t  flag_trame_rx_Uart2, tx_Uart2_actif, message_uart2_en_cours;
  uint8_t  Uart2_rx_buff[MESSAGE_SIZE+5];
  uint8_t  Uart2_tx_buff[TX_BUFFER_SIZE];
  uint8_t  Uart2_rx_head, Uart2_rx_tail, Uart2_tx_head, Uart2_tx_tail;
  uint8_t  Uart2_rx_type_trame;
  uint8_t  Uart2_rx_nb_car_recu;
  uint8_t  Uart2_rx_longu;
  uint8_t  Uart2_rx_debut_trame, Uart2_rx_attente;
#endif

void raz_Uart(uint8_t num_uart);

// RF
#ifdef CC13xx
    void raz_init_RF(void);
     void raz_RF(void);
#endif

#ifdef CC13xx
   UART_Handle hUart2;
#endif
#ifdef ESP32
  #define hUart2 0
#endif




//  -------------------------------------- Debut fonctions ------------------------------------------

 // initialise les taches de communication UART

#ifdef Uart2

void Uart2_task_create()
{
    #ifdef CC13xx
        static Task_Params RxUartTaskParams;

        Task_Params_init(&RxUartTaskParams);
        RxUartTaskParams.stackSize = 512;  // max : 244
        RxUartTaskParams.priority = 3;  // 4
        RxUartTaskParams.stack = &RxUart2TaskStack;
        RxUartTaskParams.instance->name = "RxUart2";

        #if Uart2 == 0
            // allocation statique
            Task_construct(&RxUart2Task, Recep_Uart2, &RxUartTaskParams, NULL);
        #else
            // allocation dynamique
            Uart2_handle = Task_create((Task_FuncPtr)RxUart2Task_Ptr, &RxUartTaskParams, &eb);
            if (Uart2_handle == NULL)
                {
                code_erreur = erreur_uart;
                //System_abort("Task create failed");
                }
        #endif
    #endif
}
#endif

void tache_comm_init(void)
  {

    raz_erreur ();
    send_long_actif=0;



    #ifdef CC13xx
    // Create semaphore pour l'acces unique a message_in
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    semParam.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&MessageInSem, 1, &semParam);
    MessageInSemHandle = Semaphore_handle(&MessageInSem);
    //Semaphore_Handle_label(FlashAccessSemHandle, (xdc_runtime_Types_Label *) "flash");

      // envoi periodique de messages
      Clock_Params clkParams;
      Clock_Params_init (&clkParams);

      /* setup clock periodique pour envoi periodique de messages */
      clkParams.period = period_periode * 1000 * (1000 / Clock_tickPeriod);
      clkParams.startFlag = FALSE;
      Clock_construct(&PeriodiqueClock, PeriodiqueCallback, 1, &clkParams);
      PeriodiqueClockHandle = Clock_handle(&PeriodiqueClock);

      Maj_clock_periodique(); // demarre la clock si necessaire

      period_activ    = PERIOD_ACTIV;
      period_periode  = PERIOD_PERIODE;
      period_type     = PERIOD_TYPE;
    #else
      MessageInSemHandle = xSemaphoreCreateBinary();
      SEMAPHORE_GIVE (MessageInSemHandle);
    #endif


     // code tournant telecommande
     code_tournant_prec = 0;

     #ifdef Uart2

     // Create a UART with data processing off.
      #if Uart2 == 0  // si c'est 1, on ne l'ouvre qu'en cas de besoin
          Uart2_Open();
          Uart_actif=1;
      #endif


       // Create Uart Recept Task & Timer
       #ifdef ESP32
           xTaskCreate(Recep_Uart2, "RxUart2", 2048*2, NULL, 3, Uart2Task);
           UartSt[0].h_timeout_RX = xTimerCreate("TimeoutRX2", (300), pdFALSE, ( void * ) 0, TIMEOUT_RX_Callback);  // name,period-tick, autoreload,id, callback
           UartSt[0].h_timeout_TX = xTimerCreate("TimeoutTX2", (300), pdFALSE, ( void * ) 0, TIMEOUT_TX_Callback);  // name,period-tick, autoreload,id, callback
       #endif

       #ifdef CC13xx

         // Cree la tache continue de reception interruption Uart
         Error_init(&eb);

         #if Uart2 == 0   // mode continu
         Uart2_task_create();
         #endif

         clkParams.period = 0;
         clkParams.startFlag = FALSE;
         clkParams.arg = 0;
         Clock_construct(&TimeoutRX2, TIMEOUT_RX_Callback, 1, &clkParams);
         UartSt[0].h_timeout_RX = Clock_handle(&TimeoutRX2);
         Clock_construct(&TimeoutTX2, TIMEOUT_TX_Callback, 1, &clkParams);
         UartSt[0].h_timeout_TX = Clock_handle(&TimeoutTX2);

         // messages longs
         clkParams.period = M_LONG_PERIODE  * (1000 / Clock_tickPeriod);
         Clock_construct(&Timer_Envoi_Long, TIMEOUT_M_long_Callback, clkParams.period, &clkParams);
         xTimer_envoi = Clock_handle(&Timer_Envoi_Long);

       #endif



       raz_init_Uart2();

     #endif

     #ifdef CC13xx
        raz_init_RF();
     #endif

        // definition des 256 sections en Flash : chacune fait 4096 octets
       #ifdef FLASH_EXTERNE
           #ifdef CONCENTRATOR
                // 1�re : messages pour L, 2�:enreg divers , ensuite pour Node
                // Taille : minimum 2, max 256 - debut-longueur
                char unsigned table_pile_f[NB_PILES_F][2] = { {0,90},{90,10} }; //, {20,2},{22,2},{24,2},{26,2},{28,2},{30,2},{32,2},{34,2} };
           #else  // NODE
                // 1 seule pile RF : pour messages vers Concentrateur
                const uint8_t table_pile_f[NB_PILES_F][2] = { {0,200} };
           #endif

          uint8_t i;
          for (i=0; i<NB_PILES_F; i++)
          {
              Pile_F[i].debut = table_pile_f[i][0];
              Pile_F[i].taille = table_pile_f[i][1];
              Pile_F[i].head = 0;
              Pile_F[i].tail = 0;
          }
          // Create semaphore pour l'acces unique a la flash externe
          Semaphore_Params_init(&semParam);
          semParam.mode = Semaphore_Mode_BINARY;
          Semaphore_construct(&FlashAccessSem, 1, &semParam);
          FlashAccessSemHandle = Semaphore_handle(&FlashAccessSem);
          //Semaphore_Handle_label(FlashAccessSemHandle, (xdc_runtime_Types_Label *) "flash");

        #ifdef CC13xx
          // setup timer enregistrement EEPROM 10 secondes
          clkParams.period = 0; // Timer. Period=0
          clkParams.startFlag = FALSE;
          Clock_construct(&Timer_EEPROM_Clock, Timer_EEPROM_Callback, ENREG_INDEX_FLASH_W*1000/Clock_tickPeriod*1000, &clkParams);
          TimerEEPROMHandle = Clock_handle(&Timer_EEPROM_Clock);
        #endif
     #endif
  }


// Activation de l'uart, si elle etait en sommeil
void Activ_Uart2(void)
{
    #if Uart2 == 1   // mode reveil/sleep
      if (!Uart_actif)
      {
          Uart2_Open();
          Uart_actif = 1;
          Uart2_task_create();
          //DELAY_MS(300);
      }
    #endif
}


#ifdef ESP32
void TIMEOUT_RX_Callback(TIMER_ARG pxTimer )
{
    uint8_t num_uart = (int32_t) pvTimerGetTimerID( pxTimer );
#endif
#ifdef CC13xx
void TIMEOUT_RX_Callback(TIMER_ARG num_uart )
{
#endif
    code_erreur=timeout_RX;   //timeout apres 1 car Recu
    err_donnee1= num_uart+'0';
    raz_Uart(num_uart);
}

#ifdef ESP32
void TIMEOUT_TX_Callback(TIMER_ARG pxTimer )
{
    uint8_t num_uart = (int32_t) pvTimerGetTimerID( pxTimer );
#endif
#ifdef CC13xx
void TIMEOUT_TX_Callback(TIMER_ARG num_uart )
{
#endif
    code_erreur=timeout_TX;   //timeout entre 2 car envoyes
    err_donnee1= num_uart+'0';
    raz_Uart(num_uart);
}

#ifdef ESP32
void Com_stack (void)
{
     printf(" Comm stack %i\n",  uxTaskGetStackHighWaterMark(Uart2Task));
}
#endif

/* Repete la meme trame du message long
void TIMEOUT_M_long_Callback(TIMER_ARG pxTimer )
{
    if (send_long_repeat < 3 )  // repete 3 fois
    {
      send_long_repeat++;
      xTaskNotify( AppliTask, EVENT_TRANSF, eSetBits ); // re-envoi meme trame
    }
    else  send_fin_trans(); // annulation envoi
}*/

#ifdef CC13xx
void TIMEOUT_M_long_Callback(TIMER_ARG num_uart )
{
   /* if (send_long_repeat < 3 )  // repete 3 fois
    {
      send_long_repeat++;
      timeout_mess_long();
    }
    else*/
    { // annulation envoi
        CLOCK_STOP( xTimer_envoi);
        send_long_actif=0;
    }
}
#endif



#ifdef Uart2

void Uart2_Open(void)
{
      #ifdef CC13xx
      UART_Params uartParams;
      //  const char  echoPrompt[] = "Echoing characters:\r\n";

      //hDisplaySerial = Display_open(Display_Type_UART, &params);
       UART_Params_init(&uartParams);
       uartParams.writeDataMode = UART_DATA_BINARY;
       uartParams.readDataMode = UART_DATA_BINARY;
       uartParams.readReturnMode = UART_RETURN_FULL;
       uartParams.readEcho = UART_ECHO_OFF;
       uartParams.baudRate = 57600;
       #ifdef C10_CAMERA_PIR   // uart 115kbauds
          uartParams.baudRate = 115200;
       #endif
       //uartParams.readMode = UART_MODE_CALLBACK;
       //uartParams.readCallback = &uart_callback_read;

       hUart2 = UART_open(Board_UART0, &uartParams);

       if (hUart2 == NULL) {
           // UART_open() failed
           code_erreur = erreur_uart;
           err_donnee1 = '2';
       }
       #endif

       #ifdef ESP32
        uart_config_t uart_config = {
          .baud_rate = 115200,
          .data_bits = UART_DATA_8_BITS,
          .parity = UART_PARITY_DISABLE,
          .stop_bits = UART_STOP_BITS_1,
          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        // Configure UART parameters
        esp_err_t errU;
        errU =  uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 20, &uart0_queue, 0);
        errU += uart_param_config(UART_NUM, &uart_config);
        errU += uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (errU)
        {
          code_erreur = erreur_uart;
          err_donnee1 = errU;
        }
         /*   ESP_ERROR_CHECK(uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 20, &uart0_queue, 0));
            ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
            ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));*/

        //Set uart pattern detect function.
        //uart_enable_pattern_det_baud_intr(UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
        //Reset the pattern queue length to record at most 20 pattern positions.
        //uart_pattern_queue_reset(UART_NUM, 20);
      #endif
       //c = 0xA4;
       //rxbyte = UART_read(hUart2, &c, 1);

       //UART_write(hUart2, echoPrompt, 4);  //sizeof(echoPrompt));

}

void Uart2_close(void)
{
   #ifdef CC13xx
       #if Uart2 == 1
           Task_delete(&Uart2_handle);
       #endif
       UART_close(hUart2);
    #endif
    #ifdef ESP32
       uart_driver_delete(hUart2);
    #endif
}

// Interruption
/* Reception d'un car de l'Uart -> pile rx2_buff , et flag flag_trame_rx_uart2  lorsque la trame reçue est complète
a 112kbauds : 107 caract re�us en 11,3ms (75kbauds reels)
renvoi au concentrateur : 3ms
enregistrement en ram : 45ms (si raz page flash)
*/
#ifdef CC13xx
#if Uart2 == 0
  void Recep_Uart2 (UArg arg0, UArg arg1)
#else
  static Void RxUart2Task_Ptr(UArg arg0)
#endif

#endif
#ifdef ESP32
  void Recep_Uart2(void *pvParameter)
#endif
{
  uint8_t c, free;
  int rxBytes;

  DELAY_MS (300);

  while (1)
  {
      // Begin read
      #ifdef CC13xx
          rxBytes = UART_read(hUart2, &c, 1);
      #endif
      #ifdef ESP32
          rxBytes = uart_read_bytes(UART_NUM, &c, 1, (portTickType)portMAX_DELAY);  // timeout 1 sec
      #endif
      //Semaphore_pend(UartRxSemHandle, BIOS_WAIT_FOREVER);

      if (rxBytes >= 0)
      {
          // Calcul du nb de places dispo dans le buffer (en laisser au moins 1)
          if (Uart2_rx_head < Uart2_rx_tail)
               free =  Uart2_rx_tail - Uart2_rx_head ;
          else
               free = (MESSAGE_SIZE+5) - Uart2_rx_head + Uart2_rx_tail;

          if (free < (2) )
          {
             code_erreur = erreur_RX_full;   // Rx_buffer full
             //suppression message et attente jusqu'a la fin de ce message
             Uart2_rx_head = Uart2_rx_debut_trame;
             Uart2_rx_attente=1;
             //raz_Uart(0);
          }
          if ((( (c&0x7F)>0x1F ) && ((c&0x7F) < 0x7B)) || (Uart2_rx_nb_car_recu))   // ne prend pas 0x0A ni 0xFF en premier caractere
          {
              uint8_t debu_rx = Uart2_rx_head;
              if (!Uart2_rx_attente)
              {
                  Uart2_rx_buff[Uart2_rx_head++] = c; /* copy over this byte of data dans le buffer */
                  if( Uart2_rx_head == (MESSAGE_SIZE+5) ) /* if wrapping around */
                       Uart2_rx_head = 0; /* reset pointer */
              }
              Uart2_rx_nb_car_recu ++;
              CLOCK_RESTART_TIMEOUT (UartSt[0].h_timeout_RX, TIMEOUT_rx_ct)  // timeout au bout de x secondes si on ne recoit pas la fin du message
              if (Uart2_rx_nb_car_recu==1)
              {
                  Uart2_rx_debut_trame = debu_rx;
                  #ifdef DEBUG_TIME
                      etat_cpu[index_time] = 0x31;  // phase reception courte  TODO
                      etat_detail[index_time] = 0;
                      time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                      if (index_time >MAX_Index) index_time=0;
                  #endif
                  Uart2_rx_type_trame = c & 0x80;  // enregistre le type de trame (hexa ou texte)
                  Uart2_rx_longu = 10;
              }
             #ifdef UART_AJOUT_EMETTEUR
                  if (Uart2_rx_nb_car_recu==2)
                      Uart2_rx_longu = (c & 0x7F) - 1;   // enregistre la longueur de trame pour message hexa
             #else
                  if (Uart2_rx_nb_car_recu==3)
                      Uart2_rx_longu = c & 0x7F;   // enregistre la longueur de trame pour message hexa
              #endif
              // Fin de trame
              if (Uart2_rx_type_trame)
              {
                  // Trame hexa
                 if (Uart2_rx_nb_car_recu >= (Uart2_rx_longu+4))
                 {
                  #ifdef DEBUG_TIME
                     etat_cpu[index_time] = 0x32;  // phase reception courte  TODO
                     etat_detail[index_time] = Uart2_rx_nb_car_recu;
                     time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                     if (index_time >MAX_Index) index_time=0;
                  #endif
                     Uart2_rx_nb_car_recu = 0;   // A REVOIR
                     CLOCK_STOP(UartSt[0].h_timeout_RX);  // raz timeout
                     if (!Uart2_rx_attente)
                     {
                         flag_trame_rx_Uart2++;  // nb de car recu => flag pour fin de trame
                         Task_message_recu();
                     }
                     Uart2_rx_attente = 0;
                 }
              }
              else   // Trame texte
              {
                 if ((c == car_fin_trame) || (c == 0xA))  // fin de trame => flag pour reception trame
                 {
                  #ifdef DEBUG_TIME
                     etat_cpu[index_time] = 0x32;  // phase reception courte  TODO
                     etat_detail[index_time] = Uart2_rx_nb_car_recu;
                     time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                     if (index_time >MAX_Index) index_time=0;
                  #endif

                     Uart2_rx_nb_car_recu = 0;
                     CLOCK_STOP(UartSt[0].h_timeout_RX);  // raz timeout
                     if (!Uart2_rx_attente)
                     {
                        flag_trame_rx_Uart2++;
                        Task_message_recu();
                     }
                     Uart2_rx_attente = 0;
                 }
              }
          }
       }
   }
 }


// Tache appli : TODO attendre que message_in soit libre (CC:30us) ou mettre en queue
void reception_message_Uart2(void)
{
    uint8_t long_mess, retc;

    if (flag_trame_rx_Uart2)
    {            // Reception message Uart2
        do
        {
          #ifdef DEBUG_TIME
            etat_cpu[index_time] = 0x33;  // phase reception courte  TODO
            etat_detail[index_time] = 0;
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
          #endif
            retc = SEMAPHORE_TAKE(MessageInSemHandle, 20); //   20 ms (correspondant a 1 message de 100 car 50kbps)

          #ifdef DEBUG_TIME
            etat_cpu[index_time] = 0x34;  // phase reception courte  TODO
            etat_detail[index_time] = 0;
            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
            if (index_time >MAX_Index) index_time=0;
          #endif

            if (retc)  // 1:ok
            {
                long_mess = Uart2_receive (message_in, 1); // QLHLH -> 5  QL1HLH->5
                if (long_mess) {
                    #ifdef NODE
                        Mess_statut = 0x00;  // 22:Ack/RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                    #endif
                    //ESP_LOGW(TAG, "\nmessage recu:%i %s\n", long_mess, message_in);
                    traitement_rx(long_mess);
                }
                #ifdef DEBUG_TIME
                etat_cpu[index_time] = 0x35;  // phase reception courte  TODO
                etat_detail[index_time] = 0;
                time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                if (index_time >MAX_Index) index_time=0;
                #endif

                SEMAPHORE_GIVE( MessageInSemHandle);
            }
            else // suppression message
            {
                Uart2_receive (message_in, 0);
                code_erreur = erreur_messageIn;
                err_donnee1 = '1';
            }
        }
        while ((Uart2_rx_tail != Uart2_rx_head ) && (flag_trame_rx_Uart2));
   }
}

void test_err(uint8_t code)
{
    strcpy((char *)message, "LWWxx xx y");  // HEShhhh
    message[3] = deci ( LOW_BYTE(message_test[0]) >> 4);
    message[4] = deci (LOW_BYTE(message_test[0]) & 0x0F);
    message[6] = deci ( LOW_BYTE(message_test[2]) >> 4);
    message[7] = deci (LOW_BYTE(message_test[2]) & 0x0F);
    message[9] = code + '0';
    message_test[0] = 0;
    message_test[1] = 0;
    message_test[2] = 0;
    message_test[3] = 0;
    envoi_message();
}

//type : 1:reel  0:factice
uint8_t Uart2_receive (uint8_t* data, bool type)
// Transfert message de la pile Uart2_rx_buff vers message_in  PZZx  => 3
// retourne : longueur (sans car fin, avec emetteur)
{
  uint8_t c, count, type_trame, i1,longueur_message;
  #ifdef C10_CAMERA_PIR   // Uart reorientation messages
    uint8_t destin;
  #endif

  count = 0;
  /* if the buffer is empty  */
  if( Uart2_rx_tail == Uart2_rx_head ) code_erreur = erreur_RX_buff_vide;
  else
  {
      type_trame = Uart2_rx_buff[Uart2_rx_tail] & 0x80;
      i1=1;
      longueur_message=10;
      do /* retrieve the frame/data from the buffer */
      {
         c = Uart2_rx_buff[Uart2_rx_tail++]; /* copy data from buffer */
         if( Uart2_rx_tail == (MESSAGE_SIZE+5) )
             Uart2_rx_tail = 0;   /* if wrapping around  reset pointer */
         if(count == 2)
             longueur_message = ( c & 0x7F) + 3;
         if (type_trame)
          {
            if (count >= longueur_message) i1=0;
          }
          else
          {
            if ((c==car_fin_trame) || (c==0xA))
            {
                i1=0;
                c = car_fin_trame;
            }
          }
          count++;
          if (count > MESSAGE_SIZE )
          {
            code_erreur = erreur_RX_pas_fin;  // erreur : pas de fin, ca boucle
            i1=0;
            c = car_fin_trame;
          }
          if (count==1)  // Premier caractere : destinataire
          {
              if ((c & 0x7F)=='1')  // destinataire 1 => my_adress  1yyy -> Xyyy
                  c= My_Address + (c & 0x80);
          }
          #ifdef UART_AJOUT_EMETTEUR
            if (count==2)  // 2°car : ajout emetteur 1yyy->X1yyy   Xyyy->X1yyy   Qyyy->QXyyy
            {
               /* if ((UART_AJOUT_EMETTEUR=='1') && (((message_in[0] & 0x7F) != My_Address) && ((message_in[0] & 0x7F) !='1')))
                    *data++ = My_Address | 0x80; //
                else*/
                    *data++ = UART_AJOUT_EMETTEUR;
                count++;
            }
          #endif

            #ifdef C10_CAMERA_PIR   // Uart reorientation messages
            //  reception uart:reoriente vers L des messages de l'ESP32 qui commencent par 1B et 5B
            if (count == 1)
                  destin = c & 0x7F;
            if (count == 2)  //
            {
                if ( c != '1' )   // message warning ou erreur de l'ESP32
                {
                    if (type)
                    {
                        *data = My_Address;  // L My C1 C2
                        data--;
                        uint8_t temp = *data;
                        *data = 'L';
                        data = data + 2;
                        *data++ = temp;
                        count = count+2;
                    }
                }
                else  // message normal : Remplace emetteur par My_address+0x80 quand ca vient du port serie
                {
                    if (destin == 'L')   // Permet d'avoir un message lisible pour L
                        c = My_Address | 0x80;
                    else if (destin != My_Address)
                        c = My_Address | 0x80;
                }
            }
          #endif
          if (type)
              *data++ = c;
        } while ( i1 );
  }


/*  if (message_test[1])  // TODO
  {
      message_test[2]++;
      if ((message_in[0]==0xCC) && (message_in[1]==0xE1)  && (message_in[3]=='Y'))
      {
          if ( (message_in[4]==1))
          {
              if ((message_in[2]!=0x67) || (count !=107))
                  test_err(2);
          }
          if ( (message_in[4]==2))
              message_test[1]=0;
      }
      else
          test_err(3);
  }*/


   if (flag_trame_rx_Uart2) flag_trame_rx_Uart2-- ;  // BSP_CRITICAL_STATEMENT
  if (count) count--;
  return count; /* indicate the number of bytes retrieved from the buffer, sans car fin trame */
}
#endif



#ifdef FLASH_EXTERNE

 // 0:ok  1:erreur
 uint8_t enreg_message_flash(uint8_t *buf, uint8_t section)
{
    uint8_t longu, retc, secteur, i;
    uint32_t free, index_head;

    retc = 1;  // erreur
    // ajout de la longueur pair au d�but (avec decalage)
    longu = buf[0];  // 2
    if (longu)
    {
        for (i=longu; i; i--)
            buf[i] = buf[i-1];
        longu++;
        longu += (longu & 1); // pair superieur
        buf[0] = longu;

        if ((section >= NB_PILES_F) || (!longu) || (longu>TAILLE_MAX_MESS_RF))
            return true;  // erreur

        index_head = Pile_F[section].head;

        // verif qu'il y a assez de place
        /* calcul places libres dans pile */
        if (Pile_F[section].head < Pile_F[section].tail)
           free =  Pile_F[section].tail - Pile_F[section].head ;
        else
           free = (((uint32_t)Pile_F[section].taille<<12) - index_head) + Pile_F[section].tail;

        if ( free <= (uint32_t)longu+4096 )  // pour eviter d'effacer un secteur encore en utilisation par la queue de la pile
        {
            if ( free <= 2000 )  // pb de tail passe devant head
            {
                // reset index pile
                Pile_F[section].head -= (Pile_F[section].head & 1);  // pair
                Pile_F[section].tail = Pile_F[section].head;  // raz pile flash
                sauve_eeprom_index_flash();
            }
            return true;   // buffer full : erreur
        }
        else
        {   // transfert data
            /* Wait for semaphore */
            retc = Semaphore_pend(FlashAccessSemHandle, 1000*100); // Semaphore_WAIT_FOREVER   1 seconde
            if (retc)  // 1:ok
            {
                ExtFlash_open();              // Transfert message vers Flash

                // secteur de fin de message
                secteur = (index_head + longu -1) / 4096;
                if ( ( (index_head + longu -1) % 4096)  < (uint32_t)longu )  // Raz nouveau secteur
                {
                    if (secteur >= Pile_F[section].taille)
                        secteur -= Pile_F[section].taille;
                    ExtFlash_erase( (uint32_t)(secteur+Pile_F[section].debut) << 12, 4096);  // 256 secteurs de 4096 octets   80ms 9mA
                }
                // enregistrement (0:erreur, 1:ok)  // 120us
                retc = ExtFlash_write( index_head ,  longu, (const uint8_t *) buf, section);

                // lecture apres enregistrement pour verifier
                //uint8_t mess_relu[256];
                //retc += ExtFlash_read( index_flash_head , nb, mess_relu, 0) ;
                //ret = strncmp((const char *)mess_relu, (const char *)valeur, nb);
                //if ((retc!=2) || (ret))
                 //   System_printf("err enreg flash\n");

                ExtFlash_close();
                Semaphore_post(FlashAccessSemHandle);

                index_head += longu;
                if (index_head >= ((uint32_t)Pile_F[section].taille<<12))
                        index_head -= ((uint32_t)Pile_F[section].taille<<12);
                Pile_F[section].head  = index_head;

                // enregistrement des index en EEPROM : immediat ou differe
                if (ENREG_INDEX_FLASH_W)
                {
                    if (!Clock_isActive(TimerEEPROMHandle))
                    {
                        CLOCK_RESTART_TIMEOUT(TimerEEPROMHandle, ENREG_INDEX_FLASH_W * 1000);  // millisecondes
                    }
                }
                else  // immediat
                    sauve_eeprom_index_flash();  // 2,2ms

            }
            retc = 1 - retc;  // => 0:ok  1:erreur
        }
    }
    return retc;
}

// mode:0:lecture sans supp,  1:suppression  2:lecture et suppression
// return : 0:erreur 1:ok  2:ok et dernier message
uint8_t lecture_message_flash(uint8_t *buf, uint8_t section, uint8_t mode, uint8_t Lmax)
{
    uint8_t longueur_flash, valide, retc, retd;
    uint32_t index_tail;
    uint8_t buff_fl[4];
    uint8_t retc1, wrap;
    uint32_t index_temp;

    retd=0;
    valide=0;

    longueur_flash = 0;

    if (section >= NB_PILES_F)
        return 0;

    index_tail = Pile_F[section].tail;

    /* Wait for semaphore */
    retc = Semaphore_pend(FlashAccessSemHandle, 1000*100); // Semaphore_WAIT_FOREVER   1 seconde

    if (retc)  //1:ok
    {
        //DELAY_MS(1);
        ExtFlash_open();
        retc = ExtFlash_read( index_tail , 4, buff_fl, section) ; // 2 devraient suffire : long paire, long

        // verifie que l'octet 1(longueur message) = octet 0 (longueur paire en flash) moins 1
        longueur_flash = buff_fl[0];  // ex 16  ou 18
        uint8_t long_temp_f = buff_fl[1] + 1;  // ex 15  ou 16  => 16 ou 17
        long_temp_f += (long_temp_f & 1);  // 16 ou 18
        if ((retc) && (longueur_flash<=Lmax) && (longueur_flash) && (!(longueur_flash%2))
                && (!(index_tail%2)) && (longueur_flash == long_temp_f))
        {
            // verifie si longueur ne depasse pas head
            // retc1 : 0:erreur  1:ok  2:dernier

            retc1 = 0;
            wrap = 0;
            index_temp = index_tail + longueur_flash;
            if (index_temp >= ((uint32_t)Pile_F[section].taille<<12))
            {
                    index_temp -= ((uint32_t)Pile_F[section].taille<<12);
                    wrap=1;
            }
            if (index_temp == Pile_F[section].head)
                retc1=2;  // dernier message
            else
            {
                if (index_tail <= Pile_F[section].head)
                {
                    if ((!wrap) && ( index_temp <  Pile_F[section].head))
                        retc1=1;   // 1:ok
                }
                else
                {
                    if (wrap)
                    {
                        if ( index_temp <  Pile_F[section].head)
                            retc1=1;
                    }
                    else
                        retc1=1;
                }
            }

            if (retc1)
            {
                retc=1;
                if (mode!=1)  // 1=pas de lecture
                    retc = ExtFlash_read( index_tail+1, longueur_flash, buf, section) ;
                if (retc)
                    valide=1;
            }
        }

        ExtFlash_close();
        Semaphore_post(FlashAccessSemHandle);
    }

    if (valide)
    {       // valide
        retd = retc1;
        if (mode) // mode 1 ou 2
        // suppression de ce message en flash
        {
            Pile_F[section].tail = index_temp;  // supp_message_flash (ou temp)

            // enregistrement des index en EEPROM : imm�diat ou diff�r�
            if (ENREG_INDEX_FLASH_R)
            {
                if (!Clock_isActive(TimerEEPROMHandle))
                {
                    CLOCK_RESTART_TIMEOUT(TimerEEPROMHandle, ENREG_INDEX_FLASH_R * 1000);  // millisecondes
                }
            }
            else  // imm�diat
                sauve_eeprom_index_flash();
        }
    }
    else  // erreur
    {
        retd=0;
        Pile_F[section].head -= (Pile_F[section].head & 1);  // pair
        Pile_F[section].tail = Pile_F[section].head;  // raz pile flash
        sauve_eeprom_index_flash();
    }
    return retd;  // 0:erreur  1:ok  2:ok mais dernier
}

// suppression d'un message flash, en utilisant sa longueur
// return : 0:erreur 1:ok
// avance au message suivant
bool supp_message_flash(uint8_t longu, uint8_t section)
{
    // suppression de ce message en flash
    uint32_t index_tail;

    if ((section >= NB_PILES_F) || (!longu) || (longu&1) || (longu>TAILLE_MAX_MESS_RF))
        return false;

    index_tail = Pile_F[section].tail + longu;
    if (index_tail  >= ((uint32_t)Pile_F[section].taille<<12))
            index_tail -= ((uint32_t)Pile_F[section].taille<<12);

    Pile_F[section].tail = index_tail;

    // enregistrement des index en EEPROM : imm�diat ou diff�r�
    if (ENREG_INDEX_FLASH_R)
    {
        if (!Clock_isActive(TimerEEPROMHandle))
        {
            CLOCK_RESTART_TIMEOUT(TimerEEPROMHandle, ENREG_INDEX_FLASH_R * 1000);  // millisecondes
        }
    }
    else  // imm�diat
        sauve_eeprom_index_flash();

    return true;
}

void enreg_index_flash(void)
{
    // enregistrement des index en EEPROM : immediat ou differe
    if (ENREG_INDEX_FLASH_R)  // differe suivant la valeur de Enreg_index_flash_R
    {
        if (!Clock_isActive(TimerEEPROMHandle))
        {
            CLOCK_RESTART_TIMEOUT(TimerEEPROMHandle, ENREG_INDEX_FLASH_R * 1000);  // millisecondes
        }
    }
    else  // immediat
        sauve_eeprom_index_flash();
}


// sauvegarde tous les index de la flash, immediat ou differe
void sauve_eeprom_index_flash_glob(void)
{
    if (ENREG_INDEX_FLASH_W)
    {
        if (!Clock_isActive(TimerEEPROMHandle))
        {
            Clock_setTimeout(TimerEEPROMHandle, ENREG_INDEX_FLASH_W * 1000 * 1000  / Clock_tickPeriod);
            Clock_start(TimerEEPROMHandle);
        }
    }
    else  // immediat
        sauve_eeprom_index_flash();
}


// sauvegarde tous les index de la flash
void sauve_eeprom_index_flash(void)
{
    uint8_t buff_w[8*NB_PILES_F], i;
    for (i=0; i<NB_PILES_F; i++)
    {
        memcpy (buff_w +  i*8, &Pile_F[i].head, 4);
        memcpy (buff_w+4+ i*8, &Pile_F[i].tail, 4);
        //EEPROM_write(8*i, buff_w, 8, 0);
    }
    EEPROM_write(0, buff_w, 8*NB_PILES_F, 0);
}
#endif  // FLASH_EXTERNE


void Timer_EEPROM_Callback(UArg arg0)
{
    // Post event pour enregistrer les index flash en EEPROM
    Timer_EEPROM_Callback_task();
}

void PeriodiqueCallback(UArg arg0)  // callback pour envoyer message periodique
{
    #ifdef CC13xx
      event_action_periodique(period_type);
    #endif
}



#ifdef Uart2


 void transmission_message_Uart2(void)
  {
     uint8_t new_message, a;

     if (message_uart2_en_cours)
         code_erreur = 0x3E;
     else
     {
         message_uart2_en_cours=1;
         new_message = 1;

         while ((Uart2_tx_head != Uart2_tx_tail) && (!tx_Uart2_actif) && (new_message))  // message a envoyer sur liaison serie
          {
              a = Uart2_tx_tail;
              transmission_Uart2();
              if (a == Uart2_tx_tail)
                new_message=0;
              else
                  if ((send_long_attente) && (send_long_port_dest==3)) // message envoye : test si message long en attente et place dispo
                  {
                    /*uint16_t free;
                    if (Uart2_tx_head < Uart2_tx_tail)
                      free =  Uart2_tx_tail - Uart2_tx_head ;
                    else
                      free = (TX_BUFFER_SIZE - Uart2_tx_head) + Uart2_tx_tail;
                    if (free > (TAILLE_FICHIER_TRAME+8))
                    {  // place dispo pour un nouveau message
                        send_long_attente=0;
                        strcpy((char *)message+1, "WET3");
                        message[0]=send_long_emett;
                        envoi_message ();
                    }*/
                  }
          }
         message_uart2_en_cours=0;
     }
  }
#endif







void raz_Uart(uint8_t num_uart)  // raz car en reception (suite timeout)
{
#ifdef Uart2
   Uart2_rx_attente=0;
   Uart2_rx_head=0;
   Uart2_rx_tail=0;
   Uart2_rx_nb_car_recu = 0;
   Uart2_rx_longu = 0;
   CLOCK_STOP(UartSt[0].h_timeout_RX);  // raz timeout RX
   flag_trame_rx_Uart2 = 0;
#endif
}

#ifdef Uart2

void raz_init_Uart2(void)  // raz car lors du reset
{
 raz_Uart(0);
 tx_Uart2_actif = 0;
 message_uart2_en_cours=0;
 CLOCK_STOP(UartSt[0].h_timeout_TX);  // raz timeout TX
 Uart2_tx_head = 0;
 Uart2_tx_tail = 0;
}


// enregistre le message a envoyer dans le buffer TX_Uart2
uint8_t envoi_Uart2(uint8_t long_mess)
{
   uint8_t free, c, retc, full;
   retc=1;

   /* calcul places libres dans buffer */
   if (Uart2_tx_head < Uart2_tx_tail)
      free =  Uart2_tx_tail - Uart2_tx_head ;
   else
      free = (TX_BUFFER_SIZE - Uart2_tx_head) + Uart2_tx_tail;

   full=0;
   if (message[2]=='E')  // message erreur
   {
       if ( ((long_mess+1) >=free) || ((long_mess+2) > free) )
           full=1;
   }
   else  // message normal : on laisse 13 places pour un eventuel futur message erreur
   {
       if ( ((long_mess+1) >=free) || ((long_mess+14) > free) )
           full=1;
   }
   if (full)
   {
       code_erreur = erreur_TX_full;   // Tx_buffer full
       err_donnee1 = '1';
   }
   else
   {   // transfert data
       for (c=0; c<(long_mess+1); c++)  // PRZZ ou PR1ZZ  lg=4
       {
          Uart2_tx_buff[Uart2_tx_head++] = message[c]; /* copy over this byte of data */
          if( Uart2_tx_head == TX_BUFFER_SIZE ) Uart2_tx_head = 0;  /* reset pointer  if wrapping around */
       }
       retc = 0;
       Task_envoie_message();
    }
    return retc;
 }


void transmission_Uart2 (void)
{
    uint8_t longueur_tx, c, index, type, envoi, tx_rejoint;
    uint8_t message_trans[MESSAGE_SIZE];

  // determination type de trame

  type = Uart2_tx_buff[Uart2_tx_tail] & 0x80;
  envoi=1;



  //verification que le canal DMA est disponible
  //if (hdma_usart2_tx.State == HAL_DMA_STATE_READY)
    //{
      // transfert du buffer tx_buff -> message_trans
      if (type)  // message hexa    PR1ZZ lg=5
      {
          index = Uart2_tx_tail;
          index++;
          index++;
          if( index >= TX_BUFFER_SIZE )   index = index - TX_BUFFER_SIZE;   /* if wrapping around  reset pointer */
          longueur_tx = (Uart2_tx_buff[index] & 0x7F) + 4;
          if (longueur_tx < MESSAGE_SIZE)
          {
              for (c=0; c<(longueur_tx); c++)  // PRZZ ou PR1ZZ  lg=5
              {
                  message_trans[c] = Uart2_tx_buff[Uart2_tx_tail++]; /* copy over this byte of data */
                 if( Uart2_tx_tail >= TX_BUFFER_SIZE ) Uart2_tx_tail = 0;  /* reset pointer  if wrapping around */
                 if ((Uart2_tx_tail == Uart2_tx_head) && (c != (longueur_tx-1))) envoi=0;
              }
          }
          else
          {
              envoi = 0;
              err_donnee2 = '1';
          }
      }
      else  // message texte
      {
          c = 0;
          tx_rejoint=0;
          for (longueur_tx=0; ((c != car_fin_trame) && (longueur_tx < (MESSAGE_SIZE - 1))); longueur_tx++)  // PRZZ   lg=5
          {
              c = Uart2_tx_buff[Uart2_tx_tail];
              message_trans[longueur_tx] = c; /* copy over this byte of data */
              Uart2_tx_tail++;
              if( Uart2_tx_tail == TX_BUFFER_SIZE ) Uart2_tx_tail = 0;  /* reset pointer  if wrapping around */
              if (tx_rejoint)
                  tx_rejoint=2;   // depassement
              if (Uart2_tx_tail== Uart2_tx_head)
                  tx_rejoint=1;   // fin de la pile
          }

          if ((longueur_tx == (MESSAGE_SIZE - 1)) || (tx_rejoint==2)
                  || ((tx_rejoint) && (Uart2_tx_tail != Uart2_tx_head)))
            {
              envoi=0; // pb trop long
            }
      }

      if  (!envoi)
      {
          Uart2_tx_tail = Uart2_tx_head;  // raz pile envoi
          code_erreur = erreur_TX_full;
          err_donnee1 = '2';
      }
      else
      {
          message_trans [longueur_tx] = 0;
          if ((longueur_tx > 1) && (longueur_tx < MESSAGE_SIZE))
          {
              tx_Uart2_actif = 1;
              //timeout_uart2_tx = timeout_tx_ct;
                #ifdef CC13xx
                      UART_write(hUart2, (char *)message_trans, longueur_tx);
                #endif
                #ifdef ESP32
                      uart_write_bytes(hUart2, (const char *) message_trans, longueur_tx);
                      #ifdef Uart_saut_ligne   // pour visualisation en mode debug
                         uint8_t saut_ligne;
                         saut_ligne=10;
                         uart_write_bytes(hUart2, (const char *) &saut_ligne, 1);
                      #endif
                #endif

              //Display_printf(hDisplaySerial, 0, 0, (char *)message_trans);
              tx_Uart2_actif = 0;

            /*if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)message_trans, longueur_tx)!= HAL_OK)  // Envoie Uart
              {
                   tx_Uart2_actif = 0;
                   code_erreur = erreur_TX_pas_fin;
               }*/
           }
      }
    //}
}
//#endif
#endif

// --------------------   LIAISON RF ----------------------------------------

#ifdef CC13xx

void raz_RF(void)  // raz car en reception (suite timeout)
{
}


void raz_init_RF(void)  // raz car lors du reset
{
  raz_RF();
}

#endif



//calcule longueur jusqu'à 0 ou fin_trame    // LRZx => long=4
char unsigned longueur_messa (char unsigned * message_envoye)
{
  char unsigned c, longueur_calc, type;

  type = message_envoye[0] & 0x80;

  if (type)  // message hexa
  {
      longueur_calc = (message_envoye[1] & 0x7F) + 4;
  }
  else  // message texte
  {
      c = 1;
      for (longueur_calc = 0; ((c != car_fin_trame) && (c) && (longueur_calc < (MESSAGE_SIZE - 1))); longueur_calc++)    // LRZx => long=4 (devient LPRZx)
        c = message_envoye[longueur_calc];

      if ((c != car_fin_trame) && (c))    // erreur : message[] trop long - dépassement de tableau
        {
          code_erreur = erreur_mess_tab + 2;
          err_donnee1 = message_envoye[0];
          err_donnee2 = message_envoye[1];
          message_envoye[MESSAGE_SIZE - 1] = car_fin_trame;
        }
      if ((!c) && (longueur_calc) && (longueur_calc<=MESSAGE_SIZE))   // dernier car=0 => remplace par car_fin_trame
        message_envoye[longueur_calc-1]=car_fin_trame;
  }
  if (longueur_calc > MESSAGE_SIZE) // message trop long
  {
      longueur_calc = MESSAGE_SIZE;
      code_erreur = erreur_mess;
  }
  return longueur_calc;
}

// Envoi message texte (termine par car_fin_trame ou 00)

uint8_t envoi_message (void)   // rajoute l'emetteur et CRC à message[] -> message_temp[], calcule longueur
{
    uint8_t c, crc, long_mess, retc, uart;
    retc=1;

  //raz_timer_sleep ();
  // calcul longueur (sans car fin, avec emetteur) 3 pour LZ (devient LPZf)
  crc = 0;

  uart=0;
  if (message[0] & 0x80)  // vers le port serie du destinataire
  {
      uart=1;
      message[0] &= 0x7F;
  }
  long_mess = longueur_messa(message);  // calcul longueur et ajout car_final si 0 a la fin

  if ((long_mess < 2) || (long_mess >= MESSAGE_SIZE))  // trop long ou trop court
    code_erreur = erreur_TX_empty;
  else
  {
      if (uart)
      {
          for (c = long_mess - 1; c; c--) // decalage de 1 a partir de la 2e place  XHLH -> X1HLH
          {
              message[c + 1] = message[c];
          }
          message[1] = '1';
          message[0] = message[0] & 0x7F;
          long_mess++;
      }
      for (c = long_mess - 1; c; c--) // ajout emetteur My_address  long=3 LZ 3=2  2=1  c=2 puis c=1   LPZf
        {
          message[c + 1] = message[c];
          crc = message[c] + crc;
        }
      crc = crc - car_fin_trame;
      message[1] = My_Address;

      if (message[2] & 0x80)  // rajout CRC a la fin du message
        {
          message[long_mess] = crc;
          long_mess++;
          message[long_mess] = car_fin_trame;
        }

      transf_buff (long_mess);  // Transfert dans buffer d'envoi  et envoi
  }
  return retc;
}



// Envoi message hexa (longueur definie au 2� octet = longueur apres le type de message

uint8_t envoi_message_def (void) // rajoute l'emetteur, 0x80 et CRC à message[] -> message_temp[], augmente longueur, P1ZZ
{
    uint8_t c, crc, long_mess, retc;
    retc=1;

//  raz_timer_sleep ();

  // Longueur pour P1ZZ = 1, on rajoute 3 pour le destinataire, l'emetteur,  et le type  => long=4  => PR1ZZ
  long_mess = message[1] & 0x7F;
  crc = 0 - message[1];

  if ( long_mess >= MESSAGE_SIZE - 3 )
  {
      code_erreur = erreur_mess_tab ;
      long_mess = MESSAGE_SIZE - 4;
  }
  else
  {
      if (message[0] & 0x80)  // vers le port serie du destinataire
      {
          for (c = long_mess - 1; c>1; c--) // decalage de 1 a partir de la 3e place  X2HLH -> X21HLH
          {
              message[c + 1] = message[c];
          }
          message[2]='1';
          message[0] = message[0] & 0x7F;
          message[1]++;
          long_mess++;
      }
      for (c = long_mess + 2; c; c--) // ajout emetteur My_address  long=0 P0Z 3=2  2=1  c=2 puis c=1  => long_mess=3  Px0Z
        {
          message[c + 1] = message[c];
          crc = message[c] + crc;
        }

      long_mess = long_mess + 3;
      message[1] = My_Address;
      message[0] = message[0] | 0x80; // permet d'identifier le message de long_mess d�finie

      if (message[2] & 0x80)  // rajoute CRC � la fin du message
        {
          long_mess++;
          message[long_mess] = crc;
          message[2]++; // ajout 1 � la longueur
        }

      retc = transf_buff (long_mess);  // Transfert dans buffer d'envoi  et envoi
  }
  return retc;
}


/*char unsigned passerelle_table_routage(char unsigned destinataire)
{
  char unsigned i,j;
  j = 0;
  for (i = 0; i < nb_ligne_routage; i++) // recherche de la liaison à utiliser dans la table de routage
    {
      if ((destinataire >= table_routage[i][0])
          && (destinataire <= table_routage[i][1]))
        {
          j = table_routage[i][3]; // passerelle
          break;
        }
    }
  if (!j) j=destinataire;
  return j;
}*/


// cherche dans table de routage la passerelle
uint8_t passerelle_routage(uint8_t dest)
{
    uint8_t i, type, interf;
    type = 0;
    dest &= 0x7F;
    interf = dest;
    for (i = 0; i < nb_ligne_routage; i++) // recherche de la liaison a� utiliser dans la table de routage
      {
        if ((dest >= table_routage[i][0])
            && (dest <= table_routage[i][1]))
          {
             type = table_routage[i][2]; // type liaison
             break;
          }
      }
    if (type==7)
        interf = table_routage[i][3];
    return interf;
}


// identification du port du destinataire : 0:aucun 3:uart2, 6-7:RF
uint8_t port_dest(uint8_t destinataire)
{
    uint8_t i,j;
    if ((destinataire & 0x80) && ( (destinataire & 0x7F) == My_Address))
        j=3;
    else
    {
        j = 0;
        for (i = 0; i < nb_ligne_routage; i++) // recherche de la liaison a utiliser dans la table de routage
        {
            if ((destinataire >= table_routage[i][0])  && (destinataire <= table_routage[i][1]))
            {
                j = table_routage[i][2]; // type liaison
                break;
             }
        }
    }
    return j;
}


//return 0:ok  1:erreur
uint8_t transf_buff (uint8_t longueur_mess)   // var:  PRHLHf  long=5      def: PR2HLH  long=5
{
  uint8_t destinataire, i, j, retc;
  retc=1;

  destinataire = message[0] & 0x7F;
  //long_def = message_temp[0] & 0x80; // 0 si message texte, 80 si message hexa

  if (((destinataire == My_Address) && (My_Address!='1')) || (destinataire == '0'))   // envoi sur soi-meme -loop
  {
      if (longueur_mess < MESSAGE_SIZE -1)
      {
          memcpy (message_in, message, longueur_mess + 1);
          //message_in[0] = '0' + message[0] & 0x80;  // emetteur=loop
          traitement_rx (longueur_mess);
          retc=0;
      }
  }
  else
  {
      j = 0;
      for (i = 0; i < nb_ligne_routage; i++) // recherche de la liaison a utiliser dans la table de routage
      {
          if ((destinataire >= table_routage[i][0])
              && (destinataire <= table_routage[i][1]))
            {
              j = table_routage[i][2]; // type liaison
              break;
            }
      }
      if (j)
        {

        #ifdef Uart2     // envoi par Uart2
           bool Uart2_act=1;
           #if Uart2 == 1   // mode reveil/sleep => pas d'envoi
              if (!Uart_actif)
                  Uart2_act=0;
           #endif
           if ((j==3) && (Uart2_act))
                retc = envoi_Uart2(longueur_mess);
        #endif

        #ifdef CC13xx
                  if (j==6)
                      retc = envoi_buffer_RF(message, longueur_mess, destinataire);
                  if (j==7)
                      retc = envoi_buffer_RF(message, longueur_mess,  table_routage[i][3]);
        #endif

        }
  }
  return retc;
}



// calcul l'espace dispo du buffer du destinataire concerne
uint16_t calcul_free_space (uint8_t destinat)
{
  uint16_t free=0;
  uint8_t port = port_dest (destinat);

  #ifdef Uart2
  if (port == 3)  // Uart2
  {
      if (Uart2_tx_head < Uart2_tx_tail)
      free =  Uart2_tx_tail - Uart2_tx_head ;
    else
      free = (TX_BUFFER_SIZE - Uart2_tx_head) + Uart2_tx_tail;
  }
  #endif

  #ifdef CC13xx
    if ((port==6) || (port==7))
    {
        #ifdef PILE_RF_FLASH  // Pile d'envoi en memoire Flash externe
            free = 255;
        #else
            free = calcul_espace_dispo_node(numero_device_rf(passerelle_routage('L')));
        #endif
    }
  #endif
  return free;
}

#ifdef CC13xx
void fin_trans_mess_long(void)
{
    send_long_actif = 0;
    CLOCK_STOP( xTimer_envoi);
}

void recep_mess_long_uart(void)
{
    // MESSAGE LONG
      uint16_t num_trame;
      num_trame = ((uint16_t)message_in[5]<<8) + message_in[6];

      if (!send_long_actif)   // debut de message long recu
      {
          if (!num_trame)  // initialisation
          {
              send_long_actif=1;
              send_long_autre=0;
              send_long_emett = message_in[0] & 0x7F;  // emetteur
              send_long_port_emett = 3;                // port 3:uart2
              send_long_dest = 'L'; // port du destinataire
              send_long_buffer_tx=0;  // 0:dispo RF si Uart->RF
              send_long_buffer_suiv=0;  // 0:dispo  RF du destinataire
              send_long_pile=0;         // max 3 en pile d'emission tx
              send_long_attente=0;
              send_long_num_trame = 0;
              send_long_repeat = 0;
              //demarrage timer pour raz
          }
      }
      if (send_long_actif) // si message long actif
      {
          // verif que la trame recue est soit la meme, soit la suivante
          if ((num_trame == send_long_num_trame) || (num_trame == send_long_num_trame+1))
          {
              // calcul de la place dispo en buffer TX
              uint16_t free;
              free = calcul_free_space(send_long_dest);
              if (free > (TAILLE_FICHIER_TRAME+8)*2)   // place dispo pour 2 messages => WET0
              {
                  send_long_buffer_tx=0;
                  send_long_pile++;
              }
              else if (free > (TAILLE_FICHIER_TRAME+8))  // place dispo pour ce nouveau message
              {
                  send_long_buffer_tx=1;         // ok  mais pas dispo pour le suivant => WET1
                  send_long_pile++;
              }

              else  // pas de place dispo pour renvoyer ce message => WET2
                  send_long_buffer_tx=2;

              //strcpy((char *)message+1, "WETx");
              //message[4] = send_long_buffer_tx;
              //message[0] = send_long_emett;
              //envoi_message ();

              send_long_num_trame = num_trame;
              CLOCK_START( xTimer_envoi);
          }
          else
              fin_trans_mess_long();
       }
}


void recep_mess_long_rf(code)
{
    // MESSAGE LONG recu de RF
      uint16_t num_trame;
      num_trame = ((uint16_t)message_in[5]<<8) + message_in[6];

      // Message long : Ack: 0:pas mess long 1:prem trame 2:nouv prem trame 3:ok-place 4:nok 5:nok 6:nok 7:emett dif
      if ((code==1) || (code==2))   // debut de message long recu : init
      {
          send_long_actif=1;
          send_long_num_trame = 0;
          send_long_emett = message_in[1] & 0x7F;  // emetteur
          send_long_port_emett = 7;                // port 7:RF
          send_long_port_dest = port_dest('L'); // port du destinataire
          send_long_buffer_tx=0;  // 0:dispo RF si Uart->RF
          send_long_buffer_suiv=0;  // 0:dispo  RF du destinataire
          send_long_pile=0;         // max 3 en pile d'emission tx
          send_long_attente=0;
          send_long_repeat = 0;
          //demarrage timer pour raz
          CLOCK_START( xTimer_envoi);
      }
      else if (code==3) // si message long deja actif
      {
          send_long_num_trame = num_trame;
          send_long_buffer_tx=0;
          CLOCK_START( xTimer_envoi);
      }
      else if (code != 7)
          fin_trans_mess_long();
}

// messa : message_in
uint8_t recep_long (uint8_t * messa)
{
    uint8_t code=0;
    uint8_t free;
    // Message long : Ack: 0:pas mess long 1:prem trame  3:ok-place 4:nok 5:nok 6:nok 7:emett dif  8:nouv prem trame nok

    uint16_t num_trame;
    num_trame = ((uint16_t)messa[5]<<8) + messa[6];

    if (send_long_actif)  // message long deja actif
    {
        // verif que l'emetteur est le meme
        if (send_long_emett != (messa[1] & 0x7F))
        {
            code = 7;   // emetteur different, attendre avant de renvoyer
        }
        else
        {
            // verif que la trame recue est soit la meme, soit la suivante
            if ((num_trame == send_long_num_trame) || (num_trame == send_long_num_trame+1))
            {
                code = 4;   // trame suivante - pas de place
                free = calcul_free_space (messa[0] & 0x7F);
                if (free > (((uint16_t)TAILLE_FICHIER_TRAME+8)))
                {  // place dispo pour ce message
                    send_long_attente=0;
                    code = 3;  // equivalent WET0 : ok
                }
            }
            else if (!num_trame)
                code = 8;   // reprise a zero : nok
            else
                code = 6;  // nok : decalage de trame, reprendre au debut
        }
    }
    else // 1ere trame
    {
        if (!num_trame)
        {
            code = 1;  // 1ere trame ok
        }
        else
            code = 5;  // nok
    }
    return code;
}

// tache node : suite a timeout message_long
void mess_long_event(void)
{
    if ((send_long_actif) && ((send_long_buffer_tx==1) || (send_long_buffer_tx==2)))  // si dest=RF
    {

    }
        // Message long test
    /*
        strcpy((char *)message, "LxY");  //
        message[1] = 13; // longueur
        message[3] = 0;  // type 0:premier  1:milieu  2:dernier
        if (send_long_actif==1)
            message[3] = 2;
        message[4] = send_long_num_trame;
        message[5] = 0; //type trame
        uint8_t tmp;
        for (tmp=0; tmp<10; tmp++)
            message[tmp+6] = send_long_actif + '0';
        Mess_statut = 0;  // 0:ack, pas de RX  30=pas d'ack+RX   bit4:pas d'ack  bit5:RX apres
        envoi_message_def ();
        send_long_actif--;
        CLOCK_RESTART_TIMEOUT(xTimer_envoi, M_LONG_PERIODE);  // 500 millisecondes );  //redemarre le timer
    }*/
}
#endif


void traitement_rx (char unsigned longueur_m) // var :longueur n'inclut pas le car_fin_trame (inclus emetteur) : 4 pour RLT1
{       // def :longueur n'inclut pas la longueur (inclus l'emetteur) : 4 pour RL1T1
  uint8_t a, b, crc, long_def;//, emet_m;
  uint8_t tempo1, tempo2, tempo3, tempo4;
  unsigned char volatile   * pregistre_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)
  unsigned int volatile    * pregistre_int_xdata;  // pointeur vers Memory (ROM, RAM, EEprom, registres)


  uint32_t i32;
  mess_ok=0;

  //char unsigned cpuSR;  // uint8_t

  flag_comm_transfert=0;
  //raz_timer_sleep ();
  message[0] = message_in[1]; // emetteur devient le destinataire du futur message
  //emet_m = message[0];

  if (((message_in[0] & 0x7F) != My_Address) && ((message_in[0] & 0x7F) != '0'))  // Transfert ailleurs
  {
      if (longueur_m  < MESSAGE_SIZE-1)
      {
        #ifdef CC13xx
          // Message long
          if ((message_in[0]==('L'+0x80)) && (message_in[3]=='Y'))    // MESSAGE LONG : L-Em-Lg-Y-type-num*2
          {
              uint8_t code;
              send_long_buffer_tx=0;
              if (port_dest(message_in[1])==3)  // Node et STM32 : si reception de l'uart
              {
                  message_test[1] = 1;  // TODO
                  recep_mess_long_uart();
                  code = 1;
              }
              else
              {
                  code = recep_long( message_in);   // Concentrateur : reception de RF
                  recep_mess_long_rf(code);   // Concentrateur : reception de RF
              }

              if ((send_long_actif) && (code < 4))
              {
                  if (message_in[4] == 2)   // derniere trame
                  {
                      send_long_actif = 0;
                      CLOCK_STOP( xTimer_envoi);
                  }
                  memcpy (message, message_in, longueur_m + 1);
                  flag_comm_transfert=1;
                  if ((port_dest ('L') & 0b110) == 6)  // vers RF : 6 ou 7
                  {
                       uint16_t num_trame;
                       num_trame = ((uint16_t)message_in[5]<<8) + message_in[6];
                       Mess_statut = 0x10; // pas d'Ack  // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)  bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                       if (!(num_trame%10)) Mess_statut = 0;  // Ack pour 1 message sur 10  TODO
                  }

                  transf_buff (longueur_m);  // transfert du message long
              }
          }
          else  // message normal a transferer
          #endif
          {
              memcpy (message, message_in, longueur_m + 1);
              flag_comm_transfert=1;
              transf_buff (longueur_m);  // envoi du message
          }
      }
      else
      {
          code_erreur = erreur_mess;  // et '2'
          err_donnee1 = '2';
          err_donnee2 = message_in[0];
      }
  }
    #ifdef Uart2    // Transfert les messages vers l'uart
      else if ( (message_in[0] == My_Address) && (message_in[2] == '1'))  // transfert vers Uart : XL1HLH->1LHLH - message texte
      {
          if (longueur_m+1 < MESSAGE_SIZE)
          {
              memcpy (message, message_in+1, longueur_m );
              message[0] = '1';
              message[1] = message_in[1];
              flag_comm_transfert=1;
              transf_buff (longueur_m-1);
          }
      }
      else if ((message_in[0] == (My_Address|0x80)) && (message_in[3] == '1'))  // transfert vers Uart : XL31HLH->1L2HLH - message longueur definie
      {
          if (longueur_m+1 < MESSAGE_SIZE)
          {
              memcpy (message, message_in+1, longueur_m );
              message[0] = '1'+0x80;
              message[1] = message_in[1];
              message[2] = message_in[2]-1;
              flag_comm_transfert=1;
              transf_buff (longueur_m-1);
          }
      }
    #endif
  else
  {
      if (longueur_m >= MESSAGE_SIZE)
      {
          longueur_m = MESSAGE_SIZE;
          code_erreur =erreur_mess;
      }

      long_def = message_in[0] & 0x80;
      if (long_def)         // Si longueur definie dans le message
      {
          for (a = 2; a < longueur_m; a++) // 2 pour long=3, RL0T - suppression de la longueur
            message_in[a] = message_in[a + 1];
      }


      crc = 1;

      if (message_in[2] & 0x80)   // si CRC avec 1er car de la payload
      {
          crc = 0;
          for (a = 2; a < (longueur_m - 1); a++)   // long=5 si RLT1cf
            crc = crc + message_in[a];
          message_in[2] = message_in[2] & 0x7F;
          if (message_in[longueur_m - 1] != crc)
            crc = 0;
          else
            {
              longueur_m--;
            }
          message[1] = 'S';  // Accus� reception ok : S1
          message[2] = '1';
          message[3] = car_fin_trame;
          envoi_message ();
          if (!crc)
              message[2] = '0'; // si crc errone => envoi de S0
      }

      if (crc)
      {
          #ifdef CC13xx
          Mess_statut = 0;  // statut par defaut pour tout envoi de message
          if (rxrf_numero_device > lastAddedSensorNode)   // 0:pas de device  1:device 0 etc...
              rxrf_numero_device=0;
          #endif

          // ******************************** AAAAAAAAAAAAAAAAAAAAA  ********************

          if ((message_in[2] == 'A') && (message_in[3] == 'L')) // AL Lecture entree analogique
          {
              if ( (message_in[4] >='0') &&  (message_in[4] <= '9') && (longueur_m==5)) // Read ALx  : Lecture entree analog x
              {
                  uint8_t ret;
                  uint16_t ent_anal;
                  ret = lecture_analog((message_in[4]-'0'), &ent_anal);  // 0:30(solaire)   1:27(batterie)
                  if (!ret)
                  {
                      strcpy((char *)message+1, "VAnal x: ");
                      message[7] = message_in[4];
                      ajout_int16_message((int16_t)ent_anal,  9, 2);
                      envoi_message ();

                  }
              }
              if ( (message_in[4] == 'T') && (longueur_m==5)) // Read ALT  : Lecture capteur temperature
                {
                  int16_t temp_mesure_tt;

                  //temp_mesure_tt = (AONBatMonTemperatureGetDegC() >> 8) & 0x1FF;
                  #ifdef CC13xx
                    temp_mesure_tt = AONBatMonTemperatureGetDegC() ;
                    //temp_mesure_tt = (AONBatMonTemperatureGetDegDec()>>6);
                    #endif
                  #ifdef ESP32
                    temp_mesure_tt = 10;
                  #endif
                  //AONBatMonNewTempMeasureReady();

                  strcpy((char *)message+1, "Temp cpu:  ");
                  ajout_int16_message (temp_mesure_tt, 11, 2);
                  //envoi_char_decimal( (uint8_t) temp_mesure_tt, 7);
                  envoi_message ();
                }

              if ((message_in[4] == 'V') && (longueur_m == 5)) // Read ALV  : Lecture voltage micro
              {
                  float volt_float;

                  volt_float = (float) mesure_voltage() / 1000;

                  strcpy((char *)message+1, " Voltage: ");
                  ajout_float_message(volt_float, 2, 11, 2);
                  envoi_message ();
                }
          }

          // ****************************   BBBBBBBBBBBBBBBBBBBBB  *************************

        #ifdef BROAD_BALISE_SYNCHRO
          if (message_in[2] == 'B') // Balise
          {
              #ifdef CONCENTRATOR
              if ((message_in[3] == 'L') && (message_in[4] == 'S'))
              {
                  uint8_t device_rx;
                  device_rx  = numero_device_rf(message_in[1]);
                  if (device_rx)
                  {
                      device_rx--;
                      if ((message_in[5] == 'Y') && (longueur_m == 8))         //   BLSYxx  balise temporelle de resynchro
                      {
                          uint32_t periode_bal;
                          knownNodes[device_rx].synchro = 0;
                          knownNodes[device_rx].period_activ_rx = hexaT(message_in[6])*16 + hexaT(message_in[7]);  // en secondes
                          periode_bal = (uint32_t) knownNodes[device_rx].period_activ_rx * 1000 * 100;

                          uint32_t resteC,  resteF;
                          resteC = tick_restant_avant_broadcast();  // et activation si synchro_actif=1
                          resteF = resteC % periode_bal;

                          // si resteF est trop petit (<200ms), rajout d'une periode
                          if (resteF < 20000 )
                              resteF += periode_bal;

                          // => BESYxxxx nb de 10 microsec jusqu'� prochaine fenetre. Max:255secondes=>4 octets

                          if (synchro_actif)  // 0:pas de balise, 2:balise envoyee
                          {
                              strcpy((char *)message+2, "BESY");
                              memcpy(message+6,  &resteF, 4);
                              message[10] = PERIODE_BALISE_SYNCHRO;
                              message[1] = 8;
                              Mess_statut = 0x10; // pas d'Ack // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)  bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                              envoi_message_def ();
                          }
                      }
                      if ((message_in[5] == 'Z') && (longueur_m == 6))         //   BLSZ  balise temporelle de resynchro bien recue par node
                      {
                          knownNodes[device_rx].synchro = 1;
                          knownNodes[device_rx].resynchro++;
                      }
                  }
                  device_rx  = numero_device_rf(message_in[6]);
                  if (device_rx)
                  {
                      device_rx--;
                      if ((message_in[5] == 'B') && (longueur_m == 7))   // BLSBx  : Statut du Node X :Temps avant la prochaine synchro, synchro?, nb resynchro
                      {
                          strcpy((char *)message+1, "B Rx:       Resync:    ");
                          ajout_int16_message((int16_t) (timeout_prochaine_fenetre(device_rx)/10000), 5,0);  // en 0,1seconde
                          message[5]=':';
                          message[11]=message[10];
                          message[10]=',';
                          ajout_int8_message(knownNodes[device_rx].resynchro, 20,1);
                          envoi_message();

                          //statut_node_synchro(device_rx);
                      }
                  }
                  if ((message_in[5] == 'A') && (longueur_m == 6))   // BLSA  : Temps avant la prochaine balise de synchro
                  {
                      strcpy((char *)message+1, "B Prochaine Bal:     s");
                      ajout_int16_message((int16_t) (timeout_prochaine_balise()), 17, 0x82);  // en 1seconde
                      envoi_message();

                      //statut_node_synchro(device_rx);
                  }
                  if ((message_in[5] == 'T') && (longueur_m <= 7))   // BLST(R)  : Prochaine balise et nb resynchro de chaque node et option Raz
                  {
                      strcpy((char *)message+1, "B Prochaine:     Resync:  ");

                      ajout_int16_message((int16_t) (timeout_prochaine_balise()), 13, 0x82);  // en 1seconde

                      uint8_t index=26, i;
                      for (i=0; i<lastAddedSensorNode;i++)
                      {
                          message[index++]= knownNodes[i].adresse;
                          if (knownNodes[i].synchro)
                              message[index++]=':';
                          else
                              message[index++]=' ';
                          message[index++]= deci(knownNodes[i].resynchro >> 4);
                          message[index++]= deci(knownNodes[i].resynchro & 0xF);
                          message[index++]=' ';
                          if (message_in[6] == 'R')
                              knownNodes[i].resynchro=0;  // option Raz
                      }
                      message[index]=car_fin_trame;
                      envoi_message();

                      //statut_node_synchro(device_rx);
                  }
              }
              if ((message_in[3] == 'E') && (message_in[4] == 'S') && (message_in[5] == 'Z') && (longueur_m==7))   // BESZ0/1 : arret �mission balise
              {
                  if ((message_in[6]=='0') || (message_in[6]=='1'))
                  {
                      synchro_actif = message_in[6]-'0';
                      arret_emission_balise();
                      strcpy((char *)message+1, "OK");
                      envoi_message ();
                  }
              }
              #endif // concentrator
              #ifdef NODE
              if ((message_in[3] == 'L') && (message_in[4] == 'S'))
              {
                  if ((message_in[5] == 'T') && (longueur_m == 6))   // BLST  : Statut du Node : Temps avant la prochaine demande de resynchro,  nb demande de resynchro
                  {
                      strcpy((char *)message+1, "B Demande Resynchro : Prochaine(s):       Nb:      ");
                      ajout_int16_message((int16_t) (timeout_prochaine_resynchro()), 36, 0x82);  // en 1seconde
                      ajout_int8_message(knownNodes[0].resynchro, 45,1);
                      message[45]=':';
                      envoi_message();

                      //statut_node_synchro(device_rx);
                  }
              }
              if ((message_in[3] == 'L') && (message_in[4] == 'R'))
              {
                  if ((message_in[5] == 'S') && (longueur_m == 6))   // BLRS  : lecture du rssi du bruit (� l'arret)
                  {
                      strcpy((char *)message+1, "B Bruit Rssi:      ");
                      ajout_int8_message(rssi_bruit, 15,1);
                      envoi_message();
                  }
              }
              #endif

              if (message_in[3] == 'E')
              {
                #ifdef NODE
                  if ((message_in[4] == 'S') && (message_in[5] == 'Y') && (longueur_m == 11))         //   BESYxxxx  Reception temps avant balise initiale
                  {
                      uint32_t temp_restant;

                      memcpy(&temp_restant, message_in+6, 4);
                      recept_balise_resynchro(temp_restant);

                      knownNodes[0].synchro = 1;
                      strcpy((char *)message+1, "BLSZ");
                      Mess_statut = 0x10; // pas d'Ack // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)  bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                      envoi_message ();
                      strcpy((char *)message, "LBLSZ");
                      envoi_message ();
                  }
                  if ((message_in[4] == 'P') && (longueur_m == 7))         //   BEPxx  Periode entre 2 fenetres d'ecoute (secondes:5,6,10,12,15,20,30,60)
                  {
                      modifPerioFeneBal( (message_in[5]-'0')*10 + message_in[6]-'0', 0 );
                  }
                  if ((message_in[4] == 'F') && (longueur_m == 7))         //   BEFxx  Fenetre d'ecoute (milli-secondes)
                  {
                      duree_fenetre_RX = (message_in[5]-'0')*10 + message_in[6]-'0';
                      mess_ok=1;
                  }
                #endif
              }
              if (message_in[3] == 'L')
              {
                 #ifdef NODE
                  if ((message_in[4] == 'D') && (longueur_m == 5))         //   BLD : lecture periode, fenetre, fenetre actu, ratees
                  {
                      strcpy((char *)message+1, "Balise: ");
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

                  if ((message_in[4] == 'Z') && (longueur_m == 5))    //  BLZ : reactivation balise synchro (si arretee)
                  {
                         Nouv_Deman_balise_synchro();
                  }
                #endif
              }

          }
        #endif  //   #ifdef BROAD_BALISE_SYNCHRO


          // ******************************** DDDDDDDDDDDDDDDDDD ********************

          // D : Decode un message capteur hexa en texte  QxDCPssssvvvv -> LQDSCPjour-hh:mm:ss Cvv.vv vv.vv

          if (message_in[2] == 'D')   // D�codage message hexa
          {
              uint8_t index_val, index_mess, index_in, nb_valeurs;
              uint16_t jours_time, Vbrut;
              float Vfloat;
              uint32_t second_time;
              a=message_in[3]; // type_capteur
              if (a & 0x80)   // bit 4
                  nb_valeurs = (longueur_m - 9)/((a&1)+1);  // avec date
              else
                  nb_valeurs = (longueur_m - 5)/((a&1)+1);  // sans date

              //period=period_conversion(message_in[8])/(1000*1000/Clock_tickPeriod); // periode (en secondes)

              index_val = 0;

              message[0] = 'L';           // destinataire PC
              message[1] = 'D';           // D pour Decodage capteur
              message[2] = message_in[1]; // emetteur d'origine du message
              index_mess = 3;
              message[index_mess++] = deci(a>>4);  // type_capteur
              message[index_mess++] = deci(a & 0x0F);
              message[index_mess++] = '-';
              message[index_mess++] = deci(message_in[4]>>4);  // periode
              message[index_mess++] = deci(message_in[4] & 0x0F);
              message[index_mess++] = ':';

              index_in = 5;
              if (a & 0x80)  // ajout date
              {
                  memcpy ((char *) &i32, (char *)(message_in+index_in), 4);  // timestamp
                  //strncpy ((char *) &i32, (char *)(message_in+3), 4);  // timestamp
                  jours_time = i32/(3600*24);
                  second_time = i32 - jours_time*(3600*24);
                  //message[3] = jours_time>>8;   // jours
                  //message[4] = jours_time&0xFF; // jours
                  conversion_sec_date (second_time, 5);
                  message[13] = ' ';
                  conversion_sec_heure ( second_time, 14, 1);  // 00:00:00
                  index_mess = 22;
                  index_in = index_in+4;
              }

              message[index_mess++] = ' ';

              while (index_val < nb_valeurs)
              {
                  if (a&1)  // 16 bits
                      Vbrut = (message_in[index_in+index_val*2]<<8) + (message_in[index_in+index_val*2+1]);
                  else   // 8 bits
                      Vbrut = (message_in[index_in+index_val]);
                  Vfloat=0;
                  if ((a & 0b1110) == (0b010)) // temp�rature uniquement
                      Vfloat = ((double)(int16_t)Vbrut / 65536)*165 - 40;
                  if ((a & 0b1110) == (0b100)) // hygro uniquement
                      Vfloat = ((double)Vbrut / 65536)*100;               //-- calculate relative humidity [%RH]
                  if ((a & 0b1110) == (0b110)) // temp�rature et hygrom�trie
                  {
                      if (index_val&1)  // valeurs impaires : hygro
                          Vfloat = ((double)Vbrut / 65536)*100;               //-- calculate relative humidity [%RH]
                      else  // paires : temp
                          Vfloat = ((double)(int16_t)Vbrut / 65536)*165 - 40;
                  }
                  index_val++;
                  index_mess = ajout_float_message(Vfloat, 2, index_mess, 0);
                  message[index_mess-1]=' ';
              }
              message[index_mess]=car_fin_trame;
              envoi_message();
          }

          // ******************************** EEEEEEEEEEEEEEEEEEE ********************

          if (message_in[2] == 'E')   // Activation des Entrees
          {
              #ifdef CC13xx
              uint32_t code_tournant;
              if ((message_in[3] == 'C') && (rxrf_numero_device) && (rxrf_numero_device <= lastAddedSensorNode)) // code tournant
              {
                  if ((message_in[4] == '4') &&  (longueur_m == 9)) // EC4xxxx reception code tournant pour synchro
                  {  // concentrateur uniquement
                      code_tournant = * (uint32_t *)(message_in+5);
                      knownNodes[rxrf_numero_device-1].code_tournant = code_tournant_suivant(code_tournant);
                  }
                  if ((message_in[4] == '1') &&  (longueur_m == 9)) // Concentrateur : QEC1xxxx   Telecommande : Desactivation centrale securis�e
                  {  // concentrateur uniquement
                      uint32_t cpt=1;
                      uint32_t code_teste=0;
                      code_tournant = * (uint32_t *)(message_in+5);
                      code_teste = knownNodes[rxrf_numero_device-1].code_tournant;
                      cpt = 1;
                      while (cpt < 100)
                      {
                          if (code_teste == code_tournant) break;
                          else
                          {
                              cpt++;
                              code_teste = code_tournant_suivant(code_teste);
                          }
                      }
                      if (cpt < 100)  // bon code tournant
                      {
                          knownNodes[rxrf_numero_device-1].code_tournant = code_tournant_suivant(code_teste);
                          code_tournant_timer = rand();
                          message[1] = 3;
                          message[2] = 'E';
                          message[3] = 'C';
                          message[4] = '2';
                          message[5] = code_tournant_timer;
                          //envoi_immediat = 1;
                          envoi_message_def();  // envoi EC2 au node
                          code_tournant_time1 = NB_TICKS + code_tournant_timer*2*100 + 0x630;  // min 68C=16ms
                      }
                      /*strcpy((char *)message, "Lessai code tournant:xxxx xxxx ");
                      ajout_message_32b(cpt, 21);
                      envoi_message ();*/
                  }
                  if ((message_in[4] == '2') &&  (longueur_m == 6)) // Node :QEC2x reception timer du concentrateur
                  {
                      DELAY_MS (message_in[5]*2);
                      strcpy((char *)message+1, "EC3");  // Envoi EC3 au concentrateur
                      Mess_statut = 0x20;  // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                      envoi_message ();
                  }
                  if ((message_in[4] == '3') &&  (longueur_m == 5)) // QEC3 reception par concentrateur, apr�s timer
                  {
                      uint32_t code_tournant_diff;
                      code_tournant_diff = NB_TICKS - code_tournant_time1;
                      if ((code_tournant_diff < 200)  && (rxrf_numero_device))  // 2 ms
                      {
                          strcpy((char *)message, "LxPMC0");
                          message[1]=knownNodes[rxrf_numero_device-1].adresse;
                          message[6]=car_fin_trame;
                          transf_buff (6);  // Transfert dans buffer d'envoi  et envoi
                      }
                  }

              }
              #endif

              a = message_in[4]-'0';
              if ((message_in[3] == 'N') && (a < NB_ENTREES) && (longueur_m == 7))   // ENxyz : Activation entrée X pdt Y sec à z
                {
                    //activation_entree (a, message_in[5]-'0', message_in[6]-'0');
                }
              if ((message_in[3] == 'N') && (a < NB_ENTREES) && (message_in[5]=='?') && (longueur_m == 6))  // ENx? : interog entrée X
                {
                    message[1] ='E';
                    message[2] ='N';
                    message[3] = a +'0';
                    message[4] = ':';
                    //message[5] = HAL_GPIO_ReadPin(h_etat_entree[a].entree_port, h_etat_entree[a].entree_pin) + '0';
                    message[6] = car_fin_trame;
                    envoi_message();
              }
           }


          // ******************************** HHHHHHHHHHHHHHHHHHH  ********************

          if (message_in[2] == 'H')                               // HL et HE  Heure/Date
          {
              uint32_t secondes_mes;
              //  ts ISeconds_Time;  // nanosecondes

              if (message_in[3] == 'L')
                {
                  if (message_in[4] == 'H')   lecture_heure_rtc();  // HLH Heure  QL2HLH=0xD1 0x4C 0x02 0x48 0x4C 0x48 send numbers
                  if (message_in[4] == 'D')   lecture_date_rtc();   // HLD Date
                  if (message_in[4] == 'S')                         // HLS nb de secondes depuis 1900
                  {
                      NB_SECONDES(secondes_mes); //Time(ISeconds_Time *ts);  // seconds et nanosecondes
                      message[1]=7;  // longueur
                      message[2]='H';
                      message[3]='E';
                      message[4]='S';
                      memcpy(message+5,  &secondes_mes,4);
                      #ifdef CC13xx
                          Seconds_Time ts;  // sec & nanosecondes
                          Seconds_getTime(&ts);  // seconds et nanosecondes
                          message[9] = ts.nsecs/4000000;
                      #else
                        message[9] = 0;
                      #endif
                      Mess_statut = 0x02;  // 0x30 bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                      // *((uint32_t)(message+3)) =  secondes_mes;
                      envoi_message_def();
                  }
                  if ((message_in[4] == 'X') )                          //  HLX lecture vitesse RTC
                  {
                    #ifdef CC13xx
                      strcpy((char *)message+1, "HX tuning : ");  // Envoi de la derive au PC
                      uint32_t subsec_inc;
                      subsec_inc = HWREG( AON_RTC_BASE + AON_RTC_O_SUBSECINC );
                             // = (( subSecInc       ) & AUX_WUC_RTCSUBSECINC0_INC15_0_M  );    // 0x0000FFFF
                             // HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1 ) = (( subSecInc >> 16 ) & AUX_WUC_RTCSUBSECINC1_INC23_16_M );    // 0x000000FF

                      code_hex_dec((uint8_t*) &subsec_inc, message+12, 3,0);
                      message[18] = car_fin_trame;
                      envoi_message();
                    #endif
                  }
                }
              if (message_in[3] == 'E')
                {
                     if ((message_in[4] == 'H') && (longueur_m == 11))      //  HEH101111
                     {
                         ecrit_heure_rtc();
                     }
                     if ((message_in[4] == 'D') && (longueur_m == 11))      //  HED010117
                     {
                         ecrit_date_rtc();
                     }
                     if ((message_in[4] == 'T') && (longueur_m == 5))         //  HET Demande de mise a l'heure par HLS/HES
                     {
                         strcpy((char *)message, "QHLS");
                         NB_SECONDES(validite);
                         Mess_statut = 0x20;  // Ack,RX apres  bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:differe   bit4:pas d'ack  bit5:RX apres
                         envoi_message ();
                     }
                     if ((message_in[4] == 'S') && ((longueur_m >= 9) || (longueur_m <= 15)))         //  HESxxxxyzzblf y:0,1s  z:nb millis prochaine fenetre
                     {
                         // Verification que le message recu est valide (moins de 30 sec apres la demande)
                         int32_t secondes_derive;
                         uint32_t secondes_prec;
                         uint8_t secondes_sens;


                         NB_SECONDES(secondes_prec);
                         secondes_derive = secondes_prec - validite; // entre le moment d'envoi et de reception
                         if (secondes_derive < 0) secondes_derive = -secondes_derive;


                        #ifdef ESP32
                             //printf ("maj heure %i\n", secondes_derive);
                             //printf ("long:%d lumi%d format:%d\n", longueur_m, message_in[13], message_in[14]-'0');
                            if (longueur_m == 15)
                            {
                                bal_proch_fenetre = (((uint16_t)message_in[10])<<8) + message_in[11];
                                bal_periode = message_in[12];
                                lumino_cc13 = message_in[13];
                                cam_format = message_in[14]-'0';
                                cam_format_conv();   // cam_format ->framesize, qualite, fps
                                if (Log_verbose > 3)
                                    printf ("---balise %d fenetre:%i lumi:%d - %d\n", bal_periode, bal_proch_fenetre, lumino_cc13, cam_format);
                            }
                             ArretWLSN ();
                        #endif

                         if (secondes_derive < 30)  // sinon non prise en compte du message
                         {
                             /*uint8_t i;
                             message[0]='L';
                             for (i=0; i<longueur_m; i++)
                                 message[1+i]=message_in[i];  // transfert au PC
                             message[i]=car_fin_trame;
                             envoi_message();*/

                             memcpy(&secondes_mes, message_in+5, 4);  // TODO le dernier octet (ms) n'est pas traite

                             if ((message_in[1] & 0x7F) == 'P')
                                 secondes_mes += 3600u*24*(30*365-22+25567); // en provenance de P : recaler 2000 vers 1970 puis 1900

                             #ifdef ESP32   // retirer 70 ans
                                 secondes_mes -= 3600u*24*(70*365 + 17); // en provenance de P : recaler 2000 vers 1970 puis 1900
                             #endif

                             secondes_sens='+';
                             if (secondes_prec > secondes_mes)
                             {
                                 //printf ("neg \n");
                                 secondes_derive = secondes_prec - secondes_mes;
                                 secondes_sens='-';
                             }
                             else
                                 secondes_derive = secondes_mes - secondes_prec;


                             //printf ("ttt %d %i %i\n", secondes_derive, secondes_mes, secondes_prec);

                             if (secondes_derive > 2)  // S'il y a derive de plus de 2 secondes
                             {
                                 if (secondes_derive < 1000)  // sinon c'est une initialisation / recalage
                                 {
                                     strcpy((char *)message, "LHSx:derive=+   x");  // Envoi de la derive au PC
                                     message[3]= My_Address;
                                     message[12]= secondes_sens;
                                     envoi_int_decimal(secondes_derive, 12);
                                     Mess_statut = 0;
                                     envoi_message ();
                                 }
                                 else
                                 {
                                     //printf ("uuu %d\n", secondes_derive);  // init
                                 }
                                 struct timeval tv;
                                 tv.tv_sec = (time_t) secondes_mes;
                                 //printf ("ecrit %i %li\n", secondes_mes, tv.tv_sec);  // init
                                 ECRIT_SECONDES(tv);  // mise a jour de l'heure (uniquement si derive de plus de 2 secondes)
                             }
                             #ifdef CC13xx
                             if (compteur_PHLS)  // premiere mise � l'heure reussie
                             {
                                compteur_PHLS = 1;  // fin maj initial et demarrage periodique
                                #ifdef C10_ElecVanne  // demarrage programme a 21heures
                                    demar_prog_21h(secondes_mes);
                                #endif
                             }
                             #endif

                         }
                         else
                         {
                           #ifdef DEBUG_M
                             strcpy((char *)message, "LHEcriture 30:       ");  // Envoi de la raison non prise en compte au PC
                             envoi_int_decimal(secondes_derive, 14);
                             //Mess_statut = 0;
                             //envoi_message ();
                           #endif
                         }
                     }
                     if ((message_in[4] == 'X') && (longueur_m == 9))         //  HEXxxxxx
                     {
                         memcpy(&secondes_mes, message_in+5, 4);  // TODO le dernier octet (ms) n'est pas traite
                         struct timeval tv;
                         tv.tv_sec = (time_t) secondes_mes;
                         ECRIT_SECONDES(tv);  // mise a jour de l'heure
                     }
                     #ifdef CC13xx
                     if ((message_in[4] == 'X') && (longueur_m == 11))                          //  HEXxxxxxx  modification vitesse RTC
                     {
                         uint32_t subsec_inc;
                         message[0]='L';  // renvoie a l'envoyeur
                         uint8_t i;
                         for (i=0; i<longueur_m; i++)
                             message[1+i]=message_in[i];  // transfert au PC
                         message[i]=car_fin_trame;
                         envoi_message();
                         subsec_inc=0;
                         code_dec_hex(message_in+5, (uint8_t *) &subsec_inc, 3);
                         SetupSetAonRtcSubSecIncOK(subsec_inc);  // augmentation fr�quence RTC * 2  augmenter de 36
                     }

                     if ((message_in[4] == 'X') && (longueur_m == 11))                          //  HEXxxxxxx  modification vitesse RTC
                     {
                         uint32_t subsec_inc;
                         subsec_inc=0;
                         code_dec_hex(message_in+5, (uint8_t *) &subsec_inc, 3);
                         SetupSetAonRtcSubSecIncOK(subsec_inc);  // augmentation fr�quence RTC * 2  augmenter de 36
                     }
                     #endif
                }
          }



          // ******************************** PPPPPPPPPPPPPPPPPPPPP   ********************

          #ifdef CC13xx
          if ((message_in[2] == 'P') && (message_in[3] == 'E'))                               // PE : Periodique
          {
              if ((message_in[4] == 'M') && (longueur_m==6))     // PEMx mode arret 0:marche 1�3:reduit 4:arret
              {
                  if ((message_in[5] >= '0') && (message_in[5] <= '4'))
                      chgt_mode_arret(message_in[5]-'0');
              }
              if ((message_in[4] == 'A') && (longueur_m==6))     // PEA1/0 : Active le message p�riodique
              {
                  if (message_in[5] == '0')                      //   PEA0 : D�sactive le message periodique
                  {
                      period_activ=0;
                      Maj_clock_periodique();
                  }
                  if (message_in[5] == '1')                     //    PEA1 : Active le message periodique
                  {
                      period_activ=1;
                      Maj_clock_periodique();
                  }
              }
              if ((message_in[4] == 'P') && (longueur_m==7))     // PEPxx : periode en secondes
              {
                  period_periode = (message_in[5]-'0')*10 + message_in[6]-'0';
                  Maj_clock_periodique();
              }
              if ((message_in[4] == 'T') && (longueur_m==6))     // PETx : Type de message : 1:temp�rature, 2:message RSSI
              {
                  period_type = message_in[5]-'0';
              }
          }
          if ((message_in[2] == 'P') && (message_in[3] == 'L'))
          {
              if ((message_in[4] == 'M') && (longueur_m==5))     // PLM lecture mode arret 0:marche 1�3:reduit 4:arret
              {
                  strcpy((char *)message+1, "Mode arret:x");
                  message[12] = mode_arret+'0';
                  envoi_message ();
              }
          }
          #endif // fin du CC13xx

          // ******************************** RRRRRRRRRRRRRRRRRRRRRR   ********************

          #ifdef CC13xx
          if (message_in[2] == 'R')                               // R : Radio
          {
              traitement_message_radio(longueur_m);   // Traitement message dans radiotask.c

              if (message_in[3] == 'L')
              {
                  if (message_in[4] == 'A')                      //   RLA : Lecture Comptage_radio_RXcallback
                  {
                      envoi_cpt_radio_CB();
                  }
                  if (message_in[4] == 'B')                      //   RLB : Lecture index_test
                  {
                      envoi_cpt_index_test();
                  }

                  if (message_in[4] == 'C')                      //   RLC : Lecture mode RX et periode allumage Radio
                  {
                      strcpy((char *)message+1, "Radio Mode:- Allumage:00- pdt  xms period:xmin");
                      chgt_mode_radio(1, 1, 0);  //1:lecture 1:config
                      envoi_message ();
                  }
                  if (message_in[4] == 'D')                      //   RLD : envoi mess erreur test
                  {
                      //envoi_mess_err_test();
                  }


                  if (message_in[4] == 'P')                      //   RLP : Lecture Power Tx
                  {
                      strcpy((char *)message+1, "RLP Power Tx: xx");
                      int8_t power_get;
                      EasyLink_getRfPower(&power_get);
                      ajout_message_8b ((uint8_t)power_get, 15);
                     envoi_message ();
                  }
                  if ((message_in[4] == 'R') )      //   RLR : Envoi niveau de reception � Q puis L
                  {
                      #ifdef CONCENTRATOR
                      if (longueur_m == 6)  // RLRx : envoie 30 requetes � x, 1 par seconde
                      {
                          envoie_message_reg(message_in[5],1,30);
                      }
                      else if (longueur_m<20)  // ajoute le dernier niveau rssi reception et envoie a L
                      {
                          memcpy(message, message_in+1, longueur_m);
                          message[0]='L';
                          message[longueur_m-1]=' ';
                          ajout_int8_message(retourne_last_rssi(), longueur_m, 0);
                          envoi_message ();
                      }
                      #endif
                      #ifdef NODE
                      if (longueur_m==5)
                      {
                          #ifdef DEBUG_reseau
                              uint8_t aa = 'R';
                              //  message[0]= cpt_perte_balise_synchro+'0';
                              UART_write(hUart2, &aa, 1);
                          #endif
                          strcpy((char *)message, "QRLRx:    ");
                          message[4] = message_in[0];
                          ajout_int8_message(retourne_last_rssi(), 6, 0);
                          envoi_message();
                      }
                      #endif
                  }
                  if (message_in[4] == 'S')
                  {
                      if (longueur_m==5)          //   RLS : Lecture statut (Eteint/allume)
                      {
                          //uint8_t rx_actif;
                          //rx_actif = EasyLink_get_running();    // 0:eteint   0x0A:RX activ�
                          //if (rx_actif) rx_actif=1;
                          strcpy((char *)message+1, "Radio Phase:-  Actif:-");
                          message[13]=Phase_radio()+'0';
                          //message[22]=rx_actif+'0';
                          message[22]= EasyLink_get_running()? '1' : '0' ;
                          envoi_message ();
                      }
                  }
                  a = numero_device_rf(message_in[5]);  // lettre -> numero
                  if (a)
                  {
                      a--;
                      if ((message_in[4]=='T') && (longueur_m==6))  //  RLTx  : Lecture statut des message supprimes et RX/TX echec
                      {
                          strcpy((char *)message+1, "Radio TX:x renvoi:xx  TX rate:xx  TX supp:xx RX-err:xx  Pile:   ");
                          message[10]=knownNodes[a].adresse;
                          ajout_message_8b(knownNodes[a].nb_trans_renvoye, 19);
                          ajout_message_8b(knownNodes[a].nb_trans_ratees, 31);
                          ajout_message_8b(knownNodes[a].nb_messages_effaces, 43);
                          ajout_message_8b(knownNodes[a].nb_rx_err, 53);
                          uint8_t nb_pile;
                          #ifdef PILE_FLASH
                              uint8_t b = a + OFFSET_FLASH;
                              nb_pile = Pile_F[b].head - Pile_F[b].tail;
                              if (Pile_F[b].head < Pile_F[b].tail)
                                  nb_pile = Pile_F[b].head - Pile_F[b].tail + (uint32_t)Pile_F[b].taille<<12);
                          #else
                              nb_pile = knownNodes[a].head - knownNodes[a].tail;
                              if (knownNodes[a].head < knownNodes[a].tail)
                                  nb_pile = knownNodes[a].head - knownNodes[a].tail + TAILLE_PILE_ENVOI_RADIO;
                          #endif
                          ajout_message_8b(nb_pile, 62);
                          envoi_message ();
                      }
                      if ((message_in[4]=='U') && (longueur_m==6))  //  RLUx  : Lecture mode_rx
                      {
                          strcpy((char *)message+1, "Radio TX:x mode:x  recu:xx    envoi:xx   ");
                          message[10]=knownNodes[a].adresse;
                          message[17]=knownNodes[a].mode_rx + '0';
                          ajout_message_16b(knownNodes[a].nb_reception, 25);
                          ajout_message_16b(knownNodes[a].nb_transmission, 37);
                          envoi_message ();
                      }
                  }
              }
              if (message_in[3] == 'E')
              {
                  if ((message_in[4] == 'A')  && (longueur_m==6))                    //   REAx : Ecriture affichage Rssi
                  {
                      #ifdef CONCENTRATOR
                      modif_affiche_Rssi(message_in[5]-'0');
                      #endif
                   }
                  if ((message_in[4] == 'M') && (longueur_m==6) && (message_in[5] < '7') &&  (message_in[5] > '0'))          //   REMx : Ecriture mode RX
                  {
                      chgt_mode_radio(2, 1, message_in[5]-'0');  //mode 2:ecriture  type:1
                      mess_ok=1;
                  }
                  if ((message_in[4] == 'P') && (longueur_m==7))         //   REPxx : Ecriture Power Tx (d�cimal : 00 pour -10, 01 pour 0, 15 pour 14
                  {
                      char signed as;
                      as = (message_in[5]-'0')*10 + message_in[6]-'0';
                      if (as) as=as-1; else as=-10;
                      change_power_radio(as);
                  }
                  if ((message_in[4] == 'P')  && (longueur_m==8))                    //   REPxxx : Ecriture periode allumage Radio
                  {
                      i32 = (uint16_t)(message_in[5]-'0')*10 + message_in[6]-'0';      // secondes
                      if (message_in[7]=='h')            i32 = i32*3600;        // heures
                      else
                          if (message_in[7]=='m')         i32 = i32*60;          // minutes
                      chgt_mode_radio(2, 2, i32);  //2:ecriture  2:period
                  }
                  if ((message_in[4] == 'P') && (message_in[5] == 'A') && (longueur_m==9))     //   REPAxxx : Ecriture periode allumage Radio
                  {
                      uint8_t i8;
                      i8 = (message_in[6]-'0')*100 + (message_in[7]-'0')*10 + message_in[8]-'0';

                      /*i32 = (uint16_t)(message_in[5]-'0')*10 + message_in[6]-'0';      // secondes
                      if (message_in[7]=='h')            i32 = i32*3600;        // heures
                      else
                          if (message_in[7]=='m')         i32 = i32*60;          // minutes   */
                      chgt_mode_radio(2, 2, i8);  //2:ecriture  2:period
                  }
                  #ifdef CONCENTRATOR
                     if ((message_in[4] == 'R')  && (longueur_m==7))                    //   RERxy : Ecriture mode rapide x:nb repet y:nb sec*2 entre chaque repet
                     {
                         modif_repet_radio (message_in[5]-'0', (message_in[6]-'0')*2);
                     }
                  #endif
                  }
          }
          #endif

          // ******************************** SSSSSSSSSSSSSSSSSSSSSS  ********************

          if (message_in[2] == 'S')                               // S : Statut
          {
              if (message_in[3] == 'L')
              {
                    if (message_in[4] == 'O')       // SLO : lecture statut
                    {
                        #ifdef CC13xx               // CC13xx
                            strcpy((char *)message+1, "Statut:     s  ");

                            // Indication du temps restant allumage ESP (0 si eteint)
                            #ifdef C10_CAMERA_PIR  // statut
                              uint16_t tps_restant=0;
                              if (SortieTor[Enab_ESP].consigne == 1)  // Si ESP32 actif
                              {
                                  tps_restant = resteclockESP();
                              }
                              ajout_int16_message ( tps_restant, 10,0x82);
                            #endif

                            // indication des bits de statut
                            uint8_t i, index;
                            index = 16;
                        #endif
                        #ifdef ESP32                              // ESP32
                            strcpy((char *)message+1, "Statut: Wifi:   ");
                            message[14] = wifi_mode() + '0';

                            uint8_t i, index;
                            index = 17;
                        #endif

                        for (i = 0; i<8; i++)
                        {
                            if (T_BIT(StatutOps, 7-i))  // Bit0:activ PIR  Bit1:activ PIR+Cam(necessite b0 aussi)
                            {
                                message[index++] = 'b';
                                message[index++] = 7-i+'0';
                                message[index++] = ' ';
                            }
                        }
                        message[index] = car_fin_trame;
                        envoi_message ();
                    }

                    #ifdef FLASH_EXTERNE
                    if ((message_in[4] == 'I'))
                    {
                        if (message_in[5] == 'F')  //   SLIF : lecture index flash
                        {
                            uint8_t i;
                            for (i=0; i<NB_PILES_F; i++)
                            {
                                strcpy((char *)message+1, "Index Fl H:....-.... T:....-....");
                                ajout_message_32b(Pile_F[i].head, 12);
                                ajout_message_32b(Pile_F[i].tail, 24);
                                envoi_message ();
                            }
                        }
                        if (message_in[5] == 'E')  //   SLIE : lecture index Eeprom
                        {
                            strcpy((char *)message+1, "Index Eeprom:....");
                            ajout_d_word_message(index_EEPROM, 14);
                            envoi_message ();
                        }
                    }
                    #endif

/*                  if ((message_in[4] == 'N') && (longueur_m == 5)) // SLN : Lecture configuration Noyau S : 0:pas de sleep, 1:sleep timer, 2:sleep
                    {           //     F : 0:100ms 1:250ms  2:500ms  4:1s 8:2s 40:10s
                      lecture_config_noyau ();//     R : 0:Idle  1:RX_timer   2:RX_continu
                    }               //     T : Timer RX : 1:100ms  2:200ms 5:500ms  A:1s*/

                  if ((message_in[4] == 'E') && (longueur_m == 5))     //          SLE  :  Lecture des Erreurs
                    {
                      if (comptage_erreur > nb_erreurs_enregistrees)
                          comptage_erreur = nb_erreurs_enregistrees;
                      message[1] = 'E'; // => LEL reset;nb erreurs-num erreur-donn�e1;donn�e2
                      message[2] = 'L';
                      message[3] = ' '; //nb_reset + 48;
                      message[4] = comptage_erreur + 48;
                      message[5] = ':';
                      a = 6;
                      for (b = 0;
                          ((b < comptage_erreur)
                              && (a < (MESSAGE_SIZE - 6))); b++)
                        {
                          message[a++] = deci (enr_erreur[b] >> 4);
                          message[a++] = deci (enr_erreur[b] & 15);
                          message[a++] = '-';
                        }
                      a--;
                      message[a] = car_fin_trame;
                      envoi_message ();

                      raz_erreur ();

                      // nb_reset = 0;
                    }

                  if ((message_in[4] == 'V') && (longueur_m == 5))     //          SLV  :  Lecture du type et version de l'appli
                  {
                      strcpy((char *)message+1, "SV Type:C V:00");
                      message[9] = CODE_APPLI;
                      message[13] = deci (CODE_VERSION > 4);
                      message[14] = deci (CODE_VERSION & 15);
                      envoi_message ();
                  }

                  if ((message_in[4] == 'P') && (message_in[5] == '8') && (longueur_m == 7))  // SLP8x : Lecture parametre 8bit variable
                  {
                      uint8_t param;
                      param = message_in[6]-'0';
                      if (param <PAR_VAR_MAX)
                      {
                        strcpy((char *)message+1, "S Param:   ");
                        message[9] = param + '0';
                        message[10]=':';
                        message[11] = deci (par8_var[param] >> 4);
                        message[12] = deci (par8_var[param] & 15);
                        message[13] = car_fin_trame;
                        envoi_message ();
                      }
                  }
                  if ((message_in[4] == 'P') && (message_in[5] == 'H') && (longueur_m == 7))  // SLPHx : Lecture parametre 16bit variable
                  {
                      uint8_t param;
                      param = message_in[6]-'0';
                      if (param <PAR_VAR_MAX)
                      {
                        strcpy((char *)message+1, "S Param:   ");
                        message[9] = param + '0';
                        message[10]=':';
                        ajout_d_word_message (par16_var[param], 11);
                        message[15] = car_fin_trame;
                        envoi_message ();
                      }
                  }

                }

              if (message_in[3] == 'E')
              {
                  if ((message_in[4] == 'O')  && (longueur_m == 7))      // SEOxy : ecriture statut bit x � 0 ou 1
                  {
                      if (message_in[6] == '0')  // desactivation
                      {
                          StatutOps &= ( ~(1 << (message_in[5]-'0')));
                      }
                      if (message_in[6] == '1')  // activatio
                      {
                          StatutOps |=  (1 << (message_in[5]-'0'));
                      }
                  }
                  if ((message_in[4] == 'N') && (longueur_m == 7)) // SENxy : Ecriture configuration Noyau x=y
                    {
                      //ecriture_config_noyau ();
                    }
                  if ((message_in[4] == 'V') && (message_in[5] == 'B') && (longueur_m == 6)) // SEVB : Raz valeur voltage batterie
                  {
                      volt_mesure_bas = 844;
                  }
                  if ((message_in[4] == 'P') && (message_in[5] == '8'))
                  {
                      uint8_t param;
                      param = message_in[6]-'0';
                      if ((param < PAR_VAR_MAX) && (longueur_m <= 10))      // SEP8x0  SEP8x255 : ecriture parametre x
                      {
                          par8_var[param]  = (uint8_t) decodage16i_message_in( 3 , 6 );  // 6e position
                          mess_ok=1;
                      }
                  }
                  if ((message_in[4] == 'P') && (message_in[5] == 'H'))   // SEPHx0 � SEPHx65500
                  {
                      uint8_t param;
                      param = message_in[6]-'0';
                      if ((param < PAR_VAR_MAX) && (longueur_m <= 12))      // SEPHx0  SEPHx255 : ecriture parametre x
                      {
                          par16_var[param]  = (uint16_t) decodage16i_message_in( 5 , 6 );  // 6e position
                          mess_ok=1;
                      }
                  }
                }
                #ifdef DEBUG_TIME
                if ((message_in[3] == 'D') && (message_in[4] == 'E') && (message_in[5] == 'B')   \
                        && (message_in[6] == '1') && (longueur_m == 7))  //  SDEB1  : Debug1:recupe debug_time
                {
                  #ifdef PILE_RF_FLASH // sinon ca fait trop de donn�es
                    uint32_t time0, time_a;
                    time0 = time_cpu[0];
                     for (a=0; a<index_time; a++)
                     {
                         strcpy((char *) message,"LSDEB1: ");
                         message[7] = deci(a>>4);
                         message[8] = deci(a & 0x0F);
                         message[9] = ':';

                         message[10] = deci(etat_cpu[a]>>4);    // etat_cpu
                         message[11] = deci(etat_cpu[a] & 0x0F);
                         message[12] = '-';

                         time_a = time_cpu[a] - time0;
                         time0 = time_cpu[a];
                         ajout_int16_message(time_a, 13, 0);  // time par rapport au pr�c�dent
                         message[19] = '-';

                         ajout_message_16b (etat_detail[a], 20);  // d�tail
                         message[24] = car_fin_trame;
                         envoi_message();

                     }
                     index_time=0;  // Raz
                     trigger_time=0;
                  #endif
                }
                #endif
            }

          // ******************************** TTTTTTTTTTTTTTTTTTT  ********************

          #ifdef CC13xx
          if (message_in[2] == 'T')            //         TEST
          {
              if ((message_in[3] == 'P') && (message_in[4] == 'L')
                      && (message_in[5] == 'Z') && (longueur_m == 6)) //   TPLZ : plante - test watchdog
              {
                  writeToAddress ( (void*) 19,4);
              }

              if ((message_in[3] == 'A') ) //   TAx   : envoi de la structure des nodes
              {
                  a = numero_device_rf(message_in[4]);
                  if (a)
                  {
                      strcpy ((char *)message+1, "Node x:");
                      message[6] = message_in[4];
                      code_hex_dec((uint8_t*) &knownNodes[a-1], message+8, 14, 1);
                      message[7+14*3] = car_fin_trame;
                      envoi_message ();
                  }
                  else
                  {
                      strcpy ((char *)message+1, "Node x inconnu");
                      message[6]=message_in[4];
                      envoi_message ();
                  }
              }

              if ((message_in[3] == 'B') && (longueur_m == 11)) //   TBxx : test de comm x message successifs
               //   ZxTBA4LLL10  ZxTBA4Qzz10   ZxTBA4QZL10   1, 4, et 8 (x message succ vers L, Q-z, Q-Z-L)
              {
                  if (message_in[4] == 'A')   // TBAx plusieurs messages vers Q
                  {
                      uint8_t cpt_te;
                      //ACTIV_SORTIE(0, 1, 10);  // LED2 2 flash
                      //ACTIV_BUZ(0, 2000, 4000, 5);
                      for (cpt_te = 0; cpt_te < message_in[5]-'0'; cpt_te++ )
                      {
                          memcpy ((char *)message, message_in+1, 12); // -> QTBB3QZL10
                          message[0] = message_in[6];
                          message[3] = 'B';
                          message[4] = cpt_te + '1';
                          Mess_statut = ((message_in[9]-'0')<<4) + message_in[10]-'0';
                          envoi_message ();
                      }
                  }
                  if (message_in[4] == 'B')   // TBBx message vers Z
                  {
                      memcpy ((char *)message, message_in+1, 12); // -> ZTBC3QZL10
                      message[0] = message_in[7];
                      message[3] = 'C';
                      Mess_statut = ((message_in[9]-'0')<<4) + message_in[10]-'0';
                      envoi_message ();
                  }
                  if (message_in[4] == 'C')   // TBCx message vers L
                  {
                      memcpy ((char *)message, message_in+1, 12); // -> QTBD3QZL10
                      message[0] = message_in[8];
                      message[3] = 'D';
                      Mess_statut = ((message_in[9]-'0')<<4) + message_in[10]-'0';
                      envoi_message ();
                  }
              }
              if ((message_in[3] == 'C') )                          //  TC
              {
                  if (message_in[4] == 'A')   // TCA
                  {
                      strcpy ((char *)message, "ZTCA");  // boule infinie sur la tache
                      envoi_message ();
                  }
                  if (message_in[4] == 'B')   // TCB
                  {
                      while (1)  // boule infinie immediate
                      {

                      }
                  }
              }
              if ((message_in[3] == 'D') && (message_in[4] == '1'))                          //  TD1
              {
                  strcpy ((char *)message+1, "test:xx xx");
                  message[6] = deci(message_test[5]>>4);
                  message[7] = deci(message_test[5]&15);
                  message[9] = deci(message_test[6]>>4);
                  message[10] = deci(message_test[6]&15);
                  Mess_statut = 0x02; // pas d'Ack // bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)  bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                  envoi_message ();
              }
              if ((message_in[3] == 'D') && (message_in[4] == '2'))                          //  TD2
              {
              }

              if ((message_in[3] == 'E') ) //   TExxx   :  envoie un nb de caracteres demandes, en hexa, 2 messages
              {
                  uint8_t i,nb;
                  nb = (message_in[4]-'0')*100 + (message_in[5]-'0')*10 + message_in[6] -'0';
                  if (nb < MESSAGE_SIZE)
                  {
                      strcpy ((char *)message, "LAAAA");
                      for (i=4; i<nb; i++)
                          message[i]= (i%10) + '0';
                      message[nb] = car_fin_trame;
                      //envoi_message ();

                      strcpy ((char *)message, "LBYBBB");
                      for (i=5; i<(nb+4); i++)
                          message[i]= (i%10) + '0';
                      message[1] = nb;
                      envoi_message_def ();
                  }
              }
              if ((message_in[3] == 'F') ) //   TFxxx   :  envoie un nb de caracteres demandes
                {
                  strcpy ((char *)message, "LUT");
                  uint8_t i,nb;
                  nb = (message_in[4]-'0')*100 + (message_in[5]-'0')*10 + message_in[6] -'0';
                  if (nb < MESSAGE_SIZE)
                  {
                      for (i=3; i<nb; i++)
                          message[i]= (i%10) + '0';
                      message[nb] = car_fin_trame;
                      envoi_message ();
                  }
                }
              if ((message_in[3] == 'Y') && (longueur_m == 5)) //   TYx
              {
                  uint16_t test1, t2, t3;
                  t3 = message_in[4]-'0';
                  for (test1=0; test1<(t3); test1++)
                  {
                      strcpy((char *)message, "L000AABBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBCCCCCCCCCCA0AABBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBCCCCCCCCCCA0111");
                      for (t2=2; t2<6; t2++)
                      {
                          message[t2]=test1+'0';
                      }
                      Mess_statut = 0x00;  // Ack/RX apres - bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois) bit3:diff�r�   bit4:pas d'ack  bit5:RX apres
                      envoi_message ();
                  }
              }
              if ((message_in[3] == '7') ) //   T7   :  reception de la reponse a T6
              {
                  ACTIV_PWM ( 0,1,1,1300,2);        // Bip buzzer
                  ACTIV_SORTIE(LED_cpu, 2, 20);  // activation Led N�0, mode:1, duree 0.6s
              }
          }
          #endif  // fin du CC13xx


          // ******************************** VVVVVVVVVVVVVVVVVVVV  ********************

          // Fonctions Lecture registre (VE/VL)

          if (message_in[2] == 'V')    // VE/VL : Ecriture/Lecture d'un registre
          {
              tempo1 = hexa (message_in[5]) * 16 + hexa (message_in[6]);
              tempo2 = hexa (message_in[7]) * 16 + hexa (message_in[8]);
              tempo3 = hexa (message_in[9]) * 16 + hexa (message_in[10]);
              tempo4 = hexa (message_in[11]) * 16 + hexa (message_in[12]);

              if (message_in[3] == 'E') // Ecriture Registre : VERxxxxyy (ram) VEExxxxyy (EEprom)
                {
                  if ((message_in[4] == 'R') && (longueur_m == 11)) // Ecriture Ram : VERxxxxyy
                    {
                      pregistre_xdata = ( char unsigned *) (SRAM_BASE + (uint32_t)(tempo1*256) + tempo2);
                      *pregistre_xdata = tempo3;
                    }
                  if ((message_in[4] == 'E') && (longueur_m == 11)) // Ecriture EEprom : VEExxxxyy
                    {
                     // pregistre_xdata = ( char unsigned *) (DATA_EEPROM_BASE + (uint32_t)(tempo1*256) + tempo2);
                     // *pregistre_xdata = tempo3;
                    }
                }
              if (message_in[3] == 'L')   // Lecture Registres    VLRxxxx  VLExxxx  VLXxxxxxxxx
              {
                  if ((message_in[4] == 'V') && (longueur_m == 5))        // VLV Lecture Part number-serial-numero serie
                  {
                      uint32_t part_number;
                      part_number =    ( HWREG(FCFG1_BASE + 0x00000118  )) ;  // FCFG1_O_ICEPICK_DEVICE_ID
                      strcpy((char *)message+1, "V Part Num: ");
                      ajout_message_32b (part_number, 13);
                      message[22] = car_fin_trame;
                      envoi_message();

                  }
                  if ((message_in[4] == 'R') && (longueur_m == 9))        // Lecture RAM : VLRxxxx  0x0 à 0x5000 en full word (32bits)
                    {
                      //pregistre =  (__data unsigned char *) (tempo1);
                      //tempo2 = *pregistre;
                      //pregistre_xdata = ( unsigned char *) pregistre;
                      if (tempo1 > 0x4F) tempo1=0;
                      pregistre_int_xdata = ( int unsigned *) (SRAM_BASE + (uint32_t)(tempo1*256) + (tempo2 & 0xFC));
                      //valeur_mem = *pregistre_int_xdata;
                      envoie_1_word ( (int unsigned *) pregistre_int_xdata);
                    }
                  if ((message_in[4] == 'E') && (longueur_m == 9))           // Lecture EEPROM : VLExxxx   0x0 à 0x17FF en full words (32 bits)
                    {
                      if (tempo1 > 0x17) tempo1=0;
                      //pregistre_xdata = ( char unsigned *) (DATA_EEPROM_BASE + (uint32_t)(tempo1 *256) + tempo2);
                      //tempo2 = *pregistre_xdata;
                      //envoie_1octet ( (uint8_t *) pregistre_xdata);
                    }
                  if ((message_in[4] == 'X') && (longueur_m == 13))          // Lecture Memory : VLXxxxxxxxx  0x0800 à 0x0802
                    {
                      //tempo1=0x08;
                      //if (tempo2 > 2) tempo2=0;
                      pregistre_int_xdata = ( int unsigned *) ((uint32_t)(tempo1<<24) + (uint32_t)(tempo2<<16)
                          + (uint32_t)(tempo3<<8) + tempo4);
                      //valeur_mem = *pregistre_int_xdata;
                      envoie_1_word ((int unsigned *) pregistre_int_xdata);
                    }
                  if ((message_in[4] == 'D') && (longueur_m == 9))           // VLDxxxx : Lecture 16 Registres en RAM
                    {
                      if (tempo1 > 0x4F) tempo1=0;
                      pregistre_int_xdata = (int unsigned *) (SRAM_BASE + (uint32_t)(tempo1*256) + (tempo2 & 0xFC));
                      envoie_16octets ( (uint32_t *) pregistre_int_xdata);
                    }
                  if ((message_in[3] == 'L') && (message_in[4] == 'P')
                      && (longueur_m == 10))                                 // VLPxxxxy : Lecture RAM partielle
                    {
                      pregistre_int_xdata = ( int unsigned *) (SRAM_BASE + (uint32_t)(tempo1*256) + (tempo2 & 0xFC) );
                      dump_ram_fin = message_in[9] - '0';  // fin ram
                      dump_ram_timer = 10; // 1 ligne de 55 car toutes les 25ms = 20kb/s
                    }
                }
            }

          // ******************************* WATCHDOG *********************************
        #ifdef WATCHDOG
          if ((message_in[2] == 'W') && (message_in[3] == 'L') && ((longueur_m == 4) || (longueur_m == 5)))  // WL : lecture Watchdog  WLZ:et raz
          {
              uint8_t index=14;
              strcpy ( (char *)message+1, "Watch: sleep: ");
              for (a=0; a<NB_TACHES_WDT; a++)
              {
                  index = ajout_int16_message(watchdog_cpt_sleep_max[a], index, 0x82);
                  message[index++] = '-';
              }
              index--;
              strcpy ( (char *)message+index, "  activ: ");
              index += 9;
              for (a=0; a<NB_TACHES_WDT; a++)
              {
                  index = ajout_int16_message(watchdog_cpt_activ_max[a], index, 0x82);
                  message[index++] = ' ';
                  index = ajout_int16_message(watchdog_cpt_activ_tot[a], index, 0x82);  // 0(Node):  1:(Radio):5 taches par allumage
                  message[index++] = '-';
              }
              message[index-1] = car_fin_trame;
              envoi_message();

              if (message_in[4] == 'Z')   // efface les valeurs des max
              {
                  for (a=0; a<NB_TACHES_WDT; a++)
                  {
                      watchdog_cpt_sleep_max[a] = 0;
                      watchdog_cpt_activ_max[a] = 0;
                  }

              }
          }
        #endif

          // ******************************** XXXXXXXXXXXXXXXXXXXX  ********************

          if (message_in[2] == 'X')     //  ACTIVATION SORTIES
          {
              b = message_in[4] - '0';
              if ((message_in[3] == 'E') && (b < NB_SORTIES))
              {
                  if (longueur_m == 6)   // XExy : Activation sortie X pdt Y 0,1sec    XE01 (Led0 1s) 0:sans limite
                   {
                      ACTIV_SORTIE(b,1,(int unsigned)(message_in[5]-'0')*10);
                   }
                  if (longueur_m == 7) // XExyz : Activation sortie X mode Y pdt z 0,1sec    XE021 (Led0 mode2 1 sec
                    {                                               // XE421 (ignition modem 2s mode:1)
                      ACTIV_SORTIE (b, message_in[5] - '0',  (int unsigned) (message_in[6] - '0') * 10);// 5:5s  A:17s Z:42s  {:73s  +:4 min
                    }
              }

              if ((message_in[3] == 'L') && (b < NB_SORTIES)) // XLx : Lecture ETAT SORTIE  X      XL4 => consigne - duree
              {
                  message_lecture_etat_sortie (b);
              }


              if ((message_in[3] == 'E') && (message_in[4] == 'B')  && (longueur_m == 9))  // BUZZER
                 // Activation buzzer XEBcdpd  ex:XEB1233 Z94  duree*0,1s/periode/duty
                {  // 1346   =>  1, 0,5s ,1200Hz,vol6   2446:bip  2446
                  // periode : 0-900:2200Hz  4-1700:1200Hz  6-2100:950Hz
                  // consigne, duree, periode, duty
                  // duree : 1:0,2s 2:0,4s  3:1s  4:1,6s  5:3,2s   6:6s   7:12s   8:25s   9:50s   10:1,5min
                  ACTIV_PWM ( 0,message_in[5] - '0', (uint32_t)( 1 << (message_in[6] - '0')),
                             (uint32_t)(900 + 200 *(message_in[7] - '0')), (message_in[8] - '0')*4);
                }
              if ((message_in[3] == 'S') && (longueur_m == 7))  // XSabc  SPOT LED en PWN  a:mode  b:duree  c:duty
              {
                  // Activation Spot PWM : frequence:10kHz(200)  duty 20%
                  // mode  : 0:eteint  1:allume  3:lent  7:2s-0,5
                  // duree : 1:0,2s 2:0,4s  3:1s  4:1,6s  5:3,2s   6:6s   7:12s   8:25s   9:50s   10:5min
                  // duty : 0:0,5% 1:1%  2:2%  3:3%%  4:8% 5:12%  6:16%  7:25% 8:35%  9:50%
                  // Exemple XS144, XS74
                  uint8_t duty;
                  duty = message_in[6]-'0';
                  if (duty <4)  // 0,1,2,3
                      duty = (1<<duty);
                  else   // 4 a 9
                  {
                      if (duty==4) duty=20; // 20=8%
                      if (duty==5) duty=31; // 31=12%
                      if (duty==6) duty=41; // 41=16%
                      if (duty==7) duty=64; // 64=25%
                      if (duty==8) duty=128; // 128=50%
                      if (duty==9) duty=255; // 255=100%
                  }

                  ACTIV_PWM ( 1, message_in[4] - '0', (uint32_t)( 1 << ((message_in[5] - '0')*3)), 200, duty);
                  //  ACTIV_PWM ( 1,1, Spot_allumage_nuit*10, 200, 255);  // duree:30 secondes, frequence 10kHz, duty:100%
              }

              if ((message_in[3] == 'I') && (longueur_m == 8))  // XIcdpd  SPOT LED en PWM  c:consigne  d:duree p:periode d:duty
              {  // bXI1827
                  /* Allumage LED IR des cameras PIR :
                  periode : 2700=> 750hz   900=>2200Hz   500=>4000Hz   100->20kHz
                  periode : 1(50):40kHz 4:(200):10kHz
                  duty :  20=8%  31=12%  41=16%  64=25%  128=50%  255=100%
                  uint16_t periode=500;  */

                  // Exemple XS1442:normal  XS1443:fort
                  uint8_t duty;
                  duty = message_in[7]-'0';
                  if (duty==4) duty=20; // 20=8%
                  if (duty==5) duty=31; // 31=12%
                  if (duty==6) duty=41; // 41=16%
                  if (duty==7) duty=64; // 64=25%
                  if (duty==8) duty=128; // 128=50%
                  if (duty==9) duty=255; // 255=100%
                  ACTIV_PWM ( 1,message_in[4] - '0', (uint32_t)( 1 << (message_in[5] - '0')),
                             (uint32_t)( 50 *(message_in[6] - '0')), duty);
              }
            }

          // ----------------- WWWWWWWWWWW - Message longs --------------------------

          /*if ((message_in[2] == 'W') && (message_in[3] == 'E'))
          {
              if ((message_in[4] == 'T') && (longueur_m == 6)) //--------- reponse WETx : Transfert fichier 0:ok dispo, 1:ok attente, 2:nok 3:dispo
              {
                if (message_in[5] == '2')  // nok
                {
                  if (send_long_repeat < 3 )  // repete 3 fois
                  {
                    send_long_repeat++;  // TODO il faudrait garder le message, pour pouvoi le renvoyer
                    //Event_post(nodeEventHandle, NODE_MESS_LONG);
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

          // ******************************** ZZZZZZZZZZZZZZZZZZZZ ********************

          if ((message_in[2] == 'Z') && (longueur_m == 4)) //  Z  RAZ       Zx   :  ZR :RESET ou ZZ :RAZ CPU
          {
              if (message_in[3] == 'R')    // ZR : Reset
                {
                  //HWREGB(PRCM_BASE+ PRCM_O_WARMRESET) |= 4;   // Warmrest -> systesm reset
                  //HAL_SYSTEM_RESET();    *((uint32_t *)0x40082270) = 1;
                  // HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD)  RBL COMMAND_RESET
                    #ifdef CC13xx
                      HWREG (AON_SYSCTL_BASE + 4) |= (uint32_t) (1<<31);
                    #endif
                    #ifdef ESP32
                      esp_restart();
                    #endif
                }

              if (message_in[3] == 'Z')   // ZZ : Raz erreur
                {
                  raz_erreur();
                  //nb_reset=0;
                  //noyau_raz ();

                  //Appli_init (0);
                }
          }

          traitement_message_appli(longueur_m);

          if (mess_ok)
          {
              message[0] = message_in[1] & 0x7F; // emetteur devient le destinataire du futur message
              if (message_in[1] & 0x80)  // envoie à l'emetteur qui l'enverra ensuite à l'uart attaché
                  strcpy((char *)message+1, "-OK");
              else
                  strcpy((char *)message+1, "OK");
              envoi_message ();
          }

        }
    }
}



void Maj_clock_periodique(void)
{
#ifdef CC13xx
    if (period_activ)
    {
        CLOCK_STOP(PeriodiqueClockHandle);
        Clock_setPeriod(PeriodiqueClockHandle, (period_periode *1000 * (1000 / Clock_tickPeriod)));
        Clock_start(PeriodiqueClockHandle);
   }
   else {
        CLOCK_STOP(PeriodiqueClockHandle);
    }
#endif
}


uint32_t code_tournant_suivant(uint32_t code_old)
{
    bool var;
    // bits : 26, 19, 15, 10, 2
    var = ((code_old >> 2) & 1) ^ ((code_old >> 10) & 1);
    var = var ^ ((code_old >> 15) & 1);
    var = var ^ ((code_old >> 19) & 1);
    var = var ^ ((code_old >> 26) & 1);
    return  (code_old >> 1) | (var<<31);
}


void dump_xdata_1_ligne (uint32_t * pregistre_int_xdata)
{
  int unsigned val, i2;

  envoie_addr_16 ( (uint32_t *) pregistre_int_xdata);  // 8 car
  val = *pregistre_int_xdata;

  for (i2 = 0; i2 < Dump_ram_max; i2++)
    {
      ajout_message_32b ( val, i2*10+8);
      //message[(i2 * 3) + 8] = deci (val >> 4);
      //message[(i2 * 3) + 9] = deci (val & 15);
      pregistre_int_xdata = pregistre_int_xdata + 1;
      val = *pregistre_int_xdata;
      message[(i2 * 10) + 7] = '-';
    }
  message[(i2 * 10) + 7] = car_fin_trame;
  envoi_message ();
}


uint8_t hexa (uint8_t val) // transforme 1 char ASCII (45->68) en decimal (0 � 15) = hexa (0->F)
{
  uint8_t resul;
  if ((val < 48) || (val > 70))
    resul = 0;
  else if (val < 58)
    resul = val - 48;   // chiffres
  else if (val > 64)
    resul = val - 55;   // lettres
  else
    resul = 0;
  return resul;
}

// affiche en hexa, un uint_8  ex:D3
void message_1hexa (uint8_t valeur, uint8_t position)
{
  message[position++] = deci (valeur >> 4);
  message[position] = deci (valeur & 0x0F);
}

// affiche en hexa, un uint_16  ex:45D3
void message_1int (uint16_t valeur_int, uint8_t position)
{
  message[position++] = deci (valeur_int >> 12);
  message[position++] = deci ((valeur_int & 0x0F00) >> 8);
  message[position++] = deci ((valeur_int & 0x00F0) >> 4);
  message[position] = deci (valeur_int & 0x0F);
}

uint8_t hexaT(uint8_t val)      // transforme 1 char ASCII (45->68) en hexa (0->F) puis continue jusqu'à Z ('G'->0x11, 'H'->0x12,..)
{
uint8_t resul;
    if ( (val<48) || (val>90) )
      resul = 0;
    else
      if (val < 58 ) resul = val - 48;
      else
         if (val > 64 ) resul = val - 55 ;
         else resul = 0;
return resul;
}

uint8_t Ascii2_Hexa( uint8_t *c)      // transforme 2 char ASCII (0x41 + 0x42) en sa valeur hex (0xAB)
{
    return hexaT(*c)*16 + hexaT(*(c+1));
}

uint32_t Ascii_Hexa( uint8_t *c, uint8_t nb)      // transforme nb char ASCII (0x41 + 0x42) en sa valeur hex (0xAB)
{
    uint8_t i;
    uint32_t res=0;
    for (i=0; i<nb; i++)
    {
        res += (hexaT(*(c+i))<<(i*4));
    }
    return res;
}

uint8_t deci (uint8_t val) //transforme un char hexa en son charactere ASCII   0->48 9->57 A->65 F->70 G->71
{
  uint8_t resul;
  if (val > 36)
    resul = 91;  // caractere apres Z pour indiquer une erreur
  else if (val < 10)
    resul = val + 48;
  else
    resul = val + 55;
  return resul;
}

// affiche en hexa, un uint_8  ex:D3  0xD3 -> 0x44+0x33
void ajout_message_8b (uint8_t valeur, uint8_t num)
{
  message[num++] = deci (valeur >> 4);
  message[num] = deci (valeur & 0x0F);
}

// affiche en hexa, un uint_16  ex:45D3
void ajout_message_16b (uint16_t valeur_int, uint8_t num)
{
  message[num++] = deci (valeur_int >> 12);
  message[num++] = deci ((valeur_int & 0x0F00) >> 8);
  message[num++] = deci ((valeur_int & 0x00F0) >> 4);
  message[num] = deci (valeur_int & 0x0F);
}

// affiche en hexa, un uint_32  ex:45D3 3F6A
void ajout_message_32b(uint32_t valeur, uint8_t num)
{
    ajout_message_16b (valeur>>16, num);
    message[num+4] = ' ';
    ajout_message_16b (valeur, num+5);
}


// affiche en hexa, un uint_16  ex:45D3   du pointeur_adresse
void envoie_addr_16 (uint32_t * pointeur_adresse)
{
  message[1] = 'V';  //message de la valeur lue
  message[2] = ' ';
  //ajout_message_16b((uint16_t)pointeur_adresse, 3);
  message[3] = deci (HIGH_BYTE(pointeur_adresse) >> 4);
  message[4] = deci (HIGH_BYTE(pointeur_adresse) & 0x0F);
  message[5] = deci ( LOW_BYTE(pointeur_adresse) >> 4);
  message[6] = deci (LOW_BYTE(pointeur_adresse) & 0x0F);
  message[7] = ':';
}

void envoie_1_word (uint32_t * adresse_int)
{
  envoie_addr_16 ( (uint32_t *) adresse_int);
  ajout_message_32b(*adresse_int, 8);
  message[17] = car_fin_trame;
  envoi_message ();
}

void ajout_d_word_message(uint16_t valeur, uint8_t num)
{
    message[num] = deci (HIGH_BYTE(valeur) >> 4);
    message[num+1] = deci (HIGH_BYTE(valeur) & 0x0F);
    message[num+2] = deci ( LOW_BYTE(valeur) >> 4);
    message[num+3] = deci (LOW_BYTE(valeur) & 0x0F);
}

void ajout_word_message(int unsigned valeur, uint8_t num)
{
    message[num]   = deci (HIGH1_BYTE(valeur) >> 4);
    message[num+1] = deci (HIGH1_BYTE(valeur) & 0x0F);
    message[num+2] = deci (HIGH2_BYTE(valeur) >> 4);
    message[num+3] = deci (HIGH2_BYTE(valeur) & 0x0F);
    message[num+4] = ' ';
    message[num+5] = deci (HIGH_BYTE(valeur) >> 4);
    message[num+6] = deci (HIGH_BYTE(valeur) & 0x0F);
    message[num+7] = deci ( LOW_BYTE(valeur) >> 4);
    message[num+8] = deci (LOW_BYTE(valeur) & 0x0F);
}

void envoie_16octets ( uint32_t * pointeur_envoi)
{
  uint8_t i2;
  int unsigned val;
  // uint8_t ** point_point;

  //point_point = &pregistre_xdata;
  envoie_addr_16 (pointeur_envoi);
  for (i2 = 0; i2 < 4; i2++)
    {
      val = *pointeur_envoi;
      ajout_message_32b(val, (i2*10)+8);
      pointeur_envoi += 1;
      message[(i2 * 10) + 17] = '-';
    }
  message[(i2 * 10) + 7] = car_fin_trame;
  envoi_message ();
}

// renseigne "message" par la valeur en unite horaire, a la position indiquee
// input  : valeur en secondes
// output : 43 h, 300s, 15 m
void conversion_deci_text_message(uint16_t valeur, uint8_t position)
{
    uint8_t j, i;

    j = valeur / 3600;
    if (j)    // periode en heures
    {
        message[position+3] = 'h';
        envoi_char_decimal (j, position);
    }
    else
    {
        // recupere les minutes
        j = valeur / 60;
        if (j)    // periode en minutes
        {
            message[position+3] = 'm';
            i = (j) % 60;
            envoi_char_decimal (i, position);
        }
        else
        {   // periode en seconde
            message[position+3] = 's';
            i = valeur % 60;
            envoi_char_decimal (i, position);
        }
    }
}

uint32_t period_conversion (uint8_t period)
{
    uint8_t exp;
    uint32_t resultat=1;
    exp = period & 3;
    if (!exp) resultat = (period>>2)*10;     // * 10 milliseconde
    if (exp==1) resultat = (period>>2)*1000;  // secondes
    if (exp==2) resultat = (period>>2)*1000*60;  // minutes
    if (exp==3) resultat = (period>>2)*1000*3600;  // heures
    return MS_to_TICK(resultat);
}


/*
ASCII       -> Hexa4         : char hexa(uint8_t val)       0x41 -> 0xA
ASCII       -> Hexa4         : char hexaT(uint8_t val)      0x41 -> 0xA  continue apres F
ASCII+ASCII -> Hexa8         : uint8_t Ascii2_Hexa( uint8_t *c)
ASCII+ASCII -> HexaX         : uint8_t Ascii_Hexa( uint8_t *c, uint8_t nb)

code_dec_hex -> tab          = idem mais flexible  Ascii-> Hexa
code_hex_dec -> tab          : idem mais flexible  Hexa->Ascii

Hexa4        -> ASCII        : uint8_t deci (uint8_t val)  0->0x30 9->57 A->0x41 F->70 G->71

Hexa8        -> message      : ajout_message_8b    0xAB   -> 0x41+0x42
Hexa16       -> message      : ajout_message_16b   0xABCD -> 0x41+0x42+0x43+0x44
Hexa32       -> message      : ajout_message_32b   0x4FE2 4D21

Hexa16      -> message format� : envoie_addr_16

Hexa32      -> envoie          : envoie_1_word    "V + adresse16 : contenu32"
Hexa 16octet-> envoie 16 octe  : envoie_16octets ( uint32_t * pointeur_envoi)

Decimal :
hexa16 ->  envoi_int_decimal  0x23 -> 35
hexa16 ->  ajout_int16_message (avec parm fixe)

Test de communication
Du node, en mode 1, sans pile flash :
Z1HLH ou ZzHLH : -> heure
11HLH ou 1zHLH : -> heure
01HLH ou 0zHLH : -> heure
zxxxx : retour identique
112HLH: 0xB1 0x31 0x02 0x48 0x4C 0x48
Z12HLH: 0xDA 0x31 0x02 0x48 0x4C 0x48
L1HLH ne fonctionne pas bien
L12HLH : 0xCC 0x31 0x02 0x48 0x4C 0x48
idem avec pile_flash
QzHLH : heure conc
Qz2HLH : 0xD1 0x7A 0x02 0x48 0x4C 0x48 : heure conc
ZxTBA4zzz10 ZxTBA4LLL10  ZxTBA4Qzz10   QZz  ZxTBA4QZL10   1, 4, et 8 (x message succ vers z, L, Q-z, Q-Z-L)
idem 00(Ack), 20(RX apres), 30(RX sans Ack), 02 (et deconnect conc)
08(differe) : ZZTBA1LLL08 ensuite Z->L Z->L Q->L
06(renvoie) : ZZTBA4LLL06

Du concentrator :
QLHLH ou 0LHLH : -> heure
QL2HLH: 0xD1 0x4C 0x02 0x48 0x4C 0x48 -> heure
ZLHLH : -> heure node
ZL2HLH: 0xDA 0x4C 0x02 0x48 0x4C 0x48
ZL1HLH (vers port serie connecte au node)
ZL31HLH: 0xDA 0x4C 0x03 0x31 0x48 0x4C 0x48
zxxxx -> id vers node/serie
zL1HH : 0xFA 0x4C 0x01 0x48 0x48 : HH sur node/serie
QxTBA4LLL10 QxTBA4zzz10 Zzz QxTBA4ZLL10   ZQL  QxTBA4ZQz10   1, 4, et 8 (x message succ vers z, L, Q-z, Q-Z-L)
08(differe) : QLTBA1zzz08 ensuite Q->z

En mode 6 :
du Node : QzHLH LzGG ZZTBA2QLL10 et 20
du Conc : ZLHLH
  bloqu� sur Conc zLGG->N:ZzTBA1LLL20  Qzz20  ZLHLH->ZzTBA1LLL20     ZLHLH+ZLHLD-> ZzTBA3LLL20

En mode 5 :
1 seul node : ZLHLH ZLHLD
2 nodes (Y:6 et Z:5)
*/


