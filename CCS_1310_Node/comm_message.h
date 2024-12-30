/*
 * communication_message.h
 *
 *  Created on: 22 juil. 2017
 *      Author: Tocqueville
 */

#ifndef COM_MESSAGE_H_
#define COM_MESSAGE_H_

#include <stdint.h>
//#include "stdint.h"

//  ---------------   CONFIGURATION ------------------------


#ifdef CONCENTRATOR
      #define My_Address 'Q'  // Passerelle
      #define Uart2   0 // Activation de la liaison série
#else
      #define My_Address NODE  // end device
#endif

#ifdef DEBUG_M
#ifndef dest_erreur_reset
      #define dest_erreur_reset '1'
#endif
#endif

#define car_fin_trame  13

#define MESSAGE_SIZE 140  // taille max d'un message recu ou envoye  (utilise dans 3 buffer)
#define RX_BUFFER_SIZE 140  // buffer reception pour la file de reception des messages > MESSAGE_SIZE
#define TX_BUFFER_SIZE 140  // buffer transmission pour la file d'envoi des messages  > MESSAGE_SIZE
#define TAILLE_FICHIER_TRAME 100  // longueur d'un message long
#define M_LONG_PERIODE  8000 // periode de repeat de message long en ms
//Taille max des messages : 123 car (en comptant le car de fin, mais hors expediteur)

#define HIGH1_BYTE(x)            ((uint8_t) (((uint32_t)(x)) >> 24))
#define HIGH2_BYTE(x)            ((uint8_t) (((uint32_t)(x)) >> 16) & 0xFF)
#define HIGH_BYTE(x)            ((uint8_t) (((uint32_t)(x)) >> 8) & 0xFF)
#define LOW_BYTE(x)             ((uint8_t) (((uint32_t)(x)) & 0xFF))

#define PAR_VAR_MAX 3  // nombre de parametres

#ifdef CC13xx
    #define SEMAPHORE_TAKE(x,y) Semaphore_pend(x, y*100) // x:handle y:en ms
    #define SEMAPHORE_GIVE(x) Semaphore_post(x)
#else
    #define SEMAPHORE_TAKE(x,y) xSemaphoreTake( x, y/10) // x:handle y:en ms
    #define SEMAPHORE_GIVE(x)  xSemaphoreGive( x)
#endif

// ------------------------------------Extern Variables--------------------------------------------------

extern  uint8_t message_in[];
extern  uint8_t longueur_message_in;
extern  uint8_t message[];
extern  uint8_t flag_comm_transfert;
extern  uint8_t dump_ram_fin, dump_ram_timer;
extern  uint8_t par8_var[PAR_VAR_MAX];
extern  uint16_t par16_var[PAR_VAR_MAX];
extern  uint8_t Uart_actif;
extern  uint8_t compteur_PHLS;
#ifdef CC13xx
  extern  Clock_Handle MajHeureClockHandle;
#endif
extern  uint16_t compteur_maj_heure; // interrogation periodique : Nb d'heures avant interrogation

extern uint8_t send_long_actif, send_long_repeat, send_long_buff_suiv, send_long_buffer_tx, send_long_port_emett;
extern uint8_t send_long_attente, send_long_emett, send_long_port_dest, send_long_emett, send_long_buffer_suiv, send_long_pile;
extern uint16_t send_long_num_trame, send_long_num_trame_tx, send_long_autre;
extern uint8_t StatutOps;  // statut operationnel par bit
extern char unsigned message_test[10];

// SORTIES
typedef struct
{
  uint8_t          num_Uart;      // 
  #ifdef CC13xx
    Clock_Handle      h_timeout_RX;     // handle clock
    Clock_Handle      h_timeout_TX;     // handle clock
  #endif
  #ifdef ESP32
    TimerHandle_t     h_timeout_RX;
    TimerHandle_t     h_timeout_TX;
  #endif
} UartStruct;

// ----------------  structures ram tampon-------------------

#define GROUPES_CAR_MAX  80     // en nb de car par message envoye

#ifdef DEBUG_M
    #define GROUPES_NB_MAX  2       // 1  nb de message envoye
#else
    #define GROUPES_NB_MAX  6       // 6  nb de message envoye
#endif

#define GROUPES_TEMPS_MAX 600   // 600 (10h) en minutes (max 715)


// -------------------------------- Fonctions externes  ---------------------------------------------------

void tache_comm_init(void);
void Periph_init(void);
void Com_stack (void);

void Uart2_Open(void);
void Uart2_close(void);

void reception_message_Uart2(void);
void transmission_message_Uart2(void);
void traitement_message_appli (char unsigned longueur_m);

uint8_t envoi_message(void);
uint8_t envoi_message_def(void);
uint8_t transf_buff (char unsigned longueur_mess);   // var:  PRZf  long=3      def: PR0Z  long=3
void traitement_rx (char unsigned);

void test_err(uint8_t code);  // TODO


void conversion_deci_text_message(uint16_t valeur, uint8_t position);
uint32_t code_tournant_suivant(uint32_t code_old);
void Maj_clock_periodique(void);

uint8_t hexa(char unsigned);                    // transforme un ASCII en hexa
uint8_t hexaT(char unsigned val);               // transforme un ASCII en hexa et continue apres F
uint8_t Ascii2_Hexa( uint8_t *c);               // transforme 2 char ASCII (0x41+0x43) en sa valeur hex (0xAC)
uint32_t Ascii_Hexa( uint8_t *c, uint8_t nb);   // transforme Nb char ASCII (0x41+0x42+0x44) en sa valeur hex (0xABD)

void message_1hexa(char unsigned valeur, char unsigned position);
void message_1int(uint16_t valeur_int, char unsigned position);

char unsigned deci(char unsigned);   //transforme un char hexa en son charact�re ASCII   0->48 9->57 A->65 F->70 G->71
void ajout_message_8b (uint8_t  valeur, uint8_t num);
void ajout_message_16b(uint16_t valeur, uint8_t num);
void ajout_message_32b(uint32_t valeur, uint8_t num);

void envoie_addr(uint32_t);
void envoie_addr_16 (uint32_t *);
void envoie_16octets(uint32_t *);
void envoie_1_word (int unsigned *);
void ajout_d_word_message(uint16_t valeur, char unsigned num);
void ajout_word_message(int unsigned valeur, char unsigned num);
void sauve_eeprom_index_flash_glob(void);
uint16_t calcul_free_space (uint8_t port);
uint8_t recep_long (uint8_t * messa);

uint32_t period_conversion (uint8_t period);
void demarr_lect_flash();
char unsigned longueur_messa (char unsigned * message_envoye);
uint8_t passerelle_routage(uint8_t dest);

void sauve_eeprom_index_flash(void);
uint8_t enreg_message_flash(uint8_t *buf, uint8_t section);
uint8_t lecture_message_flash(uint8_t *buf, uint8_t section, uint8_t mode, uint8_t Lmax);
bool supp_message_flash(uint8_t longu, uint8_t section);
void enreg_index_flash(void);
void Activ_Uart2(void);


// 256 pages de 0xFFF octets
struct Pile_Fl_S {
    uint8_t debut;  // numero de premiere pages
    uint8_t taille;  // nombre de pages (min 1, max 254 - EEPROM ensuite)
    uint32_t head;
    uint32_t tail;
};

struct RemoteNode {
    uint8_t adresse;    // 0:pas d'adresse definie
    uint8_t type_node;  // 1:noeud normal
    uint8_t mode_rx;    // 0:non defini  1:RX continu  4:RX period  5:RX period  6:pas de RX
    uint8_t puissance;  // 0 a 10
    int8_t latestRssi;
    uint8_t latestLength;
    uint16_t nb_reception;
    uint16_t nb_transmission;
    uint8_t  nb_trans_ratees;    // deuxieme envoi perdu
    uint8_t  nb_trans_renvoye;   // premier envoi perdu
    uint8_t  nb_messages_effaces;
    uint8_t  nb_rx_err;
    uint8_t   hors_portee_etat;    // 0:ok, 1:10s, 2:30s, etc..*3  11:7 jours   50:d�sactiv�
    uint8_t forcage;
    uint8_t head;
    uint8_t tail;
    uint8_t synchro;
    uint8_t period_activ_rx;      // si synchro : periode entre 2 fenetres
    uint8_t duree_fenetre_rx;     // si synchro : duree fenetre d'ecoute
    uint8_t resynchro;            // resynchro
    uint32_t code_tournant;
    uint8_t buffer_tx[TAILLE_PILE_ENVOI_RADIO];
};

// -------------------------------- Fonctions internes  ---------------------------------------------------


#endif /* COMMUNICATION_MESSAGE_H_ */


