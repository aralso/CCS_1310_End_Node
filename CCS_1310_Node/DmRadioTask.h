/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 */

#ifndef TASKS_DMRADIOTASKTASK_H_
#define TASKS_DMRADIOTASKTASK_H_

#include "stdint.h"
#include "Board.h"
#include "comm_message.h"
#include "RadioProtocol.h"

#include "easylink/EasyLink.h"
#include "easylink/easylink_config.h"


enum RFRadioOperationStatus {
    RFRadioStatus_Success,              // TX et Ack ok
    RFRadioStatus_Failed,               // erreur de transmission
    RFRadioStatus_FailedNotConnected,   // pas d'envoi : pb de format de message
    RFRadioStatus_TXok_RXFailed,        // envoi ok, mais le RX ne s'active pas
    RFRadioStatus_Timeout,              // envoi ok, pas d'ack, malgre resend
};



struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum RFRadioOperationStatus result;
    uint8_t statut;
    //uint8_t dernier;  // 0:pas dernier  1:dernier
};


struct Synchro_S {
    uint8_t statut;
    uint8_t period_bal;
    uint8_t period_RX;  // 0:bal a chaque fois, 1:1 fois sur 2,
    uint8_t cpt_period_RX;
    int8_t decalage;
    int16_t ecart;
    uint16_t volatilite;  // de l'ecart
    uint32_t derniere_balise;
    uint8_t nb_balises_ratees;
    uint32_t ticks_prec;
    uint32_t ticks_nouv;
    uint16_t duree_fenetre;   // duree de la fenetre d'ecoute
    uint32_t periode_fenetre;
};

typedef void (*Radio_PacketReceivedCallback)(struct Message_RF_Packet* packet, int8_t rssi);


// *******  EXTERN ***********

extern uint8_t rxrf_emetteur, rxrf_mode, rxrf_numero_device;
extern uint8_t lastAddedSensorNode;
extern uint8_t Mess_statut, mode_rx;
extern     int16_t decal_conc;
extern  int8_t rssi_bruit;


// ********* FONCTIONS **************

/* Create the RadioTask and creates all TI-RTOS objects */
void RadioTask_init(void);
void Activ_radio_rx(uint8_t premier_tx, uint16_t duree_acti, uint32_t duree_inacti, uint8_t nb_repet, uint16_t decalage);  // lancement séquence allumage

uint8_t RadioTask_getMyAddr();

/* Register the packet received callback */
void RadioTask_registerPacketReceivedCallback(Radio_PacketReceivedCallback callback);

enum RFRadioOperationStatus envoi_mess_RF(char unsigned * messageRF, unsigned char longueur_mess, uint8_t destinataire_RF, bool dernier_rx);
uint8_t envoi_buffer_RF(char unsigned * messageRF, unsigned char longueur_mess, uint8_t destinataire_RF);
enum RFRadioOperationStatus transmission_RF( char unsigned i);  // i:desttinataire

void clear_Watchdog(void);
void chgt_mode_radio(char unsigned mode, char unsigned type, uint8_t valeur);
void traitement_message_radio (char unsigned longueur_m);

uint8_t numero_device_rf(uint8_t adresse);
void envoi_pile_rf (uint8_t i, uint8_t mode);  // paramètre : numero_device a qui envoyer les messages, mode:0:pas bloqué,1:bloqué-timing, 2:tous
void envoi_keepalive(void);

void update_node(uint8_t aug_nb_recep);
bool addNewNode(uint8_t new_adresse);
static void updateNode(uint8_t active_node, uint8_t aug_nb_recep);
void change_power_radio (int8_t as);
void hors_portee_augmente(uint8_t num_adres);
void hors_portee_desactive(uint8_t num_adres);
void fin_envoi_rf(void);
uint8_t trouve_mess_RF_envoyer(uint8_t dest, uint8_t forcage);
//void fin_envoi_mess_RF(void);
void allumage_apres_envoi(void);
void extinction_radio(void);
uint8_t Phase_radio(void);
uint8_t Radio_suiv(void);
void Envoie_RF(uint8_t numero_dest, char unsigned * messageRF);
void envoi_cpt_radio_CB(void);
void envoi_cpt_index_test(void);
void Demande_balise_synchro(void);
void modifPerioFeneBal( uint8_t perio, uint8_t mode);
void Nouv_Deman_balise_synchro(void);
void arret_ecoute_radio(void);
void reprise_ecoute_radio(void);

void Radio_debut_tx (void);    // Debut phase TX
void RadioDebutKeepAlive (void);    // demande d'envoi keep alive
void RadioDebutBaliseTXdemande (void);    // demande balise TX de demande messages en attente
void RadioDebutEcouteRX (void);    // demande d'écoute RX
void RadioDebutTXBaliseSynchro(void); // envoi balise synhcro
void recept_balise_resynchro (uint32_t temp_restant);
void verif_radio_rx(void);
uint16_t calcul_espace_dispo_node(uint8_t numero_dest);
uint16_t timeout_prochaine_fenetre_node(void);
void envoi_info_balise();
uint16_t timeout_prochaine_resynchro(void);


/* set BLE advertiser settings */
//void ConcentratorRadioTask_setAdvertiser(ConcentratorAdvertiser advertiser);

#endif /* TASKS_DMRADIOTASKTASK_H_ */
