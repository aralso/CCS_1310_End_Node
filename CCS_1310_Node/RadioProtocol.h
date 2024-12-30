/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 */

#ifndef RADIOPROTOCOL_H_
#define RADIOPROTOCOL_H_

#include "stdint.h"


#define RADIO_MY_ADDRESS            My_Address //0x00
#define RADIO_CONCENTRATOR_ADDRESS  'Q'
#define PASSERELLE_ADDRESS          'Q'   // utilisé quand le node doit se connecter à un repeteur
#define PC_ADDRESS                  'L'
#define PC_NODE_SERIE_ADDRESS       'R'

// Adresse reseau
#ifdef Reseau_test
    #define AddReseau1  0x27 // LGM-Test:0x27
#else
    #define AddReseau1  0x28 // Nominal:0x28
#endif

#define AddReseau2  0x45
#define AddReseau3  0x11

// Selection du protocole radio
#define RADIO_PROTOCOLE     1       //  100:625bps    10:5000bps(long range)    1:50kbps
// Les protocoles doivent être définis dans les symboles du projet

#ifdef NODE  // S à Z
        #define RF_address_debut             'Q'
        #define RF_address_fin               'Q'

        // timeout pour phase RX : attention 150car=>27ms(si radio_protocole=1)  donc 15=> max 80 car reçus
        #define DUREE_MAX_ECOUTE_RX     15*RADIO_PROTOCOLE   // mode 4,5,6 : durée en milliseconde  100ms (LR) 15ms/20s(5kbps) => 10uA

        //#define PERIODE_BALISE_SYNCHRO    2               // Période d'envoi balise synchro, en minutes
        // Conso balise synchro : ecoute : F/P*5u F=4ms P=10s => 2uA   balise:2 minutes
        #define DUREE_FENETRE_RX          4*RADIO_PROTOCOLE     // Duree fenetre RX (mode 4,5) en millisecondes

        #define DUREE_ACTIV_apres_RX      12*RADIO_PROTOCOLE  // en milli-secondes

        #define TAILLE_PILE_ENVOI_RADIO  TX_BUFFER_SIZE    //    120 à 200  // Taille de la pile RF d'envoi pour chaque destinataires
        #define TAILLE_MAX_MESS_RF              127   // Max 127 (payload)

        // 1ère pile : messages pour le concentrateur
        #define NB_PILES_F  1   // définition des piles Flash : table_pile_f (dans comm_message.c)
#endif



#if RADIO_PROTOCOLE == 100
    #define RADIO_EASYLINK_MODULATION     EasyLink_Phy_625bpsLrm
#elif   RADIO_PROTOCOLE == 10
    #define RADIO_EASYLINK_MODULATION     EasyLink_Phy_5kbpsSlLr
   // mess=VLHLH(L=6car) => nb car=12+(5+5+L+9)*2=52car => 83ms
#elif  RADIO_PROTOCOLE == 2
    #define RADIO_EASYLINK_MODULATION     EasyLink_Phy_Custom
#elif  RADIO_PROTOCOLE == 1
    #define RADIO_EASYLINK_MODULATION     EasyLink_Phy_50kbps2gfsk
   // mess=VLHLH(L=6car) => nb car=4+4+(5+5+L+2)=21car => 3,3ms
#endif

#define RADIO_PACKET_TYPE_ACK_PACKET             1
#define RADIO_PACKET_TYPE_SIMPLE_MESSAGE_PACKET  2
#define RADIO_PACKET_TYPE_DEMANDE_MESS_PACKET    3
#define RADIO_PACKET_TYPE_KEEPALIVE_PACKET       4
#define RADIO_PACKET_TYPE_SYNCHRO                5

#define HEADER_LENGTH  5

struct PacketHeader {
    uint8_t sourceAddress;
    uint8_t packetType;
    uint8_t mode_rx;  // bit 0à3:mode RX
    uint8_t statut;   // bit0:dernier  bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)   bit3:forcage   bit4:pas d'ack  bit5:RX apres
    uint8_t compteur; // compteur pour l'ack
};


struct RF_Mess_Payload {
    struct PacketHeader header;
    uint8_t Message[TAILLE_MAX_MESS_RF];
};

struct RF_synchro_Payload {
    uint8_t sourceAddress;
    uint8_t packetType;
    uint8_t type;  // 0: balise synchro (conc)  1:balise resynchro (conc)  2:demande resynchro (node)
    uint8_t period_balise;
    uint8_t period_RX;
    uint32_t time;  // en secondes
    uint32_t prochaine_balise;  // en 100 us
};

struct Message_RF_Packet {
    struct RF_Mess_Payload RF_payload;
    uint8_t dest;
    uint8_t length;
    int8_t rssi;
    uint32_t absTime;
    uint32_t rx_TimeOut;
    uint32_t tick_rx;
};


struct AckPacket {
    struct PacketHeader header;
};


#endif /* RADIOPROTOCOL_H_ */
