/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 */

#ifndef TASKS_NODETASK_H_
#define TASKS_NODETASK_H_

/* Initializes the Node Task and creates all TI-RTOS objects */
void NodeTask_init(void);
void appui_boutton_0(void);
void appui_boutton_1(void);
void appui_2bouttons(void);
void appui_boutton_2(void);
void SCSTask_init(void);
void Task_message_recu(void);
void Task_envoie_message(void);
void Activ_clock_100ms(void);
void Desactiv_clock_100ms(void);
uint8_t retourne_last_rssi(void);
char sensor_init();
void Envoi_regulier_data(void);
uint8_t lecture_temphdc1000(void);
uint16_t resteclockESP();
void writeToAddress (uintptr_t *addr, int val);

void event_action_periodique(uint8_t type_action_periodique);

void hors_portee_augmente(uint8_t num_adres);
void hors_portee_desactive(uint8_t num_adres);
void increment_mode_affich_enreg(void);
void increment_mode_envoi_mpu(void);

void envoi_enreg_resultat (uint8_t code, uint8_t * chaine);
void Affiche_texte_resultat(uint8_t * chaine);
//void  timeout_mess_long(void);
uint16_t transf_duree_ev(uint8_t duree);
void activ_evanne(uint8_t num, uint16_t duree);
void envoie_mesure_sonde_HumSol(uint8_t type);  // 0:hexa  1:texte
void chgt_mode_arret( uint8_t nouv);
void demar_prog_21h(time_t secondes_mes);


void SensorTask_init(void);

struct Capteurs_R {
    uint8_t capteur;    // 0:temp, 1:lumi, 2:hygro, 3:accelero
    uint8_t activ;      // bit0:actif/non actif, bit1:BP0, bit2:periodique bit3:mvt
    uint8_t period;    // bit01:0:0,1s  1:1s  2:1min  3:1h
    uint8_t type;       // type de mesure sur ce capteur
    uint8_t type_seuil1;      // 0:jamais, 1:tous, 2:min, 3:max
    uint16_t val_seuil1;  //
    uint8_t action_seuil1;  // 0:enreg  1:transmission immediat 1 fois  2:1 min entre chaque  3:max_nb
    uint8_t timer_seuil1;   // timer entre 2 mesures
    uint8_t type_seuil2;      // 0:jamais, 1:tous, 2:min, 3:max
    uint16_t val_seuil2;  //
    uint8_t action_seuil2;  // 0:enreg  1:transmission immediat
    uint8_t transmis;   // 0:non 1:à chaque valeur 2:max_duree 3:max_nb  4:max_duree&nb
    uint16_t max_duree;  // en secondes
    uint8_t max_nb;     //
    uint8_t codage_trans;     // 0:pas de date   Nb de données(0à7) + codage(capteur) + Nb d'octet par donnée(0:1 1:2)
    uint8_t message[8];
};

extern float FloatValue, Float2Value;
extern uint8_t arret_radio;
extern uint8_t Config_boutons;


#define Niv_batt_tres_bas 8  // pas de radio ni led en dessous de ce niveau


#endif /* TASKS_NODETASK_H_ */
