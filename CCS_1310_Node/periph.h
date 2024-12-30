/*
 * communication_message.h
 *
 *  Created on: 22 juil. 2017
 *      Author: Tocqueville
 */

#ifndef PERIPH_H_
#define PERIPH_H_

#include <stdint.h>
//#include "stdint.h"

//  ---------------   CONFIGURATION ------------------------


// -------------------------------- Fonctions externes  ---------------------------------------------------

void Periph_init(void);



// -------------------------------- Fonctions internes  ---------------------------------------------------

// --------------------- EXTERN -----------------

#ifdef WATCHDOG
    extern     Watchdog_Handle watchdogHandle;
    extern uint8_t watchdog_tache[NB_TACHES_WDT];  // etat tache : 0:asleep 1:unknown_asleep 2:unknown_active  3:active
    extern uint32_t watchdog_cpt_activ[NB_TACHES_WDT];   // compteur d'activation de la tache
    extern uint32_t watchdog_cpt_sleep[NB_TACHES_WDT];  // compteur d'inactivation de la tache
    extern uint32_t watchdog_cpt_activ_max[NB_TACHES_WDT];   // compteur max d'activation de la tache
    extern uint32_t watchdog_cpt_activ_tot[NB_TACHES_WDT];   // compteur max d'activation de la tache
    extern uint32_t watchdog_cpt_sleep_max[NB_TACHES_WDT];  // compteur max d'inactivation de la tache
#endif


#endif /* COMMUNICATION_MESSAGE_H_ */


