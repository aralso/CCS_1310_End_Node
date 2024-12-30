/* Peripherique  NODE
 *
 *  Created on: 14 avril 2017
 *      Author:
 */


#include <DmRadioTask.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>


// BIOS Header files
#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/Power.h>

#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>
#include <ti/sysbios/hal/Seconds.h>


#include <string.h>

/* TI-RTOS Header files */
#include <ti/display/Display.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/SPI.h>
#include "extflash/ExtFlash.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/prcm.h)

#include <ti/display/DisplayUart.h>

/* Board Header files */
#include "Board.h"

#ifdef CONCENTRATOR
   #include "DmConcentratorTask.h"
#endif

#ifdef NODE
   #include "DmNodeTask.h"
#endif

#include "comm_message.h"
#include "utilitaires.h"
#include <ti/devices/CC13x0/driverlib/AON_BATMON.h>

// -------------------  CONFIG  -------------------------



// ------------------------------------ Fonctions externes ---------------------------------------------------



//  ---------------   CONFIGURATION UTILISATEUR ------------------------*/



     // -----------------  CONFIGURATION DIVERSES -----------------------------


//  -------------------------------------   Codes erreur  ----------------------------------------code_erreurs

     #define Error_initializing_board            0x30



//------------------------------------Define------------------------------------------------------


// -----------------------------------Global Constants--------------------------------------------



// ------------------------------------Global Variables--------------------------------------------------


// variables appui boutons
bool   b0prec,  b1prec, double_appui;
uint8_t phase_appui_b0, phase_appui_b1, b0_appui, b1_appui;
Uint32 b0_debut, b1_debut, b0_suite, b1_suite;
#if BOUTON_CC == 3  // 3e bouton
    uint32_t b2_debut, b2_suite, b2_appui;
    uint8_t phase_appui_b2;
    bool   b2prec;
#endif


// Timer pour appui 2 fois sur boutons
    Clock_Struct TimerButtonClock;     /* not static so you can see in ROV */
    static Clock_Handle TimerButtonClockHandle;


// ------------------------------------Local Variables--------------------------------------------------

#ifdef WATCHDOG
    Watchdog_Handle watchdogHandle;

    #define WATCHDOG_TASK_STACK_SIZE 512

    static Task_Params WatchdogTaskParams;
    Task_Struct WatchdogTask;    /* not static so you can see in ROV */
    static uint8_t WatchdogTaskStack[WATCHDOG_TASK_STACK_SIZE];
    uint8_t watchdog_activ;

    static void WatchdogTaskFunction(UArg arg0, UArg arg1);

#endif

// ------------------------------------ Global  Variables--------------------------------------------------

#ifdef WATCHDOG
    uint8_t watchdog_tache[NB_TACHES_WDT];  // etat tache : 0:asleep 1:unknown_asleep 2:unknown_active  3:active
    uint32_t watchdog_cpt_activ[NB_TACHES_WDT];   // compteur d'activation de la tache
    uint32_t watchdog_cpt_sleep[NB_TACHES_WDT];  // compteur d'inactivation de la tache
    uint32_t watchdog_cpt_activ_max[NB_TACHES_WDT];   // compteur max d'activation de la tache
    uint32_t watchdog_cpt_activ_tot[NB_TACHES_WDT];  // compteur total d'activation de la tache
    uint32_t watchdog_cpt_sleep_max[NB_TACHES_WDT];  // compteur max d'inactivation de la tache
#endif


// ------------------------------ Extern -------------------------------------------------------------


 extern S_SortieTor SortieTor[];
 extern etat_entree h_etat_entree[];



// ------------------------------------Local Prototypes---------------------------------------------------

 void TimerButtonCallback(UArg arg0);
 void PeriodiqueCallback(UArg arg0);
 void buttonCallback(PIN_Handle handle, PIN_Id pinId);


 /* Pin driver handle */
 PIN_Handle buttonPinHandle;
 PIN_Handle ledPinHandle;

 /* Global memory storage for a PIN_Config table */
 static PIN_State buttonPinState;
 static PIN_State ledPinState;


// TABLE PIN SORTIES TOR

PIN_Config LEDpinTable[] = {
     Board_PIN_LED0      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#if (NB_SORTIES > 1)
     Board_GPIO_S1      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#if (NB_SORTIES > 2)
     Board_GPIO_S2  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#if (NB_SORTIES > 3)
     Board_GPIO_S3  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#if (NB_SORTIES > 4)
     Board_GPIO_S4  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
#endif
#if (NB_SORTIES > 5)
     Board_GPIO_S5  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
#endif
#if (NB_SORTIES > 6)
     Board_GPIO_S6  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
#endif
#endif
     PIN_TERMINATE
 };


 /*
  * Application button pin configuration table:
  *   - Buttons interrupts are configured to trigger on falling edge.
  *   Configuration des entrées TOR
  */


#ifdef CC1310_carte

#ifdef C10_Chien
   PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
   PIN_TERMINATE
  };
#endif

#ifdef C10_Concentrator
      PIN_Config buttonPinTable[] = {
       Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
      PIN_TERMINATE
     };
   #endif

#ifdef C10_ElecVanne   // table entree
  PIN_Config buttonPinTable[] = {
      Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
      PIN_TERMINATE
  };
 #endif

#ifdef C10_sonde_HUmSol    // table entree
  PIN_Config buttonPinTable[] = {
      Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
      PIN_TERMINATE
  };
 #endif

   #ifdef C10_Compteur  // table entree : contact reed
     PIN_Config buttonPinTable[] = {
         Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
         PIN_TERMINATE
     };
    #endif

#ifdef C10_Cent_Meteo  // table entree : capteur pluviometre
  PIN_Config buttonPinTable[] = {
      Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
      PIN_TERMINATE
  };
 #endif

 #ifdef TELECOMMANDE  // table entree : 3 boutons
     PIN_Config buttonPinTable[] = {
         Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
         Board_PIN_BUTTON1  | PIN_INPUT_EN | Pull_B1 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_NEGEDGE,
         Board_PIN_BUTTON2  | PIN_INPUT_EN | Pull_B2 | PIN_IRQ_BOTHEDGES,
         PIN_TERMINATE
     };
    #endif

    #ifdef RADAR_DET  // table entree : contact detection et anti-arrachage
     PIN_Config buttonPinTable[] = {
         Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
         Board_PIN_BUTTON1  | PIN_INPUT_EN | Pull_B1 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_NEGEDGE,
         PIN_TERMINATE
     };
    #endif


    #ifdef C10_Spot_solaire   // table entree : 1 entrée TOR : detecteur PIR
     PIN_Config buttonPinTable[] = {
         Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
         PIN_TERMINATE
     };
    #endif

    #ifdef C10_CAMERA_PIR   // table entree : 1 entrée TOR : detecteur PIR
     PIN_Config buttonPinTable[] = {
         Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
         PIN_TERMINATE
     };
    #endif

#ifdef C10_Test
 PIN_Config buttonPinTable[] = {
     Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
     PIN_TERMINATE
 };
#endif

 #endif

#ifdef CC1350_carte
 PIN_Config buttonPinTable[] = {
     Board_PIN_BUTTON0  | PIN_INPUT_EN | Pull_B0 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_BOTHEDGES,
     Board_PIN_BUTTON1  | PIN_INPUT_EN | Pull_B1 | PIN_IRQ_BOTHEDGES, //PIN_IRQ_NEGEDGE,
     #if VERS_HARD == 4
          CC1350STK_RELAY | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,      // version 1.4.5
     #elif VERS_HARD == 5
          CC1350STK_RELAY | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS | PIN_INPUT_DIS,           // version 1.5.1
     #endif
     PIN_TERMINATE
 };
#endif
// CC1350STK_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,           /* LED initially off */
// CC1350STK_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,          /* Button is active low */
// CC1350STK_KEY_RIGHT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button is active low */



//  -------------------------------------- Debut fonctions ------------------------------------------


  void watchdogCallback(Watchdog_Handle handle)
  {
      //printf("Watchdog timer triggered!\n");
      //releaseResources();
      while (1) {};
      //watchdog_cpt_activ[0]=0;
  }


 void Periph_init(void) {

     Clock_Params clkParams;
     Clock_Params_init (&clkParams);

    // activation Watchdog
     #ifdef WATCHDOG
         Watchdog_Params Wparams;
         Watchdog_init();

         /* Create and enable a Watchdog with resets enabled */
         Watchdog_Params_init(&Wparams);
         Wparams.callbackFxn = (Watchdog_Callback)watchdogCallback;
         Wparams.resetMode = Watchdog_RESET_ON;
         Wparams.debugStallMode = Watchdog_DEBUG_STALL_ON;

         watchdogHandle = Watchdog_open(Board_WATCHDOG0, &Wparams);
         if (watchdogHandle == NULL) {
             code_erreur = 0x33;
             // Error opening Watchdog
             //while (1);
         }
         //Watchdog_setReload(watchdogHandle, 20000 * (1000 / Clock_tickPeriod));
         Watchdog_setReload(watchdogHandle, Watchdog_convertMsToTicks(watchdogHandle, 300)); // 300ms = 60 taches wdt => 20 minutes

         // initialisation supervision par tache Watchdog
         uint8_t i;
         for (i=0; i<NB_TACHES_WDT; i++)
         {
             watchdog_tache[i]=0;
             watchdog_cpt_activ[i]= 0;
             watchdog_cpt_sleep[i] = 0;
             watchdog_cpt_activ_max[i]= 0;
             watchdog_cpt_sleep_max[i] = 0;
             watchdog_cpt_activ_tot[i] = 0;
         }
         watchdog_activ=0;

         // création tâche surveillance watchdog
         Task_Params_init(&WatchdogTaskParams);
         WatchdogTaskParams.stackSize = WATCHDOG_TASK_STACK_SIZE;
         WatchdogTaskParams.priority = Task_numPriorities - 1;
         WatchdogTaskParams.stack = &WatchdogTaskStack;
         Task_construct(&WatchdogTask, WatchdogTaskFunction, &WatchdogTaskParams, NULL);

         /* création clock watchdog
         clkParams.period = 10 * (1000 * 1000 / Clock_tickPeriod);  // 10 secondes
         clkParams.startFlag = TRUE;
         Clock_construct(&WatchdogClock, WatchdogCallback, clkParams.period, &clkParams);
         WatchdogClockHandle = Clock_handle(&WatchdogClock);*/

     #endif

    // Open led pin uniquement pour CC1350
    #ifdef CC1350_carte
         // Open led Pin
         ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
         //Assert_isTrue(pinHandle != NULL, NULL);
         PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
    #endif

     // Open led pin + 1 sortie : pour CC1310-Spot
     #ifdef CC1310_carte
        #ifdef C10_Chien  // table sortie : S1 : HP chien
          // Open led Pin
          ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
          //Assert_isTrue(pinHandle != NULL, NULL);
          PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S1, 0);  // Clear HP chien
        #endif
        #ifdef C10_ElecVanne  // table sorties
          // Open led Pin
          ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
          //Assert_isTrue(pinHandle != NULL, NULL);
          PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S1, 0);   // Clear Electrovannes
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S2, 0);
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S3, 0);
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S4, 0);
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S5, 0);
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S6, 0);
        #endif
        #ifdef C10_sonde_HUmSol   // table sortie : 1 led + power on sonde
          // Open led Pin
          ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
          //Assert_isTrue(pinHandle != NULL, NULL);
          PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
          PIN_setOutputValue(ledPinHandle, Board_GPIO_S1, 0);   // power sonde : on/off
        #endif

        #ifdef C10_Compteur  // table sortie : 1 Led
          // Open led Pin
          ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
          //Assert_isTrue(pinHandle != NULL, NULL);
          PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
        #endif

        #ifdef C10_Cent_Meteo  // table sortie : 1 Led
          // Open led Pin
          ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
          //Assert_isTrue(pinHandle != NULL, NULL);
          PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
        #endif

        #ifdef TELECOMMANDE  // table sortie : 1 led
          // Open led Pin
          ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
          //Assert_isTrue(pinHandle != NULL, NULL);
          PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
        #endif
        #ifdef C10_Spot_solaire  // table sortie : 1 led
           // Open led Pin et sortie allumage spot
           ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
           //Assert_isTrue(pinHandle != NULL, NULL);
           PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
        #endif
        #ifdef C10_CAMERA_PIR  // table sortie : led, Charge_batterie, Power ESP32
           // Open led Pin et sortie allumage spot
           ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
           //Assert_isTrue(pinHandle != NULL, NULL);
           PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
           PIN_setOutputValue(ledPinHandle, Board_GPIO_S1, 1);   // Set Enable charge batterie
           PIN_setOutputValue(ledPinHandle, Board_GPIO_S2, 0);   // Clear ESP32CAM
        #endif
        #ifdef C10_Test
           // Open led Pin et sortie allumage spot
           ledPinHandle = PIN_open(&ledPinState, LEDpinTable);
           //Assert_isTrue(pinHandle != NULL, NULL);
           PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);  // Clear Led Pin
        #endif
    #endif




    // Setup boutons

        /* setup timer pour vérifier que l'on appui 2 fois sur un bouton */
        clkParams.period = 0;  // inutile
        clkParams.startFlag = FALSE;
        Clock_construct(&TimerButtonClock, TimerButtonCallback, 1, &clkParams);
        TimerButtonClockHandle = Clock_handle(&TimerButtonClock);



    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);   // Ouverture PIN Bouttons
    if(!buttonPinHandle)
    {
         code_erreur = Error_initializing_board;
         err_donnee1 = '3';
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallback) != 0)
    {
         code_erreur = Error_initializing_board;
         err_donnee1 = '4';
    }

    b0prec=PIN_getInputValue(Board_PIN_BUTTON0);  //  1 si pas appuyé
    b1prec=PIN_getInputValue(Board_PIN_BUTTON1);  //  id
    double_appui = 0;
    phase_appui_b0=0;
    phase_appui_b1=0;
    b0_debut = 0;
    b1_debut = 0;

    #if BOUTON_CC == 3
         b2prec=PIN_getInputValue(Board_PIN_BUTTON2);    //  0 si pas de boutton 2
         phase_appui_b2=0;
         b2_debut = 0;
    #endif



 }

#ifdef WATCHDOG
 // Watchdog et supervision des taches
static void WatchdogTaskFunction(UArg arg0, UArg arg1)
{
      while(1)
      {
          uint8_t wdt_reload, wdt_mess, inactif;
          uint8_t i;

          DELAY_MS (20000);  // 20 secondes (et 300ms) 1sec=>3min34 2sec=>7min  20sec=>70min
          //Power_disablePolicy();
          //DELAY_US (1000);  // 1ms

          wdt_reload=1;
          wdt_mess=0;
          inactif = 1;  // 1:inactif, 0:au moins 1 tache active

          for (i=0; i<NB_TACHES_WDT; i++)
          {
              // etat tache : 0:asleep 1:unknown_asleep 2:unknown_active  3:active

              if (watchdog_tache[i] > 1) inactif = 0;  // tache active

              if (watchdog_tache[i] == 2)
              {
                  wdt_reload=0;  // tache active bloquee => ne reload pas le wdt => envoyer 2+i
                  wdt_mess = 2 + (i<<2);  // 2 pour node, 6 pour radio
              }
              if (watchdog_tache[i] == 3)
              {
                  watchdog_tache[i] = 2;  // activ -> unknown
                  watchdog_cpt_sleep[i] = 0;  // reset cpt asleep
              }
              if (watchdog_tache[i] == 0)
              {
                  watchdog_cpt_sleep[i]++;
                  if ((watchdog_cpt_sleep[i] > WATCHDOG_SLEEP) || (!WATCHDOG_SLEEP))  // tache inactive 100.000s (25 heures) => ne reload pas => envoyer 0+i
                  {
                      wdt_reload=0;
                      wdt_mess = 1+ (i<<2);  // 1 pour node, 5 pour radio
                      watchdog_cpt_sleep[i] = 0;
                  }
              }
              if ((watchdog_cpt_activ[i] > WATCHDOG_ACTIVE)  || (!WATCHDOG_ACTIVE))  // tache activee 3000 fois en 20 secondes : trop => reset => envoyer 3+i
              {
                  wdt_reload=0;
                  wdt_mess = 3 + (i<<2);  // 3 pour node, 7 pour radio
              }
              if (watchdog_cpt_activ[i] > watchdog_cpt_activ_max[i]) watchdog_cpt_activ_max[i] = watchdog_cpt_activ[i];
              watchdog_cpt_activ_tot[i] += watchdog_cpt_activ[i];
              watchdog_cpt_activ[i]=0; // reset cpt activ
              if (watchdog_cpt_sleep[i] > watchdog_cpt_sleep_max[i]) watchdog_cpt_sleep_max[i] = watchdog_cpt_sleep[i];

          }
          if ((!wdt_reload) && (!watchdog_activ))
              watchdog_activ=1;  // declenchement watchdog

          //Power_enablePolicy();

          if (watchdog_activ)
          {
              if (watchdog_activ==1)  // premiere fois
              {
                  watchdog_activ=2;
                  strcpy ( (char *)message, "LWatch:x");
                  message[7] = wdt_mess+'0';
                  envoi_message();
                  DELAY_MS(1500);
                  strcpy ( (char *)message, "0ZR");  // reset
                  envoi_message();
              }
          }
          else
          {
                 Watchdog_clear(watchdogHandle);
          }


          if (inactif)  // ecoute continu => verif que radio rx est active
          {
                #ifdef CONCENTRATOR
                  verif_radio_rx();
                #endif
          }
      }
}
#endif

/*
 *  ======== buttonCallback ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 */
void buttonCallback(PIN_Handle handle, PIN_Id pinId)
{
    bool b0,b1,  b0chgt, b1chgt;
    #if BOUTON_CC == 3
    bool b2,  b2chgt;
    #endif


    //time_cpu[index_time++] = Clock_getTicks(); // 100.000 = 1 seconde
    //if (index_time >18) index_time=0;

     /* Debounce logic */
     //CPUdelay(8000*50);  // 1=0,1us   300.000 = 33 ms
     //DELAY_MS(33);
     // MINOR : mettre fonction en event pour éviter d'utiliser l'interruption trop longtemps

     //new_timer
     b0=PIN_getInputValue(Board_PIN_BUTTON0);
     if ((Pull_B0) & (2<<13))
         b0=1-b0;  // inversion
     b1=PIN_getInputValue(Board_PIN_BUTTON1);
     if ((Pull_B1) & (2<<13)) b1=1-b1;  // inversion
     b0chgt=0;
     b1chgt=0;
     if (b0!=b0prec) b0chgt=1;
     if (b1!=b1prec) b1chgt=1;
     b0prec = b0;
     b1prec = b1;

     #if BOUTON_CC == 3
         b2=PIN_getInputValue(CC1350STK_RELAY);
         if ((Pull_B2) & (2<<13)) b2=1-b2;  // inversion
         b2chgt=0;
         if (b2!=b2prec) b2chgt=1;
         b2prec = b2;
    #endif

     uint32_t clock_get;
     clock_get = Clock_getTicks();

    if (Config_boutons ==1 )  // Seul est actif l'appui court : pas de double appui, pas d'appui long, pas 2 boutons appuyés simultanément
    {
    //Attention : il peut y avoir plusieurs rebonds - le bouton est alors activé plusieurs fois
     if (b0chgt)  // si bouton 0 modifie
     {
         if (!b0)  // 0 si bouton appuye
         {
             //if ( ((clock_get - b0_debut) > 3000) && (b0_debut))  // bouton appuye plus de 30 ms
             //{
                 b0_appui = 1;  // appui court
                 appui_boutton_0();
             //}
             b0_debut=0;
         }
         //else   // si bouton appuye
         //    b0_debut = clock_get;  // relevé clock pour tester le bouton appuyé longtemps
     }
     if (b1chgt)  // si bouton 1 modifie
     {
         if (!b1)  // si bouton appuye
         {
             //if ( ((clock_get - b1_debut) > 3000)  && (b1_debut)) // bouton appuye plus de 30 ms
             //{
                 b1_appui = 1;  // appui court
                 appui_boutton_1();
             //}
             b1_debut=0;
         }
         // else   // si bouton appuye
         //    b1_debut = clock_get;  // relevé clock pour tester le bouton appuyé longtemps
     }
    }

    if (Config_boutons ==2 )  // appui cout, appui long, double appui, triple appui, 2 boutons simultanés
    {
     if (double_appui)  // Si les 2 boutons ont été appuyés auparavant
     {
         if ((b0) && (b1))  //  2 boutons relaches
         {
             double_appui=0;
         }
     }
     else if ((!b0) && (!b1))  // si les 2 boutons sont appuyes
     {
         appui_2bouttons();
         double_appui=1;
     }


     else if (b0chgt)  // si bouton 0 modifie
     {
         if (b0)  // si bouton relache  // 0:long  1:court  2:double  3:3fois
         {
             if ( (clock_get - b0_debut) < 3000)  // bouton relaché trop vite (moins de 30 ms)
                 phase_appui_b0 =0;
             else
             {
                 if (clock_get > (b0_debut+200000))  // bouton appuyé 2 secondes
                 {
                     Clock_stop(TimerButtonClockHandle);  // arret, si le timer était encore en cours
                     phase_appui_b0 =0;
                     b0_appui = 0;  // appui long
                     appui_boutton_0();
                 }
                 else  // Bouton relache dans les 2 secondes
                 {
                     Clock_stop(TimerButtonClockHandle);  // arret, si le timer était encore en cours
                     phase_appui_b0++;
                     b0_suite = clock_get;  // pour voir si le bouton est relaché suffisamment longtemps
                     //if (phase_appui_b0 <= 2)  // bouton relache la première ou deuxieme fois
                     //{
                         Clock_setPeriod(TimerButtonClockHandle, 0 ); // Period=0 pour un one-shot
                         Clock_setTimeout(TimerButtonClockHandle, (1200 * 1000 / Clock_tickPeriod));  // 800ms max entre les 2 appuis
                         Clock_start(TimerButtonClockHandle);
                     /*}
                     else  // bouton relache la troisieme fois
                     {
                         phase_appui_b0 =0;
                     }*/
                 }
             }
         }
         else   // sinon bouton appuyé
         {
             /*if (phase_appui_b0==2) // si on appuie la troisieme fois
             {
                 if ( (clock_get - b0_suite) > 100000)  // bouton pas reappuye trop vite (moins de 1s)
                 {
                     Clock_stop(TimerButtonClockHandle);  // arret, si le timer était encore en cours
                     b0_appui = 4;  // appui double
                     appui_boutton_0();
                 }
                 else
                     phase_appui_b0 =0;
             }*/
             b0_debut = clock_get;  // relevé clock pour tester le bouton appuyé longtemps
         }
     }

     else if (b1chgt)  // bouton 1 modifie
     {
         if (b1)  // 1=bouton relache
         {
             if ( (clock_get-b1_debut) < 3000)  // bouton relaché trop vite (moins de 30 ms)
                 phase_appui_b1 =0;
             else
             {
                 if (clock_get > (b1_debut+200000))
                 {
                     Clock_stop(TimerButtonClockHandle);  // arret, si le timer était encore en cours
                     b1_appui = 0;  // appui long
                     appui_boutton_1();
                 }
                 else  // bouton relache dans la seconde
                 {
                     Clock_stop(TimerButtonClockHandle);  // arret, si le timer était encore en cours
                     phase_appui_b1++;
                     b1_suite = clock_get;  // pour voir si le bouton est relaché suffisamment longtemps
                     Clock_setPeriod(TimerButtonClockHandle, 0 ); // Period=0 pour un one-shot
                     Clock_setTimeout(TimerButtonClockHandle, (1200 * 1000 / Clock_tickPeriod));  // 400ms max entre les 2 appuis
                     Clock_start(TimerButtonClockHandle);
                 }
             }
         }
         else   // sinon 0=bouton appuyé
         {
             b1_debut = clock_get;  // relevé clock pour tester le bouton appuyé longtemps
         }
     }
    #if BOUTON_CC == 3
     else if (b2chgt)  // bouton 2 modifie
     {
         if (b2)  // bouton relache
         {
             if (clock_get > (b2_debut+200000))
             {
                 Clock_stop(TimerButtonClockHandle);  // arret, si le timer était encore en cours
                 b2_appui = 0;  // appui long
                 appui_boutton_2();
                 }
             else  // bouton relache dans la seconde
             {
                 Clock_stop(TimerButtonClockHandle);  // arret, si le timer était encore en cours
                 phase_appui_b2++;
                 b2_suite = clock_get;  // pour voir si le bouton est relaché suffisamment longtemps
                 Clock_setPeriod(TimerButtonClockHandle, 0 ); // Period=0 pour un one-shot
                 Clock_setTimeout(TimerButtonClockHandle, (400 * 1000 / Clock_tickPeriod));  // 400ms max entre les 2 appuis
                 Clock_start(TimerButtonClockHandle);
             }
         }
         else   // sinon bouton appuyé
         {
             b2_debut = clock_get;  // relevé clock pour tester le bouton appuyé longtemps
         }
     }
    #endif
    }

/*     if ((PIN_getInputValue(Board_PIN_BUTTON0) == 0) & (PIN_getInputValue(Board_PIN_BUTTON1) == 0))
     {  // 2 boutons appuyÃ©s
     }
     else if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
     {
         appui_boutton_0();
     }
     else if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
     {
         appui_boutton_1();
     }*/
 }

void TimerButtonCallback(UArg arg0)  // callback si on n'a pas appuyé une 2° fois sur le boutons, 80ms après l'avoir relaché
{
    // 0:long  1:court  2:double  3:3fois
    if (phase_appui_b0)
    {
        b0_appui = phase_appui_b0; // appui
        appui_boutton_0();
    }

    if (phase_appui_b1)
    {
        b1_appui=phase_appui_b1; // appui
        appui_boutton_1();
    }
    phase_appui_b0 = 0;
    phase_appui_b1 = 0;
    #if BOUTON_CC == 3
       if (phase_appui_b2)
       {
           b2_appui=phase_appui_b2; // appui
           appui_boutton_2();
       }
       phase_appui_b2 = 0;
    #endif
}





