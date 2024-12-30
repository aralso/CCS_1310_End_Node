/*
 * utilitaires.c
 *
 *
 * *  3.1.5 Sept 2020 :
 * *  3.1.4 Sept 2020 : callback clock identique
 *
 *  Created on: 23 juil. 2017
 *      Author: Tocqueville
 */


/* Board Header files */
#include "Board.h"

/*#pragma DATA_ALIGN(R2R_Flash_JT, 4) Exemple d'Alignement
const uint32 R2R_Flash_JT[] =
{
  // HCI ROM-to-ROM Functions
  (uint32)HCI_CommandCompleteEvent,                      // R2R_JT_OFFSET(0)
  (uint32)HCI_CommandStatusEvent,                        // R2R_JT_OFFSET(1)
}*/


//  -------------------------------------   Define  ----------------------------------------

// Codes erreur

     // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
     // Erreurs 20 a 7F : 4 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
     // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

#define erreur_analog               0x42  // 40 et 41:periph
#define erreur_demar                0x43
#define erreur_util_rtos            0x43
#define erreur_tab                  0x44


#define nb_erreurs_envoyees      30
#define nb_erreurs_unique        0x20
#define nb_erreurs_4_fois        0x60


#define stack_size_tache_period 512

//  -------------------------------------   Type declarations  ----------------------------------------



//  -------------------------------------   Global Variables   ----------------------------------------

// Taches
// static Task_Params T1sTaskParams;
 //Task_Struct T1sTask;    /* not static so you can see in ROV */
 //static uint8_t T1sTaskStack[ stack_size_tache_period];

// static Task_Params T100msTaskParams;
 //Task_Struct T100msTask;    /* not static so you can see in ROV */
 //static uint8_t T100msTaskStack[ stack_size_tache_period];

 uint8_t code_erreur, comptage_erreur;
 uint8_t err_donnee1, err_donnee2;
 uint8_t enr_erreur[nb_erreurs_enregistrees];          // Enr_erreur enregistre les 30 premiÃ¨res erreurs
 uint8_t erreurs_unique[nb_erreurs_unique/8];    // 1:erreur dÃ©ja envoyÃ©e, plus d'envoi
 uint8_t erreurs_4_fois[nb_erreurs_4_fois/4];       // Nb d'erreurs dÃ©ja envoyÃ©es, max 4

 uint8_t nb_reset;

 S_SortieTor SortieTor[NB_SORTIES];


 etat_entree h_etat_entree[NB_ENTREES];

 EtatPwm    Pwm[NB_PWM];  // 1 ou 2

 uint8_t mode_arret;  // 0:marche  1:reduit1  2:reduit2  3:reduit3  4:arret total



uint16_t volt_mesure_bas, volt_mesure_dernier;
uint16_t voltage_panneau;
uint8_t seuil_batt_depasse;
int16_t Temp_actuelle;   // Temp au centieme de degré


#ifdef DEBUG_TIME
uint32_t time_cpu[MAX_Index+1], etat_detail[MAX_Index+1];
uint8_t etat_cpu[MAX_Index+1], index_time, trigger_time;
#endif

//  -------------------------------------   Extern Variables ----------------------------------------

#ifdef CC13xx
   extern PIN_Handle ledPinHandle;         // Led
   extern PIN_Handle buttonPinHandle;      // Bouttons
    /* Clock pour allumage periodique Sortie Tor */
    Clock_Struct SORTIE_S0_Clock;     /* not static so you can see in ROV */
    Clock_Struct SORTIE_S1_Clock;     /* not static so you can see in ROV */
    Clock_Struct SORTIE_S2_Clock;     /* not static so you can see in ROV */
    Clock_Struct SORTIE_S3_Clock;     /* not static so you can see in ROV */
    Clock_Struct SORTIE_S4_Clock;     /* not static so you can see in ROV */
    Clock_Struct SORTIE_S5_Clock;     /* not static so you can see in ROV */
    Clock_Struct SORTIE_S6_Clock;     /* not static so you can see in ROV */

    /* Clock pour allumage periodique PWM0 BUZ */
    Clock_Struct SORTIE_PWM0_Clock;     /* not static so you can see in ROV */
    //#if (NB_PWM>1)
      /* Clock pour allumage periodique PWM1 Light */
      Clock_Struct SORTIE_PWM1_Clock;     /* not static so you can see in ROV */
    //#endif
#endif
 //extern uint8_t  buzzer_actif;



//  -------------------------------------   Local functions prototypes  ----------------------------------------

void allumedebut_PWM(uint8_t num);

//  -------------------------------------   Functions descriptions  ----------------------------------------

 // initialise les taches

 void ES_init(void)
 {
     mode_arret = 0;   // actif au démarrage

     uint8_t i;

     //  Initialisation de toutes les entrees
        for (i=0; i<NB_ENTREES; i++)
        {
            h_etat_entree[i].configure = 1;
            h_etat_entree[i].max = 20;
            h_etat_entree[i].comptage = 10;
            h_etat_entree[i].etat = 1;
            h_etat_entree[i].duree_activ_manu = 0;
            h_etat_entree[i].chgt_etat = 0;
        }
        for (i=0; i<NB_SORTIES; i++)
        {
            SortieTor[i].configure = 1;
            SortieTor[i].etat_initial = 0;
            SortieTor[i].inversion =0;  // 0:eteint  1:allume
            SortieTor[i].etat_ref = 0;
        }

        // setup clock pour allumage periodique sorties
        #ifdef CC13xx
            Clock_Params clkParams;
            Clock_Params_init (&clkParams);
            clkParams.period = 0;
            clkParams.startFlag = FALSE;

            #if NB_SORTIES >0
                SortieTor[0].sortie_pin = Board_GPIO_S0;
                clkParams.arg = 0;
                Clock_construct(&SORTIE_S0_Clock, SORTIE_S_Callback, 1, &clkParams);
                SortieTor[0].h_clock = Clock_handle(&SORTIE_S0_Clock);
            #endif
            #if NB_SORTIES >1
                SortieTor[1].sortie_pin = Board_GPIO_S1;
                clkParams.arg = 1;
                Clock_construct(&SORTIE_S1_Clock, SORTIE_S_Callback, 1, &clkParams);
                SortieTor[1].h_clock = Clock_handle(&SORTIE_S1_Clock);
            #endif
            #if NB_SORTIES >2
                SortieTor[2].sortie_pin = Board_GPIO_S2;
                clkParams.arg = 2;
                Clock_construct(&SORTIE_S2_Clock, SORTIE_S_Callback, 1, &clkParams);
                SortieTor[2].h_clock = Clock_handle(&SORTIE_S2_Clock);
            #endif
            #if NB_SORTIES >3
                SortieTor[3].sortie_pin = Board_GPIO_S3;
                clkParams.arg = 3;
                Clock_construct(&SORTIE_S3_Clock, SORTIE_S_Callback, 1, &clkParams);
                SortieTor[3].h_clock = Clock_handle(&SORTIE_S3_Clock);
            #endif
            #if (NB_SORTIES > 4)
                clkParams.arg = 4;
                SortieTor[4].sortie_pin = Board_GPIO_S4;
                Clock_construct(&SORTIE_S4_Clock, SORTIE_S_Callback, 1, &clkParams);
                SortieTor[4].h_clock = Clock_handle(&SORTIE_S4_Clock);
            #endif
            #if (NB_SORTIES > 5)
                clkParams.arg = 5;
                SortieTor[5].sortie_pin = Board_GPIO_S5;
                Clock_construct(&SORTIE_S5_Clock, SORTIE_S_Callback, 1, &clkParams);
                SortieTor[5].h_clock = Clock_handle(&SORTIE_S5_Clock);
            #endif
            #if (NB_SORTIES > 6)
                clkParams.arg = 6;
                SortieTor[6].sortie_pin = Board_GPIO_S6;
                Clock_construct(&SORTIE_S6_Clock, SORTIE_S_Callback, 1, &clkParams);
                SortieTor[6].h_clock = Clock_handle(&SORTIE_S6_Clock);
            #endif


            // setup clock pour allumage periodique PWM
            #if (NB_PWM>0)
                clkParams.period = 0;
                clkParams.startFlag = FALSE;
                clkParams.arg = 0;
                Clock_construct(&SORTIE_PWM0_Clock, SORTIE_PWM_Callback, 1, &clkParams);
                Pwm[0].h_clock = Clock_handle(&SORTIE_PWM0_Clock);
                #if (NB_PWM>1)
                    clkParams.arg = 1;
                    Clock_construct(&SORTIE_PWM1_Clock, SORTIE_PWM_Callback, 1, &clkParams);
                    Pwm[1].h_clock = Clock_handle(&SORTIE_PWM1_Clock);
                #endif
            #endif
        #endif

        #ifdef ESP32
            #if NB_SORTIES >0
                SortieTor[0].h_clock = xTimerCreate("SORTIE_S0", (300), pdTRUE, ( void * ) 0, SORTIE_S_Callback);  // name,period-tick, autoreload,id, callback
                SortieTor[0].sortie_pin = Board_GPIO_S0;
            #endif
            #if NB_SORTIES >1
                SortieTor[1].h_clock = xTimerCreate("SORTIE_S1", (300), pdTRUE, ( void * ) 1, SORTIE_S_Callback);  // name,period-tick, autoreload,id, callback
                SortieTor[1].sortie_pin = Board_GPIO_S0;
            #endif
            #if NB_SORTIES >2
                SortieTor[2].h_clock = xTimerCreate("SORTIE_S2", (300), pdTRUE, ( void * ) 2, SORTIE_S_Callback);  // name,period-tick, autoreload,id, callback
                SortieTor[2].sortie_pin = Board_GPIO_S0;
            #endif
            #if NB_SORTIES >3
                SortieTor[3].h_clock = xTimerCreate("SORTIE_S3", (300), pdTRUE, ( void * ) 3, SORTIE_S_Callback);  // name,period-tick, autoreload,id, callback
                SortieTor[3].sortie_pin = Board_GPIO_S0;
            #endif
            #if (NB_SORTIES > 4)
                SortieTor[4].h_clock = xTimerCreate("SORTIE_S4", (300), pdTRUE, ( void * ) 4, SORTIE_S_Callback);  // name,period-tick, autoreload,id, callback
                SortieTor[4].sortie_pin = Board_GPIO_S0;
            #endif

            // setup clock pour allumage periodique PWM
            #if (NB_PWM>0)
                Pwm[0].h_clock = xTimerCreate("PWM_S0", (300), pdTRUE, ( void * ) 0, SORTIE_PWM_Callback);  // name,period-tick, autoreload,id, callback
                #if (NB_PWM>1)
                    Pwm[1].h_clock = xTimerCreate("PWM_S0", (300), pdTRUE, ( void * ) 1, SORTIE_PWM_Callback);  // name,period-tick, autoreload,id, callback
                #endif
            #endif
        #endif






        seuil_batt_depasse = 0;
        volt_mesure_bas = 0;  // pas de notif au debut - init apres 4s : 844=3,3Volts
        Temp_actuelle = -100;  // -1°C

        init_PWM();

       /* #ifdef DEBUG_TIME
            index_time=0;
            #ifdef CONCENTRATOR
                trigger_time=0;
            #else
                trigger_time=0;
            #endif
        #endif  */
 }



 void Clignot_sortie(uint8_t num)
 {

    // Clignotement  Sortie LED
      if (SortieTor[num].configure)
      {
             ETAT_SORTIE(num);    // Clignotement - cas normal
      }
      else  // sortie non configuree
          Turn_off_LED(num);


 }



 /* Passerelle 39F6 EU
  -1230 US 1.4.5 passerelle test
 S-03F0 EU 1.4.5
  -7E30 EU 1.4.5 3003811
 T-2E62 US 1.4.5 calib
  -1F68         calib
 V-0F60 EU 151. sans hall
 X-628C EU 151
 W-7992 EU 1.5.1
  */
  uint8_t Part_Identif(void)
  {
      uint16_t  part_number =    ( HWREGH(FCFG1_BASE + 0x00000118  )) ;  // FCFG1_O_ICEPICK_DEVICE_ID

      uint8_t p_id=0;
      if (part_number == 0x2E62) p_id=0;  // T
      if (part_number == 0x1F68) p_id=1;
      if (part_number == 0x3396) p_id=2; // LGM
      if (part_number == 0x03F0) p_id=3; // S
      if (part_number == 0x0F60) p_id=4; // V
      if (part_number == 0x628C) p_id=5; // W
      if (part_number == 0x7992) p_id=6; // X

      return p_id;
  }

 struct timeval NB_MICROSEC(void)
 {
     struct timeval tv;
     #ifdef CC13xx
         Seconds_Time ts;
         Seconds_getTime(&ts);
         tv.tv_sec  = ts.secs;
         tv.tv_usec = ts.nsecs / 1000;
     #endif
     #ifdef ESP32
          gettimeofday(&tv, NULL);
     #endif
      return tv;
  }

 void ecrit_heure_rtc(void)
 {
     struct tm sTime;
     uint32_t secondes_rtc;
     struct timeval tv;
     uint16_t date_rtc;

     NB_SECONDES(secondes_rtc);
     date_rtc = secondes_rtc / (24*3600);

     sTime.tm_hour   = (message_in[5]-'0')*10 + message_in[6]-'0';
     sTime.tm_min = (message_in[7]-'0')*10 + message_in[8]-'0';
     sTime.tm_sec = (message_in[9]-'0')*10 + message_in[10]-'0';

     secondes_rtc = date_rtc * (24*3600) + sTime.tm_hour*3600 + sTime.tm_min*60 + sTime.tm_sec;
     tv.tv_sec = (time_t) secondes_rtc;
     ECRIT_SECONDES(tv);
 }

 void lecture_date_rtc(void)
 {
     uint32_t secondes_rtc;

     NB_SECONDES(secondes_rtc);
     message[1]='H';
     message[2]='D';
     message[3]=' ';
     conversion_sec_date ( secondes_rtc, 4);
     message[12]=car_fin_trame;
     envoi_message();
 }

 void conversion_sec_date(time_t secon, uint8_t position)  //  ecrit la date dans message Ã  la position
 {
     struct tm *sTime;
     //uint16_t mois, jour;

     /*jour = secon / (24*3600);
     sTime.tm_year  = ( jour / 365 ) - 30;
     mois    = ( jour - (sTime.tm_year+30) *365);
     sTime.tm_mon = mois / 30;
     sTime.tm_mday = mois % 30;*/

     sTime=localtime(&secon);
     memcpy(message+position,"00/00/00 ",9);

     /* Display date Format : dd-mm-yy */

     sTime->tm_year -= 100; // commence a 1900
     //sTime->tm_mday ++;   // commence a 1 pour le premier jour
     sTime->tm_mon ++;    // commence a 0 pour janvier

     message[position] = (sTime->tm_mday / 10) + '0';
     message[position+1] = (sTime->tm_mday  % 10) + '0';
     message[position+3] = (sTime->tm_mon / 10) + '0';
     message[position+4] = (sTime->tm_mon % 10) + '0';
     message[position+6]= (sTime->tm_year / 10) + '0';
     message[position+7]= (sTime->tm_year % 10)  + '0';
 }

 void ecrit_date_rtc(void)
 {
     struct tm sTime = {0};
     uint32_t secondes_rtc;
     struct timeval tv;


   //  sTime.tm_wday = message_in[5]-'0';
     sTime.tm_mday =   (message_in[5]-'0')*10 + (message_in[6]-'0') ;
     sTime.tm_mon =  (message_in[7]-'0')*10 + (message_in[8]-'0') - 1;
     sTime.tm_year =   (message_in[9]-'0')*10 + (message_in[10]-'0') + 100;  // since 1900

     // Convert to number of seconds, this will also fill up tm_wday and tm_yday
     time_t seconds = mktime(&sTime);
     NB_SECONDES(secondes_rtc);
     secondes_rtc = (secondes_rtc % (24*3600)) + seconds;

     //secondes_rtc = ( (sTime.tm_year+30)*365 + sTime.tm_mon*30 + sTime.tm_mday ) *24*3600  + Seconds_get() % (24*3600);
     tv.tv_sec = (time_t) secondes_rtc;
     tv.tv_usec = 0;
     ECRIT_SECONDES(tv);
 }

 void lecture_heure_rtc(void)
 {
     /* differents formats de heure/date :
     struct tm tm;       // decomposition en jour/mois/...
     time_t tt;             // 1 uint32_t : nb de secondes depuis 1er Janvier 1970
     struct timeval tv;   // 2 composants : tv_sec en time_t(secondes en long) et tv_usec en suseconds_t(microsecondes) */

     //uint32_t secondes_rtc;
     struct timeval tv;  // microsecondes
     uint8_t milli_sec10; // nb de 10*millis

     //secondes_rtc = Seconds_get();  // seconds


     //t1 = time(NULL);
     //ltm = localtime(&t1);
     //curTime = asctime(ltm);
     //System_printf("Time(GMT): %s\n", curTime);

     tv = NB_MICROSEC();  // seconds et microsecondes
     message[1]='H';
     message[2]='H';
     message[3]=' ';
     conversion_sec_heure ( tv.tv_sec, 4,1);
     message[12]='-';
     milli_sec10 = tv.tv_usec/10000;
     message[13] = ((milli_sec10 / 10 ) +'0');
     message[14] = ((milli_sec10 % 10 ) +'0');
     message[15]=car_fin_trame;

     envoi_message();
 }

 void conversion_sec_heure(uint32_t second, uint8_t position, bool decimal)  //  Ã©crit l'heure dans message Ã  la position
 {
     struct tm sTime;

     sTime.tm_hour = ( second / 3600 ) % 24;
     sTime.tm_min  = ( second / 60 ) % 60;
     sTime.tm_sec  = ( second % 60 );

     if ( position + 9 < MESSAGE_SIZE )
     {
       /* Display time Format : hh:mm:ss */
         memcpy(message+position,"00:00:00 ",9);
         message[position] = ((sTime.tm_hour / 10 ) +'0');
         message[position+1] = ((sTime.tm_hour % 10) +'0');
         message[position+3] = ((sTime.tm_min / 10) +'0');
         message[position+4] = ((sTime.tm_min % 10) +'0');
         message[position+6] = ((sTime.tm_sec / 10) +'0');
         message[position+7]= ((sTime.tm_sec % 10) +'0');
        // message[12]= deci((sTime.SubSeconds >> 4)  );
        // message[13]= deci((sTime.SubSeconds & 0x0F)  );
     }
     else
     {
         code_erreur = erreur_tab;  // et '4'
         err_donnee1 = '4';
     }

 }

#ifdef BATT_BMS

 // conversion voltage+temp en niveau batterie
 // 0%=2,8V  10%=3,6V  80%=4V  100%=4,2V
 uint8_t conv_volt_batt (uint16_t voltage_batt, uint8_t temp_mesure_t)
 {
     uint8_t niv;
     if (voltage_batt < 2800)
         niv = 0;
     else if (voltage_batt < 3600)       // 10%  : 0,8V pour 10% : 80mv/%
         niv = (voltage_batt - 2800) / 80;
     else if (voltage_batt < 4000)       // 80% : 0,4V pour 80%  : 5mV/%
         niv = (voltage_batt - 3600) / 5 + 10;
     else
         niv = (voltage_batt - 4000) / 20 + 90;  // 10% 0,2V pour 10% : 20mV/%
     if (niv > 100)
         niv = 100;
     return niv;
 }

 // parametre return : 0:ok  1:erreur
 uint8_t lecture_volt_batterie(uint16_t * voltage_bat, uint16_t * voltage_sol)
 {
     uint8_t ret, ret2, charge;

     charge = SortieTor[Charge_TP].consigne;

     if ( charge )  // si chargeur TP actif
         ACTIV_SORTIE( Charge_TP, 0, 0 );  // desactivation du circuit de charge
    // charge batterie : 12ms pour passer de 3,6->5,1V, 1ms pour l'inverse

     ret = lecture_analog(1, voltage_bat);
     *voltage_bat*=2;
     ret2 = lecture_analog(0, voltage_sol);
     *voltage_sol*=2;

     if ((*voltage_bat < SEUIL_ARRET_CHARGE) || (charge))
         ACTIV_SORTIE( Charge_TP, 1, 0 );  // activation du circuit de charge

     if ((ret) || (ret2)) return 1;
     else return 0;
 }


// return niveau en pourcentage
uint8_t mesure_niv_batterie(uint16_t volt, int16_t temp)
{
    uint8_t niv;
     // pile : volt=3300:100%   Volt=2300:0%
     if (volt < 2300)
         niv = 0;
     else if (volt > 3300)
         niv=100;
     else
         niv = (volt - 2300) / 10;

    #ifdef BATT_BMS
       niv = conv_volt_batt (volt, 25);
    #endif
       return niv;
}
#endif

// return : en millivolts
uint16_t mesure_voltage(void)
{
    uint32_t volt_mesure_v;
    float volt_float;
    uint16_t millivolts;
    bool err = 1;

    #ifdef BATT_BMS
        err = lecture_volt_batterie (&millivolts, &voltage_panneau);
    #endif

    if (err)
    {
        #ifdef CC13xx
          volt_mesure_v = AONBatMonBatteryVoltageGet();
        #endif
        #ifdef ESP32
          volt_mesure_v = 300;
        #endif
        volt_float = (float) volt_mesure_v / 256;
        millivolts = volt_float * 1000;
    }
    return millivolts;
}

//return : 0:interne   1:externe
// en centi_degres
uint8_t mesure_temperature(S_Temp *Stemp)
{
    uint8_t ret=0;
    #ifdef CAPTEUR_TEMP  // capteur de temperature externe HDC1000 ou HDC1100
        ret = lecture_temphdc1000();  // 0:ok  1:err
        //par_var[0] = ret;
        //par_var[2] = Temp_actuelle & 0xFF;
        //par_var[1] = (Temp_actuelle >> 8);
        ret = 1 - ret;  // 0:err(->interne) 1:ok
        if (ret)  // 1:ok
        {
            Stemp->temp = Temp_actuelle;
            Stemp->hygro = Float2Value * 100;
        }
    #endif
    if (!ret)  // lecture temperature interne
    {
        #ifdef CC13xx
            Stemp->temp = AONBatMonTemperatureGetDegC() * 100;
        #endif
        #ifdef ESP32
            Stemp->temp = 1000;
        #endif
        Temp_actuelle =   Stemp->temp;
    }
    return ret;
}

// mesure voltage batterie
int16_t mesure_volt_vref(void)
{
     int16_t diff_volt;


    #ifdef CC13xx
       volt_mesure_dernier = AONBatMonBatteryVoltageGet();
    #endif
    #ifdef ESP32
       volt_mesure_dernier = 0x300;
    #endif
     diff_volt = volt_mesure_bas - volt_mesure_dernier;         // prec=270   mes=260  diff=10     bas=270   dernier=260
     //if (diff_volt < 0) diff_volt = - diff_volt;
     if (diff_volt > 12)  // baisse de 12 => 12/256 > 0.05Volt
     {  // n'enregistre que les baisses
         volt_mesure_bas = volt_mesure_dernier;
     }
     return diff_volt;
}

void envoi_voltage_bas(void)
{
     float volt_float;
     volt_float = (float) volt_mesure_bas / 256;
     strcpy((char *)message, "L Voltage: ");
     ajout_float_message(volt_float, 2, 11, 2);
     envoi_message ();
 }


uint32_t mesure_temperature32(void)
{
#define TEMP130_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define VDD_CALIB ((uint16_t) (300))
#define VDD_APPLI ((uint16_t) (330))

  //uint32_t uwADCxConvertedValue;
  int32_t temperature;

  //uwADCxConvertedValue=0;
  temperature = 0;

  /* ### - 3 - Channel configuration ######################################## */
  /* Select Channel 0 to be converted */
/*  sConfig.Channel = ADC_CHANNEL_0;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)  Error_Handler();*/

  // Configure capteur de tempÃ©rature.
    //sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    //if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)  Error_Handler();

  // Configure vref interne.
/*    sConfig.Channel = ADC_CHANNEL_VREFINT;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)  Error_Handler();*/

 /*##- 4- Start the conversion process #######################################*/
  //if (HAL_ADC_Start(&AdcHandle) != HAL_OK)   Error_Handler();

    /*##- 5- Wait for the end of conversion #####################################*/
    /*  Before starting a new conversion, you need to check the current state of
         the peripheral; if itâ€™s busy you need to wait for the end of current
         conversion before starting a new one.
         For simplicity reasons, this example is just waiting till the end of the
         conversion, but application may perform other tasks while conversion
         operation is ongoing. */
    //HAL_ADC_PollForConversion(&AdcHandle, 10);

    /* Check if the continous conversion of regular channel is finished */
    //if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
    {
      /*##-6- Get the converted value of regular channel  ########################*/
      //uwADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
    }
    /* Temperature sensor calibration value address */
    //temperature = ((uwADCxConvertedValue * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR );
    temperature = temperature * (int32_t)(130 - 30);
    temperature = temperature / (int32_t)(*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR);
    temperature = temperature + 30;

  return temperature;
}


// parametre return : 0:ok  1:erreur
uint8_t lecture_analog(uint8_t num, uint16_t * adcVolt)
{
    uint8_t ret = 1;

    #ifdef CC13xx
    #ifdef NODE
        ADC_Handle   adc;
        ADC_Params   params;
        int_fast16_t res;
        uint16_t adcValue0;


        ADC_Params_init(&params);
        adc = ADC_open(Board_ADC0+num, &params);

        if (adc == NULL)
        {
            code_erreur = erreur_analog;
            err_donnee1= '1';
        }
        else
        {
            DELAY_MS(20);

            // Blocking mode conversion
            res = ADC_convert(adc, &adcValue0);

            if (res == ADC_STATUS_SUCCESS)
            {
                *adcVolt = (uint16_t) (ADC_convertRawToMicroVolts(adc, adcValue0)/1000);
                ret = 0;
            }
            else {
                code_erreur = erreur_analog;
                err_donnee1= '2';
            }
            ADC_close(adc);
        }
    #endif
    #endif
    return ret;
}




void envoi_code_erreur (void)        // envoie l'erreur a dest_erreur_reset
{
    uint8_t i3;


/*   ACTIV_SORTIE(0, 1, 10);  // LED2 2 flash  test
   message[0] = dest_erreur_reset;
   message[1] ='E';
   message[2] = code_erreur;
   message[3] = comptage_erreur +'0';
   message[4] = car_fin_trame;
   envoi_message();*/


    // Erreurs 0 a 1F : envoye une seule fois (appli >0x10)
    // Erreurs 20 a 7F : 3 fois        (Appli >0x70)  comm:2,3 util/periph:4  radio:5  tab:6 conc/appli:7
    // Erreurs 80 a FF : tout le temps (Appli > 0xD0)

    if (comptage_erreur < nb_erreurs_envoyees)  // 30 envoyees, 20 enregistrees
    {
       i3 = 0;
       if (comptage_erreur < nb_erreurs_enregistrees)  enr_erreur[comptage_erreur]=code_erreur;
        comptage_erreur++;
        if (code_erreur < (nb_erreurs_4_fois+nb_erreurs_unique))
        {
            if (code_erreur < nb_erreurs_unique) // 0x20  envoi_une seule fois
            {
                if (code_erreur < nb_erreurs_unique)
                {
                    uint8_t index, bit;
                    index = code_erreur/8;  // si code=0xA =>index=1, bit=2;
                    bit = code_erreur % 8;
                    if (! (erreurs_unique[index] & (1<<bit)))
                    {
                        i3=1;
                        erreurs_unique[index] |= (1<<bit);
                    }
                }
            }
            else      // envoi 4 fois max
            {
                uint8_t index, bit, cpt;
                index = (code_erreur-nb_erreurs_unique)/4;  // si code=0xA =>index=3, bit=4;
                bit = (code_erreur % 4)*2;
                cpt = (erreurs_4_fois[index] >> bit) & 3 ;
                if  (cpt < 3)
                {
                    i3=1;
                    erreurs_4_fois[index] += (1 << bit);
                }
            }
        }
        else i3=1;  //  0x80 Ã  0xFF

        if (i3)
          {
          #ifdef dest_erreur_reset
           message[0] = dest_erreur_reset;
           message[1] ='E';
           message[2] ='L';
           message[3] = nb_reset + 48;
           message[4] = comptage_erreur + 48;
           message[5] = ':';
           message[6] =deci(code_erreur >> 4 );
           message[7] =deci(code_erreur & 15);
           message[8] = '-';
           message[9] = err_donnee1;
           message[10] = err_donnee2;
           i3=9;
           if (err_donnee1) i3=10;
           if (err_donnee2) i3=11;
           message[i3] = car_fin_trame;    // 11=>Longueur=13 => important pour pile full
            envoi_message();
          #endif
          }
    }
    code_erreur=0;
    err_donnee1=0;
    err_donnee2=0;
}

void raz_erreur(void)
{
    uint8_t i1;
    code_erreur=0;
    err_donnee1=0;
    err_donnee2=0;
    comptage_erreur=0;
    for (i1=0; i1<(nb_erreurs_unique/8); i1++)   erreurs_unique[i1] = 0;
    for (i1=0; i1<(nb_erreurs_4_fois/4); i1++)   erreurs_4_fois[i1] = 0;
}


void Turn_on_LED(uint8_t num)
{
    SortieTor[num].etat = 1;
    uint8_t a=1;
    if (SortieTor[num].inversion) a=0;
    #ifdef CC13xx
          PIN_setOutputValue(ledPinHandle, SortieTor[num].sortie_pin, a);
    #endif
    #ifdef ESP32
          gpio_set_level (SortieTor[num].sortie_pin, a);
    #endif
}

void Turn_off_LED(uint8_t num)
{
    SortieTor[num].etat = 0;
    uint8_t a=0;
    if (SortieTor[num].inversion) a=1;
    #ifdef CC13xx
          PIN_setOutputValue(ledPinHandle, SortieTor[num].sortie_pin, a);
    #endif
    #ifdef ESP32
          gpio_set_level (SortieTor[num].sortie_pin, a);
    #endif

    //SortieTor[which].sortie_port->BRR = SortieTor[which].sortie_pin;

/*   if (0 == which)
  {
      GPIOA->BRR = GPIO_PIN_7;
  }
  else if (1 == which)
  {
      GPIOA->BRR = GPIO_PIN_6;
  }
  else if (2 == which)
  {
      GPIOA->BRR = GPIO_PIN_5;
  }
  else if (3 == which)
  {
      GPIOA->BRR = GPIO_PIN_5;
  }
  return;*/
}


// Active la sortie num en fonction de consigne et duree
void ACTIV_SORTIE(uint8_t num, uint8_t consigne, int unsigned duree)
// num : numero de la sortie
// consigne (C_Sortie) : 0 eteint, 1 allume, 2:clignot rapide0,3s-0,3s, 3:clig lent 1,5s-1,5s, 4:flash rapide(1ms)
//          5:spot(10s:allume 4s:eteint)  6 :1 flash lent 0,01-3s  7:7s-7s  8:0,1s-0,1s  9: 0,1s-25s   11 a 19 : 1 a 9 eclats
//          20:EV (pulse-attente-pulse)
// duree : duree en 100ms (4s = 40)

// variables internes : C_Sortie_NB:nb de clignotements  C_Sortie_M:compteur Etat:0 ou 1
{
if ( (num < NB_SORTIES) )
 {
    if (consigne>19) consigne =19;
    if  ( (consigne >9) && (consigne <11) ) consigne = 4;
    SortieTor[num].consigne = consigne;
    SortieTor[num].duree = duree;
    if (!duree) SortieTor[num].etat_ref = consigne;

    if (consigne >10) SortieTor[num].nb_flash = consigne - 11; else SortieTor[num].nb_flash = 1;  // Nb de clignotements (Consigne>11 uniqu)

    CLOCK_STOP(SortieTor[num].h_clock);

    if (consigne <2) //  0 ou 1 sans timer de chgt d'etat
    {
        if (!consigne)
            Turn_off_LED(num);   // desactive la sortie
        else
            Turn_on_LED(num);   // active la sortie
        if (duree)  // si duree non nulle : demarrage clock
        {
            CLOCK_RESTART_TIMEOUT(SortieTor[num].h_clock, duree * 100);  // millisecondes
        }
    }
    else  // consigne avec timer        // active la sortie
    {
        SortieTor[num].etat=0;  // etat initial
        ETAT_SORTIE(num);
    }
 }
 else
     code_erreur=depass_tab+3;
return;
}

void ETAT_SORTIE(uint8_t num)  // change l'etat des sorties
{
    uint16_t i1;  // en millisecondes, max 65 secondes
    uint8_t cycle_fin=0;

if (num < NB_SORTIES)
{
    if ((SortieTor[num].consigne != 0) && (SortieTor[num].consigne != 1))   // Consigne de programmation
    {
        if (SortieTor[num].etat == 0)   //Etat de la LED : LED eteinte => calcul du temps d'allumage
        {
            i1=0;
            switch (SortieTor[num].consigne)
            {                                 //   on     off
                case 2 : {i1 = 300 ;break;}     //  0,3s - 0,3s
                case 3 : {i1 = 1500;break;}     //  1,5s - 1,5z
                case 4 : {i1 = 1 ;break;}     //  1ms - 0,4s
                case 5 : {i1 = 10000;break;}    //  10s  - 4s
                case 6 : {i1 = 10 ;break;}     //  0,01s - 3s
                case 7 :
                    { if (SortieTor[num].duree>130) i1 = 7000; else i1=1000; break;}
                case 8 : {i1 = 100 ;break;}      //  0,1s - 0,2s
                case 9 : {i1 = 100 ; break;}     //  0,1s - 25s
                case 11 :
                case 12 :
                case 13 :
                case 14 :
                case 15 :
                case 16 :
                case 17 :
                case 18 :
                case 19 : {i1 = 100;break;}
                default : i1 = 10;
            }
        }
        else          //LED allumee => calcul du temps d'extinction
        {

            i1=0;
            switch (SortieTor[num].consigne)
            {
                case 2 : {i1 = 300 ;break;}
                case 3 : {i1 = 1500;break;}
                case 4 : {i1 = 700 ;break;}
                case 5 : {i1 = 4000;break;}
                case 6 : {i1 = 3000 ;break;}
                case 7 : {i1 = 5000 ;break;}
                case 8 : {i1 = 200  ;break;}
                case 9 : {i1 = 25000 ;break;}     //  0,01s - 25s
                case 11 :
                case 12 :
                case 13 :
                case 14 :
                case 15 :
                case 16 :
                case 17 :
                case 18 :
                case 19 :
                {
                    if (SortieTor[num].nb_flash == 0)
                    {
                        i1 = 1200;
                        SortieTor[num].nb_flash = SortieTor[num].consigne - 11;
                    }
                    else
                    {
                        i1 = 300;
                        SortieTor[num].nb_flash --;
                    }
                    break;
                }
                default : i1 = 10;
            }
        }
        if ((SortieTor[num].duree) && ( i1/100 >= SortieTor[num].duree ))
            cycle_fin=1;
    }
    else  // 0 ou 1 => fin de cycle
    {
        cycle_fin=1;
    }

    if (cycle_fin)  // fin de cycle
    {
        ACTIV_SORTIE( num, SortieTor[num].etat_ref, 0);   // Arret clignotement sortie au bout du temps Duree
    }
    else   // cycle continue
    {
        if (SortieTor[num].etat == 0)
            Turn_on_LED(num);  // active la sortie
        else
            Turn_off_LED(num);  // desactive la sortie

        if (SortieTor[num].duree) SortieTor[num].duree -= i1/100;

        CLOCK_RESTART_TIMEOUT(SortieTor[num].h_clock, i1);  // millisecondes
    }
 }
 else code_erreur=depass_tab+4;//bit_set(erreur_sms,5);
return;
}


// Activation PWM
// param numero : 0:buzz, 1:light
// parametre Consigne : :0 eteint, 1:continu, 2:sonnerie rapide1/1, 3:sonnerie lent1/1, 4:bip rapide,5:2 tons, 6: 1 bip court espace, 7(2s-0,5s:Spot PWM) 11 a 19 : 1 a 9 flash
//      Nota : 11 Ã  19 : duree d'un cycle : ((3+1)*Nb bips+13) x100ms
// parametre duree : duree de fonctionnment du buzzer en 0,1 seconde (0 Ã  6553s - 2h)
// parametre periode : 2700=> 750hz   900=>2200Hz   500=>4000Hz
// parametre duty : permet de regler le volume 0 a 255 : optimum=20

void ACTIV_PWM( uint8_t num,uint8_t consigne, int unsigned duree, int unsigned periode1, uint8_t duty1)  //dï¿½finit les variables buzzer en fonction de 'consigne', duree, volume
{
    #if NB_PWM > 0
    if (num < NB_PWM)
    {
        Pwm[num].consigne = consigne;    // Nota : a la fin de cette routine, le buzzer s'allume ou s'eteint
        Pwm[num].nb_flash = consigne - 11;
        Pwm[num].etat = 0;      // 0:pas dï¿½marï¿½, 1:pause, 2:allume, 3:2ï¿½ ton
        Pwm[num].duree = duree;
        Pwm[num].periode1 = periode1;
        Pwm[num].duty1 = duty1;

        CLOCK_STOP(Pwm[num].h_clock);

        // Nota : le premier cycle comporte le duty precedent
        if ((consigne <2) && (duree)) //  0 ou 1 sans timer de chgt d'etat
        {
            if (!consigne)
                eteint_PWM(num);   // desactive la sortie
            else
            {
                allumedebut_PWM(num);   // active la sortie
                Pwm[num].etat = 2;      // 0:pas dï¿½marï¿½, 1:pause, 2:allume
            }
            CLOCK_RESTART_TIMEOUT(Pwm[num].h_clock, duree * 100);  // millisecondes
        }
        else  // consigne avec timer        // active la sortie
        {
            ETAT_PWM(num);
        }
    }
    #endif
}


void allumedebut_PWM(uint8_t num)
{
    #ifdef CC13xx
        PWM_start(Pwm[num].h_pwm);
        //PWM_setDutyAndPeriod (*Pwm[num].h_pwm, Pwm[num].duty1 * 0x1000000, Pwm[num].periode1);
        if (Pwm[num].duty1==255)
            PWM_setDuty(Pwm[num].h_pwm,  0xFFFFFFFF);
        else
            PWM_setDuty(Pwm[num].h_pwm, Pwm[num].duty1 * 0x1000000);

        PWM_setPeriod(Pwm[num].h_pwm, Pwm[num].periode1);
        Pwm[num].etat = 2;
    #endif
}

void allume_PWM(uint8_t num)
{
    #ifdef CC13xx
        PWM_start(Pwm[num].h_pwm);
    #endif
    Pwm[num].etat = 2;
}

void eteint_PWM(uint8_t num)  // stop timer 2
{
    #ifdef CC13xx
        PWM_stop(Pwm[num].h_pwm);
    #endif
    Pwm[num].etat= 1;
}

void ETAT_PWM(uint8_t num)
{
    uint8_t a;
    uint8_t cycle_fin=0;
    uint8_t cons = Pwm[num].consigne;

   if (cons > 1)
    {
        if (Pwm[num].etat != 2 )   //buzzer eteint => calcul du temps avant allumage
        {
            a = 0;
            if ((cons == 1) || (cons == 2)) a=2;  // 200ms
            if (cons == 3) a=15;                   // 1,5s
            if (cons == 4) a=1;                    // 100ms
            if (cons == 5) a = 10;                 // 1s
            if (cons == 6) a = 100;                // 10s
            if (cons == 7) a = 20;                 // 2s  pour Spot LED PWM
        }

        else    //buzzer sonne => calcul du temps avant extinction
        {
            switch (cons)
            {
                case 1 : {a = 2 ;break;}
                case 2 : {a = 2 ;break;}
                case 3 : {a = 15;break;}
                case 4 : {a = 4 ;break;}
                case 5 : {a = 15; break;}
                case 6 : {a = 2; break;}
                case 7 : {a = 5; break;}   // 0,5s  pour Spot LED PWM
                case 11 :
                case 12 :
                case 13 :
                case 14 :
                case 15 :
                case 16 :
                case 17 :
                case 18 :
                case 19 :
                        if (Pwm[num].nb_flash == 0)
                        {
                            a = 12;
                            Pwm[num].nb_flash = cons - 11;
                        }
                        else
                        {
                            a = 2;
                            Pwm[num].nb_flash --;
                        }
                        break;
                default : a = 0;
            }
        }
        if ( a >= Pwm[num].duree )
            cycle_fin=1;
   }
   else
   {
       cycle_fin=1;
   }

   if (cycle_fin)  // fin de cycle
   {
       eteint_PWM(num);  // stop timer
   }
   else   // le cycle continue
   {
       if (cons != 5)
       {
           if (Pwm[num].etat  < 2)
               if (Pwm[num].etat)
                   allume_PWM(num);  // pause => rallume
               else
                   allumedebut_PWM(num);  // pas demarre => demarre
           else
               eteint_PWM(num);  // stop timer
       }
       else  // pwm 2 tons
       {
           if ( Pwm[num].etat == 2 )  // passage en periode2
           {
               Pwm[num].etat = 3;
             #ifdef CC13xx
               PWM_setDuty(Pwm[num].h_pwm, Pwm[num].duty2 * 0x1000000);
               PWM_setPeriod(Pwm[num].h_pwm, Pwm[num].periode2);
             #endif
           }
           else  // passage en periode1
           {
               Pwm[num].etat = 2;
            #ifdef CC13xx
               PWM_setDuty(Pwm[num].h_pwm, Pwm[num].duty1 * 0x1000000);
               PWM_setPeriod(Pwm[num].h_pwm, Pwm[num].periode1);
            #endif
           }

       }
       Pwm[num].duree -= a;
       CLOCK_RESTART_TIMEOUT(Pwm[num].h_clock, a * 100);  // millisecondes
   }
return;
}

void init_PWM(void)
{
    #if NB_PWM >0

        PWM_Params params;
        PWM_init();
        PWM_Params_init(&params);
        params.idleLevel  = PWM_IDLE_LOW;

        // PWM in US with fractional duty cycle

        PWM_Params_init(&params);
        // dutyUnits : PWM_DUTY_US ou PWM_DUTY_FRACTION (fraction de 0x100000000)
        params.dutyUnits = PWM_DUTY_FRACTION; // PWM_DUTY_US;  //
        params.dutyValue =  10; // 1500; //PWM_DUTY_FRACTION_MAX/2;
        params.periodUnits = PWM_PERIOD_US;
        params.periodValue = 3000;
        // params.idleLevel = 0;  // par defaut

        Pwm[0].duree=0;
        Pwm[0].h_pwm = PWM_open(Board_PWM0, &params);
        if(Pwm[0].h_pwm == NULL ) {
            code_erreur = erreur_demar;
            err_donnee1 = '1';
        }

        #if (NB_PWM>1)
            Pwm[1].duree=0;
            Pwm[1].h_pwm = PWM_open(Board_PWM1, &params);
            if(Pwm[1].h_pwm == NULL ) {
                code_erreur = erreur_demar;
                err_donnee1 = '2';
            }
        #endif

    /*     pwmParams.periodUnits  = PWM_PERIOD_US;
        pwmParams.periodValue = 3000; //PERIOD_VALUE;              // in microseconds
        pwmParams.dutyUnits    = PWM_DUTY_FRACTION;
        pwmParams.dutyValue   = 1500; //PULSE1_VALUE;*/

        // PWM open  will set  pin to idle level
        //hPWM0 = PWM_open(Board_PWM0, &pwmParams);

        //Log_error0("error info");

    #endif

}

uint8_t lecture_etat_sortie(uint8_t num)
{
    return SortieTor[num].consigne;
}


void message_lecture_etat_sortie(uint8_t num)
{
    message[1] ='X';
    message[2] ='L';
    message[3] = num+'0';
    message[4] = ':';
    message[5] = SortieTor[num].consigne +'0';
    message[6] = '-';
    message[7] = SortieTor[num].duree/8 +'0';
    message[8] = car_fin_trame;
    envoi_message();
}


void hysteresis_entrees(void)
{
  uint8_t ent;

  for (ent=0; ent<NB_ENTREES; ent++)
  {
      if ((!h_etat_entree[ent].duree_activ_manu) && (h_etat_entree[ent].configure))
        {
         //  if (HAL_GPIO_ReadPin(h_etat_entree[ent].entree_port, h_etat_entree[ent].entree_pin))  // actif 1
           {
               if (h_etat_entree[ent].comptage != h_etat_entree[ent].max)
                 {
                     h_etat_entree[ent].comptage++;
                     if (h_etat_entree[ent].comptage == (h_etat_entree[ent].max / 2))
                         {
                           h_etat_entree[ent].comptage = h_etat_entree[ent].max;
                           h_etat_entree[ent].etat = 1;
                           h_etat_entree[ent].chgt_etat = 2;
                         }
                 }
           }
//           else                                                                           // inactif  0
           {
               if (h_etat_entree[ent].comptage != 0 )
                 {
                     h_etat_entree[ent].comptage--;
                     if (h_etat_entree[ent].comptage == (h_etat_entree[ent].max / 2))
                         {
                           h_etat_entree[ent].comptage = 0;
                           h_etat_entree[ent].etat = 0;
                           h_etat_entree[ent].chgt_etat = 1;
                         }
                 }
           }
        }
  }
}

void activation_entree (uint8_t a, uint8_t duree, uint8_t valeur)
{
  if ((valeur == 1) &&  (h_etat_entree[a].etat == 0))    // activation Ã  1
    {
           h_etat_entree[a].etat =1;
           h_etat_entree[a].chgt_etat=2;
           h_etat_entree[a].comptage = h_etat_entree[a].max;
           h_etat_entree[a].duree_activ_manu = duree*10;
           h_etat_entree[a].activ_manuelle=1;
    }
  if ((valeur == 0) &&  (h_etat_entree[a].etat == 1))    // activation Ã  0
    {
           h_etat_entree[a].etat =0;
           h_etat_entree[a].chgt_etat=1;
           h_etat_entree[a].comptage = 0;
           h_etat_entree[a].duree_activ_manu = duree*10;
           h_etat_entree[a].activ_manuelle=1;
    }
  if ((valeur == 2) )                                    // raz Ã  0
    {
           h_etat_entree[a].etat =0;
           h_etat_entree[a].chgt_etat=0;
           h_etat_entree[a].comptage = 0;
           h_etat_entree[a].duree_activ_manu = 0;
           h_etat_entree[a].activ_manuelle=0;
    }
  if ((valeur == 3) )                                    // raz Ã  1
    {
           h_etat_entree[a].etat =1;
           h_etat_entree[a].chgt_etat=0;
           h_etat_entree[a].comptage = h_etat_entree[a].max;
           h_etat_entree[a].duree_activ_manu = 0;
           h_etat_entree[a].activ_manuelle=0;
    }
}

void dec_activation_entree_manuelle (void)
{
  uint8_t ent;

  for (ent=0; ent<NB_ENTREES; ent++)
  {
    if (h_etat_entree[ent].duree_activ_manu)
    {
/*        if (!h_etat_entree[ent].activ_manuelle)  // Si activation manuelle de l'entrÃ©e Ã  0
         {
            if (h_etat_entree[ent].comptage >= (h_etat_entree[ent].max / 2) )
               if (h_etat_entree[ent].chgt_etat==2) h_etat_entree[ent].chgt_etat=0; else h_etat_entree[ent].chgt_etat = 1;
            h_etat_entree[ent].comptage = 0;
            h_etat_entree[ent].chgt_etat = 0;
         }
        else                                         // Si activation manuelle de l'entrÃ©e Ã  1
        {
            if (h_etat_entree[ent].comptage <= (h_etat_entree[ent].max / 2) )
               if (h_etat_entree[ent].chgt_etat==1) h_etat_entree[ent].chgt_etat=0; else h_etat_entree[ent].chgt_etat = 2;
            h_etat_entree[ent].comptage = h_etat_entree[ent].max;
            h_etat_entree[ent].chgt_etat = 1;
        }*/
        h_etat_entree[ent].duree_activ_manu--;
        if (!h_etat_entree[ent].duree_activ_manu) h_etat_entree[ent].activ_manuelle=0;
    }
  }
}


// affiche un double, avec x chifres arpÃ¨s la virgule et envoi
/*void envoi_2d_float_message (double valeur, uint8_t nb_decimale, uint8_t num)
{
    num = ajout_2d_float_message (valeur, nb_decimale, num);
    message[num] = car_fin_trame;
    envoi_message();
}*/


// affiche un double, avec x chifres arpÃ¨s la virule
/*uint8_t ajout_2d_float_message (float valeur, uint8_t nb_decimale, uint8_t num)
{  // affiche un moins pour les chiffres nÃ©gatifs

    uint16_t entier;
    uint8_t decimal, dixieme;

    if (valeur <0)
    {
        valeur = -valeur;
        message[num++] = '-';
    }
    entier = (uint16_t) valeur;
    if (entier/100) message[num++] = entier / 100 + '0';
    dixieme = (entier / 10) % 10;
    if ((entier/100) || dixieme) message[num++] = dixieme + '0';
    message[num++] = entier % 10 + '0';
    if (nb_decimale==2) {
        message[num++] = ',';
        decimal = (char unsigned) ((valeur - entier) * 100 );
        message[num++] = decimal / 10 + '0';
        message[num++] = decimal % 10 + '0'; }
    if (nb_decimale==1) {
        message[num++] = ',';
        decimal = (char unsigned) ((valeur - entier) * 10 );
        message[num++] = decimal + '0';  }
    return num;
}*/


// Decodage dans la chaine message_in d'un nombre decimal,
// Paramï¿½tres :
// nombre retournï¿½ : uint16
// digit : nombre de digit ï¿½ lire
// position : position de dï¿½part de lecture dans la chaine message_in
uint16_t  decodage16_message_in (uint8_t nombre, uint8_t position)
{
    uint8_t i;
    uint16_t valeur_lue;

    valeur_lue=0;
    for (i=0; i<nombre; i++)
    {
        valeur_lue *= 10;
        valeur_lue += message_in[position+i]-'0' ;
        if (message_in[position+i+1] == car_fin_trame) break; // fin
    }
    return valeur_lue;
}


// Decodage dans la chaine message_in d'un nombre decimal,
// Paramètres :
// nombre retourné : int16
// digit : nombre de digit à lire
// position : position de départ de lectuer dans la chaine message_in
int16_t  decodage16i_message_in (uint8_t nombre, uint8_t position)
{
    uint8_t i,neg;
    int16_t valeur_lue;
    neg=0;
    i=0;
    if (message_in[position] == '-')  // chiffre negatif
    {
        neg=1;
        i=1;
    }
    valeur_lue=0;
    while (i<(nombre+neg))
    {
        valeur_lue *= 10;
        valeur_lue += message_in[position+i]-'0' ;
        i++;
        if ((message_in[position+i] == car_fin_trame) || (!message_in[position+i]))
            i=nombre+neg;
    }

    if (neg) valeur_lue = -valeur_lue;
    return valeur_lue;
}

// affiche un int32_t (jusqu'à 100.000) - format fixe
uint8_t ajout_int32_f_message (int32_t valeur,  uint8_t position)
{  // affiche un moins pour les chiffres nÃ©gatifs

    uint8_t dixmille, mille, cent, dix;

    if (valeur <0)
    {
        valeur = -valeur;
        message[position++] = '-';
    }
    else  position++;

    dixmille = (uint8_t) (valeur / 10000);
    mille = (uint8_t) (valeur / 1000 - dixmille*10);
    cent = (uint8_t) (valeur/100 - mille*10 - dixmille*100);
    dix = (uint8_t) (valeur%100 - valeur%10 );

    if (dixmille) message[position++] = dixmille + '0';
    else  position++;
    if (mille || dixmille) message[position++] = mille + '0';
    else  position++;
    if (dixmille || mille || cent) message[position++] = cent + '0';
    else  position++;
    message[position++] = dix + '0';
    message[position++] = valeur % 10 + '0';
    return position;
}

// affiche un float, renvoie l'index
// param: 0:espace  1:zéro  2:variable
// param bit fort : 0:fin_trame
uint8_t ajout_float_message (float valeur, uint8_t nb_decimale, uint8_t pos, uint8_t param)
{  // affiche un moins pour les chiffres nÃ©gatifs

    int32_t entier;
    uint8_t dixmille, mille, cent, dix, fixe;

    fixe = param & 0x7F;
    if (valeur <0)
    {
        valeur = -valeur;
        message[pos++] = '-';
    }
    else
        if (fixe!=2)
            message[pos++] = ' ';

    entier = (int32_t) valeur;
    dixmille = (uint8_t) (entier / 10000);
    mille = (uint8_t) (entier/1000) %10;
    cent = (uint8_t) ((entier/100) %10);
    dix = (uint8_t) ((entier/10) % 10 );

    if ((fixe==1) || (dixmille)) message[pos++] = dixmille + '0'; // dixmilliers
    else  if (fixe!=2) message[pos++] =' ';

    if ((fixe==1) || (dixmille) || (mille)) message[pos++] = mille + '0'; // milliers
    else  if (fixe!=2) message[pos++] =' ';

    if ((fixe==1) || (dixmille) || (mille) || (cent)) message[pos++] = cent + '0';
    else  if (fixe!=2) message[pos++] =' ';

    if ((fixe==1) || (dixmille) || (mille) || (cent) || (dix)) message[pos++] = dix + '0';
    else  if (fixe!=2) message[pos++] =' ';

    message[pos++] = entier % 10 + '0';

    dix = (char unsigned) ((valeur - entier) * 10 );  //  ex:3,453 => 4
    cent = ((char unsigned) ((valeur - entier) * 100 ) ) % 10;
    //cent = (char unsigned) (cent - ((uint8_t)cent/100)*100 ); // ex:53
    mille = ((char unsigned) ((valeur - entier) * 1000 ) ) % 10;

    if (nb_decimale) {
        message[pos++] = '.';
        message[pos++] = dix + '0';

        if (nb_decimale==3) {
            message[pos++] = cent + '0';
            message[pos++] = mille + '0'; }
        if (nb_decimale==2) {
            message[pos++] = cent + '0'; }
    }

    if (!(param & 0x80)) message[pos++] = car_fin_trame;
    return pos;
}

// affiche un int16, renvoie l'index
// fixe: 0:espace  1:zéro  2:variable  +car_fin  0x80:pas de car_fin
uint8_t ajout_int16_message (int16_t valeur, uint8_t pos, uint8_t param)
{  // affiche un moins pour les chiffres negatifs

    uint8_t dixmille, mille, cent, dix, fixe;

    fixe = param & 0x7F;
    if (valeur <0)
    {
        valeur = -valeur;
        message[pos++] = '-';
    }
    else
        if (fixe!=2)
            message[pos++] = ' ';

    dixmille = (uint8_t) (valeur / 10000);
    mille = (uint8_t) (valeur/1000) %10;
    cent = (uint8_t) ((valeur/100) %10);
    dix = (uint8_t) ((valeur/10) % 10 );

    if ((fixe==1) || (dixmille)) message[pos++] = dixmille + '0'; // dixmilliers
    else  if (fixe!=2) message[pos++] =' ';

    if ((fixe==1) || (mille) || (dixmille)) message[pos++] = mille + '0'; // milliers
    else  if (fixe!=2) message[pos++] =' ';

    if ((fixe==1) || (cent) || (mille) || (dixmille)) message[pos++] = cent + '0';
    else  if (fixe!=2) message[pos++] =' ';

    if ((fixe==1) || (cent) || (mille) || (dixmille) || (dix)) message[pos++] = dix + '0';
    else  if (fixe!=2) message[pos++] =' ';

    message[pos++] = valeur % 10 + '0';

    if (!(param & 0x80)) message[pos++] = car_fin_trame;
    return pos;
}

// affiche un int8, renvoie l'index
// fixe: 0:espace  1:zéro  2:variable   +car_fin  0x80:pas de car_fin
uint8_t ajout_int8_message (int8_t valeur, uint8_t pos, uint8_t param)
{  // affiche un moins pour les chiffres nÃ©gatifs, juste avant le chiffre

    uint8_t cent, dix, premier, signe, fixe;

    fixe = param & 0x7F;
    premier=0;
    if (valeur <0)
    {
        signe='-';
       valeur =  - valeur;
    }
    else signe=' ';

    cent  = (uint8_t) ((valeur/100) %10);
    dix   = (uint8_t) ((valeur/10) % 10 );

    if ((fixe==1) || (cent))
    {
        if ((fixe !=2) || (signe=='-'))
            message[pos++] = signe;
        message[pos++] = cent + '0';
        premier=1;
    }
    else
        if (fixe!=2)
            message[pos++] =' ';

    if ((fixe==1) || (cent) || (dix))
    {
        if (!premier)
        {
            premier=1;
            message[pos++] = signe;
        }
        message[pos++] = dix + '0';
    }
    else
        if (fixe!=2)
            message[pos++] =' ';

    if (!premier)
    {
        if ((fixe !=2) || (signe=='-'))   // fixe=0ou1->signe fixe=2->signe moins
            message[pos++] = signe;
    }
    message[pos++] = valeur % 10 + '0';

    if (!(param & 0x80))
        message[pos++] = car_fin_trame;
    return pos;
}

// ajoute 3 ou 4 valeurs float dans le message
// fixe: 0:espace  1:zéro  2:variable
uint8_t ajout_float_multi_valeurs(float * valeur, uint8_t nb_decimale, uint8_t pos, uint8_t nb, uint8_t fixe)
{
    uint8_t i, index;

    index=pos;
    if (pos < MESSAGE_SIZE - nb*7)
    {
        for (i=0; i<nb; i++)
        {
            index = ajout_float_message(valeur[i], nb_decimale, index, fixe);
            index --;
            message[index++]=' ';
        }
        message[index] = car_fin_trame;
    }
    return index;
}

// fixe: 0:espace  1:zéro  2:variable
uint8_t ajout_int16_multi_valeurs(int16_t * valeur, uint8_t pos, uint8_t nb, uint8_t fixe)
{
    uint8_t i, index;
    index=0;
    if (pos < MESSAGE_SIZE - nb*5)
    {
        index=pos;
        for (i=0; i<nb; i++)
        {
            index = ajout_int16_message(valeur[i], index, fixe);
            index --;
            message[index++]=' ';
        }
        if (!(fixe & 0x80)) message[index] = car_fin_trame;
    }
    return index;
}


void envoi_int_decimal (uint16_t val, uint8_t position) // envoie 1 int en decimal 0 a  65534
{
  uint16_t a,b,c,d,e;

  if (position < MESSAGE_SIZE -7)
  {
      a = val / 10000;
      b = val / 1000 - a*10;
      c = val / 100 - b*10 - a*100;
      d = val / 10 - c*10 - b*100 - a *1000;
      e = val - d*10 - c*100 -b*1000 - a *10000;
      if (a) message[position] = a +'0';
      position++;
      if (a+b) message[position] = b +'0';
      position++;
      if (a+b+c) message[position] = c +'0';
      position++;
      if (a+b+c+d) message[position] = d +'0';
      position++;
      message[position++] = e + '0';
  }
}

void envoi_char_decimal (uint8_t val, uint8_t position) // envoie 1 char en decimal 0 a 255
{
    uint8_t b, c, d;
  if (position < MESSAGE_SIZE - 5)
  {
      b = val / 100;
      c = val / 10 - b * 10;
      d = val - c * 10 - b * 100;
      if (b)
        message[position] = b + '0';
      position++;
      if (b + c)
        message[position] = c + '0';
      position++;
      message[position++] = d + '0';
  }
}

// comparaison de 2 chaines de caractÃ¨re
// return : 1 identique (ou chaine 2 inclus dans chaine1), 0:diffÃ©rent

uint8_t comp_message (uint8_t * mess1, uint8_t * mess2)
{
    uint8_t i,j;
    j=1;
    for (i=0;i<250;i++)
      {
        if (!*mess2)
            break;
        else
            if (*mess1 != *mess2) j=0;
      }
    return j;
}

//*****************************************************************************
//
// SetupSetAonRtcSubSecInc
//
//*****************************************************************************
#ifdef CC13xx
void SetupSetAonRtcSubSecIncOK( uint32_t subSecInc )
{
   // Loading a new RTCSUBSECINC value is done in 5 steps:
   // 1. Write bit[15:0] of new SUBSECINC value to AUX_WUC_O_RTCSUBSECINC0
   // 2. Write bit[23:16] of new SUBSECINC value to AUX_WUC_O_RTCSUBSECINC1
   // 3. Set AUX_WUC_RTCSUBSECINCCTL_UPD_REQ
   // 4. Wait for AUX_WUC_RTCSUBSECINCCTL_UPD_ACK
   // 5. Clear AUX_WUC_RTCSUBSECINCCTL_UPD_REQ
   HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0 ) = (( subSecInc       ) & AUX_WUC_RTCSUBSECINC0_INC15_0_M  );
   HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1 ) = (( subSecInc >> 16 ) & AUX_WUC_RTCSUBSECINC1_INC23_16_M );

   HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL ) = AUX_WUC_RTCSUBSECINCCTL_UPD_REQ;
   while( ! ( HWREGBITW( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL, AUX_WUC_RTCSUBSECINCCTL_UPD_ACK_BITN )));
   HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL ) = 0;
}
#endif

// Transcodage d'une chaine en caractères décimaux vers une chaine hexa
void code_dec_hex (uint8_t* ad_entree, uint8_t* ad_sortie, uint8_t nb_octet_hex)
{
    uint8_t cpt;
    for (cpt=0; cpt<nb_octet_hex; cpt++)
    {
        *ad_sortie = ((hexa(*ad_entree))<<4) + hexa(*(ad_entree+1));
        ad_sortie++;
        ad_entree = ad_entree+2;
    }
}

// LEcture - Transcodage d'une chaine en caractères hexa vers une chaine decimaux
// type : bit01:00 sans espace 01:espace à chaque octet    bit2:0:normal  1:inversé
void code_hex_dec (uint8_t* ad_entree, uint8_t* ad_sortie, uint8_t nb_octet_hex, uint8_t type)
{
    uint8_t cpt;
//    uint8_t* l_entree;
    for (cpt=0; cpt<nb_octet_hex; cpt++)
    {
/*        l_entree = ad_entree;
        if (type &4)  // inversion
        {
            if (ad_entree%4==0)  l_entree=l_entree+1;
        }*/
        *ad_sortie++ = deci((*ad_entree) >>4);
        *ad_sortie++ = deci((*ad_entree)&0xF);
        ad_entree++;
        if ( ((type&0b11)==0b01) || (((type&0b11)==0b10) && ((cpt)&1)) || (((type&0b11)==0b11) && !((cpt+1)&3)) )  // rajout d'un espace à chaque octet
            *ad_sortie++ = ' ';
    }
}
