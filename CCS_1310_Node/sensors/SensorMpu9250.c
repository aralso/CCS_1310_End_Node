/** ============================================================================
 *  @file       SensorMpu9250.c
 *
 *  @brief      Driver for the InvenSense MPU9250 Motion Processing Unit.
 *  ============================================================================

ALGORITHMES CAPTEUR MPU:
  1:energie vibratoire (fréquence d'acquisition 1kHz)
  2:chocs              (fréquence d'acquisition 1kHz)
  3:position 3D, angles, vecteur acceleration sans gravite (fréquence d'acquisition 50Hz)
  4:vibration temporel et FFT   (fréquence d'acquisition 1kHz)


VECTEUR_ORIENTATION, VIBRATIONS et CHOCS
Bouton0 court  : mode eteint -> actif
Bouton0 long   : mode hexa -> mode texte
Bouton0 double : chgt mode : liaison série -> envoi RF -> enregistrement Flash
Bouton1 court  : mode 1 : Choc -> E.vib+choc -> E.vib+choc+accel ->  id + quaternion
Bouton1 long   : mode 2 : 0:quaternion  1:angles  2:gyro  3:accelero  4:magneto  5:val brutes accel 6:energ_vib  7:chocs
Bouton1 double : envoi balise de demande de messages en attente
2 boutons      : Raz index flash (3 bips)

Calibration accelero : Ecart type + 3 valeurs biais accelero (divide by 8 to get 2048 LSB per g)
Calibration magneto  : 1ere ligne : 3 min et 3 max  (axe X:bouton  Y:cable   Z:partie plate)
Calibration gyro     : Ecart type + 3 valeurs biais gyro (Divide by 4 to get 32.9 LSB per deg/s)


0:Quaternion : 4 valeurs q0,q1,q2,q3 normés (multiplié par 1000)
1:Angles : angles X, Y, Z en degrés
2:Gyroscopes : vitesse de rotation en rad/s sur chaque axe
3:Accelero : valeurs brutes X,Y,Z en milli-g   puis sans gravité : X,Y,Z  en milli g
4:Magnetometre : valeurs en milli-Gauss (diviser par 10 pour uTesla)
5:Enveloppe accel X (accel/freinage sur axe X) : amplitude moyenne en milli-g, durée en ms
6:Energie Vibratoire : Energie vibratoire sur 3 axes, en milli-g
7:Chocs : Axe principal, Valeur efficace sur cet axe, durée en ms
8:valeurs brutes
9:Energie vib + chocs + accel
10:mode 9 + quaternion

Mode 9 et 10:
Energie vib :V + Sat + 3 float
Chocs :      C + Sat + Axe + 2 float
Accel :      A + 3 float
quaternion : Q + 4 valeurs float

Vibration : statut, Axe, valeur efficace(mg), ratio Eff, ratio Kurt, Ecart type Ratio Kurt

Messages :
 *  SCMCALM : calib magneto
 *  SLCECA21 : activ capteur
 *  SLCECP205s : 5 s  SCECP130m   SRCECP201d
 *  SCLF1 : affiche les données enreg en flash
 *  SCMEAZx : 0 ou 1 pour enregistrement des données brutes accelero
 *  SCMDLPx  : filtre passe-bas de l'accelerometre
 *
Mode connexion : USB(avec alim) : Comm RF ou Uart
                 ou sur pile    : Comm RF uniquement


 -----------------------------------------------------------------------------
*  Init functions
* ------------------------------------------------------------------------------

setup_mpu : initialise MPU
sensorMpuSleep : 6 axes désactivés et sleep mode
Mpu9250WakeUp : Exit Low power mode - restore accRange,  and restore config

9250Init   : Open PIN, Register interrupt, Reset
9250Reset  : désactive interrupt, reset, test, AccSetRange(8G), MagInit, Sleep
9250Test   :test Who_am_I

powerOnCheck : check puis active (et reset)
PowerOff : désactive PIN power, désactive interruption, active I2C bus 0
PowerIsOn : indique position de la PIN power
Enable  : wakeUp, selectAxes, sinon Sleep
Sleep : Low power mode (4F en Pwr_mgt1)

* -----------------------------------------------------------------------------
*  Utility functions
* ------------------------------------------------------------------------------

Fonctions Accelero :
enableWom : enable accel & interrupt, disable gyro
irqStatus : check interrupt (data or Wom)
AccSetRange  : newRange (rangeReg) pour accelero (2G, 4G,..)
AccReadRange : Lit range (2G, 4G,..)
SelectAxes  : set acc/gyro/mag axes (mpuConfig)

Fonctions magnetometre :
SetByPass : allow I2C bus to control magnetometer
MagInit   : bypass, Lit Fuse calib (mag_cal), ajoute 128, turn off par mag_reset
MagReset  : bypass, reset, réactive si mpuConfig, scale & mode
MagEnable : bypass, enable (& scale/mode)or disable
MagTest   : bypass, who_am_I


* -----------------------------------------------------------------------------
*  Read  functions  -  1500
* ------------------------------------------------------------------------------

readTempData : Lecture température
AccRead   : lecture accelero  - data brute X-Y-Z
Acc_Read  : lecture accelero  - data nette X-Y-Z
AccConvert  : /32k*range

GyroRead  : lecture gyro      - data brute
Gyro_Read : lecture gyro      - data nette
GyroConvert : / 65k*500

MagRead   : lecture en mode continu
Mag_Read   : bypass, si data ready : raw * calib / 256, relance une conversion



* -----------------------------------------------------------------------------
*  Calibration
* ------------------------------------------------------------------------------

calibration_accel : calib accel et gyro et affichage resultats
calibration_gyro  : calib gyro et affichage resultats
calibrate_gyro_MPU9250 : calib gyro
calibration_magne : calib mag et affichage
magcalMPU9250 : calibration magneto

* -----------------------------------------------------------------------------
*  Vibrations et FFT functions
* ------------------------------------------------------------------------------

lecture_vibrations   : lit accelero pendant 80ms et calcul valeur efficace des vibrations/secousses
lecture_choc         : lit accelero pendant 500ms et reconnait le pattern du plus fort choc
lecture_accelero     : lit accelero pendant 80ms et calcul valeur crete
lecture_multi_fft    : lit plusieurs périodes de 1s , fait FFT, fait la moyenne

lecture_fft_accelero : lit accelero pdt 1s, puis calcule FFT

* -----------------------------------------------------------------------------
*  3D, Orientation et Quaternion
* ------------------------------------------------------------------------------

calcule_quaternion : lecture 9 axes, puis calcule Quaternion
calcule_angle :  Calcule angles à partir du quaternion
remove_gravity : supprime la gravité aux accelero, à l'aide du quaternion
MadgwickQuaternionUpdate : algorithme fusion quaternion
MahonyQuaternionUpdate : algorithme fusion quaternion

Conso :
 MPU acccel data : 450uA
 250Hz FIFO acquis : 600uA (4ms pour 20 sample) à 1500uA (1khz)
 250Hz traintement choc-vib : 200uA
 gyro : 1500uA(LP) à 3200uA
 fusion : 300uA (80ms) 1500uA (20ms)
 TX : 320uA  (13mA pdt 5ms)
*/

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <math.h>

#include "Board.h"

#include "SensorMpu9250.h"
#include "SensorOpt3001.h" // For reset of I2C bus
#include "SensorUtil.h"
#include "SensorI2C.h"

#include "utilitaires.h"
#include "comm_message.h"
#include "radioProtocol.h"

#include "FFT/DSPLib.h"   // FFT


// ajouter
#include "utilitaires.h"
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Seconds.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

#include "DmRadioTask.h"


/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/

//#define MODE_SILENCE

//#define MODE_ACTIF           0   // 0:stop  1:veille 4Hz  2:veille 30Hz  3:veille 500Hz  4:actif

// 0:quaternion  1:angles  2:gyro  3:accelero  4:magneto  5:val brutes accel 6:energ_vib  7:chocs
// 10:choc  11:choc+E.vib   12:choc+E.vib+accel  13:idem=quaternion   14:rien
#define CONFIG_MPU_INITIALE  11

// Acquisition des donnees d'accelero , pour debug
//#define DEBUG_ACC 0b1 // bit0:choc   bit1:vib   bit2:acc_lin   bit3:FFT

// Algorithmes ------------------------------------------------------
#define Nb_algorithmes 4

#define PERIOD_MESURE 600  // en milli-secondes    max PERIOD_MESURE/ FREQ_ECHAN < 75
#define FREQ_ECHAN     20 //  1:1khz  2:500Hz  4:250Hz  8:125Hz  10:100Hz  20:50Hz 100:10Hz

#define ENREG_ACC_Z  0   // 1:envoi accelero 1 axe z

#define DLPF_ACCEL   3   // filtre passe bas de l'accelerometre 1:192Hz  2:92Hz  3:48Hz  4:24Hz  5:10Hz  6:5Hz

// --------------Algorithme 1 : Energie vibratoire
#define DUREE_ALGO1  5    // a multiplier par PERIOD_MESURE : 5=500ms
#define SEUIL_VIB_A1  80    // seuil en milli-g a partir duquel on envoie les valeurs

// --------------Algorithme 2 : Chocs

// 1 choc est caractérisé par une forte acceleration suivi immédiatement d'une forte deceleration
// => Reconnaissance du pattern suivant : dérive d'accéleration importante (caractérisé par la pente et la durée)
// suivie rapidement (max 5 ms) par une dérive d'acceleration importante opposée.
// la fonction identifie le choc le plus important détecté durant les 500ms

// Paramètre d'entree de la fonction :
//    seuil_pente_min : seuil d'acroissement min de l'acceleration    typ:25 milli-g / milli-seconde)
//    amplitude_min   : amplitude min de l'acceleration du choc  en milli-g   typ:500mg
//    energie_min    : Energie (amplitude*duree/2) min du choc en mg.ms         typ:5000mg.ms
// Données de sortie : axe du choc (0,1,2), amplitude (en milli-g), duree (en milli-s), saturation (0 ou 1, si les valeurs mesurées sont proches du max admissible)

#define DUREE_ALGO2     5         // a multiplier par PERIOD_MESURE : 5=500ms
#define CHOC_PENTE_MIN   30         // 50 ok Seuil_pente_min en milli-g / milli-s
#define CHOC_AMPLITUDE_MIN    300   // 500 Amplitude choc min en milli-g
#define CHOC_ENERGIE_MIN      2500  // 2500 Energie min du choc en mg.ms (amplitude*duree/2)


// --------------Algorithme 3 : 3D, quaternion, acceleration X
// Le quaternion est calculé toutes les 20 mili-secondes.
// Ce quaternion permet de déterminer le vecteur gravité, qui est soustrait au vecteur accélération pour
// déterminer l'acceleration sans gravité
// Les angles d'Euler sont directement déduits du Quaternion


#define PERIOD_ALGO3        1   // Period  à multiplier par PERIOD_MESURE
#define PERIOD_ENVOI_QUAT   2  // a multiplier par period_algo3   - périodicite d'envoi du quaternion

#define DECLENCH_DEBUT 20  // seuil declenchement début calcul enveloppe accelero X
#define DECLENCH_FIN   15  // seuil fin
#define DUREE_ACCELERATION   1  // Duree min (unit 20 milli-seconde) pour detection acceleration
#define QUAT_MOUVEMENT  0  // 0:affichage continu   1:affichage uniquement si mouvement
#define SEUIL_AFFICH_MOUV  0.01  // Seuil a partir duquel le quaternion est affiche/envoye


// --------------Algorithme 4 : Vibrations périodiques et FFT
#define ACTIV_ALGO4      1    // Activation algo 4 (vibrations)
#define PERIOD_ALGO4_H   0    // en heures.           0 = 1 seule mesure à l'activation
#define PERIOD_ALGO4_D   1    // unité 15 secondes .  0 = 1 seule mesure à l'activation
#define AXE_MESURE       0    // Axe de mesure : 0:Axe le plus fort, 1:X,  2:Y,  3:Z
#define SEUIL_ALGO4      3    // Seuil de detection de la valeur efficace (milli-g)


// Gestion mode veille MPU
#define DUREE_ACTIF              10    // 0:pas de mise en veille - Unite secondes - Duree a partir de laquelle le capteur se met en veille s'il n'y a pas de mouvement
#define SEUIL_WOM                3      // unite:4mg , max 1020mg - Amplitude min en dessous de laquelle le capteur se met en veille
#define DUREE_VEILLE3           20      // Unite 30 secondes (max:9999)     - Duree en mode 1 (freq 4Hz, conso 13uA)
#define DUREE_VEILLE2           0      // Unité 1 seconde (max:9999)   - Duree en mode 2 (freq 30Hz, conso 30uA)
#define DUREE_VEILLE1           10      // Unité 1 seconde (max:9999)   - Duree en mode 3 (freq 500Hz, conso 200uA)

//Conso : actif : mode 10:1,8mA  mode11:2,4mA   mode12:8,7mA   mode13:10mA
     //   veille3:280uA   veille2:30uA   veille1:13uA   stop:2,7uA

//#define Board_MPU_INT    CC1350STK_MPU_INT
//#define Board_MPU_POWER  CC1350STK_MPU_POWER


// Evenements du Noyau Temps reel
#define SENSOR_EVENT_ALL                  0xFFFFFFFF
#define EVENT_INTERRUPT_MPU           (uint32_t)(1 << 0)  // 1
#define SENSOR_EVENT_READ_SENSOR      (uint32_t)(1 << 1)  // 2
#define SENSOR_EVENT_CHGT_MODE        (uint32_t)(1 << 2)  // 4
#define SENSOR_TRAITEMENT_MESSAGE     (uint32_t)(1 << 3)  // 8
#define SENSOR_EVENT_ALGO4            (uint32_t)(1 << 4)  // 0x10
#define EVENT_ACTIV_MPU               (uint32_t)(1 << 5)  // 0x20
#define EVENT_DESACTIV_MPU            (uint32_t)(1 << 6)  // 0x40


// Registers
#define SELF_TEST_X_GYRO              0x00 // R/W
#define SELF_TEST_Y_GYRO              0x01 // R/W
#define SELF_TEST_Z_GYRO              0x02 // R/W
#define SELF_TEST_X_ACCEL             0x0D // R/W
#define SELF_TEST_Y_ACCEL             0x0E // R/W
#define SELF_TEST_Z_ACCEL             0x0F // R/W

#define XG_OFFSET_H                   0x13 // R/W
#define XG_OFFSET_L                   0x14 // R/W
#define YG_OFFSET_H                   0x15 // R/W
#define YG_OFFSET_L                   0x16 // R/W
#define ZG_OFFSET_H                   0x17 // R/W
#define ZG_OFFSET_L                   0x18 // R/W

#define SMPLRT_DIV                    0x19 // R/W
#define CONFIG                        0x1A // R/W
#define GYRO_CONFIG                   0x1B // R/W
#define ACCEL_CONFIG                  0x1C // R/W
#define ACCEL_CONFIG_2                0x1D // R/W
#define LP_ACCEL_ODR                  0x1E // R/W
#define WOM_THR                       0x1F // R/W
#define FIFO_EN                       0x23 // R/W

// .. registers 0x24 - 0x36 are not applicable to the SensorTag HW configuration

#define INT_PIN_CFG                   0x37 // R/W  55
#define INT_ENABLE                    0x38 // R/W
#define INT_STATUS                    0x3A // R
#define ACCEL_XOUT_H                  0x3B // R
#define ACCEL_XOUT_L                  0x3C // R
#define ACCEL_YOUT_H                  0x3D // R
#define ACCEL_YOUT_L                  0x3E // R
#define ACCEL_ZOUT_H                  0x3F // R
#define ACCEL_ZOUT_L                  0x40 // R
#define TEMP_OUT_H                    0x41 // R
#define TEMP_OUT_L                    0x42 // R
#define GYRO_XOUT_H                   0x43 // R
#define GYRO_XOUT_L                   0x44 // R
#define GYRO_YOUT_H                   0x45 // R
#define GYRO_YOUT_L                   0x46 // R
#define GYRO_ZOUT_H                   0x47 // R
#define GYRO_ZOUT_L                   0x48 // R

// .. registers 0x49 - 0x60 are not applicable to the SensorTag HW configuration
// .. registers 0x63 - 0x67 are not applicable to the SensorTag HW configuration

#define SIGNAL_PATH_RESET             0x68 // R/W
#define ACCEL_INTEL_CTRL              0x69 // R/W
#define USER_CTRL                     0x6A // R/W
#define PWR_MGMT_1                    0x6B // R/W
#define PWR_MGMT_2                    0x6C // R/W
#define FIFO_COUNT_H                  0x72 // R/W
#define FIFO_COUNT_L                  0x73 // R/W
#define FIFO_R_W                      0x74 // R/W
#define WHO_AM_I                      0x75 // R/W
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07

// Values PWR_MGMT_1
#define MPU_SLEEP                     0x4F  // Sleep + stop all clocks
#define MPU_WAKE_UP                   0x09  // Disable temp. + intern osc


// Data sizes
#define DATA_SIZE                     6

// Output data rates
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255

// Bit values
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN                  0x20
#define BIT_ACTL                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN                 0x02
#define BIT_AUX_IF_EN                 0x20

// Magnetometer registers
#define MAG_WHO_AM_I                  0x00  // Should return 0x48
#define MAG_INFO                      0x01
#define MAG_ST1                       0x02  // Data ready status: bit 0
#define MAG_XOUT_L                    0x03  // Data array
#define MAG_XOUT_H                    0x04
#define MAG_YOUT_L                    0x05
#define MAG_YOUT_H                    0x06
#define MAG_ZOUT_L                    0x07
#define MAG_ZOUT_H                    0x08
#define MAG_ST2                       0x09  // Overflow(bit 3), read err(bit 2)
#define MAG_CNTL1                     0x0A  // Mode bits 3:0, resolution bit 4
#define MAG_CNTL2                     0x0B  // System reset, bit 0
#define MAG_ASTC                      0x0C  // Self test control
#define MAG_I2CDIS                    0x0F  // I2C disable
#define MAG_ASAX                      0x10  // x-axis sensitivity adjustment
#define MAG_ASAY                      0x11  // y-axis sensitivity adjustment
#define MAG_ASAZ                      0x12  // z-axis sensitivity adjustment

#define MAG_DEVICE_ID                 0x48


// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_1,Board_MPU9250_ADDR)
#define SENSOR_SELECT_MAG()           SensorI2C_select(SENSOR_I2C_1,Board_MPU9250_MAG_ADDR)
#define SENSOR_DESELECT()             SensorI2C_deselect()

#define FS   0    // Full scale range

#define BV(n)               (1 << (n))

#ifdef CAPTEUR_MPU

// ------- Codes_erreur
#define erreur_mpu_init     0x60
// Codes erreur
// Erreurs 0 a  1F : envoyÃ©es une seule fois (appli >0x10)
// Erreurs 20 a  7F : 4 fois        (Appli >0x50)
// Erreurs 80 a  FF : tout le temps (Appli > 0xA0)



/******************** Type declarations ******************/

#ifdef QUATER_CHOCS
// Variable permettant le calcul de l'enveloppe de l'acceleration sur l'axe X
typedef struct accel {
    uint8_t      declench;         // declenchement de l'enveloppe 1:acc, 2:freinage
    int16_t   compteur;         // compteur de mesures prises depuis le début (à 13Hz)
    float     somme;            // somme des accel
    float       max;
} accel;
#endif

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/

// Variables algorithmes

config_mpu_params mpu_params;

uint8_t config_mpu;  // 0:quaternion  1:angles  2:gyro  3:accelero  4:magneto  5:enveloppe accel 6:energ_vib  7:chocs  8:val brutes 9:choc+vib
uint8_t algo_mpu;    // bit0:energie  bit1:chocs  bit2:quaternion   bit3:vib & FFT
uint8_t capteur_mpu_init;

uint8_t mode_actif;
uint8_t mode_veille_long_cpt;

uint8_t Alim_pile=1;

// variables veille
uint8_t extinction_mpu;
uint16_t extinction_mpu_cpt;

uint8_t cpt_algo_mpu[Nb_algorithmes];
uint8_t cpt_envoi_quat;
uint8_t count_FIFO;
int16_t ac_data[PERIOD_MESURE / FREQ_ECHAN + 6][3];
double ac_somme_a1[3], ac_carre_a1[3];
accel accel_X;

uint16_t cpt_envoi_balise;
float XC_1[3], XC_2[3];  // pour filtre passe-bas
float YC_[3], YC_1[3], YC_2[3];

uint16_t compt_err_imu;

static uint8_t sel_axes, smplrt_divis;
uint8_t interrupt_actif; //1:Data_ready,   0x40:Motion,   autre:Fifo overflow or Fsync

float acc_v[3];
float mag_v[3];

// tableau pile ram Acc Z
uint8_t acc_z_ram[NB_DATA_ACC_Z*2+12];
uint8_t index_ram_acc_z;
uint8_t Enreg_acc_z;

// Algo 1
uint16_t Seuil_vib_a1;
uint8_t Duree_algo1;
uint16_t algo1_count_ac;
uint8_t satur_a1;

// Algo 2
uint8_t Duree_algo2;
uint16_t Choc_pente_min;
uint16_t Choc_amplitude_min, Choc_energie_min;
uint16_t algo2_count_ac;
uint8_t choc_axe;
int16_t choc_amplitude;
int16_t choc_duree;
uint8_t satur_a2;
uint8_t pente_nb1[3], pente_nb2[3], pente_etat[3];
uint8_t temps_attente[3];
int8_t pente_sens[3];
int32_t pente_valeur1[3], pente_valeur2[3];
uint16_t pente_id[3], pente_debut[3];
int16_t ac_precedent[3];
uint8_t init_algo2;

// Algo 3
uint8_t Duree_acceleration;
uint8_t Period_envoi_quat;
uint8_t Period_algo3, Declench_debut, Declench_fin;
uint8_t recalage_initial;
uint16_t timer_apres_debut;
uint8_t valeur_recalage;

// Algo 4 (vibrations)
uint8_t Axe_Mesure;
uint8_t Period_Algo4_H;   // 0 à 255 heures
uint8_t Period_Algo4_D;   // Unité : 15 secondes
uint8_t Period_Algo4_cpt_h;
uint8_t Seuil_Algo4;

// Mise en veille
uint8_t Seuil_Wom;
uint16_t Duree_Actif;
uint16_t Duree_veille1;
uint16_t Duree_veille2;
uint16_t Duree_veille3;

#endif

// Pins that are used by the MPU9250
static PIN_Config MpuPinTable[] =
{
    Board_MPU_INT    | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS | PIN_HYSTERESIS,
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    PIN_TERMINATE
};

static PIN_State pinGpioState;
static PIN_Handle hMpuPin;

// The application may register a callback to handle interrupts
static SensorMpu9250CallbackFn_t isrCallbackFn = NULL;


#ifdef CAPTEUR_MPU

static uint8_t magStatus;

uint8_t val_test;

#ifdef DEBUG_ACC
    #define TEST_NB 400
    int16_t test_acc[TEST_NB];
    uint16_t cpt_test_acc;
    uint8_t axe_test_acc;
    uint8_t trig_test_acc;
    uint8_t test_acc_fige;
#endif

uint16_t cpt_err_sens, cpt_valid;  // comptage des batch de data non pris en compte
uint8_t dlpf_gyro, acc_lp_odr;

uint8_t Dlpf_accel;

// VARIABLE QUATERNION & CAPTEUR
float mag_data[3];

//float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS; // Gyroscope range
uint8_t accRange;    // Accelerometre range


// Magnetometer control
uint8_t Mscale; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode;        // MAG_MODE_SINGLE, 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float mRes;      // scale resolutions per LSB for the sensors

// Magnetometer calibration
int16_t mag_bias[3];
float mag_scale[3]; // mag calibration and mag bias
static int16_t mag_cal[3];

// Accelero calibration
int16_t acc_bias[3];
float acc_scale[3];

// Accelero control
uint16_t aRes;
uint8_t acc_rapport;
float gRes;      // scale resolutions per LSB for the sensors
static uint8_t accRangeReg;  // variable temporaire pour lecture accRange

// Acelero calibration

int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius

// Gyroscope calibration
int16_t G_OFFSET[3];

// Quaternion compute
#define sampleFreq  512.0f      // sample frequency in Hz
#define betaDef     0.1f        // 2 * proportional gain

#define PI 3.1415926
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s) ou 6.0 deg/s
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
//volatile float beta = betaDef;                              // 2 * proportional gain (Kp)
float beta; // = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta; // = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.1f  // Ou plutot 0.0f

uint32_t  sumCount = 0; // used to control display output rate - Nombre total de calculs de Quaternion effectués

// Angle compute
float yaw, roll, pitch, angle[3] ;
uint32_t lastUpdate = 0; // used to calculate integration interval

float  ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float q_prec[4], acc_prec[3];  // quaternion precedent pour suivre les evolutions de mouvement

// Task, Event
#define SENSOR_TASK_STACK_SIZE  1900  // 800 + 1000
#define SENSOR_TASK_PRIORITY 5
static Task_Params sensorTaskParams;
Task_Struct sensorTask;    /* not static so you can see in ROV */
static uint8_t sensorTaskStack[SENSOR_TASK_STACK_SIZE];   //   STACK
Event_Struct sensorEvent;  /* not static so you can see in ROV */
static Event_Handle sensorEventHandle;

/* Clock for the tache Sensor0 : clock de base pour la mesure 20 ms */
Clock_Struct SensTimerClock;     /* not static so you can see in ROV */
static Clock_Handle SensTimerClockHandle;

/* Clock pour le changement de mode veille */
Clock_Struct SensChgtModeTimer;     /* not static so you can see in ROV */
static Clock_Handle SensChgtModeTimerHandle;

/* Clock pour la vibration Algo 4 : clock de base pour la mesure */
Clock_Struct Algo4Clock;     /* not static so you can see in ROV */
static Clock_Handle Algo4ClockHandle;


//  ---------  EXTERN VARIABLES  --------------

extern uint8_t mode_periodique, mode_principal;

extern uint8_t mode_affich_enreg;
extern uint8_t mode_hexa_texte;

extern uint8_t i2c_valide;
uint8_t axes_interrupt, cpt_interrupt;
int16_t mpu_data[max_data_mpu];
extern  char unsigned  code_erreur;
#endif



/* -----------------------------------------------------------------------------
*  Local Functions
* ------------------------------------------------------------------------------
*/
static void SensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId);

#ifdef CAPTEUR_MPU

// Algorithmes
void algo_energie_vib(void);
void energie_vib_synthese(void);
void algo_chocs(void);
void chocs_synthese(void);
void algo_quaternion(uint8_t * acc_val);

void config_algo4_vib (void);

static void sensorMagInit(void);
static void sensorMagEnable(bool);
static bool sensorMpu9250SetBypass(void);

void SensorMpu9250_disableWom(void);

static void sensorTaskFunction(UArg arg0, UArg arg1);

void SensTimerCallback(UArg arg0);
void SensChgtModeTimerCallback(UArg arg0);
void Algo4ClockCallback(UArg arg0);

static void interrupt_mpu9250_Callback (void);
void magcalMPU9250(int16_t *);
void getMres(void);
void getAres(void);
void getGres(void);

void calib_magneto_pile(void);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void Calibration_restore_mpu(void);
void envoi_enreg_resultat (uint8_t code, uint8_t * chaine);


/* ------------------------------------------------------
 *                        FUNCTIONS
 * -------------------------------------------------------*/


void SensorTask_init(void)
{
    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&sensorEvent, &eventParam);
    sensorEventHandle = Event_handle(&sensorEvent);

    Clock_Params clkParams;
    Clock_Params_init (&clkParams);

    // setup clock pour sensor mpu     40 octets en RAM
    clkParams.period = PERIOD_MESURE * (1000 / Clock_tickPeriod); // 20ms
    clkParams.startFlag = FALSE;
    Clock_construct(&SensTimerClock, SensTimerCallback, clkParams.period, &clkParams);
    SensTimerClockHandle = Clock_handle(&SensTimerClock);

    // setup clock pour le changement de mode veille   40 octets en RAM
    clkParams.period = 0;  // non periodique
    clkParams.startFlag = FALSE;
    Clock_construct(&SensChgtModeTimer, SensChgtModeTimerCallback, 1, &clkParams);
    SensChgtModeTimerHandle = Clock_handle(&SensChgtModeTimer);

    // setup clock pour l'algo 4 - FFT - 40 octets en RAM
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&Algo4Clock, Algo4ClockCallback, 200000, &clkParams);  // Timeout 2 secondes pour 1ère mesure
    Algo4ClockHandle = Clock_handle(&Algo4Clock);

    /* Create the sensor task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = SENSOR_TASK_STACK_SIZE;
    sensorTaskParams.priority = SENSOR_TASK_PRIORITY;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.instance->name = "MPU";
    Task_construct(&sensorTask, sensorTaskFunction, &sensorTaskParams, NULL);
}

void init_capteur_mpu(void)
{

    // Init capteur MPU
    mpu_params.mpu_mode =  2;  // bit0-1 (0:i2c off  1:power off, i2C on  2:power on),
                               // bit2-3:accel-gyro (0:veille, 1:actif 3:fifo)
                               // bit4-5:magneto (0:veille, 1:continu)
    mpu_params.interruption = 0;    // interruption // 0:pas d'interruption  1:Data_ready,   0x40:Motion,   autre:Fifo overflow or Fsync
    mpu_params.power1 = 0;
    mpu_params.sel_axes = NO_AXES;   // Axes actives : NO_AXES, ACC_AXES, ALL_AXES
    mpu_params.smplrt_divis = 0;          // SMPLRT_DIV facteur de division de la fréquence 1Khz
    mpu_params.acc_odr = 11;              // ODR LP 500Hz
    mpu_params.dlpf_accel = 2;          // dlpf_accelero : 1:192Hz  2:92Hz  6:5Hz
    mpu_params.acc_range = ACC_RANGE_2G;  // Acc_range 2G, 4G, 8G, 16G
    mpu_params.dlpf_gyro = 2;            // dlpf_gyro : 1:199Hz
    mpu_params.Gscale = GFS_250DPS;   // GScale range gyro
    mpu_params.Mscale = MFS_16BITS;   // Mscale 14 ou 16 bits
    mpu_params.Mmode = MAG_MODE_OFF;  // Mmode  MAG_MODE_OFF - MAG_MODE_CONT2 : 100 Hz


    if (!(setup_mpu()))            // Init MPU  -  true=success   410ms
        code_erreur  = erreur_mpu_init;

    //Config_mpu(&mpu_params); // Configuration en fonction de params initial  //300ms
    Calibration_initiale_mpu();  // Calibration avec valeurs de calib dans le code

    config_mpu = 10;
    mode_actif = 4;
    config_type_mpu(CONFIG_MPU_INITIALE);  // paramétrage capteur avec type initial  320ms
    Clock_stop(SensTimerClockHandle);

    axes_interrupt=0;
    cpt_interrupt=0;
    accel_X.declench=0;


    // mode stop
    mode_actif = 0;
    mpu_params.mpu_mode = (mpu_params.mpu_mode & 0xFC) | 0x1;  // bit0-1: 1 = Power off
}

void chgt_acc_Z(void)
{
    Enreg_acc_z = 1 - Enreg_acc_z;
}

void Activ_sensor_mpu(void)
{
    Event_post(sensorEventHandle, EVENT_ACTIV_MPU);
}

void Desactiv_sensor_mpu(void)
{
    Event_post(sensorEventHandle, EVENT_DESACTIV_MPU);
}

// Changement de mode actif-veille  -  0:stop  1:veille 4Hz  2:veille 30Hz  3:veille 500Hz  4:actif
void application_mode_actif_veille (uint8_t new)
{
    uint8_t ret, val;

    if (new != mode_actif)
    {
        Clock_stop(SensChgtModeTimerHandle);  // arret timer chgt mode

        if ((new==3) && (!Duree_veille1)) new--;
        if ((new==2) && (!Duree_veille2)) new--;
        if ((new==1) && (!Duree_veille3)) new--;

        if (!new)   // passage en mode stop
        {
            Clock_stop(SensTimerClockHandle);  // Arret de la clock 20millisecondes
            SensorMpu9250_disableWom();
            mpu_params.mpu_mode = (mpu_params.mpu_mode & 0xFC) | 0x1;  // bit0-1: 1 = Power off
            sensorMpuSleep();           // mise en veille MPU
            SensorMpu9250_powerOff();   // power off alim MPU
            SensorI2C_close();        // I2c off
            if (mode_principal==1)
                ACTIV_SORTIE(LED_cpu,1,6);    // 1 flash long
            if  ((mode_affich_enreg)  && (!(cpt_envoi_balise)))
                Activ_radio_rx (1, DUREE_ACTIV_RX_TEMP, 5, 1, 0);  // Réception message du concentrateur

            //ACTIV_PWM ( 0,1,10,2000,3);  // 1 bip long
        }
        else if ( new>3 )   // passage en mode actif
        {
            extinction_mpu_cpt = Duree_Actif;  // Mise en veille au bout du temps specifie
            if (!mode_actif)  // si auparavant le module etait en stop
            {
                // reveil
                SensorI2C_open();
                PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);  // power on alim PMU
                DELAY_MS(120);  // permet de stabiliser le demarrage des capteurs
                SENSOR_SELECT();  // true=success  MPU select
                val = 0x80;
                ret = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);  // Device reset
                DELAY_MS(100);
                SENSOR_DESELECT();

                ret = SensorMpu9250_test();
                if (ret)
                {
                    // Initial configuration
                    SensorMpu9250_accSetRange(ACC_RANGE_2G);
                    sensorMagInit();
                }
                // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
                sensorMpu9250SetBypass();
                SENSOR_SELECT_MAG();  // Magnétometre
                if ( SensorI2C_read_1_Reg( MAG_WHO_AM_I) != 0x48)  // Read WHO_AM_I register for AK8963 0x48=success
                    ret=0;
                SENSOR_DESELECT();

                Calibration_restore_mpu();  // Calibration avec valeurs de calib
                mpu_params.mpu_mode = (mpu_params.mpu_mode & 0xFC) | 0x2;  // bit0-1: 2 = Power on
                Config_mpu(&mpu_params); // Configuration en fonction de params initial

                if ((algo_mpu & 0x7) && (mode_principal==1))  // Algo 1 à 3
                {
                    Clock_setPeriod(SensTimerClockHandle, PERIOD_MESURE * (1000 / Clock_tickPeriod )); // Period=0 pour un one-shot
                    Clock_start(SensTimerClockHandle);  // Démarrage de la lecture capteur 20 ms
                }
            }
            else if (mode_actif < 4)   // si auparavant le module etait en veille
            {
                // Wake on Motion
                SensorMpu9250_disableWom();  // 0.5ms
                mpu_params.mpu_mode =  (mpu_params.mpu_mode & 0xFC) | 0x2;  // Power on, fifo, magneto continu
                Config_mpu(&mpu_params); // Configuration en fonction de params initial
                if (algo_mpu & 0x7)  // Algo 1 à 3
                {
                    Clock_setPeriod(SensTimerClockHandle, PERIOD_MESURE * (1000 / Clock_tickPeriod) ); // Period=0 pour un one-shot
                    Clock_start(SensTimerClockHandle);  // Démarrage de la clock 20ms
                }
            }
            ACTIV_SORTIE(LED_cpu, 8 , 2);  // 1 flash court
            cpt_envoi_balise = 0;

            // initialisation de la vitesse de recalage quaternion : rapide au debut, lent ensuite
            if ((config_mpu == 12) || (config_mpu == 13))
            {
                timer_apres_debut = 6 * (1000/PERIOD_MESURE/Period_algo3);   // 6 secondes
                recalage_initial = 1000/PERIOD_MESURE/Period_algo3 ;
                valeur_recalage = 6;  // 10 recalage initial plus rapide (les premiers cycles)
                GyroMeasERROR(8);    // 300 puis 10
                lastUpdate = 0;
            }
            else
            {
                recalage_initial=1000/PERIOD_MESURE/Period_algo3 ;
                valeur_recalage = 6;
                GyroMeasERROR(8);  // Vitesse de recalage du quaternion : 3 ideal 40:rapide
            }

            if ((mode_affich_enreg) && (config_mpu == 14))
                Activ_radio_rx (1, DUREE_ACTIV_RX_TEMP, 5, 1, 0);  // Réception message du concentrateur
            //ACTIV_PWM ( 0,4,6*new,2000,3);  // x bip court
        }
        else  // passage en mode veille  new=1, 2 ou 3
        {
            uint8_t freq;
            uint16_t timer_sec;

            if (new == 1)
            {
                freq=4;                     //  mode 1 : 4 Hz, 1 uA
                timer_sec = Duree_veille3;
                mode_veille_long_cpt = 30;  // ex : 10min = 30fois 20s   1 semaine = 30 fois 6 heures
            }
            else if (new == 2)                  // mode 2 : 30 Hz, 11 uA pendant  2 heures
            {
                freq=7;
                timer_sec = Duree_veille2;
            }
            else
            {
                freq=INV_LPA_640HZ;           // mode 3 : 500 Hz, 280 uA
                timer_sec = Duree_veille1;     // 10 sec
            }
            if (mode_actif > 3)   // si auparavant on etait actif
            {
                // Passage de actif (mode 4) à veille (mode 3)  : activation mode WOM
                Clock_stop(SensTimerClockHandle);  // Arret de la clock 20millisecondes

                // Power down magnetometer
                SENSOR_SELECT_MAG();  // Magnétometre
                SensorI2C_write_1_Reg(MAG_CNTL1, 0);  // sleep
                SENSOR_DESELECT();

                SensorMpu9250_enableWom(Seuil_Wom, freq);
                Clock_setTimeout(SensChgtModeTimerHandle, (uint32_t)  timer_sec * (1000 / Clock_tickPeriod) * 1000);
                if (timer_sec != 9999)  // si 9999, pas de clock donc le module reste indefiniement dans ce mode
                    Clock_start(SensChgtModeTimerHandle);
            }
            else if (!mode_actif)    // si auparavant on etait en mode stop
            {
                // reveil
                SensorI2C_open();
                PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);  // power on alim PMU
                DELAY_MS(120);  // permet de stabiliser le demarrage des capteurs
                SENSOR_SELECT();  // true=success  MPU select
                val = 0x80;
                ret = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);  // Device reset
                DELAY_MS(100);
                SENSOR_DESELECT();

                // Initial configuration
                SensorMpu9250_accSetRange(ACC_RANGE_2G);
                sensorMagInit();

                Calibration_restore_mpu();  // Calibration avec valeurs de calib

                // Power down magnetometer
                SENSOR_SELECT_MAG();  // Magnétometre
                SensorI2C_write_1_Reg(MAG_CNTL1, 0);  // sleep
                SENSOR_DESELECT();

                SensorMpu9250_enableWom(Seuil_Wom, freq);
                Clock_setTimeout(SensChgtModeTimerHandle, (uint32_t)  timer_sec * (1000 / Clock_tickPeriod) * 1000);
                Clock_start(SensChgtModeTimerHandle);

            }
            else    // si auparavant on etait déja dans un autre mode veille
            {
                // changement de mode veille
                // Set Frequency of Wake-up
                SensorI2C_write_1_Reg(LP_ACCEL_ODR, freq);
                DELAY_MS(10);

                Clock_setTimeout(SensChgtModeTimerHandle, (uint32_t) timer_sec * (1000 / Clock_tickPeriod) * 1000);
                Clock_start(SensChgtModeTimerHandle);
            }
            ACTIV_SORTIE (0, 8 , 3*(5-new));  // 1:4 flashs  2:3 flashs    3:2 flashs
        }
        mode_actif = new;
    }
}


// Param : 0:quaternion  1:angles  2:gyro  3:accelero  4:magneto  5:val brut accel  6:vibration  7:chocs
// 10:choc   11:choc+vib   12:enveloppe accel+vib+choc 13:idem+quaternion   14:rien
void config_type_mpu(uint8_t config)
{
    uint8_t config_mpu_old = config_mpu;
    uint8_t algo4_actif;
    uint8_t acc_power1, acc_odr, acc_fchoice;

    algo4_actif = algo_mpu & BV(3);  // conservation de l'activation algo4(FFT)

    config_mpu = config;
    if (mode_actif != 4)  application_mode_actif_veille (4);

    if (config == 14)   // désactivation clock 20ms
    {
        Clock_stop(SensChgtModeTimerHandle);  // arret timer chgt mode
        //application_mode_actif_veille (0);
        //algo_mpu = 0;
    }

    acc_odr=0;
    acc_fchoice=0;
    acc_power1=0;
    if (FREQ_ECHAN != 1)  // LP mode
    {
        acc_power1=0;  // 0:LP mode desactive
        acc_odr=FREQ_ECHAN-1; // smplrt_div     2->1(500)    4->3(250)   100->(10Hz)
        //acc_odr = 12 - FREQ_ECHAN/2 ;      // 2->11   4->10
        acc_fchoice = 0;
    }

    mpu_params.power1 = acc_power1;
    mpu_params.mpu_mode =  (mpu_params.mpu_mode & 0x3) | ((3<<2)+(1<<4));  // fifo, magneto continu
    mpu_params.smplrt_divis = FREQ_ECHAN-1; // smplrt_div   1->0(1000)  2->1(500)    4->3(250)   100->(10Hz)   // 1:div par 2

    if (config < 5)  // quaternion et autres
    {
        algo_mpu=0b100;
        mpu_params.mpu_mode =  (mpu_params.mpu_mode & 0xF3) | (1<<2);  // fifo:off
        mpu_params.interruption=0;
        mpu_params.sel_axes = ALL_AXES;
        mpu_params.dlpf_accel = 2;  // filtre passe bas 92Hz
        mpu_params.acc_odr = acc_odr;
        mpu_params.dlpf_gyro = 2;   // filtre passe bas 92Hz
        mpu_params.Mmode = MAG_MODE_CONT2;
        mpu_params.acc_range = ACC_RANGE_2G;
    }
    if (config == 2)  // gyro
    {
        mpu_params.sel_axes = GYRO_AXES;
        mpu_params.power1 = 0;
        mpu_params.dlpf_accel = 2 ;  // filtre passe bas 1:192Hz  2:92Hz  6:5Hz
        mpu_params.dlpf_gyro = 2;    // filtre passe bas 92Hz
        mpu_params.Mmode = MAG_MODE_OFF;
    }

    if (config == 5)  // valeurs brutes
    {
        algo_mpu = 0b100 ;
        mpu_params.mpu_mode =  (mpu_params.mpu_mode & 0xF3) | (1<<2);  // fifo:off
        mpu_params.interruption=0;
        axes_interrupt = 0 ;  // bit0:acc, bit1:gyro, bit2:magneto : lecture uniquement de l'accelero par la routine d'interruption
        mpu_params.sel_axes = ALL_AXES;
        mpu_params.dlpf_accel = 6;  // filtre passe bas 1:192Hz  2:92Hz 6:5Hz
        mpu_params.acc_odr = acc_odr;
        mpu_params.acc_range = ACC_RANGE_2G;
        mpu_params.dlpf_gyro = 1;   // filtre passe bas 192Hz
        mpu_params.Mmode = MAG_MODE_CONT2;
    }
    if ((config == 6) || (config == 7))  // vib ou chocs
    {
        algo_mpu = config - 5 ;  // 0b01 ou 0b10
        mpu_params.mpu_mode =  (mpu_params.mpu_mode & 0x0F);  // fifo:on, pas de magneto
        mpu_params.interruption=0;
        mpu_params.sel_axes = ACC_AXES;
        mpu_params.dlpf_accel = 2;  // filtre passe bas 2:92Hz
        mpu_params.acc_odr = acc_odr;
        mpu_params.dlpf_gyro = 2;   // filtre passe bas 2:92Hz
        mpu_params.Mmode = MAG_MODE_OFF;
        mpu_params.acc_range = ACC_RANGE_16G;
    }

    if ((config == 10) || (config == 11))  // algo 1 ou 1+2
    {
        algo_mpu = 0b11 ;
        if (config == 10)  algo_mpu = 0b10 ;
        mpu_params.mpu_mode =  (mpu_params.mpu_mode & 0x0F) ;  // fifo:on, pas de magneto
        mpu_params.power1 = acc_power1;
        mpu_params.interruption=0;
        axes_interrupt = 0 ;  // bit0:acc, bit1:gyro, bit2:magneto : lecture uniquement de l'accelero par la routine d'interruption
        mpu_params.sel_axes = ACC_AXES;
        mpu_params.dlpf_accel = Dlpf_accel ;  // plutot 2  filtre passe bas 1:192Hz  2:92Hz  3:48Hz  4:24Hz  5:10Hz  6:5Hz
        mpu_params.acc_odr = 11;
        mpu_params.acc_range = ACC_RANGE_16G;
        mpu_params.dlpf_gyro = 1;   // filtre passe bas 192Hz
        mpu_params.Mmode = MAG_MODE_OFF;
    }

    if (((config == 12) || (config == 13)) && (config_mpu_old!=12) && (config_mpu_old!=13))
    {
        recalage_initial = 1000/PERIOD_MESURE/Period_algo3 ;
        valeur_recalage = 6;  // recalage initial plus rapide (les premiers cycles)
        GyroMeasERROR(8);
        lastUpdate=0;
    }
    else
    {
        recalage_initial=1000/PERIOD_MESURE/Period_algo3 ;
        valeur_recalage = 6;
    }

    if ((config == 12) || (config == 13))  // algo 1 à 3
    {
        algo_mpu = 0b111 ;
        mpu_params.interruption=0;
        axes_interrupt = 0 ;  // bit0:acc, bit1:gyro, bit2:magneto : lecture uniquement de l'accelero par la routine d'interruption
        mpu_params.sel_axes = ACC_AXES;
        mpu_params.dlpf_accel = 2 + acc_fchoice;  // filtre passe bas 1:192Hz  2:92Hz  6:5Hz
        mpu_params.acc_range = ACC_RANGE_16G;
        mpu_params.acc_odr = acc_odr;
        mpu_params.dlpf_gyro = 1;   // filtre passe bas 192Hz
        mpu_params.Mmode = MAG_MODE_OFF;
        accel_X.declench=0;
        accel_X.compteur=0;
        timer_apres_debut = 6 * (1000/PERIOD_MESURE/Period_algo3);  // 6 secondes
    }

    Config_mpu(&mpu_params); // Configuration en fonction de params

    algo_mpu = algo_mpu | algo4_actif;

    if ( (algo_mpu & 0x7)  && (mode_actif == 4))  // Algo 1 à 3
    {
        Clock_setPeriod(SensTimerClockHandle, PERIOD_MESURE * (1000 / Clock_tickPeriod) ); // Period=0 pour un one-shot
        Clock_start(SensTimerClockHandle);  // Démarrage de la lecture capteur 20 ms
    }
}

// config accelero-gyro-magneto
// gestion power off - mode veille - FIFO
// gestion clock
void Config_mpu(config_mpu_params *Params)
{
    uint8_t val;
    uint8_t dlpf_accel;

    // Configuration interruptions
    interrupt_actif= Params->interruption;  // 0:pas d'interruption  1:Data_ready,   0x40:Motion,   autre:Fifo overflow or Fsync

    sel_axes = Params->sel_axes;  // Selection axes (ALL_AXES, ACC_AXES)

    // configuration fréquence de base
    smplrt_divis = Params->smplrt_divis;  // 0:1khZ  4:200Hz

    // Configuration Accelerometre
    dlpf_accel = Params->dlpf_accel;  // DLPF pour accelerometre 6=5Hz  2=92Hz
    acc_lp_odr = Params->acc_odr;
    accRange = Params->acc_range;  // Accelero 2G
    getAres();

    // Configuration Gyroscope
    dlpf_gyro = Params->dlpf_gyro;   // DLPF pour gyro          6=5Hz
    Gscale = Params->Gscale; // Gyroscope range
    getGres();

    // Configuration Magnetometre
    Mscale = Params->Mscale;      // Magneto : 16 bits
    Mmode = Params->Mmode;  // Magneto : MAG_MODE_CONT2 : 100 Hz continu
    getMres();

    SENSOR_SELECT();


    // wake up device
    //SensorI2C_write_1_Reg(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    //DELAY_MS(100); // Wait for all registers to reset

     // reset device
     //SensorI2C_write_1_Reg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
     // DELAY_MS(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    // SensorI2C_write_1_Reg( PWR_MGMT_1, 0x01);
    // DELAY_MS(200);


    SensorI2C_write_1_Reg( PWR_MGMT_2, sel_axes);  // Axes : Accel & gyro

   // Configure MPU6050 gyro and accelerometer
     SensorI2C_write_1_Reg( CONFIG, dlpf_gyro);      // Set low-pass filter to 1:188 Hz 6:5Hz  FIFO_mode=0
     SensorI2C_write_1_Reg( SMPLRT_DIV, smplrt_divis);  // Set sample rate to 0:1 kHz
     SensorI2C_write_1_Reg( LP_ACCEL_ODR, acc_lp_odr);  // Set odr low power accelerometer
     SensorI2C_write_1_Reg( GYRO_CONFIG, Gscale<<3);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
     SensorI2C_write_1_Reg( ACCEL_CONFIG, accRange<<3); // Set accelerometer full-scale to 2 g, maximum sensitivity
     SensorI2C_write_1_Reg( ACCEL_CONFIG_2, dlpf_accel); // Fchoice=1, Set accelerometer rate to 1 kHz and bandwidth to 41 Hz (delay:5ms)

     SENSOR_DESELECT();

     SENSOR_SELECT_MAG();  // Magnétometre
     val = (Mscale << 4) | Mmode;
     SensorI2C_writeReg(MAG_CNTL1, &val, 1);
     SENSOR_DESELECT();

     // Activation interruption
     SENSOR_SELECT();
     SensorI2C_write_1_Reg( INT_PIN_CFG, 0x22);  // BIT_BYPASS_EN  et  BIT_LATCH_EN
     if (interrupt_actif)
     {
         SensorI2C_readReg(INT_STATUS,&val,1);   // Clear interrupt
         SensorI2C_write_1_Reg( INT_ENABLE, interrupt_actif);  // Enable data ready (bit 0) interrupt or Motion
         // Enable pin for interrupt
         PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);
     }
     else
     {
         PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);
         SensorI2C_write_1_Reg( INT_ENABLE, 0);  // Disable
     }

     // activate FIFO pour accel
     if ((Params->mpu_mode & 0b1100) == 0b1100)
     {
         SensorI2C_write_1_Reg( FIFO_EN, 0x00);      // Disable FIFO
         SensorI2C_write_1_Reg( PWR_MGMT_1, 0x00);   // Turn on internal clock source
         SensorI2C_write_1_Reg( USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
         SensorI2C_write_1_Reg( USER_CTRL, 0x0C);    // Reset FIFO and DMP
         DELAY_MS(3);
         // Configure FIFO to capture accelerometer data
         SensorI2C_write_1_Reg( USER_CTRL, 0x40);   // Enable FIFO
         SensorI2C_write_1_Reg( FIFO_EN, 0x08);     // Enable accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
     }

     SensorI2C_write_1_Reg(PWR_MGMT_1, Params->power1); // enable cycle

     SENSOR_DESELECT();

     // Config Accelerometer et gyroscope
     // PWR_MGMT_1 = 0x09      Make sure accelerometer is running
     // PWR_MGMT_2 = 0x07      Enable accelerometer, disable gyro
     // ACCEL_CONFIG_2 = 0x01  Set Accel LPF setting to 184 Hz Bandwidth, 1khz, DLPF, 2ms delay
     // INT_ENABLE = BIT_WOM_EN  Enable Motion Interrupt
     // ACCEL_INTEL_CTRL = 0xC0  Enable Accel Hardware Intelligence
     // WOM_THR = thresold       Set Motion Threshold 1=4mg 10=40mg
     // LP_ACCEL_ODR = INV_LPA_20HZ   Set Frequency of Wake-up (Low power) 15Hz
     // PWR_MGMT_1 = 0x29        Enable Cycle Mode (Accel Low Power Mode)
     // ACCEL_CONFIG = accRange<<3   Select the current range

     // Fonctionnement Low_power
     /*ACCEL_CONFIG(28)=0   Acc_range=2g
     ACCEL_CONFIG_2(29)=(3<<3) F_choice_b=3 (DLPF inactif)  acc_DLPF=3 => bande acc=44Hz/5ms
     LP_ACCEL_ODR(30)=2 (1Hz)

     // Fonctionnement continu avec DLPF et 200Hz - Accel+gyro
     SMPLRT_DIV (25) = 4  (avec F_choice_b=0 & dlpf>0 - 1kHz=>200Hz=5ms
     CONFIG(26) =3        gyro_DLPF=3 => bande gyro 41Hz/6ms
     GYRO_CONFIG(27)=0    F_choice_b=0 (DLPF actif),   Gyro_range=250dps
     ACCEL_CONFIG(28)=0   Acc_range=2g
     ACCEL_CONFIG_2(29)=3 F_choice_b=0 (DLPF actif)  acc_DLPF=3 => bande acc=44Hz/5ms
     LP_ACCEL_ODR(30)=0
     FIFO_EN(35)=0
     SIGNAL_PATH_RESET(104)=0
     ACCEL_INTEL_CTRL(105)=0
     USER_CTRL(106)=0
     PWR_MGMT_1(107)=1
     PWR_MGMT_2(108)=0*/

     // Lecture des registres pour visualiser la configuration accel/gyro
     /*uint8_t val_reg[40];
     SensorI2C_readReg( 25,  &val_reg[0], 16);
     SensorI2C_readReg( 104,  &val_reg[0], 8);*/
}



static void sensorTaskFunction(UArg arg0, UArg arg1)
{
    uint8_t i, j,  data_count[2], acc_valid;
    uint16_t k;

    DELAY_MS(1600);  // permet d'attendre le demarrage du capteur mpu

    // Initialisation filtre bi-quadratique IIR
    for (j=0; j<3; j++)
    {
        YC_[j]=0;
        YC_1[j]=0;
    }

    // Init compteurs algorithmes
    for (i=0; i < Nb_algorithmes ; i++)
    {
        cpt_algo_mpu[i]=0;
    }

    index_ram_acc_z=10;
    Enreg_acc_z = ENREG_ACC_Z;

    Dlpf_accel = DLPF_ACCEL;

    // Init Algo 1 Energie vibratoire
    Seuil_vib_a1 = SEUIL_VIB_A1;
    Duree_algo1 = DUREE_ALGO1;

    for (j=0; j < 3; j++)
    {
        ac_somme_a1[j]=0;
        ac_carre_a1[j]=0;
    }
    satur_a1=0;
    algo1_count_ac=0;

    // Init Algo 2 Chocs
    Duree_algo2 = DUREE_ALGO2;
    Choc_pente_min = CHOC_PENTE_MIN;
    Choc_amplitude_min = CHOC_AMPLITUDE_MIN;
    Choc_energie_min = CHOC_ENERGIE_MIN;
    satur_a2=0;
    algo2_count_ac=0;
    for (j=0; j < 3; j++)
    {
        pente_etat[j]=0;   // 0:pas de choc, 1:premiere pente, 2:attente, 3:deuxième pente
    }
    init_algo2=1;
    choc_amplitude=0;
    choc_duree=0;

    // Init Algo 3 (quaternion et acceleration)
    Period_algo3 = PERIOD_ALGO3;
    Period_envoi_quat = PERIOD_ENVOI_QUAT;
    Declench_debut = DECLENCH_DEBUT;
    Declench_fin = DECLENCH_FIN;
    Duree_acceleration = DUREE_ACCELERATION;
    cpt_envoi_quat=0;

    // Init Algo 4
    Axe_Mesure = AXE_MESURE;
    Period_Algo4_H = PERIOD_ALGO4_H;
    Period_Algo4_D = PERIOD_ALGO4_D;
    algo_mpu = (algo_mpu & 0x7) | (ACTIV_ALGO4 << 3);
    Seuil_Algo4 = SEUIL_ALGO4;

    // Init mise en veille
    Seuil_Wom = SEUIL_WOM;
    Duree_Actif = DUREE_ACTIF * (1000 / PERIOD_MESURE); // unite seconde -> PERIOD_MESURE
    extinction_mpu_cpt = Duree_Actif;  // Initialisation Mise en veille au bout du temps specifie
    Duree_veille1 = DUREE_VEILLE1;
    Duree_veille2 = DUREE_VEILLE2;
    Duree_veille3 = DUREE_VEILLE3;
    cpt_envoi_balise = 0;
    cpt_err_sens = 0;
    cpt_valid =0;

    message[0]=PC_ADDRESS;
    if (!mode_affich_enreg)  // envoi par liaison serie
        message[0]=PC_NODE_SERIE_ADDRESS;

    //SensorI2C_open();
    if ((mode_periodique) && (mode_principal==1) )  // activation periodique mouvement
    {
        application_mode_actif_veille (4);    // modes actif
    }

    if ((algo_mpu & 0x7) && (mode_actif==4))
        Clock_start(SensTimerClockHandle);  // Démarrage de la lecture capteur Algo 1 à 3

    if (algo_mpu & 0x8)
        config_algo4_vib();                 // Démarrage de la lecture capteur Algo 4
    if ((mode_principal==2) && (mode_periodique))
        chgt_mode_algo4(1);
    else
        chgt_mode_algo4(0);

    #ifdef DEBUG_ACC
        cpt_test_acc=0;
        trig_test_acc=0;
        test_acc_fige=0;
    #endif

    capteur_mpu_init = 1;

    //trigger_time = 1;
    val_test = 0;

    while (1)
    {

        uint32_t events = Event_pend(sensorEventHandle, 0, SENSOR_EVENT_ALL, BIOS_WAIT_FOREVER);

        if (events & EVENT_ACTIV_MPU)      // Activer le capteur MPU
        {
            application_mode_actif_veille (4);
        }
        if (events & EVENT_DESACTIV_MPU)  // desactiver le capteur MPU
        {
            application_mode_actif_veille (0);
        }

        if (events & SENSOR_EVENT_READ_SENSOR)  // 2:toutes les 20 milli-secondes
        {
            if (Clock_isActive(SensTimerClockHandle))
            {
                // Read FIFO  : 50 mesures en 10ms
                acc_valid=0;
                if  ((mpu_params.mpu_mode & 0b1100) == 0b1100)  // si FIFO actif
                {
                    SENSOR_SELECT();
                    k = SensorI2C_readReg(FIFO_COUNT_H,  &data_count[0], 2);
                    count_FIFO = (((uint16_t)data_count[0] << 8) | data_count[1] ) /6;    // How many sets of full accelerometer data for averaging

                    // read data - 8ms pour 20*3 valeurs    20ms pour 50*3  24ms pour 60*3
                    // 56 si period=200ms et freq_ech=4 (ODR=250hz)
                    if ((count_FIFO < (PERIOD_MESURE / FREQ_ECHAN + 6) ) && (k))
                    {  //
                        uint8_t udata[ (PERIOD_MESURE / FREQ_ECHAN + 6) * 6 ];
                        cpt_valid++;

                        if (count_FIFO > 40)  // lecture de plus de 256 donnees
                        {
                            SensorI2C_readReg( FIFO_R_W,  &udata[0]  , 240);
                            SensorI2C_readReg( FIFO_R_W,  &udata[240], 6*(count_FIFO-40) );
                        }
                        else
                            // lecture I2c de 180 octets=24ms, trop long pour débit de 400kb/s - etonnant
                            SensorI2C_readReg( FIFO_R_W,  &udata[0], 6*count_FIFO );

                        k=0;
                        for (i=0; i < count_FIFO; i++)
                        {
                            for (j=0; j<3; j++)
                            {
                                int16_t val_acc;
                                val_acc = (int16_t) ((udata[k++] << 8) | udata[k++]  ) - acc_bias[j]/acc_rapport ;  // Form signed 16-bit integer for each sample in FIFO
                                ac_data[i][j] = val_acc;

                                //enregistrement (pour debug) des donnees accelero mesurees
                                #if (DEBUG_ACC & 0b11)
                                    if ((trig_test_acc) && (axe_test_acc==j))
                                    {
                                        test_acc[cpt_test_acc++] = (int16_t) val_acc;
                                        if ((cpt_test_acc >= TEST_NB) )
                                        {
                                            trig_test_acc=0;
                                            cpt_test_acc=0;
                                            // Mettre breakpoint ici puis copier les donnees de test_acc
                                            if (test_acc_fige)
                                                test_acc_fige=0;
                                        }
                                    }
                                #endif
                            }
                        }

                        #if ( DEBUG_TIME & 0b11 ) == 0b10
                            if ((trigger_time) && (index_time <MAX_Index))
                            {
                                etat_cpu[index_time] = 00;
                                etat_detail[index_time] = count_FIFO ;
                                time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                                if (index_time >MAX_Index)
                                    index_time=0;
                            }
                        #endif
                        if (count_FIFO)  acc_valid=1;
                    }
                    else
                    {   // Reset FIFO
                        cpt_err_sens++;
                        #if (DEBUG_TIME&0b11) == 0b10
                        if ((trigger_time) && (index_time <MAX_Index))
                        {
                            etat_cpu[index_time] = 70;
                            etat_detail[index_time] = count_FIFO ;
                            time_cpu[index_time++] = Clock_getTicks(); // 1 tick = 10 micro-second  A supprimer
                            if (index_time >MAX_Index)
                                index_time=0;
                        }
                        #endif
                        //SensorI2C_write_1_Reg( FIFO_EN, 0x00);      // Disable FIFO
                        //SensorI2C_write_1_Reg( PWR_MGMT_1, 0x00);   // Turn on internal clock source
                        //SensorI2C_write_1_Reg( USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
                        SensorI2C_write_1_Reg( USER_CTRL, 0x0C);    // Reset FIFO and DMP
                        DELAY_MS(1);  // initialement : 15ms
                        // Configure FIFO to capture accelerometer data
                        SensorI2C_write_1_Reg( USER_CTRL, 0x40);   // Enable FIFO
                        //SensorI2C_write_1_Reg( FIFO_EN, 0x08);     // Enable accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
                    }
                    SENSOR_DESELECT();
                }

                //acc_valid=0;  // mode debug : permet d'eviter l'execution des algo 1 et 2

                // Execute les algorithmes selectionnés
                if (acc_valid)   // necessite d'avoir les donnees d'accelero
                {
                    // Envoie données brutes accelero axe z
                    if (Enreg_acc_z)
                    {
                        for (i=0; i<count_FIFO; i++)
                        {
                            // enregistrement en pile des info acc
                            if (index_ram_acc_z<11) index_ram_acc_z=11;
                            acc_z_ram[index_ram_acc_z++] = ac_data[i][2] >> 8;
                            acc_z_ram[index_ram_acc_z++] = ac_data[i][2] & 0xFF;
                            // envoi si pile pleine
                            if (index_ram_acc_z >= NB_DATA_ACC_Z*2 + 11)
                            {
                                acc_z_ram[0] = NB_DATA_ACC_Z*2 + 11;  // longueur avec cet octet
                                acc_z_ram[10] = FREQ_ECHAN;  // 4:4ms
                                envoi_enreg_resultat('Z', acc_z_ram);
                                index_ram_acc_z = 0;
                            }
                        }
                    }

                    // --------------  Algorithme 1 : Energie vibratoire
                    if (algo_mpu & 1)
                    {
                        algo_energie_vib();   // 2,8ms pour 50 donnees
                        cpt_algo_mpu[0]++;
                        if (cpt_algo_mpu[0] >= Duree_algo1)  // Synthese périodique
                        {
                            energie_vib_synthese();
                            cpt_algo_mpu[0]=0;
                            for (j=0; j < 3; j++)
                            {
                                ac_somme_a1[j]=0;
                                ac_carre_a1[j]=0;
                            }
                            satur_a1=0;
                            algo1_count_ac=0;
                        }
                    }

                    // --------------  Algorithme 2 : Chocs
                    if (algo_mpu & BV(1))
                    {
                        algo_chocs();
                        cpt_algo_mpu[1]++;
                        if (cpt_algo_mpu[1] >= Duree_algo2)  // Synthese périodique
                        {
                            chocs_synthese();
                            cpt_algo_mpu[1]=0;
                            for (j=0; j < 3; j++)
                            {
                                pente_etat[j]=0;   // 0:pas de choc, 1:premiere pente, 2:attente, 3:deuxième pente
                            }
                            satur_a2=0;
                            algo2_count_ac=0;
                            choc_amplitude=0;
                            choc_duree=0;
                        }
                    }
                }

                // --------------  Algorithme 3 : 3D, quaternion, acceleration X
                if (algo_mpu & BV(2))
                {
                    cpt_algo_mpu[2]++;
                    if (cpt_algo_mpu[2] >= Period_algo3)  // Synthese périodique
                    {
                        cpt_algo_mpu[2]=0;
                        if (config_mpu == 2)  // gyro seul
                        {
                            //Read gyro
                            SENSOR_SELECT();
                            //SensorI2C_write_1_Reg(PWR_MGMT_1, 0);
                            SensorI2C_write_1_Reg( PWR_MGMT_2, GYRO_AXES);  // Axes : Accel & gyro
                            SENSOR_DESELECT();
                            //SensorI2C_close();        // I2c off
                            DELAY_MS(31);   // de Standby : minimum 5ms  de Sleep: min 29ms
                            //SensorI2C_open();        // I2c on
                            float gyr_v[3];
                            gyro_read (gyr_v); // 0,4ms

                            SENSOR_SELECT();
                            //SensorI2C_write_1_Reg(PWR_MGMT_1, 0x10);  // Gyro Standby
                            SensorI2C_write_1_Reg( PWR_MGMT_2, NO_AXES);  // Axes : Accel & gyro
                            SENSOR_DESELECT();

                            cpt_envoi_quat++;
                            if (cpt_envoi_quat >= Period_envoi_quat)
                            {
                                cpt_envoi_quat = 0;
                                strcpy((char *)message+1, "Gyros:");
                                ajout_float_multi_valeurs (gyr_v, 2, 7, 3, 0);
                                Mess_statut=0x10 ;  // bit0:dernier  bit1-2:reenvoi(00:non, 01:1 fois, 10:x fois)
                                // bit3:différé   bit4:pas d'ack  bit5:RX apres
                                envoi_message ();  // pas d'ack, pas RX apres
                            }
                        }
                        else
                            algo_quaternion(&acc_valid);  // 2,5ms
                    }
                }


                // ------------ Mise en veille si plus de mouvement

                // Mise en veille du capteur MPU si plus de mouvement
                if (Duree_Actif)   // si mise en veille activee
                {

                    extinction_mpu=1;

                    if (acc_valid==1)  // si FIFO valide : utilisation des dernières valeurs lues
                    {
                        // prend la dernière mesure
                        for (i=0; i<3; i++)
                            acc_v[i] = (float) (ac_data[count_FIFO-1][i]) / aRes * acc_scale[i];
                    }

                    // mesure amplitude mouvement
                    for (i=0; i<3; i++)
                    {
                        if ( fabs(acc_v[i] - acc_prec[i]) > ((float) Seuil_Wom / 150 ) )
                            extinction_mpu=0;    // => mouvement => pas d'extinction
                        acc_prec[i] = acc_v[i];
                    }

                    if (extinction_mpu)  // si pas de mouvement
                    {
                        if (extinction_mpu_cpt) extinction_mpu_cpt--;   // pas de mouvement pendant plusieurs cycles successifs
                        else
                            // Mise en veille
                            application_mode_actif_veille (3);
                    }
                    else
                        extinction_mpu_cpt = Duree_Actif;   // remis a zero compteur
                }

                // A l'issue de la mesure , envoi balise radio de demande de messages en attente
                // uniquement en mode "envoi vers radio"
                if (mode_affich_enreg == 1)
                {
                    if (cpt_envoi_balise) cpt_envoi_balise--;
                    if (!cpt_envoi_balise)
                    {
                        cpt_envoi_balise = ( 1000 / PERIOD_MESURE ) * 10;  // 10 secondes
                        Activ_radio_rx (1, DUREE_ACTIV_RX_TEMP, 5, 1, 0);  // Réception message du concentrateur
                    }
                }
            }
        }
        if (events & EVENT_INTERRUPT_MPU)
        {
       // interruption  0:pas d'interruption  1:Data_ready,   0x40:Motion,   autre:Fifo overflow or Fsync
            uint8_t intData[3], val;

            SENSOR_SELECT();
            val = SensorI2C_readReg(INT_ENABLE, intData, 3);   //180us
            SENSOR_DESELECT();
            //intStatus =SensorI2C_read_1_Reg(INT_STATUS);   //180us
            if ((val) && (intData[2]))  // intStatus
            {
                if ((intData[2] & 1) && (intData[0] & 1))    // Data ready
                {
                }
                if ((intData[2] & 0x40) && (intData[0] & 0x40))   // Wake on Motion
                {
                    if ((mode_actif) && (mode_actif<4))
                    {
                        SensorI2C_write_1_Reg( INT_ENABLE, 0);  // Disable
                        application_mode_actif_veille (4);  // activation mode actif
                    }
                    else
                        code_erreur = 0x44;
                }
            }
        }
        if (events & SENSOR_EVENT_CHGT_MODE)  // chgt de mode a la fin du timer
        {
            if (mode_actif > 1)
            {
                application_mode_actif_veille (mode_actif-1);
            }
            if (mode_actif == 1)
            {
                if (mode_veille_long_cpt)
                {
                    mode_veille_long_cpt--;  // relance timer
                    Clock_setTimeout(SensChgtModeTimerHandle, (uint32_t) Duree_veille3  * (1000 / Clock_tickPeriod) * 1000);
                    Clock_start(SensChgtModeTimerHandle);
                }
                else
                {
                    // passage en mode stop , au bout d'une semaine
                    application_mode_actif_veille (0);
                }
            }
        }
        if (events & SENSOR_TRAITEMENT_MESSAGE)  // message recu par liaison serie ou RF
        {
            traitement_message_sensor_mpu(longueur_message_in);
        }


        if (events & SENSOR_EVENT_ALGO4)        // Realisation de la mesure Vibration - FFT - Algo 4
        {
            #ifdef FFT_ON
            fft_mul_out fftout;
            lecture_fft_params fftParams = {
                100,     // gamme de frequence 5 à 100 Hz => echantillonage 10 fois plus élevé
                150,     // seuil en g (à diviser par 100) pour déclarer machine en marche (Eff > seuil/100)
                Axe_Mesure,  // selection axe à mesurer : 0 :auto(le plus fort RMS), sinon 1(X),2(Y) ou 3(Z)
                0,           // 0:max, 1:min(max sur 50), 2:min(max sur 100)
                0            // Filtres : 0 :pas de filtre, 1:Passe-haut 15Hz
            };

            // Reveil du capteur
            uint8_t mode_actuel = mode_actif;
            if (mode_actuel != 4 )   // reveil si nécessaire
            {
                application_mode_actif_veille(4);
            }

            lecture_multi_fft (&fftParams, &fftout);

            // Mise en veille
            if (mode_actuel != 4 )   // restoration etat precedent
            {
                application_mode_actif_veille(mode_actuel);
            }

            // Envoi des informations de FFT et vibratoires
            if (fftout.efficace > Seuil_Algo4)
            {
                uint8_t chaine [28 + 6*Nb_valeur_max];
                // Statut(1), axe(1), efficace(4), ac_ratio(4), ac_kurt(4), ecart_type_kurt(4), 4 * fft_freq(4), 4 * fft_max(2)
                chaine[0] = 28 + 6*Nb_valeur_max;  // longueur avec cet octet
                chaine[10] = fftout.statut;
                chaine[11] = fftout.axe;
                //chaine[11] = fftout.moyenne >> 8;
                //chaine[11] = fftout.moyenne ;
                memcpy (chaine+12, &fftout.efficace, 4);
                memcpy (chaine+16, &fftout.ac_ratio, 4);
                memcpy (chaine+20, &fftout.ac_ratio_kurt, 4);
                memcpy (chaine+24, &fftout.kurt_ecart_type, 4);
                memcpy (chaine+28, &fftout.fft_freq, 2 * Nb_valeur_max );
                memcpy (chaine+28+2*Nb_valeur_max, &fftout.fft_max, 4*Nb_valeur_max);
                envoi_enreg_resultat('V', chaine);
            }
            #endif
        }
    }
}
//Taille : statut(1), Axe(1), efficace(float), ratio(float), Ratio_kurt(float), ecart type kurt(float)
//   5 freq(int), 5 val(float)

// Callback toutes les 20 milli-seconde pour effectuer la relève de la FIFO (Algo 1 et 2) et mesures Algo 3
void SensTimerCallback(UArg arg0)
{
    // Post event pour effectuer une lecture du capteur
    Event_post(sensorEventHandle, SENSOR_EVENT_READ_SENSOR);
}

// Callback permettant de changer de mode actif -> veille
void SensChgtModeTimerCallback(UArg arg0)
{
    // Post event pour changement de mode veille
    Event_post(sensorEventHandle, SENSOR_EVENT_CHGT_MODE);
}

// Callback permettant de lancer la mesure vibratoire (Algo 4)
void Algo4ClockCallback(UArg arg0)
{
    Clock_stop(Algo4ClockHandle);  // inutile

    if (Period_Algo4_cpt_h >= Period_Algo4_H)   // Fin cycle periodique
    {
        // Redemarrage clock pour future mesure
        Period_Algo4_cpt_h = 0;
        // Si la periode n'est pas nulle => relance clock, sinon arret
        if (((Period_Algo4_H) || (Period_Algo4_D)) && (mode_periodique == 1))
        {
            if (Period_Algo4_D)
            {
                Clock_setTimeout(Algo4ClockHandle, Period_Algo4_D * 15 * (1000 * 1000 / Clock_tickPeriod));
                Clock_setPeriod(Algo4ClockHandle, 0 ); // Period=0 pour un one-shot
                Clock_start(Algo4ClockHandle);
            }
            else
            {
                Clock_setTimeout(Algo4ClockHandle, 3600 * (1000 * 1000 / Clock_tickPeriod));    // 1 heure
                Clock_setPeriod(Algo4ClockHandle, 0 ); // Period=0 pour un one-shot
                Clock_start(Algo4ClockHandle);
                Period_Algo4_cpt_h ++;
            }
            // Post event pour mesure Algo4
            Event_post(sensorEventHandle, SENSOR_EVENT_ALGO4);
        }

    }
    else   // Relance clock pour 1 heure
    {
        Period_Algo4_cpt_h ++;
        Clock_setTimeout(Algo4ClockHandle, 3600 * (1000 * 1000 / Clock_tickPeriod));    // 1 heure
        Clock_setPeriod(Algo4ClockHandle, 0 ); // Period=0 pour un one-shot
        Clock_start(Algo4ClockHandle);
    }

}

void lancement_algo4(void)
{
    // Post event pour mesure Algo4
    Event_post(sensorEventHandle, SENSOR_EVENT_ALGO4);
}

void chgt_mode_algo4(uint8_t newval)
{
    if (newval)   // activation Vibration-FFT
    {
        Clock_start(Algo4ClockHandle);
    }
    else
        Clock_stop(Algo4ClockHandle);

/*        if (Clock_isActive(Algo4ClockHandle))
        {
            Clock_stop(Algo4ClockHandle);
            ACTIV_SORTIE (0, 1 , 5);     // Arret : 1 flash long
        }
        else
        {
            ACTIV_SORTIE (0, 1 , 2);     // Marche : 1 flash court
        }
    }*/
}


void traitement_message_capteur_mpu(void)
{
    Event_post(sensorEventHandle, SENSOR_TRAITEMENT_MESSAGE);
}




// initialise le mpu9250 et magnétometre
// return true = success
uint8_t setup_mpu(void)
{
    uint8_t ret;

    ret=1;
    // init et reset - range : ACC_RANGE, test, puis sleep/LP
    //ret = SensorMpu9250_init();
    SensorMpu9250_registerCallback(interrupt_mpu9250_Callback); // apres init
    DELAY_MS(400);

    // SelfTest du Gyro et accelerometer - utile uniquement en mode debug
    /*float   SelfTest[6];    // holds results of gyro and accelerometer self test
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    // acc:102  99 100    gyro :260  50  73
    strcpy((char *)message, "LSelf-test: ");
    ajout_float_multi_valeurs (SelfTest, 1, 11, 6, 0);
    envoi_message ();*/

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    sensorMpu9250SetBypass();
    SENSOR_SELECT_MAG();  // Magnétometre
    if ( SensorI2C_read_1_Reg( MAG_WHO_AM_I) != 0x48)  // Read WHO_AM_I register for AK8963 0x48=success
        ret=0;
    SENSOR_DESELECT();
    //DELAY_MS(1000);
    compt_err_imu=0;  // compteur d'erreurs de lecture du capteur

    return ret;
}


// --------------  ALGORITHMES  ------------------
//------------------------------------------------

// Algorithme 1 : Energie vibratoire

// Calcule en continu la moyenne et la l'energie
void algo_energie_vib(void)
{
    uint8_t j, i;
    //double ac_float;

    for (i = 0; i < count_FIFO; i++)
    {
        for (j=0; j<3; j++)
        {
            //ac_float =  (double) (ac_data[i][j] ) * acc_scale[j] / aRes;
            ac_somme_a1[j]  += ac_data[i][j];
            ac_carre_a1[j] += (double) ac_data[i][j] * ac_data[i][j];
            //if (abs(ac_data[i][j]) > 29000)  satur_a1=1;     // proche de la saturation
        }
    }
    algo1_count_ac += count_FIFO;
}

void energie_vib_synthese(void)
{
    uint8_t j, k;
    float ecart_type_ac[3];
    uint8_t chaine[30];

    k=0;
    for (j=0; j<3; j++)
    {
        // Ecart type = racine (1/n * somme carrés - moyenne^2)
        ecart_type_ac[j] = sqrt (ac_carre_a1[j] / algo1_count_ac - ac_somme_a1[j]/algo1_count_ac * ac_somme_a1[j] /algo1_count_ac) * 1000;  //
        ecart_type_ac[j] = ecart_type_ac[j] * acc_scale[j] / aRes;
        if (ecart_type_ac[j] > Seuil_vib_a1)
        {
            k=1;
            #if (DEBUG_ACC & 0b10)
                trig_test_acc=1;
                axe_test_acc=j;
            #endif
        }
    }
    // Affichage de l'energie vibratoire
    if (k)
    {
        chaine[0] = 22;  // longueur avec cet octet
        memcpy (chaine+10, ecart_type_ac, 12);
        envoi_enreg_resultat('E', chaine);
    }
}



// Algorithme 2 : Chocs
void algo_chocs(void)
{
    int16_t pente, seuil_pente;
    uint8_t i, j;
    float Y_out;

    seuil_pente = Choc_pente_min * 2 * (FREQ_ECHAN); // conversion de mg/ms en brut/echantillon

    for (i = 0; i < count_FIFO; i++)
    {
        for (j=0; j<3; j++)
        {

            // Filtre passe bande IIR : passe-bas 5Hz

                Y_out =  - F_b1_C * YC_1[j]  - F_b2_C * YC_2[j];
                Y_out += (float) ac_data[i][j]*F_a0_C + F_a1_C*XC_1[j] + F_a2_C*XC_2[j] ;

                if ((i<2) && (init_algo2))
                {
                    Y_out = ac_data[i][j];
                    if ((j==2) && (i==1)) init_algo2=0;
                }
                // sauvegarde valeurs précédentes
                XC_2[j] = XC_1[j];
                XC_1[j] = ac_data[i][j];
                YC_2[j] = YC_1[j];
                YC_1[j] = Y_out;

                if ((!i) && (init_algo2))
                    pente = (int16_t) Y_out - YC_2[j];
                else
                    pente=0;

            if (abs((int16_t) Y_out) > 29000)  satur_a2=1;     // proche de la saturation


            if (pente_etat[j])   // 0:pas de choc, 1:premiere pente, 2:attente, 3:deuxième pente
            {
                if (pente_etat[j]==1)   // premiere pente
                {
                    if (pente_sens[j]*pente > seuil_pente)  // pente_sens : 1:pente montante  -1:pente descendante
                    {  // meme sens => on ajoute
                        pente_nb1[j]++;
                        pente_valeur1[j] += pente;
                    }
                    else // fin de premiere pente
                    {
                        // verification que la premiere pente est suffisante
                        if (abs(pente_valeur1[j]) > (Choc_amplitude_min*2) )
                        {
                            if (pente_sens[j]*pente < -seuil_pente)  // changement de sens => etat=3
                            {
                                pente_etat[j]=3;
                                pente_sens[j] = -pente_sens[j];
                                pente_valeur2[j]=pente;
                                pente_nb2[j]=1;
                            }
                            else // fin de premiere pente
                            {
                                 pente_etat[j]=2;  // attente
                                 // Duree max entre le pattern d'acceleration et celui de deceleration
                                 temps_attente[j] = (2000.0 / pente_valeur1[j] * pente_nb1[j] * pente_sens[j]);
                                 pente_id[j] = algo2_count_ac;  // on conserve etat, id, nb, valeur, actif
                            }
                        }
                        else   // remise a zero
                            pente_etat[j] = 0;
                    }
                }
                else if (pente_etat[j]==2)   // attente
                {
                    if (pente_sens[j]*pente < -seuil_pente)  // changement de sens
                    {
                        pente_etat[j]=3;
                        pente_sens[j] = -pente_sens[j];
                        pente_valeur2[j]=pente;
                        pente_nb2[j]=1;
                    }
                    else if (algo2_count_ac > (pente_id[j] + temps_attente[j]))
                    {
                        pente_etat[j]=0;  // arret : choc non confirmé
                    }
                }
                else if (pente_etat[j]==3)   // deuxieme pente
                {
                    if (pente_sens[j]*pente > seuil_pente)
                    {  // meme sens => on ajoute
                        pente_nb2[j]++;
                        pente_valeur2[j] += pente;
                    }
                    else // on arrete : verif du choc
                    {
                        int16_t duree, amplitude;

                        pente_etat[j]=0;
                        // caractérisation du choc
                        duree = ( algo2_count_ac - pente_debut[j] ) * (FREQ_ECHAN);
                        if (abs(pente_valeur2[j]) > (Choc_amplitude_min*2))
                        {
                            float force;
                            // amplitude = max entre l'amplitude montante et la descendante
                            if (abs(pente_valeur2[j]) > abs(pente_valeur1[j]) )
                                amplitude = abs(pente_valeur2[j]);
                            else
                                amplitude = abs(pente_valeur1[j]);
                            force = (float) duree*amplitude;
                            // comparaison avec précédents chocs pour identifier le plus fort
                            if (force > (float) choc_amplitude * choc_duree)
                            {
                                choc_amplitude = amplitude;
                                choc_duree = duree;
                                choc_axe = j;
                            }
                        }
                    }
                }
            }
            if (!pente_etat[j])  // pas actif
            {
                if (abs(pente)>seuil_pente)  // nouveau debut de choc
                {
                    pente_etat[j]=1;
                    if (pente>0) pente_sens[j] = 1; else pente_sens[j]=-1; // 0:pas de choc, 1:pente montante  -1:pente descendante
                    pente_valeur1[j]=pente;
                    pente_nb1[j]=1;
                    pente_debut[j] = algo2_count_ac;
                    #if (DEBUG_ACC & 0b1)
                    // pour debug : enregistrement axe Z uniquement
                    if ( (!test_acc_fige) && (j==2) )
                    {
                        trig_test_acc=1;
                        axe_test_acc=j;
                        uint8_t k;
                        for (k=0; k<count_FIFO; k++)
                        {
                            test_acc[k] = (int16_t) ac_data[k][j];
                        }
                        cpt_test_acc = count_FIFO;
                    }
                    #endif
                }
            }
            ac_precedent[j] =  ac_data[i][j];
        }
        algo2_count_ac++;
    }
}

void chocs_synthese(void)
{
    uint8_t chaine[30];
    float resultat_mpu;


    // Choc superieur au seuil d'energie_min
    if  ( (uint32_t) choc_amplitude /4 *choc_duree > (Choc_energie_min * FREQ_ECHAN) )
    {
        #if (DEBUG_ACC & 0b1)
            if (choc_axe == 2)
                test_acc_fige = 1;
        #endif
        //amplitude = (float) choc_amplitude / aRes * 1000;

    // Affichage des chocs
        chaine[0] = 17;  // longueur avec cet octet puis MSTLttttt
        chaine[10] = choc_axe;
        resultat_mpu = (float) choc_amplitude / aRes * 1000;
        memcpy (chaine+11, &resultat_mpu, 4);
        memcpy (chaine+15, &choc_duree, 2);
        envoi_enreg_resultat('C', chaine);
    }
}

// Algorithme 3 : 3D, quaternion et acceleration
void algo_quaternion(uint8_t * acc_val)
{
    uint8_t retm, index, i;
    uint32_t Now;        // used to calculate integration interval
    float  ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
    float gyr_v[3];
    float deltat;
    float Y_;

    float resultat_mpu[3];

    Now = Clock_getTicks();  // precision 30 micro-second
    if (!lastUpdate)  // 1ere itération
        lastUpdate = Now - 2000;  // moins 20 millisecondes
    deltat = ((Now - lastUpdate)/100000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    // Read_accel
    if (*acc_val)  // utilisation des dernières valeurs lues
    {
        // prend la dernière mesure
        for (i=0; i<3; i++)
            acc_v[i] = (float) (ac_data[count_FIFO-1][i]) / aRes * acc_scale[i];
    }
    else   // lecture acc
    {
        if ( acc_read( acc_v) )  // 0,4ms
        {
            *acc_val = 2;
        }
    }
    // prepare magneto
    SENSOR_SELECT_MAG();  // Magnétometre
    SensorI2C_write_1_Reg(MAG_CNTL1, (Mscale << 4) | MAG_MODE_SINGLE);
    SENSOR_DESELECT();

    //Read gyro
    SENSOR_SELECT();
    SensorI2C_write_1_Reg( PWR_MGMT_2, ALL_AXES);  // Axes : Accel & gyro
    SENSOR_DESELECT();
    DELAY_MS(35);   // de Standby : minimum 5ms  de Sleep: min 29ms
    gyro_read (gyr_v); // 0,4ms
    SENSOR_SELECT();
    SensorI2C_write_1_Reg( PWR_MGMT_2, ACC_AXES);  // Axes : Accel
    SENSOR_DESELECT();

    // read magneto
    retm = Mag_Read( mag_v);  // mode single

    // modification vitesse de recalage après la periode initiale
    if (recalage_initial)
    {
        recalage_initial--;
        if (!recalage_initial)
        {
            GyroMeasERROR(valeur_recalage);  // Vitesse de recalage du quaternion : 3 ideal 40:rapide
            //GyroMeasError = PI * (value / 180.0f);  // 3:lent, 40:rapide
        }
    }
    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
    // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    ax = acc_v[0];
    ay = acc_v[1];
    az = acc_v[2];
    gx = gyr_v[0];
    gy = gyr_v[1];
    gz = gyr_v[2];
    mx = mag_v[0];
    my = mag_v[1];
    mz = -mag_v[2];

    if (!retm)
    {
        MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz, deltat);  // 1ms

        if (timer_apres_debut)  timer_apres_debut--;  // calcul acceleration lineaire pas fait les premieres secondes, pour permettre le calage du module
        if (!timer_apres_debut)
        {
            if ((config_mpu == 12) || (config_mpu == 13))  // calcul enveloppe accel/freinage sans gravite sur l'axe X
            {
                // suppression gravite
                float acc_sans[3];
                acc_sans[0] = acc_v[0];
                acc_sans[1] = acc_v[1];
                acc_sans[2] = acc_v[2];
                remove_gravity (acc_sans);

                // Filtre passe bas quadratique IIR Fc:5Hz  Echant:50Hz

                /*Y_2 = Y_1;
                Y_1 = Y_;
                Y_ =  - F_b1 * Y_1  - F_b2 * Y_2;

                Y_ += (float) X_*F_a0 + F_a1*X_1 + F_a2*X_2 ;*/
                Y_ = acc_sans[0] * 1000;

                //enregistrement (pour debug) des donnees accelero mesurees
                #if (DEBUG_ACC & 0b100)
                    if (trig_test_acc)
                    {
                        test_acc[cpt_test_acc++] = (int16_t) Y_;
                        if ((cpt_test_acc >= TEST_NB) )
                        {
                            // Mettre breakpoint ici puis copier les donnees de test_acc
                            trig_test_acc=0;
                            cpt_test_acc=0;
                        }
                    }
                #endif

                /*strcpy((char *)message+1, " Val X : ");
                ajout_float_multi_valeurs (&Y_out, 0,9, 1, 0);
                envoi_message ();
                DELAY_MS(10);*/

                // fin calcul enveloppe
                if (((accel_X.declench==1) && (Y_ < Declench_fin)) || ((accel_X.declench==2) && (Y_ > -Declench_fin)))
                {
                    //clock_fin = Clock_getTicks();
                    accel_X.declench=0;
                    accel_X.somme = accel_X.somme / accel_X.compteur;
                    // Suppression des accelerations faibles
                    float absolu;
                    uint8_t chaine[30];

                    absolu = accel_X.somme;
                    if (accel_X.somme<0) absolu = -absolu;
                    if ((absolu > Declench_debut*2) && (accel_X.compteur > Duree_acceleration))
                    {
                        // Impression de l'enveloppe acceleration/freinage
                        chaine[0] = 22;  // longueur avec cet octet
                        resultat_mpu[0] = accel_X.somme;
                        resultat_mpu[1] = accel_X.max;
                        resultat_mpu[2] = (float) accel_X.compteur * PERIOD_MESURE * PERIOD_ALGO3;
                        memcpy (chaine+10, resultat_mpu, 12);
                        envoi_enreg_resultat ('A', chaine);
                        #if (DEBUG_ACC & 0b100)
                            trig_test_acc=1;
                        #endif
                    }
                }
                // bébut calcul enveloppe
                if ((!accel_X.declench) && ((Y_ > Declench_debut) ||  (Y_ < -Declench_debut)))
                {
                    accel_X.declench=1;
                    if (Y_<0) accel_X.declench=2;
                    accel_X.somme = 0;
                    accel_X.max =0;
                    accel_X.compteur =0;
                    //clock_deb = Clock_getTicks();
                }

                // enregistrement nouvelle valeur : moyenne et max
                //if (((accel_X.declench==1) && (Y_ > Declench_fin)) || ((accel_X.declench==2) && (Y_ < -Declench_fin)))
                if (accel_X.declench)
                {
                    accel_X.somme += Y_;
                    float absolu1;
                    absolu1 = Y_;
                    if (Y_<0) absolu1 = -Y_;
                    if (accel_X.max < absolu1) accel_X.max = absolu1;
                    //if (accel_X.compteur < 300) tab_acc[accel_X.compteur]= (int16_t) Y_;
                    accel_X.compteur++;
                }
            }
        }

        cpt_envoi_quat++;
        if (cpt_envoi_quat >= Period_envoi_quat)  // Synthese périodique
        {
            cpt_envoi_quat=0;

            if ((config_mpu == 0) || (config_mpu == 13))  // affichage quaternion pour visu 3D
            {
                uint8_t quat_mvt;
                float quat[4];
                uint8_t chaine[30];

                quat_mvt = 1;
                if (QUAT_MOUVEMENT)  //   identification s'il y a eu un mouvement important
                {
                    float total_mouv;
                    total_mouv=0;
                    for (i=0; i<4; i++)
                    {
                        total_mouv += fabs(q_prec[i]-q[i]);
                        q_prec[i] = q[i];
                    }
                    if (total_mouv < SEUIL_AFFICH_MOUV ) quat_mvt  = 0;
                }

                if (quat_mvt)  //   n'envoie que s'il y a un mouvement important (si activé)
                {
                    quat[0]=q[1]*1000;
                    quat[1]=q[2]*1000;
                    quat[2]=q[3]*1000;
                    quat[3]=q[0]*1000;

                    chaine[0] = 21;  // longueur avec cet octet
                    memcpy(chaine+5, &quat[0], 16);
                    envoi_enreg_resultat('Q', chaine);
                }
            }

            if (config_mpu ==1)  // affichage angles en degrés
            {
                calcule_angle();       // 0.4ms
                strcpy((char *)message+1, "Angle:");
                index = ajout_float_multi_valeurs (angle, 0, 7, 3, 0);
                envoi_message ();
            }
            if (config_mpu ==2)  // affichage gyro en rad/s
            {
                strcpy((char *)message+1, "Gyros:");
                index = ajout_float_multi_valeurs (gyr_v, 2, 7, 3, 0);
                envoi_message ();
            }
            if (config_mpu ==3)  // affichage accelero en milli-g avec et sans gravite
            {
                float acc_sans[3], acc_milig[3];
                acc_sans[0] = acc_v[0];
                acc_sans[1] = acc_v[1];
                acc_sans[2] = acc_v[2];
                remove_gravity (acc_sans);

                acc_sans[0] *= 1000;  // sans gravite, en milli-g
                acc_sans[1] *= 1000;
                acc_sans[2] *= 1000;
                acc_milig[0] = acc_v[0]*1000;
                acc_milig[1] = acc_v[1]*1000;
                acc_milig[2] = acc_v[2]*1000;
                strcpy((char *)message+1, "Accel:");
                index = ajout_float_multi_valeurs (acc_milig, 0, 7, 3, 0);
                index = ajout_float_multi_valeurs (acc_sans, 0, index, 3, 0);
                envoi_message ();
            }
            if (config_mpu ==4)  // affichage magneto en milli-gauss
            {
                strcpy((char *)message+1, "Magne:");
                index = ajout_float_multi_valeurs (mag_v, 0, 7, 3, 0);
                envoi_message ();
            }

            if (config_mpu == 5)  // Affichage valeurs brutes
            {
                int16_t valeurs[3];
                uint8_t mode_mpu;
                mode_mpu=0;                 // 0:accel  1:gyro  2:magneto
                lecture_valeurs_brutes(mode_mpu, valeurs);
                strcpy((char *)message, "R brut:  ");
                if (!mode_mpu) message[7]='A';
                else if (mode_mpu==1) message[7]='G'; else message[7]='M';
                index = ajout_int16_multi_valeurs (valeurs, 9, 3, 0);
                envoi_message ();  // 7ms
            }
        }
    }
}


// Application de la config pour la clock de l'algo 4
void config_algo4_vib (void)
{
    Clock_stop(Algo4ClockHandle);
    if ( algo_mpu & BV(3) )  // Activation
    {
        // Redemarrage clock pour future mesure
        Period_Algo4_cpt_h = 0;
        if ((PERIOD_ALGO4_H) || (PERIOD_ALGO4_D))  // Si la periode n'est pas nulle => lance clock, sinon one-shot
        {
            Clock_setTimeout(Algo4ClockHandle, Period_Algo4_D * 15 * (1000 * 1000 / Clock_tickPeriod));
            Clock_setPeriod(Algo4ClockHandle, 0 ); // Period=0 pour un one-shot
            Clock_start(Algo4ClockHandle);
        }
        else   // one-shot, au bout de 2 secondes
        {
            Clock_setTimeout(Algo4ClockHandle, 2 * (1000 * 1000 / Clock_tickPeriod));
            Clock_setPeriod(Algo4ClockHandle, 0 ); // Period=0 pour un one-shot
            Clock_start(Algo4ClockHandle);
        }
    }
}



// 0:alim par connecteur   1:alim par pile
void chgt_alim_pile(uint8_t pile)
{
    if ((pile) )  // alim par pile
    {
        Alim_pile = 1;
        calib_magneto_pile();  // prise en compte des parametres magnetiques de la pile
        #ifdef Uart2
            //Uart2_close();
        #endif
    }
    if ((!pile) )  // alim par connecteur
    {
        Alim_pile = 0;
        calib_magneto_pile();
        #ifdef Uart2
            //Uart2_Open();
        #endif
    }
}



// Eteint <-> Actif
void increment_mode_actif_veille(void)  // bouton 0 court
{
    uint8_t new;

    if (!mode_actif)
        new = 4;
    else
        new = 0;
    /*new = mode_actif + 1;
    if (new == 1) new = 4;
    if (new > 4) new = 0;*/
    application_mode_actif_veille(new);
}



// Reception message (par radio ou liaison serie)
void traitement_message_sensor_mpu(char unsigned longueur_m)
{
    // messages reçus :

    if ((message_in[2] == 'C') && (message_in[3] == 'M'))
    {
        if (message_in[4] == 'A')
        {
            if (message_in[5] == '1') //  CMA1 - Paramètres Algo 1 (Energie vibratoire)
            {
                if ((message_in[6] == 'S') && (longueur_m==11))      // CMA1S  Seuil de Vibration minimum
                {
                    Seuil_vib_a1 = decodage16_message_in(4, 7);  // 4 chiffres, 7°position
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'D')  && (longueur_m==10))      // CMA1D  Duree de totalisation de l'energie vibratoire
                {
                    Duree_algo1 = (uint8_t) decodage16_message_in( 3, 7);  // 3 chiffres, 7°position
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
            }
            if (message_in[5] == '2') //  CMA2 - Paramètres Algo 2 (chocs)
            {
                if ((message_in[6] == 'D')  && (longueur_m==10))      // CMA2D  Duree de totalisation des chocs
                {
                    Duree_algo2 = (uint8_t) decodage16_message_in( 3, 7);  // 3 chiffres, 7°position
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'P') && (longueur_m==10))      // CMA2P  Pente minimum de detection
                {
                    Choc_pente_min = decodage16_message_in( 3, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'A') && (longueur_m==11))      // CMA2A  Amplitude minimum de detection
                {
                    Choc_amplitude_min = decodage16_message_in( 4, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'E') && (longueur_m==12))      // CMA2E  Energie minimum de detection
                {
                    Choc_energie_min = decodage16_message_in(5, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
            }
            if (message_in[5] == '3') //  CMA3 - Paramètres Algo 3 (Quaternion, Acceleration)
            {
                if ((message_in[6] == 'P') && (longueur_m==10))      // CMA3P  Periode d'affichage / envoi en ms
                {
                    Period_algo3 = (uint8_t) decodage16_message_in( 3, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'S') && (longueur_m==10))      // CMA3S  Seuil de declenchement d'enveloppe acceleration
                {
                    Declench_debut = (uint8_t) decodage16_message_in( 3, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'F') && (longueur_m==10))      // CMA3F  Fin de déclencement d'enveloppe acceleration
                {
                    Declench_fin = (uint8_t) decodage16_message_in( 3, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'D') && (longueur_m==10))      // CMA3D  Duree min de detection acceleration
                {
                    Duree_acceleration = decodage16_message_in( 3, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
            }
            if (message_in[5] == '4') //  CMA4 - Paramètres Algo 4 (Vibrations)
            {
                if ((message_in[6] == 'O') && (longueur_m==8))      // CMA4O  Activation/désactivation Algo 4
                {
                    uint8_t i = message_in[7]-'0';
                    if ((!i) || (i==1))   // 0 ou 1
                    {
                        algo_mpu = (algo_mpu & 0x7) | (i<<3) ;
                        config_algo4_vib();
                        ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                    }
                }
                if ((message_in[6] == 'A') && (longueur_m==8))      // CMA4Ax  Axe selectionne pour la mesure
                {
                    uint8_t i = message_in[7]-'0';
                    if  (i<4)   // 0, 1, 2 ou 3
                    {
                        Axe_Mesure = i;
                        ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                    }
                }
                if ((message_in[6] == 'P') && (longueur_m==11))      // CMA4P    Period de mesure Algo 4
                {
                    uint16_t valeur = decodage16_message_in( 4, 7);   // Unite : 15 secondes
                    Period_Algo4_H = valeur / 240 ; // Nombre d'heures
                    Period_Algo4_D = valeur - Period_Algo4_H * 240; // Nm de periodes de 15 secondes
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'S') && (longueur_m==10))      // CMA4S    Seuil de detection valeur efficace Algo 4
                {
                    uint16_t valeur = decodage16_message_in( 3, 7);   // Unite : 1 milli-g
                    Seuil_Algo4 = valeur  ;
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
            }
        }

        if ((message_in[5] == 'A') && (message_in[6] == 'Z')) //  CMEAZ - Enregistrement donnes brutes accelero Axe Z
        {
            if ((message_in[4]=='E') &&  (longueur_m==8))   // CMEAZ0  DésActivation    CMEAZ1  Activation
                Enreg_acc_z = message_in[7]-'0';
            if ((message_in[4]=='L') &&  (longueur_m==7))   // CMLAZ   Lecture
            {
                strcpy((char *)message+1, "CMLAZx");
                message[6] = Enreg_acc_z+'0';
                envoi_message ();
            }
        }


        if (message_in[4] == 'D')    // CMDLPx  : filtre passe-bas de l'accelerometre
        {
            if ((message_in[5] == 'L') && (message_in[6] == 'P') && (longueur_m==8))
            {
                uint8_t i = message_in[7]-'0';
                if  (i<7)   // 0 à 6
                {
                    Dlpf_accel = i;
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
            }
        }

        if (message_in[4] == 'V')
        {
            if ((message_in[5] == 'S') && (longueur_m==9)) //  CMVS - Seuil en milli-g de mise en veille
            {
                Seuil_Wom = decodage16_message_in( 3, 6);
                ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
            }
            if ((message_in[5] == 'D') )                         //  CMVD - Durée pour mise en veille
            {
                if ((message_in[6] == '1') && (longueur_m==11)) //  CMVD1 - Durée pour mise en veille 1
                {
                    Duree_Actif = decodage16_message_in( 4, 7) * (1000 / PERIOD_MESURE );  // unite sec
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == '2') && (longueur_m==11)) //  CMVD2 - Durée pour mise en veille 2
                {
                    Duree_veille1 = decodage16_message_in( 4, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == '3') && (longueur_m==11)) //  CMVD3 - Durée pour mise en veille 3
                {
                    Duree_veille2 = decodage16_message_in( 4, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
                if ((message_in[6] == 'A') && (longueur_m==11)) //  CMVDA - Durée pour mise en arret / stop
                {
                    Duree_veille3 = decodage16_message_in( 4, 7);
                    ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                }
            }
        }

        if (message_in[4] == 'M') //  CMMx  Changement mode d'acquisition données
        {
            uint8_t mode_capt = (message_in[5]-'0')*10 + message_in[6]-'0';
            if ((mode_capt < 14) && (longueur_m==7))  // 00:quaternion, 10:choc,...
            {
                config_type_mpu(mode_capt);  // application de la configuration sélectionnée
                ACTIV_SORTIE (0, 1 , 8);     // 1 flash court
            }
        }
        if (message_in[4] == 'A') //  CMAx  Changement mode actif_veille   0:arret   1:actif
        {
            if ((message_in[5] >= '0') && (message_in[5] < '2') && (longueur_m==6))
            {
                application_mode_actif_veille((message_in[5] - '0')*4);  // 0 ou 4
            }
        }
        if (message_in[4] == 'T') //  CMTx  Changement mode
        {
            if ((message_in[5] >= '0') && (message_in[5] < '3') && (longueur_m==6))  // 0:liaison série  1:Radio RF   2:enreg Flash
            {
                mode_affich_enreg = message_in[5] - '0';
                #ifndef MODE_SILENCE
                if (!mode_affich_enreg)
                    ACTIV_PWM ( 0,1, 10, 2000,3);  // 1 bip long
                else
                    ACTIV_PWM ( 0,4, 6*mode_affich_enreg, 2000, 3);  // 1-2 bip court
                #endif
            }
        }
        if (message_in[4] == 'H') //  CMHx  Changement mode hexa-texte   0:hexa  1:texte
        {
            if ((message_in[5] >= '0') && (message_in[5] < '2') && (longueur_m==6))
            {
                mode_hexa_texte = message_in[5] - '0';
                ACTIV_SORTIE (0, 2 , 6*(mode_hexa_texte+1));
            }
        }
        if ((message_in[4] == 'C') && (message_in[5] == 'A') && (message_in[6] == 'L') && (longueur_m==8)) //  CMCALx  Calibration
        {
            if (message_in[7] == 'A')    // CMCALA  calibration accelero
            {
                ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                calibration_accel();
            }
            if (message_in[7] == 'G')    // CMCALG  calibration gyro
            {
                ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                calibration_gyro();
            }
            if (message_in[7] == 'M')    // CMCALM  calibration magneto
            {
                ACTIV_SORTIE (0, 1 , 2);     // 1 flash court
                calibration_magne();
            }
        }
        // deja utilisé : A, C, H, M, T, V
        if (message_in[4] == 'Z') //  CMZxx  Test
        {
            if (longueur_m==7)
            {
                val_test = decodage16_message_in( 2, 5);
                ACTIV_SORTIE (0, 1 , 2);
            }
        }
    }
}


#define NB_MODULES  4   // Nb de modules calibres dans le code

// active les donnees de calibration du magnetometre, en fonction du parametre pile (avec pile ou sans pile)
// cette correction permet de tenir compte de l'influence magnetique de la présence de la pile et du devpack
void calib_magneto_pile(void)
{
    uint8_t  part_id = Part_Identif();

    // COEFF CALIBRATION MAGNETOMETRE : 1er chiffre :sans pile, 2e chiffre:avec pile
    int16_t const OFFSET_MAG_X[NB_MODULES*2]={139, -21, -66, 13, -52, -208, -49, 40};
    int16_t const OFFSET_MAG_Y[NB_MODULES*2]={-30, 160, 155, 152, 40, 24, 166, -1365};
    int16_t const OFFSET_MAG_Z[NB_MODULES*2]={-116, -183, 149, 140, 284, 514, 72, -610};
    float const SCALE_MAG_X[NB_MODULES*2]={1.012, 1.013, 1.016, 1.0085, 1.011, 1.016, 1.009, 1.048};
    float const SCALE_MAG_Y[NB_MODULES*2]={1.024, 1.021, 1.041, 1.0040, 1.023, 1.445, 1.031, 1.419};
    float const SCALE_MAG_Z[NB_MODULES*2]={0.966, 0.969, 0.948, 0.9877, 0.967, 0.755, 0.9626, 0.746};

    // Get magnetometer calibration from AK8963 ROM
    // Paris : Latitude : 48.8534 Nord  Longitude : 2.3488 Est
    // F=48,1uT  H=20,9uT  Z=43,3uT   X=20,9uT  Y=0,3uT  D=0,71°  I=64,2°
    mag_bias[0] = OFFSET_MAG_X[part_id*2 + Alim_pile];
    mag_bias[1] = OFFSET_MAG_Y[part_id*2 + Alim_pile];
    mag_bias[2] = OFFSET_MAG_Z[part_id*2 + Alim_pile];
    mag_scale[0]= SCALE_MAG_X[part_id*2 + Alim_pile];
    mag_scale[1]= SCALE_MAG_Y[part_id*2 + Alim_pile];
    mag_scale[2]= SCALE_MAG_Z[part_id*2 + Alim_pile];

}


void Calibration_initiale_mpu(void)
{

    // Chaque device calibré est numerote de 0 à xx

   // COEFF CALIBRATION ACCELEROMETRE
   int16_t const OFFSET_ACC_X[NB_MODULES]={2524, 458, 398, 140};
   int16_t const OFFSET_ACC_Y[NB_MODULES]={1458, 532, 250, 218};
   int16_t const OFFSET_ACC_Z[NB_MODULES]={-3110, 118, -578, 474};
   float const SCALE_ACC_X[NB_MODULES]={0.999, 1.003, 0.996, 0.996};
   float const SCALE_ACC_Y[NB_MODULES]={0.995, 0.998, 0.998, 0.996};
   float const SCALE_ACC_Z[NB_MODULES]={0.979, 0.985, 0.9818, 0.9834};

   // COEFF CALIBRATION GYROSCOPE
   int16_t const OFFSET_GYR_X[NB_MODULES]={-8, 65, 150, -76};
   int16_t const OFFSET_GYR_Y[NB_MODULES]={-43, -17, -62, -38};
   int16_t const OFFSET_GYR_Z[NB_MODULES]={-66, -9, -22, 6};


   uint8_t  part_id = Part_Identif();

   // Calibration Gyroscope
     G_OFFSET[0] = OFFSET_GYR_X[part_id];
     G_OFFSET[1] = OFFSET_GYR_Y[part_id];
     G_OFFSET[2] = OFFSET_GYR_Z[part_id];
     Calibration_restore_mpu();


   // Calibration Accelero
     acc_bias[0] = OFFSET_ACC_X[part_id];
     acc_bias[1] = OFFSET_ACC_Y[part_id];
     acc_bias[2] = OFFSET_ACC_Z[part_id];
     acc_scale[0]= SCALE_ACC_X[part_id];
     acc_scale[1]= SCALE_ACC_Y[part_id];
     acc_scale[2]= SCALE_ACC_Z[part_id];

     calib_magneto_pile();
}

// restoration de la calibration apres power-off
void Calibration_restore_mpu(void)
{
   // Calibration Gyroscope
    int8_t data[6];
    data[0] = (G_OFFSET[0]/2  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (G_OFFSET[0]/2)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (G_OFFSET[1]/2  >> 8) & 0xFF;
    data[3] = (G_OFFSET[1]/2)       & 0xFF;
    data[4] = (G_OFFSET[2]/2  >> 8) & 0xFF;
    data[5] = (G_OFFSET[2]/2)       & 0xFF;
    // Push gyro biases to hardware registers
    SENSOR_SELECT();
    SensorI2C_writeReg( XG_OFFSET_H, (uint8_t *) data,6);
    data[0] = 12;
    SensorI2C_readReg( XG_OFFSET_H, (uint8_t *) data,6);
    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorMpu9250SetBypass
*
* @brief       Allow the I2C bus to control the magnetomoter
*
* @return      true if success
*/
static bool sensorMpu9250SetBypass(void)
{
    bool success;
    uint8_t val;

    if (SENSOR_SELECT())
    {
        val = BIT_BYPASS_EN | BIT_LATCH_EN;
        success = SensorI2C_writeReg(INT_PIN_CFG, &val, 1);
        DELAY_MS(10);

        SENSOR_DESELECT();
    }
    else
    {
        success = false;
    }

    return success;
}


#endif

/*******************************************************************************
* @fn          SensorMpu9250_init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool SensorMpu9250_init(void)
{
    // Pins used by MPU
    hMpuPin = PIN_open(&pinGpioState, MpuPinTable);

    // Register MPU interrupt
    PIN_registerIntCb(hMpuPin, SensorMpu9250_Callback);

    // Application callback initially NULL
    isrCallbackFn = NULL;

    return SensorMpu9250_reset();
}


/*******************************************************************************
* @fn          SensorMpu9250_reset
*
* @brief       This function resets the MPU
*
* @return      True if success
*/
bool SensorMpu9250_reset(void)
{
    uint8_t val;
    bool ret;

    //beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
    //zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    //accRange = ACC_RANGE_2G; //ACC_RANGE_INVALID;
    //mpuConfig = 0;   // All axes off

    #ifdef CAPTEUR_MPU
        magStatus = 0;
    #endif

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Device reset
    val = 0x80;
    SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    SENSOR_DESELECT();

    DELAY_MS(100);

    ret = SensorMpu9250_test();
    if (ret)
    {
        // Initial configuration
        #ifdef CAPTEUR_MPU
            SensorMpu9250_accSetRange(ACC_RANGE_2G);
            sensorMagInit();
        #endif

        // Power save
        sensorMpuSleep();
    }

    return ret;
}





/*******************************************************************************
* @fn          sensorMpuSleep
*
* @brief       Place the MPU in low power mode
*
* @return
*/
void sensorMpuSleep(void)
{
    bool success;
    uint8_t val;

    //ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }

    val = NO_AXES;  // All axes disabled
    success = SensorI2C_writeReg(PWR_MGMT_2, &val, 1);
    if (success)
    {
        val = MPU_SLEEP;
        success = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    }

    SENSOR_DESELECT();

    SENSOR_SELECT_MAG();  // Magnétometre
    val = 0;
    SensorI2C_writeReg(MAG_CNTL1, &val, 1);  // sleep
    SENSOR_DESELECT();

}





/*******************************************************************************
* @fn          SensorMpu9250_powerOff
*
* @brief       This function turns off the power supply to MPU9250
*
* @return      none
*/
void SensorMpu9250_powerOff(void)
{

    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    // Force an access on I2C bus #0 (sets the I2C lines to a defined state)
    SensorOpt3001_test();  // true=sucess

    // Turn off power supply
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

}

/*******************************************************************************
 *  @fn         SensorMpu9250_Callback
 *
 *  Interrupt service routine for the MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void SensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == Board_MPU_INT)
    {
        if (isrCallbackFn != NULL)
        {
            isrCallbackFn();
        }
    }
}

/*******************************************************************************
 * @fn          SensorMpu9250_test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorMpu9250_test(void)
{
    uint8_t val;
    static bool first = true;

    //ST_ASSERT(SensorMpu9250_powerIsOn());

    // Select Gyro/Accelerometer
    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Make sure power is ramped up
    if (first)
    {
        DELAY_MS(100);
        first = false;
    }


    // Check the WHO AM I register
    ST_ASSERT(SensorI2C_readReg(WHO_AM_I, &val, 1));
    ST_ASSERT(val == 0x71);

    SENSOR_DESELECT();

    return true;
}


#ifdef CAPTEUR_MPU

/*******************************************************************************
* @fn          SensorMpu9250_accSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*
* @return      true if write succeeded
*/
bool SensorMpu9250_accSetRange(uint8_t newRange)
{
    bool success;

    // if (newRange == accRange) return true;

    //ST_ASSERT(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    accRangeReg = (newRange << 3);

    // Apply the range
    success = SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1);
    SENSOR_DESELECT();

    if (success)
    {
        accRange = newRange;
        getAres();
    }

    return success;
}

/*******************************************************************************
* @fn          sensorMagInit
*
* @brief       Initialize the compass
*
* @return      none
*/
static void sensorMagInit(void)
{
    uint8_t val;
    //ST_ASSERT_V(SensorMpu9250_powerIsOn());

    // Bypass Enable and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master

    if (!sensorMpu9250SetBypass())
    {
        return;
    }

    if (SENSOR_SELECT_MAG())
    {
        static uint8_t rawData[3];

        // Enter Fuse ROM access mode
        val = MAG_MODE_FUSE;
        SensorI2C_writeReg(MAG_CNTL1, &val, 1);
        DELAY_MS(10);

        // Get calibration data
        if (SensorI2C_readReg(MAG_ASAX, &rawData[0], 3))
        {
            // Return x-axis sensitivity adjustment values, etc.
            mag_cal[0] =  (int16_t)rawData[0] + 128;
            mag_cal[1] =  (int16_t)rawData[1] + 128;
            mag_cal[2] =  (int16_t)rawData[2] + 128;
        }

        // Turn off the sensor by doing a reset
        val = 0x01;
        SensorI2C_writeReg(MAG_CNTL2, &val, 1);

        SENSOR_DESELECT();
    }
}




/*******************************************************************************
 * @fn          sensorMagEnable
 *
 * @brief       Enable or disable the compass part of the MPU9250
 *
 * @return      none
 */
static void sensorMagEnable(bool enable)
{
    uint8_t val;

    ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (!sensorMpu9250SetBypass())
    {
        return;
    }

    if (SENSOR_SELECT_MAG())
    {
        if (enable)
        {
            // Set magnetometer data resolution and sample ODR
            val = (Mscale << 4) | Mmode;
        }
        else
        {
            // Power down magnetometer
            val = 0x00;
        }
        SensorI2C_writeReg(MAG_CNTL1, &val, 1);

        SENSOR_DESELECT();
    }
}

/*******************************************************************************
 * @fn          SensorMpu9250_magTest
 *
 * @brief       Run a magnetometer self test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorMpu9250_magTest(void)
{
    uint8_t val;
    //ST_ASSERT(SensorMpu9250_powerIsOn());

    // Connect magnetometer internally in MPU9250
    sensorMpu9250SetBypass();

    // Select magnetometer
    SENSOR_SELECT_MAG();

    // Check the WHO AM I register
    val = 0xFF;
    ST_ASSERT(SensorI2C_readReg(MAG_WHO_AM_I, &val, 1));
    ST_ASSERT(val == MAG_DEVICE_ID);

    SENSOR_DESELECT();

    return true;
}



/*******************************************************************************
* @fn          sensorMpu9250WakeUp
*
* @brief       Exit low power mode, restore config
*
* @return      none
*/
void SensorMpu9250_WakeUp(void)
{
    SENSOR_SELECT();

    SensorI2C_write_1_Reg(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    DELAY_MS(100); // Wait for all registers to reset

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
     SensorI2C_write_1_Reg( PWR_MGMT_1, 0x01);
     DELAY_MS(200);

 //   SensorI2C_write_1_Reg(PWR_MGMT_1, MPU_WAKE_UP);

    // All axis initially disabled
    SensorI2C_write_1_Reg(PWR_MGMT_2, sel_axes);
    //mpuConfig = 0;

    // Restore the range - utile ??
    SensorI2C_write_1_Reg(ACCEL_CONFIG, accRange << 3);

    // Clear interrupts
    SensorI2C_read_1_Reg(INT_STATUS);

    // sensorMagInit();

    //sensorMpu9250SetBypass();

    SENSOR_DESELECT();
}



/*******************************************************************************
* @fn          SensorMpu9250_powerIsOn
*
* @brief       Return 'true' if MPU power is on
*
* @return      state of MPU power
*/
bool SensorMpu9250_powerIsOn(void)
{
    return PIN_getOutputValue(Board_MPU_POWER) == Board_MPU_POWER_ON;
}

/*******************************************************************************
* @fn          SensorMpu9250_registerCallback
*
* @brief       Register a call-back for interrupt processing
*
* @return      none
*/
void SensorMpu9250_registerCallback(SensorMpu9250CallbackFn_t pfn)
{
    isrCallbackFn = pfn;
}

// Interruption generée par capteur MPU
//  1:Data_ready,   0x40:Motion,   autre:Fifo overflow or Fsync
void interrupt_mpu9250_Callback(void)  // lecture ok pour MPU 9250
{
        Event_post(sensorEventHandle, EVENT_INTERRUPT_MPU);
}





/*******************************************************************************
* @fn          SensorMpu9250_enableWom
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool SensorMpu9250_enableWom(uint8_t threshold, uint8_t frequence)
{
    uint8_t val;

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Make sure accelerometer is running
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));
    DELAY_MS(10);

    // ?? SMPLRT_DIV à remettre à zéro ?

    // Enable accelerometer, disable gyro
    val = 0x07;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));
    DELAY_MS(10);

    // Set Accel LPF setting to 184 Hz Bandwidth
    val = 0x01;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG_2, &val, 1));
    DELAY_MS(10);

    // Enable Motion Interrupt
    val = BIT_WOM_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));
    DELAY_MS(10);

    // Enable Accel Hardware Intelligence
    val = 0xC0;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));
    DELAY_MS(10);

    // Set Motion Threshold
    val = threshold;
    ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));
    DELAY_MS(10);

    // Set Frequency of Wake-up
    val = frequence; //INV_LPA_20HZ;
    ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));
    DELAY_MS(10);

    // Enable Cycle Mode (Accel Low Power Mode)
    val = 0x29;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));
    DELAY_MS(10);

    // Select the current range
    //accRangeReg = 0 << 3;
    //ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1));

    // Clear interrupt
    SensorI2C_readReg(INT_STATUS,&val,1);
    DELAY_MS(10);

    SENSOR_DESELECT();

    // Enable pin for wake-on-motion interrupt
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);

    return true;
}


// Disable WOM
void SensorMpu9250_disableWom(void)
{
    // Disable Accel Hardware Intelligence
    SensorI2C_write_1_Reg(ACCEL_INTEL_CTRL, 0);

    // Clear Motion Threshold
    SensorI2C_write_1_Reg(WOM_THR, 0);

    // Clear Frequency of Wake-up
    SensorI2C_write_1_Reg(LP_ACCEL_ODR, 0);
}


/*******************************************************************************
* @fn          SensorMpu9250_irqStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t SensorMpu9250_irqStatus(void)
{
    uint8_t intStatus;

    intStatus = 0;
    ST_ASSERT(SensorMpu9250_powerIsOn());

    if (SENSOR_SELECT())
    {
        if (!SensorI2C_readReg(INT_STATUS,&intStatus,1))
        {
            intStatus = 0;
        }
        SENSOR_DESELECT();
    }

    return intStatus;
}

void GyroMeasERROR(uint16_t value)
{
    GyroMeasError = PI * (value / 180.0f);  // 3:lent, 40:rapide
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta

}



/*******************************************************************************
* @fn          SensorMpu9250_accReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*/
uint8_t SensorMpu9250_accReadRange(void)
{
    ST_ASSERT(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Apply the range
    SensorI2C_readReg(ACCEL_CONFIG, &accRangeReg, 1);
    SENSOR_DESELECT();

    return (accRangeReg>>3) & 3;
}

/*******************************************************************************
* @fn          SensorMpu9250_magStatus
*
* @brief       Return the magnetometer status
*
* @return      mag status
*/
uint8_t SensorMpu9250_magStatus(void)
{
    return magStatus;
}



void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

/*******************************************************************************
 * @fn          SensorMpu9250_gyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      none
 ******************************************************************************/
float SensorMpu9250_gyroConvert(int16_t data)
{
    //-- calculate rotation, unit deg/s, range -250, +250
    return (data * 1.0) / (65536 / 500);
}



void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}




void getAres() {
  switch (accRange)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case ACC_RANGE_2G:
          aRes = 32768/2;
          break;
    case ACC_RANGE_4G:
          aRes = 32768/4;
          break;
    case ACC_RANGE_8G:
          aRes = 32768/8;
          break;
    case ACC_RANGE_16G:
          aRes = 32768/16;
          break;
  }
  acc_rapport = aRes / 256;
}



// ---------------------------------------------------------------------
//          READ FUNCTIONS
//-----------------------------------------------------------------------


// Read Temperature ------------------------

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  SENSOR_SELECT();
  SensorI2C_readReg(TEMP_OUT_H,  &rawData[0],2);  // Read the two raw data registers sequentially into data array
  SENSOR_DESELECT();
  return ((int16_t) ((rawData[0] << 8) | rawData[1])) ;  // Turn the MSB and LSB into a 16-bit value
}


// lecture valeurs brutes du capteur MPU9250
//paramètre : mode_mpu : 0:accelero  1:gyro  2:magneto
void lecture_valeurs_brutes(uint8_t mode_mpu, int16_t *valeurs)
{
    uint8_t j, rawData[7], val;
    int16_t idata16[3];

    if (!mode_mpu)  // accelero
    {
        SENSOR_SELECT();
        val = SensorI2C_readReg(ACCEL_XOUT_H, (uint8_t*)rawData, DATA_SIZE);
        for (j=0; j<3; j++)
        {
            //if (val)
                idata16[j] = (int16_t) ((rawData[2*j] << 8) | rawData[1+2*j]);
            /*else
                idata16|j] = 0;*/
        }
    }
    if (mode_mpu==1)  // gyro
    {
        SENSOR_SELECT();
        SensorI2C_readReg(GYRO_XOUT_H, (uint8_t*)rawData, DATA_SIZE);
        for (j=0; j<3; j++)
        {
            idata16[j] = (int16_t) ((rawData[2*j] << 8) | rawData[1+2*j]);
        }
    }
    if (mode_mpu==2)  // magneto
    {
        SENSOR_SELECT_MAG();
        SensorI2C_readReg(MAG_ST1,&val,1);
        if (val & 0x01)    // Check magnetometer data ready bit
        {
            SensorI2C_readReg(MAG_XOUT_L, &rawData[0],7);  // Burst read of all compass values + ST2 register
            if (!(rawData[6] & 0x08))  // Check ST2 if magnetic sensor overflow set, if not report data
            {
                // Turn the MSB and LSB into a signed 16-bit value,
                // data stored as little Endian
                idata16[0] = (int16_t) ((rawData[1] << 8) | rawData[0]);
                idata16[1] = (int16_t) ((rawData[3] << 8) | rawData[2]);
                idata16[2] = (int16_t) ((rawData[5] << 8) | rawData[4]);
            }
        }
    }
    SENSOR_DESELECT();
    for (j=0;j<3;j++)
    {
        valeurs[j] = idata16[j];
    }
}



// Accel read  -> retourne acc_v , en g
// Retur True if success
// Prise en compte calibration (biais et scale)
bool acc_read(float * acc_net)
{
    bool success;
    uint8_t ii, udata8[6];
    int16_t idata16[3];

    // Burst read of all accelerometer values
    if (!SENSOR_SELECT()) return false;
    //c = SensorI2C_read_1_Reg( ACCEL_CONFIG_2);  // pour test d'activation

    success = SensorI2C_readReg(ACCEL_XOUT_H, (uint8_t*)udata8, DATA_SIZE);
    SENSOR_DESELECT();

    if (success)
    {
        for (ii=0;ii<3;ii++)
        {
            idata16[ii] = (int16_t) ((udata8[2*ii] << 8) | udata8[1+2*ii]);
            acc_net[ii] = (float) (idata16[ii] - acc_bias[ii]) / aRes * acc_scale[ii];  // save mag biases in G for main program
        }
    }
    return success;
}




// Gyro read  -> retourne gyr_v
bool gyro_read(float * gyr_net)
{
    bool success;
    uint8_t ii, rawData[6];
    int16_t data[3];

    // Burst read of all gyrometer values
    if (!SENSOR_SELECT()) return false;
    success = SensorI2C_readReg(GYRO_XOUT_H, (uint8_t*)rawData, DATA_SIZE);
    SENSOR_DESELECT();

    if (success)
    {
        for (ii=0;ii<3;ii++)
        {
            data[ii] = ((int16_t) rawData[2*ii] << 8) | rawData[1+2*ii];
            //-- calculate rotation, unit deg/s, range -250, +250
            gyr_net[ii] = (data[ii] * 1.0) / (65536 / 500);
        }
    }
    return success;
}


//lecture d'une valeur de magnetometre en mode continu
// Nota : activer Mmode avant
uint8_t Mag_Read(float * mag_net)
{
    // Mag_cal : 298  301  289
    uint8_t ii, ret;
    int16_t mag_temp[3];

    ret = SensorMpu9250_magRead_quick(mag_temp);  // Read the mag data
    if (!ret)
    {
        for (ii=0;ii<3;ii++)
        {
            mag_net[ii] = (float) ((mag_temp[ii]- mag_bias[ii]) *mRes*mag_cal[ii] / 256 * mag_scale[ii]) ;  // save mag biases in G for main program
        }
    }
    return ret;
}




/*******************************************************************************
* @fn          SensorMpu9250_magRead
*
* @brief       Read data from the compass - X, Y, Z - 3 words
*
* @return      Magnetometer status
*/
uint8_t SensorMpu9250_magRead(int16_t *data)
{
    uint8_t val;
    uint8_t rawData[7];  // x/y/z compass register data, ST2 register stored here,
    // must read ST2 at end of data acquisition
    magStatus = MAG_NO_POWER;
    ST_ASSERT(SensorMpu9250_powerIsOn());

    magStatus = MAG_STATUS_OK;

    // Connect magnetometer internally in MPU9250 - normalement deja fait
    SENSOR_SELECT();
    val = BIT_BYPASS_EN | BIT_LATCH_EN;
    if (!SensorI2C_writeReg(INT_PIN_CFG, &val, 1))
    {
        magStatus = MAG_BYPASS_FAIL;
    }
    SENSOR_DESELECT();

    if (magStatus != MAG_STATUS_OK)
    {
        return false;
    }

    // Select this sensor
    SENSOR_SELECT_MAG();

    if (SensorI2C_readReg(MAG_ST1,&val,1))
    {
        // Check magnetometer data ready bit
        if (val & 0x01)
        {
            // Burst read of all compass values + ST2 register
            if (SensorI2C_readReg(MAG_XOUT_L, &rawData[0],7))
            {
                val = rawData[6]; // ST2 register to finish read cycle

                // Check if magnetic sensor overflow set, if not report data
                if(!(val & 0x08))
                {
                    // Turn the MSB and LSB into a signed 16-bit value,
                    // data stored as little Endian
                    data[0] = (int16_t) ((rawData[1] << 8) | rawData[0]);
                    data[1] = (int16_t) ((rawData[3] << 8) | rawData[2]);
                    data[2] = (int16_t) ((rawData[5] << 8) | rawData[4]);

                    // Sensitivity adjustment
                    /*data[0] = data[0] * mag_cal[0] >> 8;
                    data[1] = data[1] * mag_cal[1] >> 8;
                    data[2] = data[2] * mag_cal[2] >> 8;*/
                }
                else
                {
                    magStatus = MAG_OVERFLOW;
                }
            }
            else
            {
                magStatus = MAG_READ_DATA_ERR;
            }
        }
        else
        {
            magStatus = MAG_DATA_NOT_RDY;
        }
    }
    else
    {
        magStatus = MAG_READ_ST_ERR;
    }

    // Set magnetometer data resolution and sample ODR
    // Start new conversion
    //val = (Mscale << 4) | Mmode;
    //SensorI2C_writeReg(MAG_CNTL1, &val, 1);

    SENSOR_DESELECT();

    return magStatus;
}

// lecture magnetometre rapide
uint8_t SensorMpu9250_magRead_quick(int16_t *data)
{
    uint8_t val;
    uint8_t rawData[7];  // x/y/z compass register data, ST2 register stored here,
    // must read ST2 at end of data acquisition
    magStatus = MAG_STATUS_OK;

    // Select this sensor
    SENSOR_SELECT_MAG();

    if (SensorI2C_readReg(MAG_ST1,&val,1))
    {
        // Check magnetometer data ready bit
        if (val & 0x01)
        {
            // Burst read of all compass values + ST2 register
            if (SensorI2C_readReg(MAG_XOUT_L, &rawData[0],7))
            {
                val = rawData[6]; // ST2 register to finish read cycle

                // Check if magnetic sensor overflow set, if not report data
                if(!(val & 0x08))
                {
                    // Turn the MSB and LSB into a signed 16-bit value,
                    // data stored as little Endian
                    data[0] = (int16_t) ((rawData[1] << 8) | rawData[0]);
                    data[1] = (int16_t) ((rawData[3] << 8) | rawData[2]);
                    data[2] = (int16_t) ((rawData[5] << 8) | rawData[4]);

                    // Sensitivity adjustment
                    /*data[0] = data[0] * mag_cal[0] >> 8;
                    data[1] = data[1] * mag_cal[1] >> 8;
                    data[2] = data[2] * mag_cal[2] >> 8;*/
                }
                else
                {
                    magStatus = MAG_OVERFLOW;
                }
            }
            else
            {
                magStatus = MAG_READ_DATA_ERR;
            }
        }
        else
        {
            magStatus = MAG_DATA_NOT_RDY;
        }
    }
    else
    {
        magStatus = MAG_READ_ST_ERR;
    }

    // Set magnetometer data resolution and sample ODR
    // Start new conversion
    //val = (Mscale << 4) | Mmode;
    //SensorI2C_writeReg(MAG_CNTL1, &val, 1);

    SENSOR_DESELECT();

    return magStatus;
}


// ---------------------------------------------------------------------
//          CALIBRATION
//-----------------------------------------------------------------------



// Protocole de calibration accelerometre : position vers le bas pendant 1 sec,
// successivement sur chacun des 6 axes
// print : 3 valeurs min et 3 valeurs max (divide by 8 to get 2048 LSB per g)
void calibration_accel(void)
{
    int16_t calib[6];
    uint8_t mode_preced, index;

    strcpy((char *)message+1, " Calib accelero : stable sur 6 positions");
    envoi_message ();
    DELAY_MS(500);

    mode_preced = mode_actif;
    if (mode_actif != 4 ) application_mode_actif_veille(4);

    calib_accelero(calib);

    Config_mpu(&mpu_params);        // Restore configuration
    if (mode_preced != 4) application_mode_actif_veille(mode_preced);

    strcpy((char *)message+1, " Cal acc : min-max");
    ajout_int16_multi_valeurs (calib, 11, 6, 0);
    envoi_message ();
    DELAY_MS(80);

    strcpy((char *)message+1, "Acc:biais-scale  ");
    index = ajout_int16_multi_valeurs (acc_bias, 17, 3, 0);  // Ecart type apres calib
    index = ajout_float_multi_valeurs (acc_scale, 3, index, 3, 0);  // Ecart type apres calib
    envoi_message ();
    DELAY_MS(80);

}



// Calibration des accelerometres, par la méthode des min et max
// Protocole de calibration :  positionner le capteur vers le bas pendant 1 seconde (immobile) successivement
// sur chacun des 6 axes
// données de sortie : min max sur chaque axe et enregistrement des 3 biais et 3 gains
void calib_accelero (int16_t *calib)
{
#define NB_MESURES  10   // Nombre de groupe de 40 mesures pour identifier que le module est immobile
#define SEUIL_ACC   9000 // Seuil de variance en amplitude en dessous duquel le module est immobile
#define SEUIL_GYRO  5000 // Seuil de variance en amplitude en dessous duquel le module est immobile

    uint8_t j, ii,  packet_count, data[12], valid_tous, valid_axe[6], immobile,  axe_cpt;
    uint16_t compteur_accel;
    int8_t axe_ok, axe_prec;
    int16_t ac_temp[3],  gyro_temp[3];
    int16_t max[3] = {-20000, -20000, -20000}, min[3] = {20000, 20000, 20000};
    float  variance_acc[3], variance_gyro[3], acc_max[3];
    double somme_acc[3], somme_gyr[3], somme_carre_acc[3], somme_carre_gyr[3];

    SensorMpu9250_WakeUp();    // wake up device

    SENSOR_SELECT();

    SensorI2C_write_1_Reg( PWR_MGMT_2, ALL_AXES);  // Axes : Accel & gyro

    // Configure MPU6050  accelerometer for bias calculation
    SensorI2C_write_1_Reg( CONFIG, 0x01);      // Set low-pass filter to 6=5Hz (ou 1:188 Hz)
    SensorI2C_write_1_Reg( SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    SensorI2C_write_1_Reg( GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    SensorI2C_write_1_Reg( ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    SensorI2C_write_1_Reg( ACCEL_CONFIG_2, 1); // Fchoice=1, Set accelerometer rate to 1 kHz and bandwidth to 190 Hz

    SensorI2C_write_1_Reg( INT_ENABLE, 0x00);   // Disable all interrupts
    SensorI2C_write_1_Reg( FIFO_EN, 0x00);      // Disable FIFO
    SensorI2C_write_1_Reg( PWR_MGMT_1, 0x00);   // Turn on internal clock source
    SensorI2C_write_1_Reg( USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    SensorI2C_write_1_Reg( USER_CTRL, 0x0C);    // Reset FIFO and DMP
    DELAY_MS(15);

    for (j=0; j<6; j++)
    {
        valid_axe[j]=0;
    }
    valid_tous=0;
    axe_cpt=0;
    axe_prec=0;
    compteur_accel=0;

    while (!valid_tous)    // chaque repetition dure 40 millisecondes
    {
        // Configure FIFO to capture accelerometer data
        SensorI2C_write_1_Reg( USER_CTRL, 0x40);   // Enable FIFO
        SensorI2C_write_1_Reg( FIFO_EN, 0x78);     // Enable accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
        DELAY_MS(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

      // At end of sample accumulation, turn off FIFO sensor read
        SensorI2C_write_1_Reg( FIFO_EN, 0x00);        // Disable accelerometer sensors for FIFO
        SensorI2C_readReg(FIFO_COUNT_H,  &data[0], 2); // read FIFO sample count
        packet_count = (((uint16_t)data[0] << 8) | data[1] ) / 12;    // How many sets of full accelerometer data for averaging

        if ((packet_count > 35) && (packet_count < 45))
        {
            for (j=0; j<3; j++) {
                somme_acc[j] = 0;
                somme_carre_acc[j] = 0;
                somme_gyr[j] = 0;
                somme_carre_gyr[j] = 0;
            }
            for (ii = 0; ii < packet_count; ii++)   // durée=27ms pour 80*3 valeurs
            {
                SensorI2C_readReg( FIFO_R_W,  &data[0],12); // read data
                for (j=0; j<3; j++) {
                    ac_temp[j] = (int16_t) ((data[j*2] << 8) | data[1+2*j]  ) ;  // Form signed 16-bit integer for each sample in FIFO
                    somme_acc[j]  += ac_temp[j];
                    somme_carre_acc[j] += (double) ac_temp[j] * ac_temp[j];

                    gyro_temp[j] = ((int16_t) ((data[6+j*2] << 8) | data[7+2*j]));
                    somme_gyr[j]  += gyro_temp[j];
                    somme_carre_gyr[j] += (double) gyro_temp[j] * gyro_temp[j];
                }
            }

            // Calcul du mouvement global du module
            immobile = 1;
            axe_ok = 0;
            for (j=0; j<3; j++) {
                variance_acc[j]  = somme_carre_acc[j]  / packet_count - somme_acc[j]/packet_count * somme_acc[j] /packet_count;
                if ( variance_acc[j] > SEUIL_ACC) immobile=0;
                if (somme_acc[j]/packet_count > 11000 )    axe_ok = j+1;
                if (somme_acc[j]/packet_count < -11000 )   axe_ok = -j-1;
                acc_max[j] = somme_acc[j]/packet_count;
                if (acc_max[j]>0)
                    acc_max[j] += sqrt(variance_acc[j]);
                else
                    acc_max[j] -= sqrt(variance_acc[j]);

                variance_gyro[j] = somme_carre_gyr[j] / packet_count - somme_gyr[j]/packet_count * somme_gyr[j] /packet_count;
                if ( variance_gyro[j] > SEUIL_GYRO) immobile=0;

                compteur_accel++;
                if (!(compteur_accel%6))  // affichage d'1 sur 6
                /*if (message[0]!='L')  // Affichage des valeurs en continu si on n'est pas en liaison RF*/
                {
                    strcpy((char *)message+1, " Cal acc :   ");
                    if (immobile) message[11]='I';
                    ajout_float_multi_valeurs (acc_max, 1, 13, 3, 0);
                    envoi_message ();
                }
                DELAY_MS(40);
            }

            // si le module est immobile pendant 1 groupe de mesure (40ms), vérification que pendant 20 groupes de mesure, le module reste immobile
            if  ((immobile) && (axe_ok))
            {
                if ((axe_ok == axe_prec) || (!axe_prec))
                {
                    axe_cpt++;
                    if (axe_cpt > NB_MESURES)  // immobile pendant 1 seconde (20 lectures) sur le meme axe
                    {
                        ACTIV_SORTIE (0,1,5);    // 1 flash long
                        #ifndef MODE_SILENCE
                        ACTIV_PWM ( 0,1,5,2000,3);  // 1 bip long
                        #endif
                        strcpy((char *)message+1, "Validation Calibration ------ ");
                        envoi_message ();
                        DELAY_MS(1000);
                        if (axe_ok < 0)
                        {
                            min[-axe_ok-1] = somme_acc[-axe_ok-1]/packet_count - sqrt(variance_acc[-axe_ok-1]);
                            valid_axe[-axe_ok+2]= 1;  // 3 à 5
                            axe_cpt = 0;
                        }
                        else
                        {
                            max[axe_ok-1] = somme_acc[axe_ok-1]/packet_count + sqrt(variance_acc[axe_ok-1]);
                            valid_axe[axe_ok-1]=1;
                            axe_cpt = 0;
                        }
                    }
                    axe_prec = axe_ok;
                }
                else
                {
                    axe_prec = 0;
                    axe_cpt = 0;
                }
            }
            else
            {
                axe_prec = 0;
                axe_cpt = 0;
            }
         }
        //  Vérification s'il reste des axes a etalonner
        valid_tous=1;
        for (j=0; j<6; j++)
        {
            if (!valid_axe[j]) valid_tous=0;
        }
    }
    SENSOR_DESELECT();

    DELAY_MS(500);
    ACTIV_SORTIE (0,2,10);    // bips
    #ifndef MODE_SILENCE
    ACTIV_PWM ( 0,2,10,2000,3);  //  bips
    #endif
    DELAY_MS(1000);

    acc_bias[0]  = (max[0] + min[0])/2;  // get average x bias in counts
    acc_bias[1]  = (max[1] + min[1])/2;  // get average y bias in counts
    acc_bias[2]  = (max[2] + min[2])/2;  // get average z bias in counts

    acc_scale[0]  = 16384.0 *2 /(max[0] - min[0]);  // get average x bias in counts
    acc_scale[1]  = 16384.0 *2 /(max[1] - min[1]);  // get average y bias in counts
    acc_scale[2]  = 16384.0 *2 /(max[2] - min[2]);  // get average z bias in counts

    for (j=0; j<3; j++)
    {
        calib[j]= min[j];
        calib[j+3]= max[j];
    }
}




// Immobile,  pendant 40 milli seconde
// print : Ecart type  + 3 valeurs gyro (Divide by 4 to get 32.9 LSB per deg/s)
void calibration_gyro(void)
{
    int16_t calib[4];
    uint8_t mode_preced;

    strcpy((char *)message+1, " Calib gyro : immobile");
    envoi_message ();
    DELAY_MS(500);

    mode_preced = mode_actif;
    if (mode_actif != 4 ) application_mode_actif_veille(4);

    calibrate_gyro_MPU9250(calib);  // Calibrate gyro, load biases in bias registers
    Config_mpu(&mpu_params);        // Restore configuration
    if (mode_preced != 4) application_mode_actif_veille(mode_preced);

    strcpy((char *)message+1, "Calib-gyro:         ");
    ajout_int16_multi_valeurs (calib, 11, 4, 0);
    envoi_message ();

}


// Function which accumulates gyro data . It calculates the average
// of the at-rest readings and then loads the resulting offsets into gyro bias registers.
// Lecture de 40 données (pendant 40ms) et moyenne de celles-ci
// Le module doit être immobile
// Taille 1000 octets
void calibrate_gyro_MPU9250(int16_t * calib)
{
  uint8_t j, data[6]; // data array to hold gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t ac_gy_bias[3]  = {0,0,0};
  double gy_carre[3]  = {0,0,0};
  double variance;

  SENSOR_SELECT();
 // reset device
  SensorI2C_write_1_Reg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  DELAY_MS(100);

 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  SensorI2C_write_1_Reg( PWR_MGMT_1, 0x01);
  SensorI2C_write_1_Reg( PWR_MGMT_2, 0x00);
  DELAY_MS(200);

// Configure device for bias calculation
  SensorI2C_write_1_Reg( INT_ENABLE, 0x00);   // Disable all interrupts
  SensorI2C_write_1_Reg( FIFO_EN, 0x00);      // Disable FIFO
  SensorI2C_write_1_Reg( PWR_MGMT_1, 0x00);   // Turn on internal clock source
  //SensorI2C_write_1_Reg( 0x24, 0x00); // Disable I2C master I2C_MST_CTRL - pas pour sensortag ?
  SensorI2C_write_1_Reg( USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  SensorI2C_write_1_Reg( USER_CTRL, 0x0C);    // Reset FIFO and DMP
  DELAY_MS(15);

// Configure MPU6050 gyro for bias calculation
  SensorI2C_write_1_Reg( CONFIG, 6);      // Set low-pass filter to 188 Hz
  SensorI2C_write_1_Reg( SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  SensorI2C_write_1_Reg( GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity

//  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec

  // Configure FIFO to capture gyro data for bias calculation
  SensorI2C_write_1_Reg( USER_CTRL, 0x40);   // Enable FIFO
  SensorI2C_write_1_Reg( FIFO_EN, 0x70);     // Enable gyro sensors for FIFO  (max size 512 bytes in MPU-9250)
  DELAY_MS(80); // accumulate 80 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  SensorI2C_write_1_Reg( FIFO_EN, 0x00);        // Disable gyro sensors for FIFO
  SensorI2C_readReg(FIFO_COUNT_H,  &data[0], 2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/6;// How many sets of full gyro data for averaging

  for (ii = 0; ii < packet_count; ii++)
  {
        int16_t ac_gy_temp[3] ;
        SensorI2C_readReg( FIFO_R_W,  &data[0],6); // read data for averaging
        for (j=0; j<3; j++) {  // Taille:124 octets
            ac_gy_temp[j] = (int16_t) ((data[j*2] << 8) | data[1+2*j]  ) ;  // Form signed 16-bit integer for each sample in FIFO
            ac_gy_bias[j] += (int32_t) ac_gy_temp[j]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            gy_carre[j] += (double) ac_gy_temp[j] * ac_gy_temp[j];  // Somme des carres
        }
   }

  SENSOR_DESELECT();

  variance = 0;
  for (j=0; j<3; j++) {
      ac_gy_bias[j] /= (int32_t) -packet_count* 1.5; // Normalize sums to get average count biases
      variance += (gy_carre[j]/packet_count);
  }


  // calcul variance
  calib[0] = sqrt (variance - ( ac_gy_bias[0]*ac_gy_bias[0] + ac_gy_bias[1]*ac_gy_bias[1] + ac_gy_bias[2]*ac_gy_bias[2]));

  for (j=0; j<3; j++) {   // bias gyro
      calib[j+1] = (int16_t) ac_gy_bias[j];
  }

  G_OFFSET[0] = ac_gy_bias[0];
  G_OFFSET[1] = ac_gy_bias[1];
  G_OFFSET[2] = ac_gy_bias[2];

  Calibration_restore_mpu();

}


// Faire un mouvement en 8 pendant 15 secondes
// Affiche les résultats  min-max
void calibration_magne(void)
{
    uint8_t mode_preced, index;

    strcpy((char *)message+1, " Calib Magneto : figure en 8");
    envoi_message ();
    DELAY_MS(500);

    mode_preced = mode_actif;
    if (mode_actif != 4 )
    {
        application_mode_actif_veille(4);
    }
    Clock_stop(SensTimerClockHandle);

    Mmode = MAG_MODE_CONT2; // 100 Hz continu
    sensorMagEnable(1);  // enable (Mscale-16bits & Mmode)

    // calibration magnetometre
    int16_t mag_data[6];  // min et max;
    magcalMPU9250(mag_data);  // 118 248 -133  1,03 1,03 0,946
     //  95  275 -147    1,00    1,02    0,97

    // Restoration de la configuration
    Config_mpu(&mpu_params);        // Restore configuration
    if (mode_preced != 4)  // si avant le module etait en veille
        application_mode_actif_veille(mode_preced);
    else
        Clock_start(SensTimerClockHandle);

    DELAY_MS(10); // Min:100us

    /*strcpy((char *)message+1, "Calib-Mag: ");
    index = ajout_int16_multi_valeurs (mag_bias, 12, 3, 0);
    ajout_float_multi_valeurs (mag_scale, 2, index, 3, 0);
    envoi_message ();*/
    strcpy((char *)message+1, "Mag:biais-scale  ");
    index = ajout_int16_multi_valeurs (mag_bias, 17, 3, 0);  // Ecart type apres calib
    index = ajout_float_multi_valeurs (mag_scale, 3, index, 3, 0);  // Ecart type apres calib
    envoi_message ();
    DELAY_MS(50);
}


// Calibration magnetometre : faire une figure en 8
// param min-max
// Les résultats de calibration sont enregistrés dans mag_bias et mag_scale
void magcalMPU9250(int16_t * mag_data)
{
     uint16_t ii = 0, sample_count = 0;
     uint8_t jj, retc, index;
     int16_t mag_tmp[3];
     int16_t mag_max[3] = {-2000, -2000, -2000}, mag_min[3] = {2000, 2000, 2000};
     //int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

     //Serial.println("Mag Calibration: Wave device in a figure eight until done!");

     DELAY_MS(200);  // first setup
     sample_count = 2000;  // 20 secondes
     for(ii = 0; ii < sample_count; ii++)
     {
         DELAY_MS(12);  // at 8 Hz ODR, new mag data is available every 125 ms (100Hz, 10ms)
         retc = SensorMpu9250_magRead(mag_tmp);  // Read the mag data
         if (!retc) {
             for ( jj = 0; jj < 3; jj++)
             {
                  if(mag_tmp[jj] > mag_max[jj]) mag_max[jj] = mag_tmp[jj];
                  if(mag_tmp[jj] < mag_min[jj]) mag_min[jj] = mag_tmp[jj];
                  // X : -246 à 228          -286 à 225
                  // Y : -20 à 456           -121 à 399
                  // Z : -74 à 428           -110  à 416
             }
             if (!(ii % 40) )  // affichage 1 chaque 500ms
             {
                 strcpy((char *)message+1, "Min-max  ");
                 index = ajout_int16_multi_valeurs (mag_min, 9, 3, 0); // valeur min et max
                 index = ajout_int16_multi_valeurs (mag_max, index, 3, 0); // valeur min et max
                 envoi_message ();
             }
         }
         else
             retc++;
     }

    // Get hard iron correction
     mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
     mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
     mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    /* dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
     dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
     dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];*/

   /*  dest1[0] = (float) mag_bias[0]*mRes*mag_cal[0] / 256;  // save mag biases in G for main program
     dest1[1] = (float) mag_bias[1]*mRes*mag_cal[1] / 256;
     dest1[2] = (float) mag_bias[2]*mRes*mag_cal[2] / 256;*/

    // Get soft iron correction estimate
     mag_tmp[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
     mag_tmp[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
     mag_tmp[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

     float avg_rad = mag_tmp[0] + mag_tmp[1] + mag_tmp[2];
     avg_rad /= 3.0;

     mag_scale[0] = avg_rad/((float)mag_tmp[0]);
     mag_scale[1] = avg_rad/((float)mag_tmp[1]);
     mag_scale[2] = avg_rad/((float)mag_tmp[2]);


     for (ii=0; ii<3; ii++)
     {
         mag_data[ii]   = mag_min[ii];
         mag_data[ii+3] = mag_max[ii];
     }
}


// ---------------------------------------------------------------------
//          VIBRATIONS ET FFT FUNCTIONS
//-----------------------------------------------------------------------


// Lit accelero pendant 400ms, et calcul en continu la moyenne et la valeur efficace
// Sortie :
// ecart_type : valeur efficace sur chaque axe
// satur : 0:OK,  1 si donnees d'accelero proche de la saturation


void lecture_vibrations(vibra_out *out)
{
    uint8_t packet_count, ii, j, data[6], satur, repet;
    int16_t p_count, ac_int, ac_data[83][3];
    double moyenne[3],  ac_carre[3];
    double ac_float ;

    SENSOR_SELECT();

    uint8_t val1;
    SensorI2C_readReg(INT_STATUS,&val1,1);   // Clear interrupt
    SensorI2C_readReg(INT_ENABLE,&val1,1);   // Clear interrupt


    //SensorI2C_write_1_Reg( FIFO_EN, 0x00);      // Disable FIFO
    //SensorI2C_write_1_Reg( PWR_MGMT_1, 0x00);   // Turn on internal clock source
    //SensorI2C_write_1_Reg( USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    SensorI2C_write_1_Reg( USER_CTRL, 0x0C);    // Reset FIFO and DMP
    //DELAY_MS(15);

    p_count=0;
    satur=0;
    for (j=0; j<3;j++)
    {
        moyenne[j]=0;
        ac_carre[j]=0;
    }

    for (repet=0; repet <1; repet++)  // chaque repetition dure 80 milliseconde
    {
        // Configure FIFO to capture accelerometer data
        SensorI2C_write_1_Reg( USER_CTRL, 0x40);   // Enable FIFO
        SensorI2C_write_1_Reg( FIFO_EN, 0x08);     // Enable accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
        DELAY_MS(80); // accumulate 80 samples in 80 milliseconds = 480 bytes

      // At end of sample accumulation, turn off FIFO sensor read
        SensorI2C_write_1_Reg( FIFO_EN, 0x00);        // Disable accelerometer sensors for FIFO
        SensorI2C_readReg(FIFO_COUNT_H,  &data[0], 2); // read FIFO sample count
        packet_count = (((uint16_t)data[0] << 8) | data[1] ) /6;    // How many sets of full accelerometer data for averaging

        if ((packet_count < 83) && (packet_count > 70))
        {
            for (ii = 0; ii < packet_count; ii++)   // durée=27ms pour 80*3 valeurs
            {
                  SensorI2C_readReg( FIFO_R_W,  &data[0],6); // read data for record and averaging
                  for (j=0; j<3; j++)
                  {
                      ac_int = (int16_t) ((data[2*j] << 8) | data[1+2*j]  ) ;  // Form signed 16-bit integer for each sample in FIFO
                      ac_data[ii][j] = ac_int;  // enregistrement de la dernière repetition pour analyse en mode debug
                      ac_float =  (double) (ac_int ) * acc_scale[j] / aRes;  // in g - acc_bias[0]
                      moyenne[j]  += ac_float;
                      ac_carre[j] += ac_float * ac_float;
                      if ((ac_int > 27000) || (ac_int < -27000))
                          satur=1;                        // proche de la saturation
                  }
            }
            p_count += packet_count;
        }
    }
    SensorI2C_write_1_Reg( USER_CTRL, 0x00);   // Disable FIFO
    SENSOR_DESELECT();

    out->ecart_type[0] = ac_data[0][0];  // pour ne pas afficher de warning

    for (j=0; j<3; j++)
    {
        //out->moyenne[j] = moyenne[j] / p_count;
        out->ecart_type[j] = sqrt (ac_carre[j] / p_count - moyenne[j]/p_count * moyenne[j] /p_count) * 1000;  //
    }
    out->satur = satur;
}




#ifdef FFT_ON

// Lit accelero pendant 1s à 1kHz (1024 valeurs) puis fait le rapport crete/efficace, et renvoie le ratio de la plus forte crete
// Return : true si ok, false si probleme
// Paramètres d'entrée (params) :
//    gamme de frequence : 5 à 100 Hz => frequence d'échantillonage 10 fois plus elevée - Non exploité
// Données de sortie (out) :
//    statut : 1 si machine en fct, 0 si arret, 2:valeurs accelero saturées (changer de gamme/range)
//    axe : axe le plus fort sélectionné (si axe en input=0)
//    moyenne :  moyenne des valeurs accelero : pas très utile
//    efficace : ecart-type (valeur RMS) par rapport à la moyenne
//    ac_ratio : facteur de crete : doit être entre 1,4 et 2.
//    ac_ratio_kurt : facteur de kurtosis - doit être inferieur a 3
//    FFT_freq : tableau des 5 valeurs de fréquences avec les amplitudes les plus fortes
//    FFT_max  : tableau des 5 valeurs d'amplitudes des fréquences ci-dessus

uint8_t lecture_fft_accelero (lecture_fft_params *fftParams, fft_out *out)
{
    uint8_t success,  j, z, satur=0, data[6]; // data array to hold accelerometer  x, y, z, data
    int16_t ac_data[numSamples],  ac_effi; //  val_new, val_prec1, val_prec2;
    uint16_t ac_crete[3]={0,0,0},  packet_count , ii, k;
    uint32_t  temp_carre;
    float ac_moyenne[3]={0,0,0}, moyenne;
    float  ac_carre[3]={0,0,0};
    float ac_kurtosis[3]={0,0,0};
    float ac_ecart_type[3], max_val=0;
    float fft_value;
    float X_, X_1, X_2;
    float Y_, Y_1, Y_2;

    msp_fft_q15_params params = {
        numSamples,
        true,
        msp_cmplx_bitrev_table_512_ui16,
        msp_cmplx_twiddle_table_512_q15,
        msp_split_table_512_q15
    };

    // Déterminer l'axe ayant la valeur efficace la plus élevée => ok à la fin de la première salve de mesures
    // Déterminer la fréquence fondamentale F0 pour faire coller l'échantillon a un nombre de période entière du signal : pas la peine si nb de periode > 20
    // Faire en 2 mesures : première pour determiner axe , deuxième pour enregistrer valeurs => ok
    // Adapter la fréquence d'échantillonage à la fréquence fondamentale : idéalement F0*10. OK en fonction de la gamme d'entrée
    // Diminuer l'impact d'une crete isoée dans le facteur de crête : OK : min (max de crete sur 50 valeurs)
    // Ponderer les evenements de crete isolés : utiliser le facteur de Kurtosis => ok
    // Vérification du fonctionnement de la machine : OK. Ecart_type > seuil (0,05g)

    out->statut=0;


    if (!SENSOR_SELECT()) return false;

   // reset device
    SensorI2C_write_1_Reg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    DELAY_MS(100);

   // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
   // else use the internal oscillator, bits 2:0 = 001
    SensorI2C_write_1_Reg( PWR_MGMT_1, 0x01);
    SensorI2C_write_1_Reg( PWR_MGMT_2, 0x07);  // 0x07  Accelerometer On, Gyro off
    DELAY_MS(200);

  // Configure device
    //SensorI2C_write_1_Reg( INT_ENABLE, 0x00);   // Disable all interrupts
    SensorI2C_write_1_Reg( FIFO_EN, 0x00);      // Disable FIFO
    SensorI2C_write_1_Reg( PWR_MGMT_1, 0x00);   // Turn on internal clock source
    SensorI2C_write_1_Reg( USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    SensorI2C_write_1_Reg( USER_CTRL, 0x0C);    // Reset FIFO and DMP
    DELAY_MS(15);

  // Configure MPU6050  accelerometer
    //SensorI2C_write_1_Reg( CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    SensorI2C_write_1_Reg( SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    //SensorI2C_write_1_Reg( GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    SensorI2C_write_1_Reg( ACCEL_CONFIG, RANGE_ACC_FFT<<3); // Set accelerometer full-scale to 16 g(3). Diviser par 2048
    SensorI2C_write_1_Reg( ACCEL_CONFIG_2, 0); // = 0x0  Set Accel LPF setting to 218 Hz Bandwidth, 1khz, DLPF, 2ms delay

    // Etape 1 : 80 valeurs lues avec FIFO pour détermination axe le plus fort (ecart_type max)

    // Configure FIFO to capture accelerometer data
    SensorI2C_write_1_Reg( USER_CTRL, 0x40);   // Enable FIFO
    SensorI2C_write_1_Reg( FIFO_EN, 0x08);     // Enable accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
    DELAY_MS(80); // accumulate 80 samples in 80 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
    SensorI2C_write_1_Reg( FIFO_EN, 0x00);        // Disable accelerometer sensors for FIFO
    SensorI2C_readReg(FIFO_COUNT_H,  &data[0], 2); // read FIFO sample count
    packet_count = (((uint16_t)data[0] << 8) | data[1] ) /6;    // How many sets of full accelerometer data for averaging

    out->axe = 0;

    if ((packet_count < 83) && (packet_count > 70))
    {
        for (ii = 0; ii < packet_count; ii++)   // durée=27ms pour 80*3 valeurs
        {
              SensorI2C_readReg( FIFO_R_W,  &data[0],6); // read data for record and averaging
              for (j=0; j<3; j++) {
                  ac_effi = (int16_t) ((data[j*2] << 8) | data[1+2*j]  ) ;  // Form signed 16-bit integer for each sample in FIFO
                  ac_data[ii*3+j]  = ac_effi;  // inutile de stocker les valeurs
                  ac_moyenne[j] += ac_effi;
                  if (ac_effi <0) ac_effi = (-ac_effi);               // valeur absolue
                  if (ac_effi >27000)
                      satur=1;                        // proche de la saturation
                  temp_carre = (int32_t) ac_effi * ac_effi;           // Carre des ecarts
                  ac_carre[j] += temp_carre;                          // Somme des carres
              }
        }
        for (j=0; j<3; j++) {
              ac_moyenne[j] /= (float) packet_count;                          // moyenne
              ac_ecart_type[j] = (float) sqrt( ac_carre[j]/packet_count - ac_moyenne[j]*ac_moyenne[j]);    // Ecart type / valeur efficace

              if (ac_ecart_type[j] > max_val) {    // sélectionne la valeur efficace la plus forte
                   max_val = ac_ecart_type[j];
                   out->axe = j;                                          // axe le plus fort
                   if (max_val > fftParams->seuil_marche) out->statut=1;  // machine en marche
               }
        }
        // modifie eventuellement l'axe à mesurer
        if (fftParams->sel_axe)
                out->axe = fftParams->sel_axe - 1;
        moyenne = ac_moyenne[out->axe];

    // Etape 2 : 512 valeurs lues sur l'axe sélectionné

        // Configure MPU6050  accelerometer
          //SensorI2C_write_1_Reg( CONFIG, 0x01);      // Set low-pass filter to 188 Hz
          SensorI2C_write_1_Reg( SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
          SensorI2C_write_1_Reg( ACCEL_CONFIG, RANGE_ACC_FFT<<3); // Set accelerometer full-scale to 16 g(3). Diviser par 2048
          SensorI2C_write_1_Reg( ACCEL_CONFIG_2, 0); // 8:4Khz, 2=90Hz 3=45Hz 4=21Hz 0=220Hz  Set Accel LPF setting to 218 Hz Bandwidth, 1khz, DLPF, 2ms delay
          DELAY_MS(2);

          k=0;
          //val_prec1=0;
          //val_new=0;
          ac_moyenne[0]=0;
          ac_crete[1]=0;   // crete intermediaire
          ac_crete[0]= 60000;
          ac_carre[0]=0;
          ac_kurtosis[0]=0;

          X_1 = 0;
          X_ = 0;
          // float count;
          // count = Clock_getTicks();
          // Lecture data et moyenne  660ms
          for (ii = 0; ii<numSamples; ii++)
          {
              // 4Khz => 187ms  1kHz=> 512ms
              while (!(SensorI2C_read_1_Reg( INT_STATUS) & 1)) { };  // Attente donnée dispo  // TODO rajouter timeout
              //DELAY_MS(1);
              success = SensorI2C_readReg(ACCEL_XOUT_H + 2*out->axe, (uint8_t*)data, 2);
              if (success) {
                  X_2 = X_1;
                  X_1 = X_;
                  X_ =   (int16_t) ((data[0] << 8) | data[1]) -  moyenne ;
                  if (((X_+moyenne) > 27000) || ((X_+moyenne) < -27000))
                      satur=1;                        // proche de la saturation

                  // Filtre passe bande IIR
                  if (fftParams->filtre_PB)
                  {
                      Y_2 = Y_1;
                      Y_1 = Y_;
                      if (ii==0)
                          Y_ = 0;
                      else if (ii==1)
                          Y_ = - F_b1 * Y_1;
                      else
                          Y_ =  - F_b1 * Y_1  - F_b2 * Y_2;

                      Y_ += (float) X_*F_a0 + F_a1*X_1 + F_a2*X_2 ;
                      ac_data[ii] = (int16_t) Y_;
                  }
                  else
                      ac_data[ii] = (int16_t) X_;

                  #ifdef DEBUG_ACC
                     test_acc[cpt_test_acc++] = ac_data[ii];
                     if ( cpt_test_acc >= TEST_NB)
                     {
                         // Mettre breakpoint ici
                         cpt_test_acc=0;
                     }
                   #endif

                   ac_moyenne[0] += ac_data[ii];
              }
          }
          //count = Clock_getTicks() - count;
          SensorI2C_write_1_Reg( PWR_MGMT_2, 0x3F);  // 0x3F  Accelerometer Off, Gyro off

          ac_moyenne[0] /= (float) numSamples;                          // moyenne
          out->moyenne = ac_moyenne[0] + moyenne;

          // calculs ecart type, crete, ratio,...  10ms
          for (ii = 0; ii<numSamples; ii++)
          {
                ac_effi = ac_data[ii] - ac_moyenne[0];
                if (ac_effi <0) ac_effi = (-ac_effi);               // valeur absolue
                temp_carre = (int32_t) ac_effi * ac_effi;           // Carre des ecarts
                ac_carre[0] += (float) temp_carre;                          // Somme des carres / variance
                ac_kurtosis[0] += (float) temp_carre * temp_carre; // Somme des puissances 4
                if (ac_effi > ac_crete[1])
                    ac_crete[1]=ac_effi;     // crete
                if ((fftParams->fact_crete) && (ii) && !(ii%(25<<1)))  {  // chaque 50 valeurs on prend le min de la crete fftParams->fact_crete
                    if (ac_crete[1] < ac_crete[0]) ac_crete[0] = ac_crete[1];
                    ac_crete[1] = 0;
                }
         }
          if (!fftParams->fact_crete) ac_crete[0] = ac_crete[1];  // facteur de crete si on doit prendre le max

          ac_ecart_type[0] = (float) sqrt( ac_carre[0]/numSamples);    // Ecart type / valeur efficace
          out->efficace = ac_ecart_type[0];
          out->ac_ratio = (float) ac_crete[0] / ac_ecart_type[0] ;            // facteur de crete
          out->ac_ratio_kurt = (float) (ac_kurtosis[0] / ac_carre[0] ) / ac_carre[0] * numSamples;   // facteur de Kurtosis


          // Take FFT of ac_data - 5ms
          // Fréquence echantillonage 1kHz avec 256 valeurs => chaque point = 4Hz
          msp_fft_scale_q15(&params, ac_data);

          // Sélection des 5 valeurs les plus fortes (fréquence et amplitude)
          for (j=0; j<Nb_valeur_fft_max; j++) out->fft_max[j] = 0;  // raz amplitudes
          // Filtre passe haut sur FFT : Non prise en compte des fréquences faibles : k<5
          ii= val_min_fft * 2;
          for(k= val_min_fft; k<numSamples/2; k++)   //       0 1 2 3   Samp=4 => Samp/2=2   k=1
          {
              fft_value = (float) ac_data[ii]*ac_data[ii] + (float) ac_data[ii+1]*ac_data[ii+1] ; // Somme partie reelle et imaginaire
              fft_value = sqrt(fft_value) / RANGE_ACC_FFT_coeff * 1000;  // valeur en milli-g
              ii=ii+2;
              for (j=0; j<Nb_valeur_fft_max ;j++)
              {
                  if (fft_value > out->fft_max[j])
                  {
                      //décalage valeurs max et fréquence correspondantes
                      if (j != (Nb_valeur_fft_max-1))
                          for (z=(Nb_valeur_fft_max-1); z>j; z--)  {
                              out->fft_max[z]  = out->fft_max[z-1];
                              out->fft_freq[z] = out->fft_freq[z-1];
                          }
                      out->fft_max[j] = fft_value;  // valeur en milli-g;
                      out->fft_freq[j] =  (float) 1000 / numSamples * k;  // frequence d'échantillonnage 1KHz
                      break;
                  }
              }
          }

    }

    SENSOR_DESELECT();

    if (satur) out->statut=2;                        // valeurs lues proches de la saturation => modifier la range de l'accelero

    return true ;
    // alerte si ratio > 3 ou si valeur_crete > seuil(10g)
}

// lecture de plusieurs FFT (500ms chacunes) pour faire la moyenne des différentes informations.
// Sorties : identiques à lecture_fft, moyennées.
//           + kurt_ecart_type : ecart-type du facteur de Kurtosis (donné en pourcentage de ce facteur)

void lecture_multi_fft (lecture_fft_params *fftParams, fft_mul_out *out)
{
    fft_out fftout;
    uint8_t fft_occ[Nb_valeur_max];
    float fft_val[Nb_valeur_max];
    uint8_t k, i, j, occup=0, position, z;

    // Raz
    out->efficace=0;
    out->ac_ratio=0;
    out->ac_ratio_kurt=0;
    out->kurt_ecart_type=0;

    for (k=0; k< Nb_valeur_max ; k++)
    {
        out->fft_freq[k]=0;
        fft_occ[k]=0;
        out->fft_max[k]=0;
    }

    for (i=0; i<Nb_fft_succ ;i++) // lecture de plusieurs fft successives
    {
        for (j=0; j< Nb_valeur_max ; j++)  // remise à zéro pour chaque série, des amplitudes
        {
            fft_val[j]=0;
        }

        lecture_fft_accelero(fftParams, &fftout);

        out->statut = fftout.statut;
        out->axe = fftout.axe;

        out->efficace += (float) fftout.efficace / RANGE_ACC_FFT_coeff * 1000;  // Valeur efficace en milli-g
        out->ac_ratio += fftout.ac_ratio;
        out->ac_ratio_kurt += fftout.ac_ratio_kurt;
        out->kurt_ecart_type += fftout.ac_ratio_kurt * fftout.ac_ratio_kurt;  // somme des carrés


        for (k=0; k< Nb_valeur_fft_max ; k++)
        {

            // Vérifie si on a déja vu , dans la meme série ou precedente, une frequence proche (donc d'amplitude plus élevée)
            z=0;
            for (position=0; position < Nb_valeur_max ; position++)
            {
                if ((fftout.fft_freq[k] >= out->fft_freq[position]-6) && (fftout.fft_freq[k] <= out->fft_freq[position] +6))
                {
                    z=1;  // valeur proche trouvée
                    break;
                }
            }

            if (z)  // si valeur trouvée ou proche => ajout
            {
                //if (fftout.fft_max[k] > fft_val[position])  // si amplitude est supérieure => garde cette frequence
                //        out->fft_freq[position] = fftout.fft_freq[k];
                fft_val[position] += fftout.fft_max[k];
            }
            else       // nouvelle frequence - rajout s'il reste de la place
            {
                if (occup < Nb_valeur_max)
                {
                    out->fft_freq[occup] = fftout.fft_freq[k];
                    fft_val[occup] = fftout.fft_max[k];
                    occup++;
                }
            }
        }

        // cherche si ces fréquences ont déja été identifiées sur les précédentes séries
        for (j=0; j< Nb_valeur_max ; j++)
        {
            if ( fft_val[j])  // si une valeur non nulle d'amplitude vient d'être identifiée
            {
                if (out->fft_max[j])  // si il y en avait déja une avant
                {
                    out->fft_max[j] += fft_val[j];
                    fft_occ[j]++;
                }
                else  // si cette valeur est nouvelle
                {
                    out->fft_max[j] = fft_val[j];
                    fft_occ[j] = 1;
                }
            }
        }
    }

    for (k=0; k< Nb_valeur_max ; k++)  // moyenne des valeurs amplitude
    {
        if (fft_occ[k])
            out->fft_max[k] /= fft_occ[k];
    }

    out->ac_ratio /= Nb_fft_succ;
    out->ac_ratio_kurt /= Nb_fft_succ;
    out->efficace /= Nb_fft_succ;
    out->kurt_ecart_type  =  (sqrt( out->kurt_ecart_type/Nb_fft_succ - out->ac_ratio_kurt * out->ac_ratio_kurt ) ) / out->ac_ratio_kurt * 100;
}

#endif

#ifdef QUATER_CHOCS


// Calcule angles en degrés à partir du quaternion
void calcule_angle()
{
     // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
     // In this coordinate system, the positive z-axis is down toward Earth.
     // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
     // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
     // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
     // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
     // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
     // applied in the correct order which for this configuration is yaw, pitch, and then roll.
     // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
       yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
       pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
       roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
       pitch *= 180.0f / PI;
       yaw   *= 180.0f / PI;
       yaw   += 0.48; // Déclinaison magnetique Paris 1.34;
       /* Declination at Potheri, Chennail ,India  Model Used:  IGRF12  Help
                                                                  Latitude:    12.823640Â° N
                                                                  Longitude:   80.043518Â° E
                                                                  Date Declination
                                                                  2016-04-09   1.34Â° W  changing by  0.06Â° E per year (+ve for west )*/
       roll  *= 180.0f / PI;
       angle[0] = yaw;   // direction
       angle[1] = roll;  // roulis
       angle[2] = pitch; // descente/tangage
}

void remove_gravity(float *acc)
{
    /*compensate the accelerometer readings from gravity.
    # @param q the quaternion representing the orientation of a 9DOM MARG sensor array
    # @param acc the readings coming from an accelerometer expressed in g
    #
    # @return a 3d vector representing dynamic acceleration expressed in g
    def gravity_compensate(q, acc):*/
   float g[3];

      // get expected direction of gravity
      g[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
      g[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
      g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

      // compensate accelerometer readings with the expected direction of gravity
      acc[0] = acc[0] - g[0];
      acc[1] = acc[1] - g[1];
      acc[2] = acc[2] - g[2];
}

//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
// 19/02/2012   SOH Madgwick    Magnetometer measurement is normalised
//
//=====================================================================================================

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
// Temps de calcul : 0,7ms

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability:W,X,Y,Z
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
            if ((q[0] < -2) || (q[0] > 2))   // erreur : raz quaternion
            {
                q[0] = 1;
                q[1] = 0;
                q[2] = 0;
                q[3] = 0;
            }
}
#endif  // fin FFT

#endif  // fin CAPTEUR_MPU

//====================================================================================================
// END OF CODE
//====================================================================================================

