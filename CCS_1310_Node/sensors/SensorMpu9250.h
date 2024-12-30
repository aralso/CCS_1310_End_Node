/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       SensorMpu9250.h
 *
 *  @brief      Driver for the InvenSense MPU9250 Motion Processing Unit.
 *
 *  ============================================================================
 */
#ifndef SENSOR_MPU9250_H
#define SENSOR_MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                          Includes
 * -----------------------------------------------------------------------------
 */
#include "stdint.h"
#include "stdbool.h"

/* -----------------------------------------------------------------------------
 *                                          Constants
 * -----------------------------------------------------------------------------
 */
/*// Set initial input parameters
enum accRange {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};*/

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

/*enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};*/


// Calcul Quaternion et orientation , fft, choc
#define QUATER_CHOCS
// FFT_MT
//#define FFT_ON   // activation du module FFT et analyse temporel pour machines tournantes

#define numSamples 512          // modifier aussi param FFT
#define Nb_valeur_fft_max 5     // Nb de valeurs conservées apres chaque FFT unitaire (frequence et amplitude)
#define Nb_valeur_max     5     // min:4 Apres moyenne des FFt successives : frequence, occurence, amplitude
#define RANGE_ACC_FFT  0                // 0:2g  1:4g  2:8g  3:16g
#define RANGE_ACC_FFT_coeff  16384.0    // 2g=>16384  16g:2048
#define Nb_fft_succ 10         // Nb de FFT successivement effectuée, avec moyenne des valeurs ensuite
#define val_min_fft 5          // Non prise en compte des fréquences faibles : k<5(10Hz) (filtre passe haut sur FFT

#ifdef QUATER_CHOCS
#define max_data_mpu 18 // données capteur mpu lues par la fonction interruption - multiple de 3

#define NB_DATA_ACC_Z  30  // Nb de données brutes int16 envoyées à chaque message

//#define filter_Low_5Hz
#endif

#define filter_Low_5Hz_250

#ifdef FFT_ON
#define filter_High_15Hz
#endif


// Filtre IIR : http://www.earlevel.com/main/2010/12/20/biquad-calculator/

#ifdef filter_High_15Hz  // Filtre IIR quadratique passe-haut - frequence de coupure 15Hz - Freq echa 1khz Q=0.722
    #define     F_a0  0.936732300298712
    #define     F_a1  -1.873464600597424
    #define     F_a2  0.936732300298712
    #define     F_b1  -1.8692981039614844
    #define     F_b2  0.8776310972333635
#endif
#ifdef filter_Low_6Hz   // Filtre IIR passe bas 5Hz, echantillonage 13Hz Q=0,8071
    #define     F_a0  0.6196851463898615
    #define     F_a1  1.239370292779723
    #define     F_a2  0.6196851463898615
    #define     F_b1  1.061110989543438
    #define     F_b2  0.417629596016008
#endif

#ifdef filter_Low_5Hz   // Filtre IIR passe bas 5Hz, echantillonage 50Hz Q=0,9071
    #define     F_a0  0.07212395839530776
    #define     F_a1  0.14424791679061552
    #define     F_a2  0.07212395839530776
    #define     F_b1  -1.2220879622755618
    #define     F_b2  0.5105837958567928
#endif

#ifdef filter_Low_5Hz_250   // Filtre IIR passe bas 5Hz, echantillonage 250Hz Q=0,7
    #define     F_a0_C  0.00361869062995985
    #define     F_a1_C  0.00723738125991971
    #define     F_a2_C  0.00361869062995985
    #define     F_b1_C  -1.82118969321166
    #define     F_b2_C  0.835664455731508
#endif

#ifdef filter_High_2Hz   // Filtre IIR passe haut 2Hz, echantillonage 1kHz Q=0,7071
    #define     F_a0  0.9911535113858849
    #define     F_a1  -1.9823070227717698
    #define     F_a2  0.9911535113858849
    #define     F_b1  -1.9822287623675816
    #define     F_b2  0.982385283175958
#endif
// Values PWR_MGMT_2
#define ALL_AXES                       0
#define NO_AXES                        0x3F
#define GYRO_AXES                     0x38
#define ACC_AXES                      0x07

// Accelerometer ranges
#define ACC_RANGE_2G      0
#define ACC_RANGE_4G      1
#define ACC_RANGE_8G      2
#define ACC_RANGE_16G     3
#define ACC_RANGE_INVALID 0xFF

// Axis bitmaps
#define MPU_AX_GYR        0x07
#define MPU_AX_ACC        0x38
#define MPU_AX_MAG        0x40
#define MPU_AX_ALL        0x7F

// Interrupt status bit
#define MPU_DATA_READY    0x01
#define MPU_MOVEMENT      0x40

// Magnetometer status
#define MAG_STATUS_OK     0x00
#define MAG_READ_ST_ERR   0x01
#define MAG_DATA_NOT_RDY  0x02
#define MAG_OVERFLOW      0x03
#define MAG_READ_DATA_ERR 0x04
#define MAG_BYPASS_FAIL   0x05
#define MAG_NO_POWER      0x06

// Mode Magnetometer
#define MAG_MODE_OFF                  0x00
#define MAG_MODE_SINGLE               0x01
#define MAG_MODE_CONT1                0x02   // 8 Hz
#define MAG_MODE_CONT2                0x06   // 100 Hz
#define MAG_MODE_FUSE                 0x0F

// Resolution magnetometre
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB



/* ----------------------------------------------------------------------------
 *                                           Typedefs
 * -----------------------------------------------------------------------------
*/
typedef void (*SensorMpu9250CallbackFn_t)(void);

typedef struct config_mpu_params {
    uint8_t mpu_mode;
    uint8_t interruption;
    uint8_t power1;
    uint8_t sel_axes;
    uint8_t smplrt_divis;
    uint8_t acc_odr;
    uint8_t dlpf_accel;
    uint8_t acc_range;
    uint8_t dlpf_gyro;
    uint8_t Gscale;
    uint8_t Mscale;
    uint8_t Mmode;
} config_mpu_params;


typedef struct vib_out {
    uint8_t                  axe;                // axe le plus fort.
    int16_t                  moyenne;           // moyenne
    uint16_t                 efficace;           // valeur efficace / ecart type
    float                    ac_ratio;           // ratio de crete
    float                    ac_ratio_kurt;      // ratio de kurtosis
} vib_out;

typedef struct lecture_fft_params {
    uint8_t                gamme;             // Gamme de frequence  5 à 100 Hz
    uint8_t                seuil_marche;      // seuil en g (à diviser par 100) pour déclarer machine en marche (Eff > seuil/100)
    uint8_t                sel_axe;           // selection axe à mesurer : 0 :auto(le plus fort RMS), sinon 1(X),2(Y) ou 3(Z)
    uint8_t                fact_crete;        // 0:max, 1:min(max sur 50), 2:min(max sur 100)
    uint8_t                filtre_PB;         // Filtre passe-bande : 0 :pas de filtre, 1:Passe-haut 15Hz
} lecture_fft_params;


typedef struct vibra_out {
    float ecart_type[3];
    uint8_t satur;
} vibra_out;

typedef struct Choc_params {
    uint8_t seuil_pente_min; // seuil d'acroissement min de l'acceleration    typ:25 = 50mg/ms
    uint16_t amplitude_min;  // amplitude min de l'acceleration du choc  en milli-g  type:1000mg
    uint16_t energie_min;    // Energie (amplitude*duree/2) min du choc en mg.ms         typ:5000mg.ms
} Choc_params;

typedef struct Choc_out {
    uint8_t axe;        // Axe X(0) Y(1) Z(2)
    float amplitude;    // 0:pas de choc, sinon amplitude en milli-g
    int16_t duree;      // en milli-s
    uint8_t satur;      // 1:saturation
} Choc_out;

typedef struct fft_out {
    uint8_t                  statut;            // 0:machine a l'arret, 1:machine en marche,   2:saturation valeurs accelero
    uint8_t                  axe;                // axe le plus fort (0:X, 1:Y, 2:Z)
    int16_t                  moyenne;           // moyenne
    uint16_t                 efficace;           // valeur efficace / ecart type
    float                    ac_ratio;           // ratio de crete
    float                    ac_ratio_kurt;      // ratio de kurtosis
    float                    fft_max[Nb_valeur_fft_max];         // Amplitudes maximales des plus fortes fréquences
    uint16_t                 fft_freq[Nb_valeur_fft_max];        // Fréquences les plus fortes en Hz
} fft_out;

typedef struct fft_mul_out {      // moyenne sur les multiples fft
    uint8_t                  statut;            // 0:machine a l'arret, 1:machine en marche,  2:saturation valeurs accelero
    uint8_t                  axe;                // axe le plus fort (0:X, 1:Y, 2:Z)
    int16_t                  moyenne;           // moyenne
    float                    efficace;           // valeur efficace / ecart type
    float                    ac_ratio;           // ratio de crete
    float                    ac_ratio_kurt;      // ratio de kurtosis
    float                    kurt_ecart_type;    // pourcentage ecart_type / moyenne du facteur de Kurtosis
    float                    fft_max[Nb_valeur_max];         // Amplitudes maximales des plus fortes fréquences
    uint16_t                 fft_freq[Nb_valeur_max];        // Fréquences les plus fortes en Hz
} fft_mul_out;

/* -----------------------------------------------------------------------------
 *                                          Functions
 * -----------------------------------------------------------------------------
 */

void SensorTask_init(void);
void init_capteur_mpu(void);
uint8_t setup_mpu(void);
void Config_mpu(config_mpu_params *Params);
void config_type_mpu(uint8_t config);

bool SensorMpu9250_init(void);
bool SensorMpu9250_reset(void);
void SensorMpu9250_registerCallback(SensorMpu9250CallbackFn_t);
bool SensorMpu9250_test(void);
void SensorMpu9250_powerOff(void);
bool SensorMpu9250_powerIsOn(void);
void SensorMpu9250_WakeUp(void);
void Activ_sensor_mpu(void);
void Desactiv_sensor_mpu(void);
void chgt_acc_Z(void);

void traitement_message_sensor_mpu(char unsigned longueur_m);

void SensorMpu9250_enable(uint16_t config);
bool SensorMpu9250_enableWom(uint8_t threshold, uint8_t frequence);
uint8_t SensorMpu9250_irqStatus(void);
void traitement_message_capteur_mpu(void);

bool SensorMpu9250_accSetRange(uint8_t range);
uint8_t SensorMpu9250_accReadRange(void);

// ------   READ  -------------------
bool SensorMpu9250_accRead(uint16_t *rawData);
float SensorMpu9250_accConvert(int16_t rawValue);

bool SensorMpu9250_gyroRead(uint16_t *rawData);
float SensorMpu9250_gyroConvert(int16_t rawValue);

bool SensorMpu9250_magTest(void);
uint8_t SensorMpu9250_magRead(int16_t *pRawData);
uint8_t SensorMpu9250_magRead_quick(int16_t *pRawData);
uint8_t SensorMpu9250_magStatus(void);
void SensorMpu9250_magReset(void);
bool gyro_read(float * gyr_net);
bool acc_read(float * acc_net);

void MPU9250SelfTest(float * destination);
void configMPU9250(void);
uint8_t Mag_Read(float * mag_net);
uint8_t Mag_Read_single(float * mag_net);

void application_mode_actif_veille (uint8_t new);
void chgt_mode_algo4(uint8_t newval);
void lancement_algo4(void);

// ----------  CALIBRATION ----------------
void Calibration_initiale_mpu(void);
void calibrateMPU9250(int16_t * calib);      // calib acc et gryo immobile
void calibrate_gyro_MPU9250(int16_t * calib);  // calib gyro - immobile
void calibration_magne(void);
void calibration_accel(void);
void calibration_gyro(void);
void calib_accelero (int16_t *calib);

void increment_mode_actif_veille(void);  // bouton 0 long
void Acc_Read_single(float * acc_net);

void calcule_angle(void);
void calcule_quaternion(void);
void remove_gravity(float *acc);

//  -----------  VIBRATIONS-CHOCS ----------------
uint8_t lecture_accelero(float * ac_ratio, float * ac_efficace);
uint8_t lecture_fft_accelero( lecture_fft_params *fftParams, fft_out *out);
void lecture_multi_fft (lecture_fft_params *fftParams, fft_mul_out *out);
void lecture_vibrations (vibra_out *vb_out);
void lecture_chocs(Choc_params *params, Choc_out *out);
void sensorMpuSleep(void);
void GyroMeasERROR(uint16_t); // vitesse de recalage du quaternion
void restore_config_acc_gyro(void);
void lecture_valeurs_brutes(uint8_t mode_mpu, int16_t *valeurs);

void interrupt_9250(void);


/*******************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif
