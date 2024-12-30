
/*
 *  ======== EEPROM_Emulated.c ========
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "Board.h"

#include <ti/drivers/dpl/HwiP.h>
#include "EEPROM_emu.h"
#include "extflash/ExtFlash.h"
#include <ti/sysbios/knl/Semaphore.h>


uint16_t index_EEPROM;

// extern
extern Semaphore_Handle FlashAccessSemHandle;


//Local prototype
uint8_t EEPROM_valid(void);
uint8_t EEPROM_recherche(void);
uint8_t EEPROM_raz(void);

#ifdef FLASH_EXTERNE

void EEPROM_init(void)
{
    uint8_t ret;

    index_EEPROM = 0xFFFF;  // valeur non définie
    ret = Semaphore_pend(FlashAccessSemHandle, 1000*100); // Semaphore_WAIT_FOREVER   1 seconde
    if (ret)
    {
        ret = ExtFlash_open();
        EEPROM_recherche();
        ExtFlash_close();
        Semaphore_post(FlashAccessSemHandle);
    }
}


/*
 *  ======== EEPROM_read =======
 */
// return : 0 ok, 1:erreur flash 2:erreur adresse  3:timeout semaphore  4:autre erreur
uint8_t EEPROM_read(uint8_t offset, void *buffer, uint8_t bufferSize)
{
    uint8_t retc, ret, cpt;
    uint8_t buff_header[SHEADERSIZE];

    retc = 3;

    ret = Semaphore_pend(FlashAccessSemHandle, 1000*100); // Semaphore_WAIT_FOREVER   1 seconde
    if (ret)
    {
        // Vérification de la validité de l'adresse cible
        if (( (offset+bufferSize) > EEPROMSIZE) || (offset >= EEPROMSIZE) || (bufferSize > EEPROMSIZE) )
        {
            retc = 2;  // erreur adresse
        }
        else
        {
            retc = 4;
            ExtFlash_open();
            cpt=2;
            while (cpt)
            {
                // vérification de la validité de l'index
                ret=0;
                if (EEPROM_valid() )
                {
                    // erreur => recherche de l'index valide
                    ret = EEPROM_recherche();
                }
                if (!ret)
                {
                    // lecture header
                    ret = ExtFlash_read( EEPROM_REGIONS_BASE+index_EEPROM, 1, buff_header, 0x80) ;
                    if ((ret) && (buff_header[0]==0x5F))   // header ok
                    {
                        // lecture données
                        ret = ExtFlash_read( EEPROM_REGIONS_BASE+index_EEPROM+SHEADERSIZE+offset , bufferSize, buffer, 0x80) ;
                        if (ret) { retc=0; cpt=0; }  // OK
                    }
                }
                if (retc)   // erreur de recherche initiale ou de header ou de lecture
                    EEPROM_recherche();
                if (cpt) cpt--;
            }
            ExtFlash_close();
        }
        Semaphore_post(FlashAccessSemHandle);
    }

    return retc;
}


/*
 *  ======== EEPROM_write =======
 */
// return : 0 ok, 1:ok mais avec raz 2: ok mais perte des autres valeurs EEPROM  3:erreur adresse  4:erreur semaphore 5:autre erreur
uint8_t EEPROM_write(uint8_t offset, void *buffer, uint8_t bufferSize, uint8_t flags)
{
    uint8_t retc, ret, cpt, lecture_ok;
    uint8_t buff_read[EEPROMSIZE+SHEADERSIZE];
    uint8_t buff_header[SHEADERSIZE];
    uint16_t index_precedent, index_suivant;
    uint8_t chgt_secteur;

    retc = 4;
    lecture_ok=0;

    ret = Semaphore_pend(FlashAccessSemHandle, 1000*100); // Semaphore_WAIT_FOREVER   1 seconde
    if (ret)
    {
        if (( (offset+bufferSize) > EEPROMSIZE) || (offset >= EEPROMSIZE) || (bufferSize > EEPROMSIZE) )
        {
            retc = 3;
        }
        else
        {
            retc=5;
            ExtFlash_open();
            cpt=2;
            while (cpt)  // 1er passage : avec index initial   2° passage : apres recherche 3° passage apres raz
            {
                // vérification de la validité de l'index
                lecture_ok=0;
                ret=0;
                if (EEPROM_valid() )
                {
                    // erreur => recherche de l'index valide
                    ret = EEPROM_recherche();
                }
                if (ret < 2)
                {
                    // lecture de l'enregistrement EEPROM complet
                    ret = ExtFlash_read( EEPROM_REGIONS_BASE + index_EEPROM, SHEADERSIZE+ EEPROMSIZE, buff_read, 0x80) ;
                    if ((ret) && (buff_read[0]==0x5F))   // header ok
                    {
                        // lecture valide
                        index_precedent = index_EEPROM;
                        lecture_ok = 1;
                        memcpy (buff_read + SHEADERSIZE + offset, buffer, bufferSize);

                        // determination de l'emplacement suivant
                        chgt_secteur=0;
                        if (index_EEPROM + 2*(EEPROMSIZE + SHEADERSIZE) < SECTORSIZE)
                            index_suivant = index_EEPROM + EEPROMSIZE + SHEADERSIZE;  // enreg suivant
                        else
                        {
                            // changement de secteur
                            chgt_secteur=2;  // nouveau
                            if (index_EEPROM >= SECTORSIZE)
                                chgt_secteur=1;
                            index_suivant = SHEADERSIZE + SECTORSIZE*(chgt_secteur-1);
                        }
                        // verification que l'emplacement suivant est vierge
                        ret = ExtFlash_read( EEPROM_REGIONS_BASE + index_suivant, 1, buff_header, 0x80) ;
                        if ((ret) && (buff_header[0]==0xFF))   // header ok
                        {
                            index_EEPROM = index_suivant;
                            // Activation de l'enregistrement nouveau et enregistrement valeurs
                            buff_read[0] = 0x5F;
                            buff_read[1] = 0x5F;
                            ret = ExtFlash_write( EEPROM_REGIONS_BASE + index_EEPROM,  SHEADERSIZE+EEPROMSIZE , (const uint8_t *) buff_read, 0x80);

                            // activation nouveau secteur si necessaire
                            if (chgt_secteur)
                            {
                                // activation header du nouveau secteur
                                buff_header[0] = 0x5F;
                                buff_header[1] = 0x5F;
                                ExtFlash_write( EEPROM_REGIONS_BASE + (chgt_secteur-1)*SECTORSIZE, 2 , (const uint8_t *) buff_header, 0x80);
                            }

                            // desactivation de l'enregistrement ancien
                            buff_header[0] = 0x0F;
                            buff_header[1] = 0x0F;
                            ret += ExtFlash_write( EEPROM_REGIONS_BASE + index_precedent, 2 , (const uint8_t *) buff_header, 0x80);

                            // effacement de l'ancien secteur, si necessaire
                            if (chgt_secteur)
                            {
                                chgt_secteur = 2-chgt_secteur;  // nouv1 -> 1   nouv2 ->0
                                ExtFlash_erase( EEPROM_REGIONS_BASE + chgt_secteur*SECTORSIZE, SECTORSIZE);  // secteur x de 4096 octets   80ms 9mA
                            }
                            cpt=0;
                            retc=0;
                        }
                    }
                }
                if (retc)   // erreur de recherche initiale ou de header ou de lecture ou du nouveau secteur/enreg
                    EEPROM_recherche();
                if (cpt) cpt--;
            }
            if (retc)  // 2 passages echec => raz et enreg
            {
                ret = EEPROM_raz();
                if (!ret)
                {
                    // copie le buffer dans buff_read pour le cas ou la lecture avait fonctionne
                    memcpy (buff_read + SHEADERSIZE + offset, buffer, bufferSize);
                    ret = ExtFlash_write( EEPROM_REGIONS_BASE + SHEADERSIZE*2,  EEPROMSIZE , (const uint8_t *) buff_read+SHEADERSIZE, 0x80);
                    if (ret)
                        if (lecture_ok) retc=1;  // ok mais avec raz
                        else  retc=2;  // ok mais avec perte des autres valeurs EEPROM
                }
            }
            ExtFlash_close();
        }
        Semaphore_post(FlashAccessSemHandle);
    }
    return retc;
}

// validite index EEPROM
// 0:OK  1:non
uint8_t EEPROM_valid(void)
{
    uint8_t retc;

    retc=0;
    if ((index_EEPROM < 2) || (index_EEPROM >= (2*SECTORSIZE))
            || ((index_EEPROM-2)%(EEPROMSIZE+SHEADERSIZE)) )
    {
        retc = 1;
    }
    return retc;
}

// recherche index EEPROM
// 0:ok  1:raz  2:non
uint8_t EEPROM_recherche(void)
{
    uint8_t ret, retc, secteur1, secteur2, secteur_ut, cpt;
    uint8_t buff_read[2];
    uint16_t i;

    index_EEPROM = 2;
    retc=2;

    secteur_ut=0;
    secteur1=0;
    secteur2=0;

    // Recherche du secteur en utilisation
    // lecture header premier secteur
    ret = ExtFlash_read( EEPROM_REGIONS_BASE , 2, buff_read, 0x80) ;
    if (ret)
    {
        if (buff_read[0] == 0xFF) secteur1=1;  // vierge
        else if (buff_read[0] == 0x5F)
            { secteur1=2; secteur_ut=1;}  // utilisation
    }

    // lecture header deuxieme secteur
    ret = ExtFlash_read(EEPROM_REGIONS_BASE + SECTORSIZE , 2, buff_read, 0x80) ;
    if (ret)
    {
        if (buff_read[0] == 0xFF) secteur2=1;  // vierge
        else if (buff_read[0] == 0x5F)
            { secteur2=2; secteur_ut=2;} // utilisation
    }

    if (!secteur_ut)  // aucun secteur en utilisation
    {
        ret = EEPROM_raz();
        if (!ret)
            retc=1;
    }
    else
    {
        cpt=2;
        while (cpt)
        {
            index_EEPROM = SECTORSIZE * (secteur_ut-1);

            // Recherche de l'enregistrement en utilisation
            for (i=2; i<SECTORSIZE; i+=(EEPROMSIZE+SHEADERSIZE))
            {
                // lecture header de chaque enregistrement jusqu'a celui ok
                ret = ExtFlash_read(EEPROM_REGIONS_BASE + index_EEPROM + i , 1, buff_read, 0x80) ;
                if ( (ret) && (buff_read[0] == 0x5F))
                {
                    retc= 0;
                    cpt = 0;
                    index_EEPROM += i;
                    break;
                }
            }

            if (cpt)
            {
                // changement de secteur, si possible : si les 2 etaient en utilisation 2->1
                cpt--;
                if ((secteur_ut==2) && (secteur1==2))
                    secteur_ut=1;
            }
        }

        if (retc == 2)  // index non trouve
        {
            ret = EEPROM_raz();
            if (!ret)
                retc = 1;
        }
        else
        {
            // vérification que l'autre secteur est vierge, sinon effacement de l'autre
            if ((secteur_ut==1) && (secteur2!=1))
                    ExtFlash_erase( EEPROM_REGIONS_BASE + SECTORSIZE, SECTORSIZE);  // efface secteur 2
            if ((secteur_ut==2) && (secteur1!=1))
                    ExtFlash_erase( EEPROM_REGIONS_BASE , SECTORSIZE);  // efface secteur 1

        }
    }
    return retc;
}

uint8_t EEPROM_raz_appli(void)
{
    uint8_t ret;

    ret = Semaphore_pend(FlashAccessSemHandle, 1000*100); // Semaphore_WAIT_FOREVER   1 seconde
    if (ret)
    {
        ExtFlash_open();
        ret = EEPROM_raz();
        ExtFlash_close();
        Semaphore_post(FlashAccessSemHandle);
    }
return ret;
}

// raz EEPROM pour premiere initialisation ou suite a erreur
// 0:ok 1:non
uint8_t EEPROM_raz(void)
{
    uint8_t buff_head[2*SHEADERSIZE];
    uint8_t ret, retc;

    retc=0;
    // initialisation EEPROM
    index_EEPROM =  2;   // secteur 0, apres header
    ret = ExtFlash_erase( EEPROM_REGIONS_BASE, SECTORSIZE);  // secteur 0 de 4096 octets   80ms 9mA
    if (!ret) retc=1;
    ret = ExtFlash_erase( EEPROM_REGIONS_BASE + SECTORSIZE, SECTORSIZE);  // secteur 1 de 4096 octets   80ms 9mA
    if (!ret) retc=1;

    // activation header secteur 0 et header enreg0
    uint8_t i;
    for (i=0; i<2*SHEADERSIZE; i++)
        buff_head[i]= 0x5F;
    ret = ExtFlash_write( EEPROM_REGIONS_BASE,  2*SHEADERSIZE, (const uint8_t *) buff_head, 0x80);
    if (!ret) retc=1;

    return retc;
}

#endif
