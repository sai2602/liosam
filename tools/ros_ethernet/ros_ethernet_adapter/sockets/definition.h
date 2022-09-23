#ifndef DEFINITION_H_INCLUDED
#define DEFINITION_H_INCLUDED

#include <stdint.h>
#include "version.h"

//#define SERIAL_CHECKSUM 1 // 000000005c1775d7,  RaspberryPi
//#define SERIAL_CHECKSUM 82 // 000000003fdada07, CommScope
//#define SERIAL_CHECKSUM 81 // 0000000031f53f4a, ppm
//#define SERIAL_CHECKSUM 2 // 000000003c0fbb4,   2014RP4901
//#define SERIAL_CHECKSUM 92 // 000000006a018839, 2014RP4902
//#define SERIAL_CHECKSUM 12 // 0000000016846b1b, 2015RP0401
//#define SERIAL_CHECKSUM 15 // 0000000078ed71eb, 2015RP1601

/*
    Globale Definitionsvereinbarungen zur Interprozesskommunikation
*/

typedef struct
{
    uint8_t iPrn;
    double  fPseudorange;
    double  fCarrierphase;
}   sGpsRawSat;

typedef struct
{
    uint16_t    iWeek;
    uint32_t    iTow;
    uint8_t     nSats;
    sGpsRawSat* pSat;
}   sGpsRaw;

typedef struct sGpsRawItem
{
    sGpsRaw*            item;
    struct sGpsRawItem* next;
}   sGpsRawList;

typedef struct
{
    double fX;
    double fY;
    double fZ;
}   sGpsPos;

typedef struct
{
    double fHeading;
    double fPitch;
    double fBaseE;
    double fBaseN;
    double fBaseU;
}   sGpsAttitude;

typedef struct
{
    double probability;
    sGpsAttitude attitude;
}   sGpsSolution;

typedef struct
{
    float fAccelerationX;
    float fAccelerationY;
    float fAccelerationZ;
    float fGyroX;
    float fGyroY;
    float fGyroZ;
}   sInertial;

/*
typedef struct
{
    uint8_t iPrn;
    uint8_t iSfid;
    uint8_t data[30];
}   sGpsNav;

typedef struct sGpsNavItem
{
    sGpsNav*            item;
    struct sGpsNavItem* next;
}   sGpsNavList;
*/

typedef struct sGpsNavItem
{
    uint8_t             iPrn;
    uint8_t             iSfid;
    uint8_t             data[30];
    struct sGpsNavItem* next;
}   sGpsNavList;

typedef struct
{
    uint8_t             nUsedSatellites;
    int32_t             iEpoch;
}   sGpsLog;

typedef struct
{
    int iSettingMinAmountSatellites;
    int iSettingTimeEpochStartWaiting;
    int iSettingTimeEpochStartRun;
}   sGpsConfig;

#endif // DEFINITION_H_INCLUDED
