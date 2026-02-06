#ifndef UBX_NAV_SOL_H
#define UBX_NAV_SOL_H

#include <stdint.h>
#include <stdbool.h>

/* UBX protocol constants */
#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62
#define UBX_CLASS_NAV  0x01
#define UBX_ID_NAV_SOL 0x06
#define NAV_SOL_PAYLOAD_LEN 52

#pragma pack(push, 1)
/* Raw NAV-SOL (binary UBX payload) */
typedef struct {
    uint32_t iTOW;
    int32_t  fTOW;
    int16_t  week;
    uint8_t  gpsFix;
    uint8_t  flags;
    int32_t  ecefX;
    int32_t  ecefY;
    int32_t  ecefZ;
    uint32_t pAcc;
    int32_t  ecefVX;
    int32_t  ecefVY;
    int32_t  ecefVZ;
    uint32_t sAcc;
    uint16_t pDOP;
    uint8_t  reserved1;
    uint8_t  numSV;
    uint32_t reserved2;
} UBX_NavSol;
#pragma pack(pop)

/* Decoded NAV-SOL (human-friendly) */
typedef struct {
    /* Derived LLA */
    double latitude_deg;    // [deg]
    double longitude_deg;   // [deg]
    double height_m;        // [m]

    /* Direct fields */
    uint32_t iTOW;          // [ms]
    int32_t  fTOW;          // [ns]
    int16_t  week;          // GPS week
    uint8_t  gpsFix;        // Fix type
    uint8_t  flags;         // Fix flags
    double   ecefX_m;       // [m]
    double   ecefY_m;       // [m]
    double   ecefZ_m;       // [m]
    double   pAcc_m;        // [m]
    double   ecefVX_ms;     // [m/s]
    double   ecefVY_ms;     // [m/s]
    double   ecefVZ_ms;     // [m/s]
    double   sAcc_ms;       // [m/s]
    double   pDOP;          // [unitless]
    uint8_t  numSV;         // satellites
} UBX_NavSolData;

/* Function prototype */
bool UBX_ParseNavSolFrame(uint8_t *buffer, uint16_t bufferLen, UBX_NavSol *solRaw, UBX_NavSolData *solData);

#endif // UBX_NAV_SOL_H
