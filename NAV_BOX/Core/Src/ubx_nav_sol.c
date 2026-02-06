#include "ubx_nav_sol.h"
#include <string.h>
#include <math.h>

/* WGS84 constants */
#define WGS84_A 6378137.0
#define WGS84_E2 6.69437999014e-3

/* Fletcher checksum */
static void UBX_CalculateChecksum(uint8_t *data, uint16_t length, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < length; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/* ECEF → Geodetic */
static void ecefToGeodetic(double X, double Y, double Z, double *lat, double *lon, double *h) {
    double a = WGS84_A;
    double e2 = WGS84_E2;
    double b = a * sqrt(1 - e2);
    double ep = sqrt((a*a - b*b) / (b*b));
    double p = sqrt(X*X + Y*Y);
    double th = atan2(a * Z, b * p);

    *lon = atan2(Y, X);
    *lat = atan2(Z + ep*ep*b*pow(sin(th),3), p - e2*a*pow(cos(th),3));

    double N = a / sqrt(1 - e2 * sin(*lat) * sin(*lat));
    *h = p / cos(*lat) - N;

    *lat *= 180.0 / M_PI;
    *lon *= 180.0 / M_PI;
}

/* Parser */
bool UBX_ParseNavSolFrame(uint8_t *buffer, uint16_t bufferLen, UBX_NavSol *solRaw, UBX_NavSolData *solData) {
    if (bufferLen < 8 + NAV_SOL_PAYLOAD_LEN) {
        return false;
    }

    if (buffer[0] != UBX_SYNC_CHAR1 || buffer[1] != UBX_SYNC_CHAR2) {
        return false;
    }

    if (buffer[2] != UBX_CLASS_NAV || buffer[3] != UBX_ID_NAV_SOL) {
        return false;
    }

    uint16_t len = buffer[4] | (buffer[5] << 8);
    if (len != NAV_SOL_PAYLOAD_LEN) {
        return false;
    }

    uint8_t ck_a, ck_b;
    UBX_CalculateChecksum(&buffer[2], 4 + NAV_SOL_PAYLOAD_LEN, &ck_a, &ck_b);
    if (ck_a != buffer[6 + NAV_SOL_PAYLOAD_LEN] || ck_b != buffer[7 + NAV_SOL_PAYLOAD_LEN]) {
        return false;
    }

    /* Copy into raw struct */
    memcpy(solRaw, &buffer[6], sizeof(UBX_NavSol));

    /* Fill decoded struct */
    solData->iTOW   = solRaw->iTOW;
    solData->fTOW   = solRaw->fTOW;
    solData->week   = solRaw->week;
    solData->gpsFix = solRaw->gpsFix;
    solData->flags  = solRaw->flags;
    solData->numSV  = solRaw->numSV;

    solData->ecefX_m = solRaw->ecefX / 100.0;
    solData->ecefY_m = solRaw->ecefY / 100.0;
    solData->ecefZ_m = solRaw->ecefZ / 100.0;
    solData->pAcc_m  = solRaw->pAcc / 100.0;

    solData->ecefVX_ms = solRaw->ecefVX / 100.0;
    solData->ecefVY_ms = solRaw->ecefVY / 100.0;
    solData->ecefVZ_ms = solRaw->ecefVZ / 100.0;
    solData->sAcc_ms   = solRaw->sAcc / 100.0;

    solData->pDOP = solRaw->pDOP / 100.0;

    /* Convert ECEF → LLA */
    double lat, lon, h;
    ecefToGeodetic(solData->ecefX_m, solData->ecefY_m, solData->ecefZ_m, &lat, &lon, &h);

    solData->latitude_deg  = lat;
    solData->longitude_deg = lon;
    solData->height_m      = h;

    return true;
}
