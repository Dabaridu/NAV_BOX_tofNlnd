#ifndef SENS_FUSION_H
#define SENS_FUSION_H
/* sens_fusion.h
 * Header for sensor fusion algorithms and related functions
 */

 typedef struct {
    float xX;
    float xY;
    float xA;
    float xPx;
    float xPv;
    float xPa;

    float yX;
    float yY;
    float yA;
    float yPx;
    float yPv;
    float yPa;

    float zX;
    float zY;
    float zA;
    float zPx;
    float zPv;
    float zPa;
 }state_x;

#endif /* SENS_FUSION_H */