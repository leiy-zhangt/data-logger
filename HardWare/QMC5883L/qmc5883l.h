#ifndef __QMC5883L_H
#define __QMC5883L_H

#include "sys.h"
#include "iic.h"

#define QMC5883L_addr 0X0D<<1
#define QMC5883L_DARY PCin(8)
#define QMC5883L_Range 8

void QMC5883L_Configuration(void);
void QMC5883L_MagnetismRead(uint8_t *buffer,double *data);
double QMC5883L_TemperatureRead(void);
#endif
