/*
 * com_HMC5883L.h
 *
 * Created: 11.03.2016 15:51:27
 *  Author: erlenese
 */ 


#ifndef COM_HMC5883L_H_
#define COM_HMC5883L_H_

void vCOM_init(void);

void vCOM_getData(int16_t *xCom, int16_t *yCom, int16_t *zCom);

#endif /* COM_HMC5883L_H_ */