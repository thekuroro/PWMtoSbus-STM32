/*
 * Sbus.h
 *
 *  Created on: 9 Oct 2016
 *      Author: Kuroro
 */

#ifndef SBUS_SBUS_H_
#define SBUS_SBUS_H_

#define SBUS_FRAME_LENGH	25
#define _sbusHeader 		0x0F
#define _sbusFooter 		0x00
#define _sbusLostFrame 		0x20
#define _sbusFailSafe 		0x10

void SBUS_write(uint16_t* channels);

#endif /* SBUS_SBUS_H_ */
