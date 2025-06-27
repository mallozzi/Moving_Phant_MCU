/* 
 * File:   registerHandler.h
 * Author: richa
 *
 * Created on October 1, 2019, 6:08 PM
 */

#ifndef REGISTERHANDLER_H
#define	REGISTERHANDLER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif
#include <stdint.h>
extern uint16_t lastValueWritten;
void setRegisterValue(uint8_t regNum, uint16_t dataVal);
uint16_t getRegisterValue(uint8_t regNum);
#endif	/* REGISTERHANDLER_H */

