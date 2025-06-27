

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MS_COMMUNICATION_H
#define	MS_COMMUNICATION_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>
#include "enums.h"
 
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

void enableMSFifo();
void sendCommandToSecondary(Command whichCommand);
void sendVariableToSecondary(Variable whichVar, uint16_t value);
void send32bVariableToSecondary(Variable32 whichVar, uint32_t value);
void sendBoolVarToSecondary(BoolVariable whichVar, bool boolVal);
//void sendBoolVarToSecondary(BoolVariable whichVar, bool boolVal);
void sendParamtersToSecondary();

void receiveVariableFromSecondary();
void receive32bVariableFromSecondary();
void receiveBoolVarFromSecondary();

#endif	/* MS_COMMUNICATION_H */

