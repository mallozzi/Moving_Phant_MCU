

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MS_COMMUNICATION_SLV_H
#define	MS_COMMUNICATION_SLV_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "enums_slave.h"
#include <stdbool.h>


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

void processCommandFromPrimary();
void receiveVariableFromPrimary();
void receive32bVariableFromPrimary();
void receiveBoolVarFromPrimary();

void sendVariableToPrimary(Variable whichVar, uint16_t value);
void send32bVariableToPrimary(Variable32 whichVar, uint32_t value);
void sendBoolVarToPrimary(Variable whichVar, bool value);

#endif	/* MS_COMMUNICATION_SLV_H */

