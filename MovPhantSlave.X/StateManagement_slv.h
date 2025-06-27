
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef STATE_MNGMNT_SLV_H
#define	STATE_MNGMNT_SLV_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

void startMotion();
void stopMotion();


#endif	/* STATE_MNGMNT_SLV_H */

