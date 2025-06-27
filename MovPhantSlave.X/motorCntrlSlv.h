

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

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

void setMotorOutput(int16_t pwmSignedDutyCycleInt);

#endif	/* MOTOR_CONTROL_H */

