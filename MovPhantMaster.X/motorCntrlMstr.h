

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MOTORCNTRLMSTR_H
#define	MOTORCNTRLMSTR_H

#include <xc.h> // include processor files - each processor file is guarded.  



#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#include <stdbool.h>

//void readSecondaryQuadEncoder();        // reads the position and velocity registers of the quadrature encoder on the secondary core

#endif	/* MOTORCNTRLMSTR_H */

