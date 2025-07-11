
#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include "motorCntrlMstr.h"
#include "enums.h"
#include "MSCommunication_mstr.h"



//void readSecondaryQuadEncoder() {
//// reads the position and velocity registers of the quadrature encoder on the secondary core
//// The results are stored in the global variables g_secondaryQuadEncPos and g_secondaryQuadEncVel.
//// The read is done by sending command requests through the master-slave interface.    
//// Proper usage of this should be to set the g_pingVar to 0, then call this command,
//// followed by requesting a ping with a different number to be put in g_pingVar when finished. This will
//// ensure the the encoder has been read before executing subsequent code that might use it.
//    sendCommandToSecondary(READ_QUAD_ENC);
//}        
