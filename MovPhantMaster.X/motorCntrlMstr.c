
#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include "globals.h"
#include "motorCntrlMstr.h"
#include "enums.h"
#include "MSCommunication_mstr.h"




void readSecondaryQuadEncoder() {
// reads the position and velocity registers of the quadrature encoder on the secondary core
// The result is stored in the global variable g_secondaryQuadEncPos.
// The read is done by sending command requests through the master-slave interface.    
// It is important to be aware that the encoder position is not seen by the primary core until 
// the Slave-write-master-read FIFO is read.
    
    sendCommandToSecondary(READ_QUAD_ENC); // encoder position will be in g_secondaryQuadEncPos after next Master-Slave FIFO read

}        
