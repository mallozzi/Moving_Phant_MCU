
#include <xc.h>
#include <libpic30.h>
#include <stdlib.h>
#include "globals.h"
#include "motorCntrlMstr.h"
#include "enums.h"
#include "MSCommunication_mstr.h"




void readSecondaryQuadEncoder() {
// reads the position and velocity registers of the quadrature encoder on the secondary core
// The results are stored in the global variables g_secondaryQuadEncPos and g_secondaryQuadEncVel.
// The read is done by sending command requests through the master-slave interface.    
// Proper usage of this should be to set the g_pingVar to 0, then call this command,
// followed by requesting a ping with a different number to be put in g_pingVar when finished. This will
// ensure the the encoder has been read before executing subsequent code that might use it.
    
    static uint16_t pingval=19;                 // arbitrary integer
    sendCommandToSecondary(READ_QUAD_ENC);
    // perform ping to confirm that it has been read. Ping should take a few instruction cycles to complete
    g_pingVar = 0;
    sendPingRequestToSecondary(pingval);
    LATDbits.LATD10 = 1;                        // Turn on LED 1 so that we can detect if we got stalled
    while(g_pingVar != pingval);                // wait for ping to be returned. Should take a few instruction cycles
    LATDbits.LATD10 = 0;                        // Turn off LED 1 if we were got through the ping request
    // encoder position will now be in g_secondaryQuadEncPos, velocity in g_secondaryQuadEncVel
}        
