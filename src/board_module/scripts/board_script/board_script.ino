#include "minimal_module.h"
#include "communication_module.h"
#include "drivers_module.h"

void setup(void){
    setupSerials();
    comms_setup();
    setupDrivers();
}

void loop(void){
    comms_update();
}