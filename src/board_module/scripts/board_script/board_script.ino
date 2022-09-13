
#include "minimal_module.h"
#include "communication_module.h"
#include "drivers_module.h"
#include "servos_module.h"

void setup(void){
    setupSerials();
    comms_setup();
    setupDrivers();
    setupServos();
}

void loop(void){
    comms_update();
}
