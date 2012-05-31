/*
 * state_machine.h
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include "process.h"
#include <stdint.h>
#include "../../../../../shared/hexabus_packet.h"

PROCESS_NAME(state_machine_process);

// Internal stuff: these structs implement a simple table layout

// op for datetime: bits 0..6 denote dependency on hour, minute, second, ...; bit 7 sets whether to check for >= or <.
// date/time transitions need to be stored separately. They are also executed seperately, each time before the "normal" transitions are executed

// Used for wrapping the eeprom access

#define SM_CONDITION_LENGTH 0
#define SM_TRANSITION_LENGTH 1
#define SM_DATETIME_TRANSITION_LENGTH 2

// Actual structs

struct condition {
  uint8_t sourceIP[16]; // IP
  uint8_t sourceEID;    // EID we expect data from
  uint8_t op;           // predicate function
  uint8_t datatype;     // The constant to compare with
  char    data[4];      // Leave enough room for largest chunk of data, uint32_t/float in this case
} __attribute__ ((packed));

struct transition {
  uint8_t fromState;      // current state
  uint8_t cond;           // index of condition that must be matched
  uint8_t eid;            // id of endpoint which should do something
  uint8_t goodState;      // new state if everything went fine
  uint8_t badState;       // new state if some went wrong
  struct hxb_value value;  // Data for the endpoint
} __attribute__ ((packed));


// Defintion of events that are important to the state machine
// One general event for all data that can be possibly received
extern process_event_t sm_data_received_event;
extern process_event_t sm_rulechange_event;
#endif /* STATE_MACHINE_H_*/
