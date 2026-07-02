#ifndef __EVENTS_H__
#define __EVENTS_H__

#include "supervisor.h"
#include <stdlib.h>

#define CONTROLLABLE_EVENTS_COUNT 10

// create events
extern Event a1;
extern Event b1;
extern Event a2;
extern Event b2;
extern Event sp;
extern Event a0;
extern Event b0;
extern Event a3;
extern Event a4;
extern Event b3;
extern Event b4;
extern Event sm;
extern Event sg;
extern Event iBp;
extern Event fBp;
extern Event fBm;
extern Event iBm;
extern Event iBg;
extern Event fBg;
extern Event ie;
extern Event fe;
extern Event botao;
extern Event Smet;

extern Event *controllable_event_list[CONTROLLABLE_EVENTS_COUNT];

#endif // __EVENTS_H__
