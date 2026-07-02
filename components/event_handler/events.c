#include "events.h"
#include <stdlib.h>
// create events
Event a1 = {CONTROLLABLE, 0, SUP_DEBUG_STR("a1"), NULL};
Event b1 = {UNCONTROLLABLE, 1, SUP_DEBUG_STR("b1"), NULL};
Event a2 = {CONTROLLABLE, 2, SUP_DEBUG_STR("a2"), NULL};
Event b2 = {UNCONTROLLABLE, 3, SUP_DEBUG_STR("b2"), NULL};
Event sp = {UNCONTROLLABLE, 4, SUP_DEBUG_STR("sp"), NULL};
Event a0 = {CONTROLLABLE, 5, SUP_DEBUG_STR("a0"), NULL};
Event b0 = {UNCONTROLLABLE, 6, SUP_DEBUG_STR("b0"), NULL};
Event a3 = {CONTROLLABLE, 7, SUP_DEBUG_STR("a3"), NULL};
Event a4 = {CONTROLLABLE, 8, SUP_DEBUG_STR("a4"), NULL};
Event b3 = {UNCONTROLLABLE, 9, SUP_DEBUG_STR("b3"), NULL};
Event b4 = {UNCONTROLLABLE, 10, SUP_DEBUG_STR("b4"), NULL};
Event sm = {UNCONTROLLABLE, 11, SUP_DEBUG_STR("sm"), NULL};
Event sg = {UNCONTROLLABLE, 12, SUP_DEBUG_STR("sg"), NULL};
Event iBp = {CONTROLLABLE, 13, SUP_DEBUG_STR("iBp"), NULL};
Event fBp = {UNCONTROLLABLE, 14, SUP_DEBUG_STR("fBp"), NULL};
Event fBm = {UNCONTROLLABLE, 15, SUP_DEBUG_STR("fBm"), NULL};
Event iBm = {CONTROLLABLE, 16, SUP_DEBUG_STR("iBm"), NULL};
Event iBg = {CONTROLLABLE, 17, SUP_DEBUG_STR("iBg"), NULL};
Event fBg = {UNCONTROLLABLE, 18, SUP_DEBUG_STR("fBg"), NULL};
Event ie = {CONTROLLABLE, 19, SUP_DEBUG_STR("ie"), NULL};
Event fe = {CONTROLLABLE, 20, SUP_DEBUG_STR("fe"), NULL};
Event botao = {UNCONTROLLABLE, 21, SUP_DEBUG_STR("botao"), NULL};
Event Smet = {UNCONTROLLABLE, 22, SUP_DEBUG_STR("Smet"), NULL};

// ie e fe removidos da cascata automatica: se ficassem aqui, o estado
// "armado e vazio" (contador 0) geraria o livelock ie->fe->ie->fe (recursao
// infinita). Sao disparados manualmente via trigger_event() no main.c.
Event *controllable_event_list[CONTROLLABLE_EVENTS_COUNT] = {&a1,&a2,&a0,&a3,&a4,&iBp,&iBm,&iBg};
