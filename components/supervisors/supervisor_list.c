#include "supervisor_list.h"
#include "supBotaoBr.h"

// make list with all supervisors
// first create all supervisors
extern SupervisorList supBotaoBr_list;

// then recreate and linking them
SupervisorList sup_list = {&supBotaoBr, NULL};
