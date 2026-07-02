idf_component_register(SRCS 
                    "supBotaoBr.c"
                    "supervisor_list.c"
                    INCLUDE_DIRS "."
                    REQUIRES supervisor event_handler)
