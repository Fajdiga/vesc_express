// JFBMS Master configuration parser

#ifndef JFBMS_MASTER_CONFPARSER_H_
#define JFBMS_MASTER_CONFPARSER_H_

#include "datatypes.h"
#include <stdint.h>
#include <stdbool.h>

// Constants
#define MAIN_CONFIG_T_SIGNATURE		2747705582
#define SERIALIZED_CONFIG_LENGTH	10

// Functions
int32_t jfbms_master_confparser_serialize_main_config_t(uint8_t *buffer, const main_config_t *conf);
bool jfbms_master_confparser_deserialize_main_config_t(const uint8_t *buffer, main_config_t *conf);
void jfbms_master_confparser_set_defaults_main_config_t(main_config_t *conf);

// JFBMS_MASTER_CONFPARSER_H_
#endif
