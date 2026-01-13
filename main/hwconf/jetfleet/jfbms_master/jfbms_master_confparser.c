// JFBMS Master configuration parser

#include <string.h>
#include "buffer.h"
#include "conf_general.h"
#include "jfbms_master_confparser.h"

int32_t jfbms_master_confparser_serialize_main_config_t(uint8_t *buffer, const main_config_t *conf) {
	int32_t ind = 0;

	buffer_append_uint32(buffer, MAIN_CONFIG_T_SIGNATURE, &ind);

	buffer_append_int16(buffer, conf->num_slaves, &ind);
	buffer[ind++] = conf->can_baud_rate;
	buffer_append_int16(buffer, conf->slave_timeout_ms, &ind);

	return ind;
}

bool jfbms_master_confparser_deserialize_main_config_t(const uint8_t *buffer, main_config_t *conf) {
	int32_t ind = 0;

	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != MAIN_CONFIG_T_SIGNATURE) {
		return false;
	}

	conf->num_slaves = buffer_get_int16(buffer, &ind);
	conf->can_baud_rate = buffer[ind++];
	conf->slave_timeout_ms = buffer_get_int16(buffer, &ind);

	return true;
}

void jfbms_master_confparser_set_defaults_main_config_t(main_config_t *conf) {
	conf->num_slaves = CONF_NUM_SLAVES;
	conf->can_baud_rate = CONF_CAN_BAUD_RATE;
	conf->slave_timeout_ms = CONF_SLAVE_TIMEOUT_MS;
}
