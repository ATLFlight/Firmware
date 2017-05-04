#ifndef _SERIALIZE_SENSOR_H_
#define _SERIALIZE_SENSOR_H_

#include <uORB/topics/sensor_combined.h>
#include <px4_log.h>
#include "nanocdr/nanoCdr.h"

#define BUFFER_LENGTH 200

void serialize_sensor_combined(const struct sensor_combined_s, char * output, uint32_t * s_length);

#endif
