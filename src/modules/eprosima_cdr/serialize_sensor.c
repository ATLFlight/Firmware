#include "serialize_sensor.h"

#include <uORB/topics/vehicle_attitude.h>

void serialize_sensor_combined(const struct sensor_combined_s input, char * output, uint32_t * s_length){

    // nanoBuffer using the user defined buffer (output)
    struct nanoBuffer * nanoBufferWriter = NULL;
    newStaticAlignedBuffer(output, BUFFER_LENGTH, &nanoBufferWriter);

    // nanoCDR struct for managing the nanoBuffer
    struct nanoCDR * nanoCDRWriter = NULL;
    newNanoCDR(&nanoCDRWriter, nanoBufferWriter);

    // Serialize a sensor_combined_s message
    serializeUnsignedLong(input.timestamp, nanoCDRWriter);
    serializeFloatArray(input.gyro_rad, 3, nanoCDRWriter);
    serializeFloat(input.gyro_integral_dt, nanoCDRWriter);
    serializeInt(input.accelerometer_timestamp_relative, nanoCDRWriter);
    serializeFloatArray(input.accelerometer_m_s2, 3, nanoCDRWriter);
    serializeFloat(input.accelerometer_integral_dt, nanoCDRWriter);
    serializeInt(input.magnetometer_timestamp_relative, nanoCDRWriter);
    serializeFloatArray(input.magnetometer_ga, 3, nanoCDRWriter);
    serializeInt(input.baro_timestamp_relative, nanoCDRWriter);
    serializeFloat(input.baro_alt_meter, nanoCDRWriter);
    serializeFloat(input.baro_temp_celcius, nanoCDRWriter);

    *s_length = nanoBufferWriter->m_serializedBuffer;

    free(nanoBufferWriter);
    free(nanoCDRWriter);
}
