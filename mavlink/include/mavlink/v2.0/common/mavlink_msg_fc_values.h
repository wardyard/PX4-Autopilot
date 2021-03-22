#pragma once
// MESSAGE FC_VALUES PACKING

#define MAVLINK_MSG_ID_FC_VALUES 369


typedef struct __mavlink_fc_values_t {
 uint64_t time_boot_ms; /*< [ms] Timestamp (time sincesystem boot).*/
 float stack_voltage; /*< [V] Stack voltage*/
 float load_current; /*< [A] Load current*/
 float power; /*< [W] Power*/
 float energy; /*< [Wh] Energy*/
 float bat_voltage; /*< [V] Battery voltage*/
 float bat_current; /*< [A] Battery current*/
 float ld_voltage; /*< [V] LD voltage*/
 float temp_sensor_1_celc; /*< [C] Temperature sensor1*/
 float temp_sensor_2_celc; /*< [C] Temperature sensor2*/
 float temp_sensor_3_celc; /*< [C] Temperature sensor3*/
 float temp_sensor_4_celc; /*< [C] Temperature sensor4*/
 float tst_temperature_celc; /*< [C] TST temperature*/
 float pcb_temperature; /*< [C] PCB temperature*/
 float side_h2_pressure; /*< [BAR] Stack side H2 supplypressure*/
 float mvprs; /*< [mVPrs] Pressure in mV*/
 float fan_speed; /*< [%] Fan speed*/
 uint8_t operation_status; /*<  Operation status*/
 char message[131]; /*<  Message String*/
} mavlink_fc_values_t;

#define MAVLINK_MSG_ID_FC_VALUES_LEN 204
#define MAVLINK_MSG_ID_FC_VALUES_MIN_LEN 204
#define MAVLINK_MSG_ID_369_LEN 204
#define MAVLINK_MSG_ID_369_MIN_LEN 204

#define MAVLINK_MSG_ID_FC_VALUES_CRC 245
#define MAVLINK_MSG_ID_369_CRC 245

#define MAVLINK_MSG_FC_VALUES_FIELD_MESSAGE_LEN 131

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FC_VALUES { \
    369, \
    "FC_VALUES", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fc_values_t, time_boot_ms) }, \
         { "stack_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_fc_values_t, stack_voltage) }, \
         { "load_current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_fc_values_t, load_current) }, \
         { "power", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fc_values_t, power) }, \
         { "energy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_fc_values_t, energy) }, \
         { "bat_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_fc_values_t, bat_voltage) }, \
         { "bat_current", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_fc_values_t, bat_current) }, \
         { "ld_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_fc_values_t, ld_voltage) }, \
         { "temp_sensor_1_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_fc_values_t, temp_sensor_1_celc) }, \
         { "temp_sensor_2_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_fc_values_t, temp_sensor_2_celc) }, \
         { "temp_sensor_3_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_fc_values_t, temp_sensor_3_celc) }, \
         { "temp_sensor_4_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_fc_values_t, temp_sensor_4_celc) }, \
         { "tst_temperature_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_fc_values_t, tst_temperature_celc) }, \
         { "pcb_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_fc_values_t, pcb_temperature) }, \
         { "side_h2_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_fc_values_t, side_h2_pressure) }, \
         { "mvprs", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_fc_values_t, mvprs) }, \
         { "operation_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_fc_values_t, operation_status) }, \
         { "fan_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_fc_values_t, fan_speed) }, \
         { "message", NULL, MAVLINK_TYPE_CHAR, 131, 73, offsetof(mavlink_fc_values_t, message) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FC_VALUES { \
    "FC_VALUES", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fc_values_t, time_boot_ms) }, \
         { "stack_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_fc_values_t, stack_voltage) }, \
         { "load_current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_fc_values_t, load_current) }, \
         { "power", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fc_values_t, power) }, \
         { "energy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_fc_values_t, energy) }, \
         { "bat_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_fc_values_t, bat_voltage) }, \
         { "bat_current", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_fc_values_t, bat_current) }, \
         { "ld_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_fc_values_t, ld_voltage) }, \
         { "temp_sensor_1_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_fc_values_t, temp_sensor_1_celc) }, \
         { "temp_sensor_2_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_fc_values_t, temp_sensor_2_celc) }, \
         { "temp_sensor_3_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_fc_values_t, temp_sensor_3_celc) }, \
         { "temp_sensor_4_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_fc_values_t, temp_sensor_4_celc) }, \
         { "tst_temperature_celc", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_fc_values_t, tst_temperature_celc) }, \
         { "pcb_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_fc_values_t, pcb_temperature) }, \
         { "side_h2_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_fc_values_t, side_h2_pressure) }, \
         { "mvprs", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_fc_values_t, mvprs) }, \
         { "operation_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_fc_values_t, operation_status) }, \
         { "fan_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_fc_values_t, fan_speed) }, \
         { "message", NULL, MAVLINK_TYPE_CHAR, 131, 73, offsetof(mavlink_fc_values_t, message) }, \
         } \
}
#endif

/**
 * @brief Pack a fc_values message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time sincesystem boot).
 * @param stack_voltage [V] Stack voltage
 * @param load_current [A] Load current
 * @param power [W] Power
 * @param energy [Wh] Energy
 * @param bat_voltage [V] Battery voltage
 * @param bat_current [A] Battery current
 * @param ld_voltage [V] LD voltage
 * @param temp_sensor_1_celc [C] Temperature sensor1
 * @param temp_sensor_2_celc [C] Temperature sensor2
 * @param temp_sensor_3_celc [C] Temperature sensor3
 * @param temp_sensor_4_celc [C] Temperature sensor4
 * @param tst_temperature_celc [C] TST temperature
 * @param pcb_temperature [C] PCB temperature
 * @param side_h2_pressure [BAR] Stack side H2 supplypressure
 * @param mvprs [mVPrs] Pressure in mV
 * @param operation_status  Operation status
 * @param fan_speed [%] Fan speed
 * @param message  Message String
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fc_values_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_boot_ms, float stack_voltage, float load_current, float power, float energy, float bat_voltage, float bat_current, float ld_voltage, float temp_sensor_1_celc, float temp_sensor_2_celc, float temp_sensor_3_celc, float temp_sensor_4_celc, float tst_temperature_celc, float pcb_temperature, float side_h2_pressure, float mvprs, uint8_t operation_status, float fan_speed, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FC_VALUES_LEN];
    _mav_put_uint64_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 8, stack_voltage);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power);
    _mav_put_float(buf, 20, energy);
    _mav_put_float(buf, 24, bat_voltage);
    _mav_put_float(buf, 28, bat_current);
    _mav_put_float(buf, 32, ld_voltage);
    _mav_put_float(buf, 36, temp_sensor_1_celc);
    _mav_put_float(buf, 40, temp_sensor_2_celc);
    _mav_put_float(buf, 44, temp_sensor_3_celc);
    _mav_put_float(buf, 48, temp_sensor_4_celc);
    _mav_put_float(buf, 52, tst_temperature_celc);
    _mav_put_float(buf, 56, pcb_temperature);
    _mav_put_float(buf, 60, side_h2_pressure);
    _mav_put_float(buf, 64, mvprs);
    _mav_put_float(buf, 68, fan_speed);
    _mav_put_uint8_t(buf, 72, operation_status);
    _mav_put_char_array(buf, 73, message, 131);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FC_VALUES_LEN);
#else
    mavlink_fc_values_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.stack_voltage = stack_voltage;
    packet.load_current = load_current;
    packet.power = power;
    packet.energy = energy;
    packet.bat_voltage = bat_voltage;
    packet.bat_current = bat_current;
    packet.ld_voltage = ld_voltage;
    packet.temp_sensor_1_celc = temp_sensor_1_celc;
    packet.temp_sensor_2_celc = temp_sensor_2_celc;
    packet.temp_sensor_3_celc = temp_sensor_3_celc;
    packet.temp_sensor_4_celc = temp_sensor_4_celc;
    packet.tst_temperature_celc = tst_temperature_celc;
    packet.pcb_temperature = pcb_temperature;
    packet.side_h2_pressure = side_h2_pressure;
    packet.mvprs = mvprs;
    packet.fan_speed = fan_speed;
    packet.operation_status = operation_status;
    mav_array_memcpy(packet.message, message, sizeof(char)*131);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FC_VALUES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FC_VALUES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FC_VALUES_MIN_LEN, MAVLINK_MSG_ID_FC_VALUES_LEN, MAVLINK_MSG_ID_FC_VALUES_CRC);
}

/**
 * @brief Pack a fc_values message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time sincesystem boot).
 * @param stack_voltage [V] Stack voltage
 * @param load_current [A] Load current
 * @param power [W] Power
 * @param energy [Wh] Energy
 * @param bat_voltage [V] Battery voltage
 * @param bat_current [A] Battery current
 * @param ld_voltage [V] LD voltage
 * @param temp_sensor_1_celc [C] Temperature sensor1
 * @param temp_sensor_2_celc [C] Temperature sensor2
 * @param temp_sensor_3_celc [C] Temperature sensor3
 * @param temp_sensor_4_celc [C] Temperature sensor4
 * @param tst_temperature_celc [C] TST temperature
 * @param pcb_temperature [C] PCB temperature
 * @param side_h2_pressure [BAR] Stack side H2 supplypressure
 * @param mvprs [mVPrs] Pressure in mV
 * @param operation_status  Operation status
 * @param fan_speed [%] Fan speed
 * @param message  Message String
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fc_values_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_boot_ms,float stack_voltage,float load_current,float power,float energy,float bat_voltage,float bat_current,float ld_voltage,float temp_sensor_1_celc,float temp_sensor_2_celc,float temp_sensor_3_celc,float temp_sensor_4_celc,float tst_temperature_celc,float pcb_temperature,float side_h2_pressure,float mvprs,uint8_t operation_status,float fan_speed,const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FC_VALUES_LEN];
    _mav_put_uint64_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 8, stack_voltage);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power);
    _mav_put_float(buf, 20, energy);
    _mav_put_float(buf, 24, bat_voltage);
    _mav_put_float(buf, 28, bat_current);
    _mav_put_float(buf, 32, ld_voltage);
    _mav_put_float(buf, 36, temp_sensor_1_celc);
    _mav_put_float(buf, 40, temp_sensor_2_celc);
    _mav_put_float(buf, 44, temp_sensor_3_celc);
    _mav_put_float(buf, 48, temp_sensor_4_celc);
    _mav_put_float(buf, 52, tst_temperature_celc);
    _mav_put_float(buf, 56, pcb_temperature);
    _mav_put_float(buf, 60, side_h2_pressure);
    _mav_put_float(buf, 64, mvprs);
    _mav_put_float(buf, 68, fan_speed);
    _mav_put_uint8_t(buf, 72, operation_status);
    _mav_put_char_array(buf, 73, message, 131);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FC_VALUES_LEN);
#else
    mavlink_fc_values_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.stack_voltage = stack_voltage;
    packet.load_current = load_current;
    packet.power = power;
    packet.energy = energy;
    packet.bat_voltage = bat_voltage;
    packet.bat_current = bat_current;
    packet.ld_voltage = ld_voltage;
    packet.temp_sensor_1_celc = temp_sensor_1_celc;
    packet.temp_sensor_2_celc = temp_sensor_2_celc;
    packet.temp_sensor_3_celc = temp_sensor_3_celc;
    packet.temp_sensor_4_celc = temp_sensor_4_celc;
    packet.tst_temperature_celc = tst_temperature_celc;
    packet.pcb_temperature = pcb_temperature;
    packet.side_h2_pressure = side_h2_pressure;
    packet.mvprs = mvprs;
    packet.fan_speed = fan_speed;
    packet.operation_status = operation_status;
    mav_array_memcpy(packet.message, message, sizeof(char)*131);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FC_VALUES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FC_VALUES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FC_VALUES_MIN_LEN, MAVLINK_MSG_ID_FC_VALUES_LEN, MAVLINK_MSG_ID_FC_VALUES_CRC);
}

/**
 * @brief Encode a fc_values struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fc_values C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fc_values_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fc_values_t* fc_values)
{
    return mavlink_msg_fc_values_pack(system_id, component_id, msg, fc_values->time_boot_ms, fc_values->stack_voltage, fc_values->load_current, fc_values->power, fc_values->energy, fc_values->bat_voltage, fc_values->bat_current, fc_values->ld_voltage, fc_values->temp_sensor_1_celc, fc_values->temp_sensor_2_celc, fc_values->temp_sensor_3_celc, fc_values->temp_sensor_4_celc, fc_values->tst_temperature_celc, fc_values->pcb_temperature, fc_values->side_h2_pressure, fc_values->mvprs, fc_values->operation_status, fc_values->fan_speed, fc_values->message);
}

/**
 * @brief Encode a fc_values struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fc_values C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fc_values_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fc_values_t* fc_values)
{
    return mavlink_msg_fc_values_pack_chan(system_id, component_id, chan, msg, fc_values->time_boot_ms, fc_values->stack_voltage, fc_values->load_current, fc_values->power, fc_values->energy, fc_values->bat_voltage, fc_values->bat_current, fc_values->ld_voltage, fc_values->temp_sensor_1_celc, fc_values->temp_sensor_2_celc, fc_values->temp_sensor_3_celc, fc_values->temp_sensor_4_celc, fc_values->tst_temperature_celc, fc_values->pcb_temperature, fc_values->side_h2_pressure, fc_values->mvprs, fc_values->operation_status, fc_values->fan_speed, fc_values->message);
}

/**
 * @brief Send a fc_values message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time sincesystem boot).
 * @param stack_voltage [V] Stack voltage
 * @param load_current [A] Load current
 * @param power [W] Power
 * @param energy [Wh] Energy
 * @param bat_voltage [V] Battery voltage
 * @param bat_current [A] Battery current
 * @param ld_voltage [V] LD voltage
 * @param temp_sensor_1_celc [C] Temperature sensor1
 * @param temp_sensor_2_celc [C] Temperature sensor2
 * @param temp_sensor_3_celc [C] Temperature sensor3
 * @param temp_sensor_4_celc [C] Temperature sensor4
 * @param tst_temperature_celc [C] TST temperature
 * @param pcb_temperature [C] PCB temperature
 * @param side_h2_pressure [BAR] Stack side H2 supplypressure
 * @param mvprs [mVPrs] Pressure in mV
 * @param operation_status  Operation status
 * @param fan_speed [%] Fan speed
 * @param message  Message String
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fc_values_send(mavlink_channel_t chan, uint64_t time_boot_ms, float stack_voltage, float load_current, float power, float energy, float bat_voltage, float bat_current, float ld_voltage, float temp_sensor_1_celc, float temp_sensor_2_celc, float temp_sensor_3_celc, float temp_sensor_4_celc, float tst_temperature_celc, float pcb_temperature, float side_h2_pressure, float mvprs, uint8_t operation_status, float fan_speed, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FC_VALUES_LEN];
    _mav_put_uint64_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 8, stack_voltage);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power);
    _mav_put_float(buf, 20, energy);
    _mav_put_float(buf, 24, bat_voltage);
    _mav_put_float(buf, 28, bat_current);
    _mav_put_float(buf, 32, ld_voltage);
    _mav_put_float(buf, 36, temp_sensor_1_celc);
    _mav_put_float(buf, 40, temp_sensor_2_celc);
    _mav_put_float(buf, 44, temp_sensor_3_celc);
    _mav_put_float(buf, 48, temp_sensor_4_celc);
    _mav_put_float(buf, 52, tst_temperature_celc);
    _mav_put_float(buf, 56, pcb_temperature);
    _mav_put_float(buf, 60, side_h2_pressure);
    _mav_put_float(buf, 64, mvprs);
    _mav_put_float(buf, 68, fan_speed);
    _mav_put_uint8_t(buf, 72, operation_status);
    _mav_put_char_array(buf, 73, message, 131);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FC_VALUES, buf, MAVLINK_MSG_ID_FC_VALUES_MIN_LEN, MAVLINK_MSG_ID_FC_VALUES_LEN, MAVLINK_MSG_ID_FC_VALUES_CRC);
#else
    mavlink_fc_values_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.stack_voltage = stack_voltage;
    packet.load_current = load_current;
    packet.power = power;
    packet.energy = energy;
    packet.bat_voltage = bat_voltage;
    packet.bat_current = bat_current;
    packet.ld_voltage = ld_voltage;
    packet.temp_sensor_1_celc = temp_sensor_1_celc;
    packet.temp_sensor_2_celc = temp_sensor_2_celc;
    packet.temp_sensor_3_celc = temp_sensor_3_celc;
    packet.temp_sensor_4_celc = temp_sensor_4_celc;
    packet.tst_temperature_celc = tst_temperature_celc;
    packet.pcb_temperature = pcb_temperature;
    packet.side_h2_pressure = side_h2_pressure;
    packet.mvprs = mvprs;
    packet.fan_speed = fan_speed;
    packet.operation_status = operation_status;
    mav_array_memcpy(packet.message, message, sizeof(char)*131);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FC_VALUES, (const char *)&packet, MAVLINK_MSG_ID_FC_VALUES_MIN_LEN, MAVLINK_MSG_ID_FC_VALUES_LEN, MAVLINK_MSG_ID_FC_VALUES_CRC);
#endif
}

/**
 * @brief Send a fc_values message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fc_values_send_struct(mavlink_channel_t chan, const mavlink_fc_values_t* fc_values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fc_values_send(chan, fc_values->time_boot_ms, fc_values->stack_voltage, fc_values->load_current, fc_values->power, fc_values->energy, fc_values->bat_voltage, fc_values->bat_current, fc_values->ld_voltage, fc_values->temp_sensor_1_celc, fc_values->temp_sensor_2_celc, fc_values->temp_sensor_3_celc, fc_values->temp_sensor_4_celc, fc_values->tst_temperature_celc, fc_values->pcb_temperature, fc_values->side_h2_pressure, fc_values->mvprs, fc_values->operation_status, fc_values->fan_speed, fc_values->message);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FC_VALUES, (const char *)fc_values, MAVLINK_MSG_ID_FC_VALUES_MIN_LEN, MAVLINK_MSG_ID_FC_VALUES_LEN, MAVLINK_MSG_ID_FC_VALUES_CRC);
#endif
}

#if MAVLINK_MSG_ID_FC_VALUES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fc_values_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_boot_ms, float stack_voltage, float load_current, float power, float energy, float bat_voltage, float bat_current, float ld_voltage, float temp_sensor_1_celc, float temp_sensor_2_celc, float temp_sensor_3_celc, float temp_sensor_4_celc, float tst_temperature_celc, float pcb_temperature, float side_h2_pressure, float mvprs, uint8_t operation_status, float fan_speed, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 8, stack_voltage);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power);
    _mav_put_float(buf, 20, energy);
    _mav_put_float(buf, 24, bat_voltage);
    _mav_put_float(buf, 28, bat_current);
    _mav_put_float(buf, 32, ld_voltage);
    _mav_put_float(buf, 36, temp_sensor_1_celc);
    _mav_put_float(buf, 40, temp_sensor_2_celc);
    _mav_put_float(buf, 44, temp_sensor_3_celc);
    _mav_put_float(buf, 48, temp_sensor_4_celc);
    _mav_put_float(buf, 52, tst_temperature_celc);
    _mav_put_float(buf, 56, pcb_temperature);
    _mav_put_float(buf, 60, side_h2_pressure);
    _mav_put_float(buf, 64, mvprs);
    _mav_put_float(buf, 68, fan_speed);
    _mav_put_uint8_t(buf, 72, operation_status);
    _mav_put_char_array(buf, 73, message, 131);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FC_VALUES, buf, MAVLINK_MSG_ID_FC_VALUES_MIN_LEN, MAVLINK_MSG_ID_FC_VALUES_LEN, MAVLINK_MSG_ID_FC_VALUES_CRC);
#else
    mavlink_fc_values_t *packet = (mavlink_fc_values_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->stack_voltage = stack_voltage;
    packet->load_current = load_current;
    packet->power = power;
    packet->energy = energy;
    packet->bat_voltage = bat_voltage;
    packet->bat_current = bat_current;
    packet->ld_voltage = ld_voltage;
    packet->temp_sensor_1_celc = temp_sensor_1_celc;
    packet->temp_sensor_2_celc = temp_sensor_2_celc;
    packet->temp_sensor_3_celc = temp_sensor_3_celc;
    packet->temp_sensor_4_celc = temp_sensor_4_celc;
    packet->tst_temperature_celc = tst_temperature_celc;
    packet->pcb_temperature = pcb_temperature;
    packet->side_h2_pressure = side_h2_pressure;
    packet->mvprs = mvprs;
    packet->fan_speed = fan_speed;
    packet->operation_status = operation_status;
    mav_array_memcpy(packet->message, message, sizeof(char)*131);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FC_VALUES, (const char *)packet, MAVLINK_MSG_ID_FC_VALUES_MIN_LEN, MAVLINK_MSG_ID_FC_VALUES_LEN, MAVLINK_MSG_ID_FC_VALUES_CRC);
#endif
}
#endif

#endif

// MESSAGE FC_VALUES UNPACKING


/**
 * @brief Get field time_boot_ms from fc_values message
 *
 * @return [ms] Timestamp (time sincesystem boot).
 */
static inline uint64_t mavlink_msg_fc_values_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field stack_voltage from fc_values message
 *
 * @return [V] Stack voltage
 */
static inline float mavlink_msg_fc_values_get_stack_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field load_current from fc_values message
 *
 * @return [A] Load current
 */
static inline float mavlink_msg_fc_values_get_load_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field power from fc_values message
 *
 * @return [W] Power
 */
static inline float mavlink_msg_fc_values_get_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field energy from fc_values message
 *
 * @return [Wh] Energy
 */
static inline float mavlink_msg_fc_values_get_energy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field bat_voltage from fc_values message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_fc_values_get_bat_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field bat_current from fc_values message
 *
 * @return [A] Battery current
 */
static inline float mavlink_msg_fc_values_get_bat_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ld_voltage from fc_values message
 *
 * @return [V] LD voltage
 */
static inline float mavlink_msg_fc_values_get_ld_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field temp_sensor_1_celc from fc_values message
 *
 * @return [C] Temperature sensor1
 */
static inline float mavlink_msg_fc_values_get_temp_sensor_1_celc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field temp_sensor_2_celc from fc_values message
 *
 * @return [C] Temperature sensor2
 */
static inline float mavlink_msg_fc_values_get_temp_sensor_2_celc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field temp_sensor_3_celc from fc_values message
 *
 * @return [C] Temperature sensor3
 */
static inline float mavlink_msg_fc_values_get_temp_sensor_3_celc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field temp_sensor_4_celc from fc_values message
 *
 * @return [C] Temperature sensor4
 */
static inline float mavlink_msg_fc_values_get_temp_sensor_4_celc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field tst_temperature_celc from fc_values message
 *
 * @return [C] TST temperature
 */
static inline float mavlink_msg_fc_values_get_tst_temperature_celc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field pcb_temperature from fc_values message
 *
 * @return [C] PCB temperature
 */
static inline float mavlink_msg_fc_values_get_pcb_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field side_h2_pressure from fc_values message
 *
 * @return [BAR] Stack side H2 supplypressure
 */
static inline float mavlink_msg_fc_values_get_side_h2_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field mvprs from fc_values message
 *
 * @return [mVPrs] Pressure in mV
 */
static inline float mavlink_msg_fc_values_get_mvprs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field operation_status from fc_values message
 *
 * @return  Operation status
 */
static inline uint8_t mavlink_msg_fc_values_get_operation_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field fan_speed from fc_values message
 *
 * @return [%] Fan speed
 */
static inline float mavlink_msg_fc_values_get_fan_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field message from fc_values message
 *
 * @return  Message String
 */
static inline uint16_t mavlink_msg_fc_values_get_message(const mavlink_message_t* msg, char *message)
{
    return _MAV_RETURN_char_array(msg, message, 131,  73);
}

/**
 * @brief Decode a fc_values message into a struct
 *
 * @param msg The message to decode
 * @param fc_values C-struct to decode the message contents into
 */
static inline void mavlink_msg_fc_values_decode(const mavlink_message_t* msg, mavlink_fc_values_t* fc_values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fc_values->time_boot_ms = mavlink_msg_fc_values_get_time_boot_ms(msg);
    fc_values->stack_voltage = mavlink_msg_fc_values_get_stack_voltage(msg);
    fc_values->load_current = mavlink_msg_fc_values_get_load_current(msg);
    fc_values->power = mavlink_msg_fc_values_get_power(msg);
    fc_values->energy = mavlink_msg_fc_values_get_energy(msg);
    fc_values->bat_voltage = mavlink_msg_fc_values_get_bat_voltage(msg);
    fc_values->bat_current = mavlink_msg_fc_values_get_bat_current(msg);
    fc_values->ld_voltage = mavlink_msg_fc_values_get_ld_voltage(msg);
    fc_values->temp_sensor_1_celc = mavlink_msg_fc_values_get_temp_sensor_1_celc(msg);
    fc_values->temp_sensor_2_celc = mavlink_msg_fc_values_get_temp_sensor_2_celc(msg);
    fc_values->temp_sensor_3_celc = mavlink_msg_fc_values_get_temp_sensor_3_celc(msg);
    fc_values->temp_sensor_4_celc = mavlink_msg_fc_values_get_temp_sensor_4_celc(msg);
    fc_values->tst_temperature_celc = mavlink_msg_fc_values_get_tst_temperature_celc(msg);
    fc_values->pcb_temperature = mavlink_msg_fc_values_get_pcb_temperature(msg);
    fc_values->side_h2_pressure = mavlink_msg_fc_values_get_side_h2_pressure(msg);
    fc_values->mvprs = mavlink_msg_fc_values_get_mvprs(msg);
    fc_values->fan_speed = mavlink_msg_fc_values_get_fan_speed(msg);
    fc_values->operation_status = mavlink_msg_fc_values_get_operation_status(msg);
    mavlink_msg_fc_values_get_message(msg, fc_values->message);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FC_VALUES_LEN? msg->len : MAVLINK_MSG_ID_FC_VALUES_LEN;
        memset(fc_values, 0, MAVLINK_MSG_ID_FC_VALUES_LEN);
    memcpy(fc_values, _MAV_PAYLOAD(msg), len);
#endif
}
