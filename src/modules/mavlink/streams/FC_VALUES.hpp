/****************************************************************************************
* Custom MAVLink stream for the "custom_messages.xml" message definition in mavlink/include/mavlink/v2.0/message_definitions
* File based on the one found at https://www.hackster.io/mdobrea/communication-through-custom-uorb-and-mavlink-messages-269ebf
* and https://docs.px4.io/master/en/ros/mavros_custom_messages.html
* @author Ward Bogaerts, AeroDelft, ward.bogaerts@aerodelft.nl
*****************************************************************************************/

#ifndef FC_VALUES_HPP
#define FC_VALUES_HPP

#include <uORB/topics/fc_values.h>  //placed in: build/px4_fmu-v5/uORB/topics
#include <v2.0/common/common.h>
#include "v2.0/common/mavlink_msg_fc_values.h"
// I think you can add another mavlink_msg_XXXX.h here if you've defined multiple messages in
// mavlink/include/mavlink/v2.0/message_definitions/custom_messages.xml

class MavlinkStreamFCValues : public MavlinkStream //class for defining a fuel cell values MAVLink stream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    { return new MavlinkStreamFCValues(mavlink); }

    // In a member function declaration or definition, override specifier ensures that
    // the function is virtual and is overriding a virtual function from a base class.
    const char*get_name() const override
    { return MavlinkStreamFCValues::get_name_static(); }

    // The constexpr specifier declares that it is possible to
    // evaluate the value of the function or variable at compile time.
    static constexpr const char *get_name_static()
    { return "FC_VALUES";  }

    uint16_t get_id() override
    { return get_id_static(); }

    static constexpr uint16_t get_id_static()
    { return MAVLINK_MSG_ID_FC_VALUES; } //defined in mavlink/../v2.0/custom_messages/mavlink_msg_fc_values.h

    unsigned get_size() override
    { return MAVLINK_MSG_ID_FC_VALUES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; } //defined in mavlink/../v2.0/custom_messages/mavlink_msg_fc_values.h

private:
    uORB::Subscription _sub{ORB_ID(fc_values)}; //build/px4_fmu-v5/uORB/topics

    /* do not allow top copying this class */
    MavlinkStreamFCValues(MavlinkStreamFCValues &);
    MavlinkStreamFCValues& operator = (const MavlinkStreamFCValues &);

protected:
    explicit MavlinkStreamFCValues(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
        struct fc_values_s _fc_values;  //make sure fc_values_s is the definition of uORB topic
					//check this in build/px4_fmu-v5/uORB/fc_values.h

        if (_sub.update(&_fc_values))
        {
        mavlink_fc_values_t _msg_fc_values;  	// mavlink_fc_values_t is the
                                        	// definition of your custom MAVLink message
                                                // mavlink/../v2.0/custom_messages/mavlink_msg_fc_values.h

	//couple the message names from the xml file to the msg names from the .msg file
        _msg_fc_values.time_boot_ms  	= _fc_values.timestamp;
        _msg_fc_values.stack_voltage 	= _fc_values.stack_voltage;
	    _msg_fc_values.load_current	= _fc_values.load_current;
	    _msg_fc_values.power 		= _fc_values.power;
	    _msg_fc_values.energy 		= _fc_values.energy;
	    _msg_fc_values.bat_voltage 	= _fc_values.bat_voltage;
	    _msg_fc_values.bat_current 	= _fc_values.bat_current;
	    _msg_fc_values.ld_voltage 	= _fc_values.ld_voltage;
	    _msg_fc_values.temp_sensor_1_celc 	= _fc_values.temp_sensor_1_celc;
	    _msg_fc_values.temp_sensor_2_celc 	= _fc_values.temp_sensor_2_celc;
	    _msg_fc_values.temp_sensor_3_celc 	= _fc_values.temp_sensor_3_celc;
	    _msg_fc_values.temp_sensor_4_celc 	= _fc_values.temp_sensor_4_celc;
	    _msg_fc_values.tst_temperature_celc 	= _fc_values.tst_temperature_celc;
	    _msg_fc_values.pcb_temperature 	= _fc_values.pcb_temperature_celc; // here the.xml file differs from the .msg file !!
	    _msg_fc_values.side_h2_pressure = _fc_values.side_h2_pressure;
	    _msg_fc_values.mvprs 		= _fc_values.mvprs;
	    _msg_fc_values.fan_speed 	= _fc_values.fan_speed;
	    _msg_fc_values.operation_status = _fc_values.operation_status;

        for(int i=0; i<131; i++) //arrays are copied via a loop
            _msg_fc_values.message[i] = _fc_values.message[i];

        mavlink_msg_fc_values_send_struct(_mavlink->get_channel(),
                                              &_msg_fc_values);

        PX4_WARN("uorb => mavlink - message was sent !!!!");

        return true;
        }

    return false;
    }
};
#endif // FC_VALUES_HPP; NEXT: go to src/modules/mavlink/mavlink_messages.cpp
