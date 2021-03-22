#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/fc_values.h>

extern "C" __EXPORT int inject_myUORB_main(int argc, char *argv[]);

int inject_myUORB_main(int argc, char *argv[])
{
    PX4_INFO("Hello, I am only a test program able to inject FC_VALUES messages.");

    // Declare structure to store data that will be sent
    struct fc_values_s fcVal;

    // Clear the structure by filling it with 0s in memory
    memset(&fcVal, 0, sizeof(fcVal));

    // Create a uORB topic advertisement
    orb_advert_t fc_values_pub = orb_advertise(ORB_ID(fc_values), &fcVal);

    for (int i=0; i<40; i++)
        {
        char myStr[]={"this is a FC values test"}; memcpy(fcVal.message, myStr, 24);
        fcVal.timestamp = hrt_absolute_time();
	fcVal.stack_voltage = 1.0*i;
	fcVal.load_current = 2.0*i;
	fcVal.power = i*i;
	fcVal.energy = 0.5*i;
	fcVal.bat_current = 21.0;
	fcVal.bat_voltage = 2.5*i;
	fcVal.ld_voltage = 2.5*i;
	fcVal.temp_sensor_1_celc = 0.99*i;
	fcVal.temp_sensor_2_celc = 0.89*i;
	fcVal.temp_sensor_3_celc = 0.79*i;
	fcVal.temp_sensor_4_celc = 0.69*i;
	fcVal.tst_temperature_celc = 10.0*i;
	fcVal.pcb_temperature_celc = 50.0;
	fcVal.side_h2_pressure = i*6.0;
	fcVal.mvprs = 0.05*i;
	fcVal.operation_status = i;
	fcVal.fan_speed = 0.5*i;

        orb_publish(ORB_ID(fc_values), fc_values_pub, &fcVal);

        //sleep for 2s
        usleep (2000000);
        }

    PX4_INFO("inject_myUORB finished!");

    return 0;
}
