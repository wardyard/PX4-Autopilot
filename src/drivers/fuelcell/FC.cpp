/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * driver for reading out fuel cell data. Fuel cell sends out data via RS-232.
 * RS-232 gets transformed to UART, directly connected to TELEM4 port of Pixhawk
 * Read out data from UART and store it in uORB topic fc_values.msg
 */

/**
 * inspired on GPS, Volz, MCP9808 an d CM8JL65 drivers
 */

/**
 * baud rate: 19200, data bits: 8, stop bits: 0, no parity, no handshaking
 */

/**
 * frame format (each character is 1 byte long):
 * F C S = 0 0 . 8 V , 0 0 . 0 7 A , 0 0 0 . 1 W , 0 0 0 . 0 0 W h , 0 1 5 . 2 C , 0 1 3 . 5 C , 0 1 5 . 6 C , 0 1 5 . 4 C , 0 . 6 2 B , 0 4 . 5 3 B , 3 9 . 1 V , 0 2 5 . 6 C , 5 , 0 0 0 . 0 C , 2 0 . 0 % LF
 */

#include <termios.h>
#include <fcntl.h>
#include <drivers/drv_hrt.h>
//#include <perf/perf_counter.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
//#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/fc_values.h>
//#include <uORB/Subscription.hpp>
//#include <uORB/topics/FC_values.h>
//#include <uORB/topics/gps_inject_data.h>
//#include <uORB/topics/sensor_gps.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
//#include "devices/src/ashtech.h"      GAVE ERRORS
//#include "devices/src/emlid_reach.h"
//#include "devices/src/mtk.h"
//#include "devices/src/ubx.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
//#include <algorithmfwd>
//#include "iostream.h"
//#include <toolchain/cygwin64/lib/gcc/x86_64-pc-cygwin/9.3.0/include/c++/iostream.h>
//#include <toolchain/cygwin64/lib/gcc/x86_64-pc-cygwin/9.3.0/include/c++/iterator.h>

//#define ORB_DECLARE(FC_values) extern const struct orb_metadata __orb_##FC_values__EXPORT


using namespace time_literals; //TODO: what's this?

/* Configuration Constants */
static constexpr uint32_t FC_MEASURE_INTERVAL{50_ms};    // 50ms default sensor conversion time. TODO: whta's a right value for this?
//first character of each line is 'F'
static constexpr unsigned char START_FRAME_DIGIT1{0x46}; //ASCII 'F'
//needed ASCII number codes
static constexpr unsigned char ASCII_0{0x30}; //ASCII '0'
static constexpr unsigned char ASCII_1{0x31}; //ASCII '1'
static constexpr unsigned char ASCII_2{0x32}; //ASCII '2'
static constexpr unsigned char ASCII_3{0x33}; //ASCII '3'
static constexpr unsigned char ASCII_4{0x34}; //ASCII '4'
static constexpr unsigned char ASCII_5{0x35}; //ASCII '5'
static constexpr unsigned char ASCII_6{0x36}; //ASCII '6'
static constexpr unsigned char ASCII_7{0x37}; //ASCII '7'
static constexpr unsigned char ASCII_8{0x38}; //ASCII '8'
static constexpr unsigned char ASCII_9{0x39}; //ASCII '9'
//needed ASCII letter codes
static constexpr unsigned char ASCII_A{0x41}; //ASCII 'A'
static constexpr unsigned char ASCII_B{0x42}; //ASCII 'B'
static constexpr unsigned char ASCII_C{0x43}; //ASCII 'C'
static constexpr unsigned char ASCII_S{0x53}; //ASCII 'S'
static constexpr unsigned char ASCII_V{0x56}; //ASCII 'V'
static constexpr unsigned char ASCII_W{0x57}; //ASCII 'W'
static constexpr unsigned char ASCII_h{0x68}; //ASCII 'h'
//ASCII special characters
static constexpr unsigned char ASCII_LINE_FEED{0x0A}; //ASCII 'LF': new line. At end of every frame
static constexpr unsigned char ASCII_PERCENT{0x25}; //ASCII '%'
static constexpr unsigned char ASCII_COMMA{0x2C}; //ASCII ','
static constexpr unsigned char ASCII_POINT{0x2E}; //ASCII '.'
static constexpr unsigned char ASCII_EQUALS{0x3D}; //ASCII '='
static constexpr unsigned char ASCII_SPACE{0x20}; //ASCII ' '

static constexpr uint8_t PARSER_BUF_LENGTH{132}; //including LINE_FEED character

static constexpr float MIN_STACK_VOLTAGE{32.4}; //0.6 V/cell, 54 cells
static constexpr float STACK_SHUTOFF_VOLTAGE{27.0}; //0.5 V/cell, 54 cells
//static constexpr float MAX_LOAD_CURRENT{};
//static constexpr float MIN_LOAD_CURRENT{};
//static constexpr float MIN_POWER{};
static constexpr float MAX_POWER{1500.0}; //W
//static constexpr float MIN_ENERGY{};
//static constexpr float MAX_ENERGY{};
static constexpr float MIN_BAT_VOLTAGE{3.4}; //V
static constexpr float BAT_SHUTOFF_VOLTAGE{3.0}; //V
//static constexpr float MIN_BAT_CURRENT{};
//static constexpr float MAX_BAT_CURRENT{};
//static constexpr float MIN_LD_VOLTAGE{};
//static constexpr float MAX_LD_VOLTAGE{};
static constexpr float MIN_TEMP_SENSOR_1{0.0};
static constexpr float MAX_TEMP_SENSOR_1{67.0};
static constexpr float MIN_TEMP_SENSOR_2{0.0};
static constexpr float MAX_TEMP_SENSOR_2{67.0};
static constexpr float MIN_TEMP_SENSOR_3{0.0};
static constexpr float MAX_TEMP_SENSOR_3{67.0};
static constexpr float MIN_TEMP_SENSOR_4{0.0};
static constexpr float MAX_TEMP_SENSOR_4{67.0};
//static constexpr float MIN_TST_TEMP{};
//static constexpr float MAX_TST_TEMP{};
//static constexpr float MIN_PCB_TEMP{};
//static constexpr float MAX_PCB_TEMP{};
static constexpr float MIN_SIDE_H2_PRESSURE{0.7}; //bar
static constexpr float MAX_SIDE_H2_PRESSURE{0.9}; //bar
//static constexpr float MIN_MVPRS{};
//static constexpr float MAX_MVPRS{};
//static constexpr float MIN_FAN_SPEED{};
//static constexpr float MAX_FAN_SPEED{};


class FC : public px4::ScheduledWorkItem //TODO: public ModuleBase<FC>, drivers needs to run continuously? + implement custom_command?
{
public:

/**
 * Default Constructor
 * @param port The serial port to open for communicating with the sensor.
 * @param rotation The sensor rotation relative to the vehicle body.
 */
    FC(const char *port);

/** Virtual destructor */
    virtual ~FC() override;

/**
 * Method : init()
 * This method initializes the general driver
 */
    int init();

/**
 * Diagnostics - print some basic information about the driver.
 */
    void print_info();

private:

    int _file_descriptor{-1};

// const char *_port = "/dev/ttyS4"; //serial port TELEM4
    char _port[20] {};

    uORB::PublicationMultiData<fc_values_s> _sensor_FC_pub {ORB_ID(fc_values)}; //generates a subscription to the uORB topic FC_vales?

    fc_values_s &_report = _sensor_FC_pub.get();
/**
 * see ModuleBase
 */
//int task_spawn(int argc, char *argv[]);

/**
 * see ModuleBase
 */
//int print_usage(const char *reason)

/**
 * see ModuleBase
 */
//int print_status() override;  //TODO: why is this fella grey?

/**
 * Reads data from serial UART and places it into a buffer.
 */
    int collect();

/**
 * parse a normally formatted data line
 * @return PX4_OK if succesful
 */
    int data_parser(fc_values_s &report); //for correctly formatted data line

/**
 * sends a message line to fc_values.msg.message field. Is an array of char
 * @return
 */
    int message_to_uORB(int index, fc_values_s &report);

/**
 * Opens and configures the UART serial communications port.
 * @param speed The baudrate (speed) to configure the serial UART port.
 */
    int open_serial_port(const speed_t speed = B19200); //baudrate 19200

/**
 * checks the values published to the topic and send sout warning if values exceed certain limits
 */
    int check_values(fc_values_s &report);

/**
 * checks the message published in the topic and compares it to some standard messages
 * acts accordingly
 */
    int check_messages(fc_values_s &report);

/**
 * Perform a reading cycle; collect from the previous measurement
 * and start a new one.
 */
    void Run() override;

/**
 * Initialise the automatic measurement state machine and start it.
 * @note This function is called at open and error time.  It might make sense
 *       to make it more aggressive about resetting the bus in case of errors.
 */
    void start();

/**
 * Stops the automatic measurement state machine.
 */
    void stop();

/**
 * performance timers
 */
    perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")}; //local counter
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")}; //local counter

/**
 * data buffer
 */
    char dataline[PARSER_BUF_LENGTH]{}; //frame length is 132 bytes, including '\n' and spaces used to be unsigned char

//PARSE_STATE _parse_state{PARSE_STATE::WAITING_FRAME};

    unsigned char _frame_data[PARSER_BUF_LENGTH]{};
};


FC::FC(const char *port) : ScheduledWorkItem(MODULE_NAME,px4::serial_port_to_wq(port))
                            //_sensor_FC_pub(ORB_ID(FC_values))
{
    //store port name in _port
    strncpy(_port, port, sizeof(_port) - 1);

    //enforce null termination
    _port[sizeof(_port) - 1] = '\0';

    //advertise uORB topic TODO: check with Jakob if this has to be advertise_multi
    //_sensor_FC_pub{ORB_ID(FC_values), 75}
    _sensor_FC_pub.advertise();
    //&report = _sensor_FC_pub.get();
}

FC::~FC() {
    // Ensure we are truly inactive.
    stop();

    perf_free(_sample_perf);
    perf_free(_comms_errors);

    //unadvertise uORB topic
    _sensor_FC_pub.unadvertise();
}

int
FC::init() {
    start();

    return PX4_OK;
}
/*
int
FC::task_spawn(int argc, char *argv[])
{
    PX4_INFO("task spawn called");
    FC *instance = new FC("/dev/ttyS3"); //TODO: what's this instance for?

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init() == PX4_OK) {
            PX4_INFO("task spawn and init successful");
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}
*/
int
FC::open_serial_port(const speed_t speed) {
    // File descriptor initialized?
    if (_file_descriptor > 0) {
        PX4_INFO("serial port already open");
        return PX4_OK;
    }

    // Configure port flags for read, non-controlling, non-blocking. TODO: figure out blocking or non-blocking
    int flags = (O_RDONLY | O_NOCTTY | O_NONBLOCK);

    // Open the serial port.
    _file_descriptor = ::open(_port, flags);

    if (_file_descriptor < 0) {
        PX4_ERR("open failed (%i)", errno);
        return PX4_ERROR;
    }

    termios uart_config = {};

    // Store the current port configuration. attributes.
    tcgetattr(_file_descriptor, &uart_config);

    // Clear ONLCR flag (which appends a CR for every LF).
    uart_config.c_oflag &= ~ONLCR;

    // No parity, one stop bit.
    // No parity, one stop bit.
    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    // Set the input baud rate in the uart_config struct.
    int termios_state = cfsetispeed(&uart_config, speed);

    if (termios_state < 0) {
        PX4_ERR("CFG: %d ISPD", termios_state);
        ::close(_file_descriptor);
        return PX4_ERROR;
    }
    //TODO: check if output settings are needed
    /*
    // Set the output baud rate in the uart_config struct.
    termios_state = cfsetospeed(&uart_config, speed);

    if (termios_state < 0) {
        PX4_ERR("CFG: %d OSPD", termios_state);
        ::close(_file_descriptor);
        return PX4_ERROR;
    }
     */

    // Apply the modified port attributes.
    termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config); //TODO: what does this do?

    if (termios_state < 0) {
        PX4_ERR("baud %d ATTR", termios_state);
        ::close(_file_descriptor);
        return PX4_ERROR;
    }

    PX4_INFO("successfully opened UART port %s", _port);
    return PX4_OK;
}

void
FC::print_info() {
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}
/*
int
FC::print_usage(const char *reason){
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
FC readout driver which reads out the FC data values transmitted via RS-232 protocol via the TELEM4 port.
Data is read in byte by byte (in ASCII code) and stored in a data array of 132 bytes long. This is the size of a normal data line
including the LINE_FEED character at the end of each line
Data line can be a message or a series of FC measurement values

uORB topic: FC_values

FOR MEASUREMENTS
    Data line is split up into parts corresponding to fuel cell parameters. Parameter values are added to uORB topic FC_values
FOR MESSAGE
    Full data line, containing the message, is added to corresponding uORB parameter in uORB topic FC_vlaues
)DESCR_STR");

    return 0;
}
*/
/*
int
FC::print_status() {
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);

    return 0;
}
*/
void
FC::Run() {
    // Ensure the serial port is open.
    open_serial_port();

    // Perform collection.
    collect();

    check_messages(_report);
    check_values(_report);
}

void
FC::start() {
    // Schedule the driver at regular intervals. TODO: shouldn't this driver work continuously?
    ScheduleOnInterval(FC_MEASURE_INTERVAL);

    PX4_INFO("driver started");
}

void
FC::stop() {
    // Clear the work queue schedule.
    ScheduleClear();

    // Ensure the serial port is closed.
    ::close(_file_descriptor);
}

int
FC::collect() //TODO: code conditions if bytes_read is not <= 0 AND closing of timer and stuff
{
    perf_begin(_sample_perf); //start timer

    int index = 0;
    int bytes_read = -1;

    for (int i = 0; i < PARSER_BUF_LENGTH; i++) { //re-initialize dataline array with empty characters
        dataline[i] = '\0';
    }

    uint8_t single_byte[1] {};
    bool end_of_line = false;

    while (!end_of_line) { //read a single output byte of FC rs-232
        bytes_read = ::read(_file_descriptor, &single_byte[0], 1); //so bytes_read can serve as index as well
        if (bytes_read > 0 && index < PARSER_BUF_LENGTH) {
            dataline[index] = (char) single_byte[0]; //TODO: check this conversion
            index = index + 1;
        } else if (bytes_read <= 0) {
            end_of_line = false;
            PX4_ERR("no bytes read in");
            //perf_count(_comms_errors);
            //perf_end(_sample_perf);
            //return PX4_ERROR;
        } else if (index >= PARSER_BUF_LENGTH) {
            //too long of an array. Keep on reading in bytes but don't store them
            //redundant statement
        }
        if(single_byte[0] == ASCII_LINE_FEED){
            end_of_line = true;
        }
    }
    PX4_INFO("successfully read in the data line");

    //check if dataline is a normal dataline
    if (index == PARSER_BUF_LENGTH - 1) { //so 132 iterations in while loop
        PX4_INFO("dataline of correct size");
        perf_end(_sample_perf); //TODO: what is the use of this timer?
        return data_parser(_report);   // PX4_ERROR or PX4_OK
    } else if (index < PARSER_BUF_LENGTH - 1 && index > 0) {
        PX4_INFO("dataline is message");
        perf_end(_sample_perf); //TODO: what is the use of this timer?
        return message_to_uORB(index, _report);
    }

    perf_end(_sample_perf);
    return PX4_ERROR;
}

int
FC::message_to_uORB(int index, fc_values_s &report) {
    //uORB subscription stuff TODO: ask Jakob
    //fc_values_s &report = _sensor_FC_pub.get();

    //int length_message = index + 1;
    //char message[PARSER_BUF_LENGTH-1];
    //for(int i = 0; i<PARSER_BUF_LENGTH; i++){
    //    message[i] = '\0';
    //}
    //strcpy(message, dataline); //used to be std::copy
    memcpy(report.message, dataline, PARSER_BUF_LENGTH-1);
    //report.message = message; //error: invalid array assignment
    report.timestamp = hrt_absolute_time();
    bool ret = _sensor_FC_pub.update();

    //delete[] message; //free up RAM

    if (ret) {
        PX4_INFO("message sent to uORB");
        return PX4_OK;
    }
    PX4_ERR("failed to send message to uORB");
    return PX4_ERROR;
}

int
FC::data_parser(fc_values_s &report) {
    //here a normal data line gets parsed, so split up into pieces and values get assigned to parameters in fc_values.msg
    /** works if dataline contains correct sequence of 132 bytes, including LINE_FEED */
    if (dataline[0] == START_FRAME_DIGIT1) { //if first character equals 'F'
        //read out every character according to the FC sequence pattern
        //ex: FCS=50.9V,00.11A,005.8W,000.00Wh,BAT=41.2V,00.2A,LD=41.1V, 19.0C,18.2C,18.6C,17.8C,TST=00.0C,PCB=018.6C, 0.00B,056.4mVPrs, 30.0%,5,
        int index = 0;
        //stack voltage [V]
        index = 4; //first digit of stack voltage
        char digits1[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                          dataline[index + 3]}; //{'5', '0', '.', '9' } V
        float value;
        value = strtof(digits1, NULL);
        if(value != HUGE_VALF){
            report.stack_voltage = value;
        } else {PX4_ERR("error reading stack voltage value");}
        //_sensor_FC_pub.update();

        //load current [A]
        index = 10; //first digit of load current
        char digits2[] = {dataline[index], dataline[index + 1], dataline[index + 2], dataline[index + 3],
                          dataline[index + 4]}; //{'0','0','.','1','1'} A
        value = strtof(digits2, NULL);
        if(value != HUGE_VALF){ //normally doesn't happen
            report.load_current = value;
        } else {PX4_ERR("error reading load current value");}
        //_sensor_FC_pub.update();

        //power [W]
        index = 17; //first digit of power
        char digits3[] = {dataline[index], dataline[index + 1], dataline[index + 2], dataline[index + 3],
                          dataline[index + 4]}; //{'0','0','5','.','8'} W
        value = strtof(digits3, NULL);
        if(value != HUGE_VALF){ //normally doesn't happen
            report.power = value;
        } else {PX4_ERR("error reading power value");}
        //_sensor_FC_pub.update();

        //energy [Wh]
        index = 24; //first digit of energy
        char digits4[] = {dataline[index], dataline[index + 1], dataline[index + 2], dataline[index + 3],
                          dataline[index + 4], dataline[index + 5]}; //{'0', '0', '0', '.', '0', '0' } Wh
        value = strtof(digits4, NULL);
        if(value != HUGE_VALF){
            report.energy = value;
        } else {PX4_ERR("error reading energy value");}
        //_sensor_FC_pub.update();

        //BAT voltage? TODO: what's this?
        index = 37; //first digit of BAT voltage
        char digits5[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                          dataline[index + 3]}; //{'4', '1', '.', '2' } V
        value = strtof(digits5, NULL);
        if(value != HUGE_VALF) {
            report.bat_voltage = value;
        } else {PX4_ERR("error reading BAT voltage value");}
        //_sensor_FC_pub.update();

        //BAT current? TODO: what's this?
        index = 43; //first digit of BAT current
        char digits6[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                          dataline[index + 3]}; //{'0', '0', '.', '2' } A
        value = strtof(digits6, NULL);
        if(value != HUGE_VALF){
            report.bat_current = value;
        } else {PX4_ERR("error reading BAT current value");}
        //_sensor_FC_pub.update();


        //LD voltage? TODO: what's this?
        index = 52; //first digit of LD voltage
        char digits7[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                          dataline[index + 3]}; //{'4', '1', '.', '1' } V
        value = strtof(digits7, NULL);
        if(value != HUGE_VALF){
            report.ld_voltage = value;
        } else {PX4_ERR("error reading LD voltage value");}
        //_sensor_FC_pub.update();

        //temp sensor 1
        index = 59; //first digit of temperature 1
        char digits8[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                          dataline[index + 3]}; //{'1', '9', '.', '0' } C
        value = strtof(digits8, NULL);
        if(value != HUGE_VALF){
            report.temp_sensor_1_celc = value;
        } else {PX4_ERR("error reading temp 1 value");}
        //_sensor_FC_pub.update();

        //temp sensor 2
        index = 65; //first digit of temperature 2
        char digits9[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                          dataline[index + 3]}; //{'1', '8', '.', '2' } C
        value = strtof(digits9, NULL);
        if(value != HUGE_VALF){
            report.temp_sensor_2_celc = value;
        } else {PX4_ERR("error reading temp 2 value");}
        //_sensor_FC_pub.update();

        //temp sensor 3
        index = 71; //first digit of temperature 3
        char digits10[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                           dataline[index + 3]}; //{'1', '8', '.', '6' } C
        value = strtof(digits10, NULL);
        if(value != HUGE_VALF){
            report.temp_sensor_3_celc = value;
        } else {PX4_ERR("error reading temp 3 value");}
        //_sensor_FC_pub.update();

        //temp sensor 4
        index = 77; //first digit of temperature 4
        char digits11[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                           dataline[index + 3]}; //{'1', '7', '.', '8' } C
        value = strtof(digits11, NULL);
        if(value != HUGE_VALF){
            report.temp_sensor_4_celc = value;
        } else {PX4_ERR("error reading temp 4 value");}
        //_sensor_FC_pub.update();

        //TST temperature TODO: what's this?
        index = 87; //first digit of temperature TST
        char digits12[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                           dataline[index + 3]}; //{'0', '0', '.', '0' } C
        value = strtof(digits12, NULL);
        if(value != HUGE_VALF){
            report.tst_temperature_celc = value;
        } else {PX4_ERR("error reading TST temp value");}
        //_sensor_FC_pub.update();

        //PCB temperature TODO: what's this?
        index = 97; //first digit of temperature PSB
        char digits13[] = {dataline[index], dataline[index + 1], dataline[index + 2], dataline[index + 3],
                           dataline[index + 4]}; //{'0', '1','8', '.', '6' } C
        value = strtof(digits13, NULL);
        if(value != HUGE_VALF){
            report.pcb_temperature_celc = value;
        } else {PX4_ERR("error reading value");}
        //_sensor_FC_pub.update();

        //stzck side H2 supply pressure TODO: which pressure is this? not sure
        index = 105; //first digit of pressure
        char digits14[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                           dataline[index + 3]}; //{'0', '0','.', '0'} B
        value = strtof(digits14, NULL);
        if(value != HUGE_VALF){
            report.side_h2_pressure = value;
        } else {PX4_ERR("error reading side H2 pressure value");}
        //_sensor_FC_pub.update();

        //mVPrs TODO: what's this?
        index = index + 6; //first digit of mVPrs
        char digits15[] = {dataline[index], dataline[index + 1], dataline[index + 2], dataline[index + 3],
                           dataline[index + 4]}; //{'0', '5','6','.', '4'} mVPrs
        value = strtof(digits15, NULL);
        if(value != HUGE_VALF){
            report.mvprs = value;
        } else {PX4_ERR("error reading mVPrs value");}
        //_sensor_FC_pub.update();

        //fan speed [%]
        index = index + 12; //first digit of fan speed
        char digits16[] = {dataline[index], dataline[index + 1], dataline[index + 2],
                           dataline[index + 3]}; //{'3','0','.', '0'} %
        value = strtof(digits16, NULL);
        if(value != HUGE_VALF){
            report.fan_speed = value;
        } else {PX4_ERR("error reading fan speed value");}
        //_sensor_FC_pub.update();

        //operation status: INTEGER
        index = index + 6; //operaion status
        int status = dataline[index] - '0'; //convert for example '1' to 1
        if(status >=0 && status <=9){ //if it's a number
            report.operation_status = dataline[index] - '0';
        } else {PX4_ERR("error reading operation status");}
        //_sensor_FC_pub.update();

        report.timestamp = hrt_absolute_time();
        _sensor_FC_pub.update();

        delete[] &digits1;
        delete[] &digits2;
        delete[] &digits3;
        delete[] &digits4;
        delete[] &digits5;
        delete[] &digits6;
        delete[] &digits7;
        delete[] &digits8;
        delete[] &digits9;
        delete[] &digits10;
        delete[] &digits11;
        delete[] &digits12;
        delete[] &digits13;
        delete[] &digits14;
        delete[] &digits15;
        delete[] &digits16;

        if (dataline[index + 2] == ASCII_LINE_FEED) {
            PX4_INFO("last character LF");
            return PX4_OK;
        }
        PX4_ERR("last character not LF");
        return PX4_ERROR;
    }
    PX4_ERR("data line does not start with 'F'");
    return PX4_ERROR;
}
/*
int
FC::data_parser(uint8_t parserbuf, PARSE_STATE &state)
{
    FC_vales &report = _sensor_FC_pub.get();
    int j = 0;
    while(j < sizeof(parserbuf)){
        uint8_t kar;
        kar = parserbuf[j];
        if(kar != START_FRAME_DIGIT1){
            _parse_state = PARSE_STATE::SKIP_LINE;
        }
        switch(state) {
            case PARSE_STATE::FCS: //so fuel cell voltage: FCS=
                if(kar == START_FRAME_DIGIT1){ //first ASCII character equals 'F'
                    j = j +4 ; //skip the next four characters:'CS='
                }
                if(kar == ASCII_C){ //index is at C of FCS=
                    j = j+ 3; //skip the next three characters: 'S='
                }
                if(kar == ASCII_S){ //index is at S of FCS=
                    j = j + 2;
                }
                if(kar == ASCII_EQUALS){ //index is at = of FCS=
                    j = j + 1;
                }
                char value[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3]};
                j = j + 3; //put index on last digit (first decimal)
                j = j + 4; //put index on first digit of load current
                float stack_voltage;
                stack_voltage = strtof(value, NULL);
                report.stack_voltage = stack_voltage;
                _parse_state = PARSE_STATE::LOAD_CURRENT;

                break;

            case PARSE_STATE::LOAD_CURRENT:
                char value2[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3], parserbuf[j+4]};
                j = j + 4; //put index on last digit of load current value
                j = j + 4; //put index on first digit of power value
                float load_crrnt;
                load_crrnt = strtof(value2, NULL);
                report.load_current = load_crrnt;
                _parse_state = PARSE_STATE::POWER;

                break;

            case PARSE_STATE::POWER:
                char value3[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3], parserbuf[j+4]};
                j = j + 4; //put index on last digit of power value
                j = j + 4; //put index on first digit of energy value
                float pwr;
                pwr = strtof(value3, NULL);
                report.power = pwr;
                _parse_state = PARSE_STATE::ENERGY;

                break;

            case PARSE_STATE::ENERGY:
                char value4[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3], parserbuf[j+4], parserbuf[j+5]};
                j = j + 5; //put index at last digit of energy
                j = j + 5; //put index on first digit of tamp sensor 1
                float enrgy;
                enrgy = strtof(value4, NULL);
                report.energy = enrgy;
                _parse_state = PARSE_STATE::TEMP_1;

                break;

            case PARSE_STATE::TEMP_1:
                char value5[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3], parserbuf[j+4]};
                j = j + 4; //put index at last digit of energy
                j = j + 4; //put index on first digit of tamp sensor 1
                float temp;
                temp = strtof(value5, NULL);
                report.temp_sensor_1_celc = temp;
                _parse_state = PARSE_STATE::TEMP_2;

                break;

            case PARSE_STATE::TEMP_2:
                char value6[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3], parserbuf[j+4]};
                j = j + 4; //put index at last digit of energy
                j = j + 4; //put index on first digit of tamp sensor 1
                float temp;
                temp = strtof(value6, NULL);
                report.temp_sensor_2_celc = temp;
                _parse_state = PARSE_STATE::TEMP_3;

                break;

            case PARSE_STATE::TEMP_3:
                char value7[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3], parserbuf[j+4]};
                j = j + 4; //put index at last digit of energy
                j = j + 4; //put index on first digit of tamp sensor 1
                float temp;
                temp = strtof(value7, NULL);
                report.temp_sensor_3_celc = temp;
                _parse_state = PARSE_STATE::TEMP_4;

                break;

            case PARSE_STATE::TEMP_4:
                char value8[] = {parserbuf[j], parserbuf[j+1], parserbuf[j+2], parserbuf[j+3], parserbuf[j+4]};
                j = j + 4; //put index at last digit of energy
                j = j + 4; //put index on first digit of tamp sensor 1
                float temp;
                temp = strtof(value8, NULL);
                report.temp_sensor_4_celc = temp;
                _parse_state = PARSE_STATE::STACK_SUPPLY_PRESSURE;

                break;
        }
     }
    return PX4_ERROR;
}
 */

int
FC::check_values(fc_values_s &report) {
    //fc_values_s &report = _sensor_FC_pub.get();
    if(report.stack_voltage < MIN_STACK_VOLTAGE && report.stack_voltage > STACK_SHUTOFF_VOLTAGE){
        PX4_ERR("stack voltage LOW");
    } else if (report.stack_voltage < STACK_SHUTOFF_VOLTAGE){
        PX4_ERR("stack voltage SHUTOFF");
    }

    if (report.power > MAX_POWER){
        PX4_ERR("power HIGH");
    }

    if (report.bat_voltage < MIN_BAT_VOLTAGE && report.bat_voltage > BAT_SHUTOFF_VOLTAGE){
        PX4_ERR("battery voltage LOW");
    } else if(report.bat_voltage < BAT_SHUTOFF_VOLTAGE){
        PX4_ERR("battery voltage SHUTOFF");
    }

    if (report.temp_sensor_1_celc > MAX_TEMP_SENSOR_1){
        PX4_ERR("temp sensor 1 HIGH");
    } else if (report.temp_sensor_1_celc < MIN_TEMP_SENSOR_1){
        PX4_ERR("temp sensor 1 LOW");
    }
    if (report.temp_sensor_2_celc > MAX_TEMP_SENSOR_2){
        PX4_ERR("temp sensor 2 HIGH");
    } else if (report.temp_sensor_2_celc < MIN_TEMP_SENSOR_2){
        PX4_ERR("temp sensor 2 LOW");
    }
    if (report.temp_sensor_3_celc > MAX_TEMP_SENSOR_3){
        PX4_ERR("temp sensor 3 HIGH");
    } else if (report.temp_sensor_3_celc < MIN_TEMP_SENSOR_3){
        PX4_ERR("temp sensor 3 LOW");
    }
    if (report.temp_sensor_4_celc > MAX_TEMP_SENSOR_4){
        PX4_ERR("temp sensor 4 HIGH");
    } else if (report.temp_sensor_4_celc < MIN_TEMP_SENSOR_4){
        PX4_ERR("temp sensor 4 LOW");
    }

    if (report.side_h2_pressure > MAX_SIDE_H2_PRESSURE){
        PX4_ERR("side H2 pressure HIGH");
    } else if (report.side_h2_pressure < MIN_SIDE_H2_PRESSURE){
        PX4_ERR("side H2 pressure LOW");
    }

    return PX4_OK;
}

int FC::check_messages(fc_values_s &report) {
    //fc_values_s &report = _sensor_FC_pub.get();
    char message[131];
    strcpy(message, report.message);
    if(strcmp(message,"DB3:SetNormalShutdown_cFCS ...") == 0){
        PX4_ERR("SHUTDOWN");
    }

    return PX4_OK;
}

/***********************************************************************************************************************
 * functions and namespace needed to define main function
 * *********************************************************************************************************************
 */

/**
 * Local functions in support of the shell command.
 */
namespace fc
{

    FC	*g_dev;

    int reset(const char *port);
    int start(const char *port);
    int status();
    int stop();
    int usage();

/**
 * Reset the driver.
 */
    int
    reset(const char *port)
    {
        if (stop() == PX4_OK) {
            return start(port);
        }

        return PX4_ERROR;
    }

/**
 * Start the driver.
 */
    int
    start(const char *port)
    {
        if (port == nullptr) {
            PX4_ERR("invalid port");
            return PX4_ERROR;
        }

        if (g_dev != nullptr) {
            PX4_INFO("already started");
            return PX4_OK;
        }

        // Instantiate the driver.
        g_dev = new FC(port);

        if (g_dev == nullptr) {
            PX4_ERR("object instantiate failed");
            return PX4_ERROR;
        }

        if (g_dev->init() != PX4_OK) {
            PX4_ERR("driver start failed");
            delete g_dev;
            g_dev = nullptr;
            return PX4_ERROR;
        }

        return PX4_OK;
    }

/**
 * Print the driver status.
 */
    int
    status()
    {
        if (g_dev == nullptr) {
            PX4_ERR("driver not running");
            return PX4_ERROR;
        }

        g_dev->print_info();

        return PX4_OK;
    }

/**
 * Stop the driver
 */
    int stop()
    {
        if (g_dev != nullptr) {
            delete g_dev;
            g_dev = nullptr;
        }

        return PX4_ERROR;
    }

    int
    usage()
    {
        PX4_INFO("usage: fc command [options]");
        PX4_INFO("command:");
        PX4_INFO("\treset|start|status|stop");
        PX4_INFO("options:");
        //PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
        PX4_INFO("\t-d --device_path");
        return PX4_OK;
    }

} //namespace fc

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int fc_main(int argc, char *argv[])
{
    //uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
    const char *device_path = "/dev/ttyS3"; //TODO: check if it's possible to add "/dev/ttyS3" as inital value
    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        /*
          switch (ch) {
            case 'R': {
                int rot = -1;

                if (px4_get_parameter_value(myoptarg, rot) != 0) {
                    PX4_ERR("rotation parsing failed");
                    return -1;
                }

                rotation = (uint8_t)rot;
                break;
            }

            case 'd':
                device_path = myoptarg;
                break;

            default:
                PX4_WARN("Unknown option!");
                return fc::usage();
        }
         */
    }

    if (myoptind >= argc) {
        return fc::usage();
    }

    // Reset the driver.
    if (!strcmp(argv[myoptind], "reset")) {
        return fc::reset(device_path);
    }

    // Start/load the driver.
    if (!strcmp(argv[myoptind], "start")) {
        return fc::start(device_path);
    }

    // Print driver information.
    if (!strcmp(argv[myoptind], "status")) {
        return fc::status();
    }

    // Stop the driver
    if (!strcmp(argv[myoptind], "stop")) {
        return fc::stop();
    }

    return fc::usage();
}
