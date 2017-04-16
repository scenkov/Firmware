/****************************************************************************
 *
 * Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_ltc2946_wrapper.cpp
 * Lightweight driver to access the Ltc2946 of the DriverFramework.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_ltc.h>

#include <board_config.h>

#include <ltc2946/LTC2946.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_ltc2946_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfLtc2946Wrapper : public LTC2946
{
public:
	DfLtc2946Wrapper();
	~DfLtc2946Wrapper();


	/**
	 * Start automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		start();

	/**
	 * Stop automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		stop();

private:
	int _publish(struct ltc2946_sensor_data &data);

	orb_advert_t		_ltc_topic;

	int			_ltc_orb_class_instance;

	perf_counter_t		_ltc_sample_perf;

};

DfLtc2946Wrapper::DfLtc2946Wrapper() :
	LTC2946(LTC2946_DEVICE_PATH),
	_ltc_topic(nullptr),
	_ltc_orb_class_instance(-1),
	_ltc_sample_perf(perf_alloc(PC_ELAPSED, "df_ltc_read"))
{
}

DfLtc2946Wrapper::~DfLtc2946Wrapper()
{
	perf_free(_ltc_sample_perf);
}

int DfLtc2946Wrapper::start()
{

//	PX4_ERR(">>>>>> Ltc2946Wrapper::start() <<<<<<<<<<<<");


	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("Ltc2946 init fail: %d", ret);
		return ret;
	}

	ret = LTC2946::start();

	if (ret != 0) {
		PX4_ERR("Ltc2946 start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfLtc2946Wrapper::stop()
{
	/* Stop sensor. */
	int ret = LTC2946::stop();

	if (ret != 0) {
		PX4_ERR("Ltc2946 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfLtc2946Wrapper::_publish(struct ltc2946_sensor_data &data)
{
	perf_begin(_ltc_sample_perf);

	ltc_report ltc_report = {};
	ltc_report.timestamp = hrt_absolute_time();

    // Board
	float v_now = m_sensor_data.board_voltage_V/4095.0 * 102.4;                //102.4V is maximum voltage on this input
	//ltc_report.board_voltage = v_now;

	float r_sense = R_SENSE_APM_5V;
	float i_now = m_sensor_data.board_current_A/4095.0 * 0.1024 / r_sense;     //0.1024V is maximum voltage on this input, 0.001 Ohm resistor

	// ESC
	float esc_v_now = m_sensor_data.esc_voltage_V/4095.0 * 102.4;                //102.4V is maximum voltage on this input

	r_sense = R_SENSE_APM_VBATT;
	float esc_i_now = m_sensor_data.esc_current_A/4095.0 * 0.1024 / r_sense;     //0.1024V is maximum voltage on this input, 0.001 Ohm resistor

	if (m_sensor_data.read_counter == 1) {
		ltc_report.board_voltage = v_now;       //if first time, initialize voltage to value now
		ltc_report.board_current = i_now;
		ltc_report.esc_voltage = esc_v_now;     //if first time, initialize voltage to value now
		ltc_report.esc_current = esc_i_now;
	}

	ltc_report.board_voltage = ltc_report.board_voltage * 0.01 + v_now * 0.99;     //filter voltage
	ltc_report.board_current = ltc_report.board_current * 0.01 + i_now * 0.99;     //filter current

	ltc_report.esc_voltage = ltc_report.esc_voltage * 0.01 + esc_v_now * 0.99;     //filter voltage
	ltc_report.esc_current = ltc_report.esc_current * 0.01 + esc_i_now * 0.99;     //filter current

	ltc_report.esc_and_board_consumption = ltc_report.esc_current + ((ltc_report.board_current * ltc_report.board_voltage) / ltc_report.esc_voltage);

/*
	DF_LOG_ERR("<<<<<<<<<<<<<<<<<<<LTC2946::     esc_and_board_consumption %f",ltc_report.esc_and_board_consumption);

	if (m_sensor_data.read_counter % 300 == 0)
	{
		DF_LOG_ERR("...............LTC2946::_measure voltage = %f, current %f , %d",
				ltc_report.board_voltage,
				ltc_report.board_current,
				m_sensor_data.read_counter );

		DF_LOG_ERR("...............LTC2946::    _measure ESC = %f, current %f , %d",
				ltc_report.esc_voltage,
				ltc_report.esc_current,
				m_sensor_data.read_counter );
	}
*/
	// TODO: when is this ever blocked?
	if (!(m_pub_blocked)) {

		if (_ltc_topic == nullptr) {
			_ltc_topic = orb_advertise_multi(ORB_ID(sensor_ltc), &ltc_report,
							  &_ltc_orb_class_instance, ORB_PRIO_DEFAULT);

		} else {
			orb_publish(ORB_ID(sensor_ltc), _ltc_topic, &ltc_report);
		}
	}

	/* Notify anyone waiting for data. */
	DevMgr::updateNotify(*this);

	perf_end(_ltc_sample_perf);

	return 0;
};


namespace df_ltc2946_wrapper
{

DfLtc2946Wrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
	g_dev = new DfLtc2946Wrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfLtc2946Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfLtc2946Wrapper start failed");
		return ret;
	}

	// Open the LTC2946 sensor
	DevHandle h;
	DevMgr::getHandle(LTC2946_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
				LTC2946_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	return 0;
}

void
usage()
{
	PX4_WARN("Usage: df_ltc2946_wrapper 'start', 'info', 'stop'");
}

} // namespace df_ltc2946_wrapper


int
df_ltc2946_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_ltc2946_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_ltc2946_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_ltc2946_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_ltc2946_wrapper::info();
	}

	else {
		df_ltc2946_wrapper::usage();
		return 1;
	}

	return ret;
}
