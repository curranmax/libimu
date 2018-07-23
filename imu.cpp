
#include "imu.h"

#include <lpsensor/LpmsSensorI.h>
#include <lpsensor/LpmsSensorManagerI.h>

#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <utility>

void incrementData(ImuData& val, ImuData& incr) {
	if(val.openMatId != 0 && val.openMatId != incr.openMatId) {
		// Error?
	}

	for(int x = 0; x < 3; ++x) {
		val.a[x] += incr.a[x];
		val.g[x] += incr.g[x];
		val.b[x] += incr.b[x];
		val.w[x] += incr.w[x];
		val.r[x] += incr.r[x];

		val.aRaw[x] += incr.aRaw[x];
		val.gRaw[x] += incr.gRaw[x];
		val.bRaw[x] += incr.bRaw[x];

		val.linAcc[x] += incr.linAcc[x];
	}

	for(int x = 0; x < 4; ++x) {
		val.q[x] += incr.q[x];
	}

	for(int x = 0; x < 9; ++x) {
		val.rotationM[x]  += incr.rotationM[x];
		val.rotOffsetM[x] += incr.rotOffsetM[x];
	}

	val.pressure    += incr.pressure;
	val.gTemp       += incr.gTemp;
	val.altitude    += incr.altitude;
	val.temperature += incr.temperature;
}

void averageData(ImuData& val, int num) {
	for(int x = 0; x < 3; ++x) {
		val.a[x] /= float(num);
		val.g[x] /= float(num);
		val.b[x] /= float(num);
		val.w[x] /= float(num);
		val.r[x] /= float(num);

		val.aRaw[x] /= float(num);
		val.gRaw[x] /= float(num);
		val.bRaw[x] /= float(num);

		val.linAcc[x] /= float(num);
	}

	for(int x = 0; x < 4; ++x) {
		val.q[x] /= float(num);
	}

	for(int x = 0; x < 9; ++x) {
		val.rotationM[x]  /= float(num);
		val.rotOffsetM[x] /= float(num);
	}

	val.pressure    /= float(num);
	val.gTemp       /= float(num);
	val.altitude    /= float(num);
	val.temperature /= float(num);
}


IMU::~IMU() {
	disconnectDevice();
}

std::pair<bool, ImuData> IMU::getData(int num_iters) const {
	ImuData v;
	if(num_iters > 0 || sensor->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED) {
		return std::make_pair(false, v);
	}

	ImuData sum_data;

	for(int i = 0; i < num_iters; ++i) {
		// TODO fix this busy wait.
		while(!sensor->hasImuData()) {}

		v = sensor->getCurrentData();

		incrementData(sum_data, v);
	}

	if(num_iters > 0) {
		averageData(sum_data, num_iters);
	}

	return std::make_pair(true, sum_data);
}

void IMU::connectDevice() {
	manager = std::unique_ptr<LpmsSensorManagerI>(LpmsSensorManagerFactory());

	sensor = manager->addSensor(DEVICE_LPMS_B2, address.c_str());
}

void IMU::disconnectDevice() {
	if(sensor != nullptr) {
		manager->removeSensor(sensor);
	}
}
