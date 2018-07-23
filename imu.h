
#ifndef _IMU_H_
#define _IMU_H_

#include <lpsensor/LpmsSensorI.h>
#include <lpsensor/LpmsSensorManagerI.h>

#include <memory>
#include <string>
#include <utility>

// Wrapper for a Lpms-B2 sensor
class IMU {
public:
	IMU(const std::string& _address)
			: manager(), sensor(nullptr), address(_address) {
		connectDevice();
	}

	~IMU();

	IMU() = delete;
	IMU(const IMU& imu) = delete;
	const IMU& operator=(const IMU& imu) = delete;

	std::pair<bool, ImuData> getData(int num_iters) const;
	std::pair<bool, ImuData> getData() const { return getData(1); }
private:
	void connectDevice();
	void disconnectDevice();

	std::unique_ptr<LpmsSensorManagerI> manager;

	LpmsSensorI* sensor;
	std::string address;
};

#endif
