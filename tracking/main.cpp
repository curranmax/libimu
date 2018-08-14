
#include "vec.h"

#include <imu/imu.h>

#include <lpsensor/ImuData.h>

#include <gm_utils.h>
#include <simple_fso.h>

#include <math.h>
#include <signal.h>
#include <unistd.h>

#define pi 3.14159265

#define DEFAULT_ADDRESS  "00:04:3E:9B:A3:7E"

bool loop = false;
void loopHandler(int s) {
	loop = false;
}

int main(int argc, char const *argv[]) {
	// Check command line input
	if(argc < 1 || argc > 3) {
		std::cerr << "Usage is:" << std::endl << "tracking [FSO_DATA_FILE] [OUTPUT_FILE]" << std::endl;
	}

	std::string tx_fso_filename = "";
	if(argc >= 2) {
		tx_fso_filename = argv[1];
	}

	std::string out_filename = "";
	if(argc >= 3) {
		out_filename = argv[2];
	}

	// If fso data given, connects to the GMs.
	SimpleFSO* tx_fso = nullptr;
	int init_gm1 = 0, init_gm2 = 0;
	if(tx_fso_filename != "" && tx_fso_filename != "none") {
		tx_fso = new SimpleFSO(tx_fso_filename);

		if(!tx_fso->isGM1Connected() || !tx_fso->isGM2Connected()) {
			std::cerr << "Couldn't connect to both GMs" << std::endl;
			exit(1);
		}

		init_gm1 = tx_fso->getGM1Val();
		init_gm2 = tx_fso->getGM2Val();
	}

	std::ofstream *ofstr = nullptr;
	if(out_filename != "") {
		ofstr = new std::ofstream(out_filename, std::ofstream::out);
	}

	// Connects to the IMU
	IMU imu(DEFAULT_ADDRESS);

	int timeout_s = 10;
	bool is_connected = imu.waitForConnection(timeout_s);
	if(!is_connected) {
		std::cerr << "Unable to connect to IMU within " << timeout_s << " second timeout" << std::endl;
		exit(1);
	}

	// Collects baseline readings for no movement
	Vec calibration;
	int calibration_count = 1000;
	int max_num_failure = 100, num_failure = 0;
	for(int i = 0; i < calibration_count; ++i) {
		std::cout << "Calibration iter " << i + 1 << std::endl;
		auto output = imu.getData();
		if(!output.first) {
			num_failure++;
			if(num_failure >= max_num_failure) {
				break;
			}

			--i;
			continue;
		}
			
		calibration.x += output.second.linAcc[0];
		calibration.y += output.second.linAcc[1];
		calibration.z += output.second.linAcc[2];
	}
	calibration /= float(calibration_count);

	if(num_failure >= max_num_failure) {
		std::cerr << "Couldn't calibrate data" << std::endl;
		exit(1);
	}

	// ---------------
	// |  Constants  |
	// ---------------
	// Location of Transmitter relative to reciever
	const Vec tx_loc(0.0, 2.83, 0.0);
	
	// Number of samples to gather at each iteration
	const int num_samples = 3;

	// Values used in our rest mechanic
	int zero_count = 0;
	const float zero_eps = 0.3;
	const int zero_count_reset = 3;

	// ---------------
	// |  Variables  |
	// ---------------
	
	// Number of values to save
	const int history = 2;
	
	// For each list, element at index i is from iteration (cur_iter - i). So element 0 is from the current iteration, element 1 is from the pervious iteration, ...
	std::vector<Vec> acc(history), velocity(history), position(history);

	// Used in calculating actual time difference between samples for integration.
	float prev_timestamp = -1.0;
	float t = float(num_samples) / 400; // the unit integration time intervel, eg: 512hz t=1/512 second, will change based on number of data took for average.

	// Writes out initial parameters
	if(ofstr != nullptr) {
		(*ofstr) << "PARAMS" << std::endl;
		(*ofstr) << "calibration_count|int|" << calibration_count << std::endl;
		(*ofstr) << "calibration|Vec|" << calibration << std::endl;
		(*ofstr) << "tx_loc|Vec|" << tx_loc << std::endl;
		(*ofstr) << "zero_eps|float|" << zero_eps << std::endl;
		(*ofstr) << "zero_count_reset|int|" << zero_count_reset << std::endl;
		(*ofstr) << "START" << std::endl;
	}

	// Sets it up so that a "Ctrl+c" stops the loop
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = loopHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, nullptr);

	loop = true;
	while(loop) {
		// Gets data from the IMU
		auto output = imu.getData(num_samples);
		if(output.first) {
			// See ImuData.h in LpSensor/include/ for more details.
			ImuData d = output.second;

			// Calculate time since last update.
			float t_delta = 0.0;
			if(prev_timestamp < 0) {
				// Uses the approximate update time for the first iteration.
				t_delta = t;
			} else {
				t_delta = d.timeStamp - prev_timestamp;
			}
			prev_timestamp = d.timeStamp;

			//*********************************** Transform IMU acceleration to world frame *****************************************************
			float m[4] = {0.0, 0.0, 0.0, 0.0};
			m[0] = d.q[0];//w
			m[1] = d.q[1];//x
			m[2] = d.q[2];//y
			m[3] = d.q[3];//z

			// Calculate rotation matrix.
			float mRot[3][3] = {
				{
					float(1.0 - 2.0 * pow(m[2], 2.0) - 2.0 * pow(m[3], 2.0)),
					float(2.0 * m[1] * m[2] - 2.0 * m[0] * m[3]),
					float(2.0 * m[1] * m[3] + 2.0 * m[0] * m[2])
				}, {
					float(2.0 * m[1] * m[2] + 2.0 * m[0] * m[3]),
					float(1.0 - 2.0 * pow(m[1], 2.0) - 2.0 * pow(m[3], 2.0)),
					float(2.0 * m[2] * m[3] - 2.0 * m[0] * m[1])
				}, {
					float(2.0 * m[1] * m[3] - 2.0 * m[0] * m[2]),
					float(2.0 * m[2] * m[3] + 2.0 * m[0] * m[1]),
					float(1.0 - 2.0 * pow(m[1], 2.0) - 2.0 * pow(m[2], 2.0))
				}
			};
			
			// Calculate determinant of rotation matrix.
			float deter = 0.0;
			for(int i = 0; i < 3; ++i) {
				deter = deter + (mRot[0][i] * (mRot[1][(i + 1) % 3] * mRot[2][(i + 2) % 3] - mRot[1][(i + 2) % 3] * mRot[2][(i + 1) % 3]));
			}
			
			// Calculate inverse rotation matrix
			float mInv[3][3];
			for (int i = 0; i < 3; ++i){
			  for (int j = 0; j < 3; ++j)
				mInv[i][j] =
					(
						(mRot[(j + 1) % 3][(i + 1) % 3] * mRot[(j + 2) % 3][(i + 2) % 3]) -
						(mRot[(j + 1) % 3][(i + 2) % 3] * mRot[(j + 2) % 3][(i + 1) % 3])
					) / deter;
			}

			// Use inverse rotation matrix to find acceleration in the world frame
			Vec imu_acc(d.linAcc[0] - calibration.x, d.linAcc[1] - calibration.y, d.linAcc[2] - calibration.z);
			Vec acc_world;
			for (int i = 0; i < 3; ++i){
			  for (int j = 0; j < 3; ++j){
				acc_world[i] += (mInv[i][j] * imu_acc[j] * 9.8);
			  }
			}


			//*********************************** Reset velocity if acceleration is low. *****************************************************
			if(acc_world.mag() < zero_eps) {
				zero_count++;
			} else {
				zero_count = 0;
			}

			//*********************************** Perform discrete integration. *****************************************************
			acc[0] = acc_world;

			if(zero_count < zero_count_reset) {
				// Integrate acceleration to get velocity
				velocity[0] = velocity[1] + (acc[1] + (acc[0] - acc[1]) / 2.0) * t_delta;
			} else {
				// Resets velocity to zero if rest limit has been reached.
				velocity = std::vector<Vec>(history);
			}

			// Integrates position from velocity
			position[0] = position[1] + (velocity[1] + (velocity[0] + velocity[1]) / 2.0) * t_delta;

			//*********************************** Calculates angles for GM mirrors. *****************************************************
			Vec cur_imu_pos = position[0];
			// 5) get current Euler angle and calculate angle resposne with current integrated position
			float tx_angle_1 = 0, tx_angle_2 = 0; // In degrees
			tx_angle_1 = acos((cur_imu_pos.z - tx_loc.z) / sqrt(pow(cur_imu_pos.x - tx_loc.x, 2) + pow(cur_imu_pos.y - tx_loc.y, 2) + pow(cur_imu_pos.z - tx_loc.z, 2))) * 180.0 / pi - 90.0;
			tx_angle_2 = -acos((cur_imu_pos.x - tx_loc.z) / sqrt(pow(cur_imu_pos.x - tx_loc.x, 2) + pow(cur_imu_pos.y - tx_loc.y, 2))) * 180.0 / pi + 90.0;
			
			// Adjust the tx_fso if it is connected.
			int tx_gm1_val = 0, tx_gm2_val = 0;
			tx_gm1_val = init_gm1 + degreeToGMUnit(tx_angle_1);
			tx_gm2_val = init_gm2 + degreeToGMUnit(tx_angle_2);

			if(tx_fso != nullptr) {
				tx_fso->setGM2Val(tx_gm2_val);
			}
			   
			// ****************************** Output ****************************************************
			std::cout << "-----------------------------------------------------------------------" << std::endl;
			std::cout << "q " << d.q[0] << ", " << d.q[1] << ", " << d.q[2] << ", " << d.q[3] << std::endl;
			std::cout << "mRot" << std::endl
					<< mRot[0][0] << ", " << mRot[0][1] << ", " << mRot[0][2] << std::endl
					<< mRot[1][0] << ", " << mRot[1][1] << ", " << mRot[1][2] << std::endl
					<< mRot[2][0] << ", " << mRot[2][1] << ", " << mRot[2][2] << std::endl;
			std::cout << "deter " << deter << std::endl;
			std::cout << "mInv" << std::endl
					<< mInv[0][0] << ", " << mInv[0][1] << ", " << mInv[0][2] << std::endl
					<< mInv[1][0] << ", " << mInv[1][1] << ", " << mInv[1][2] << std::endl
					<< mInv[2][0] << ", " << mInv[2][1] << ", " << mInv[2][2] << std::endl;
			

			std::cout << "imu_acc " << imu_acc << std::endl;
			
			std::cout << "t_delta " << t_delta << std::endl;

			std::cout << "acc_world " << acc_world << std::endl;
			std::cout << "velocity " << velocity[0] << std::endl;
			std::cout << "position " << position[0] << std::endl;

			std::cout << "tx_angle_1 " << tx_angle_1 << std::endl;
			std::cout << "tx_angle_2 " << tx_angle_2 << std::endl;

			std::cout << "tx_gm1_val " << tx_gm1_val << std::endl;
			std::cout << "tx_gm2_val " << tx_gm2_val << std::endl;;

			if(ofstr != nullptr) {
				(*ofstr) << "q|int[4]|" << d.q[0] << "," << d.q[1] << "," << d.q[2] << "," << d.q[3] << " ";
				(*ofstr) << "mRot|matrix|" << mRot[0][0] << "," << mRot[0][1] << "," << mRot[0][2] << ";"
										   << mRot[1][0] << "," << mRot[1][1] << "," << mRot[1][2] << ";"
										   << mRot[2][0] << "," << mRot[2][1] << "," << mRot[2][2] << " ";
				(*ofstr) << "deter|int|" << deter << " ";
				(*ofstr) << "mInv|matrix|" << mInv[0][0] << "," << mInv[0][1] << "," << mInv[0][2] << ";"
										   << mInv[1][0] << "," << mInv[1][1] << "," << mInv[1][2] << ";"
										   << mInv[2][0] << "," << mInv[2][1] << "," << mInv[2][2] << " ";
				(*ofstr) << "imu_acc|Vec|" << imu_acc << " ";
				(*ofstr) << "t_delta|float|" << t_delta << " ";
				(*ofstr) << "acc_world|Vec|" << acc_world << " ";
				
				(*ofstr) << "acc|vector<Vec>|";
				for(unsigned int i = 0; i < acc.size(); ++i) {
					(*ofstr) << acc[i];
					if(i < acc.size() - 1) {
						(*ofstr) << ";";
					}
				}
				(*ofstr) << " ";

				(*ofstr) << "velocity|vector<Vec>|";
				for(unsigned int i = 0; i < velocity.size(); ++i) {
					(*ofstr) << velocity[i];
					if(i < velocity.size() - 1) {
						(*ofstr) << ";";
					}
				}
				(*ofstr) << " ";

				(*ofstr) << "position|vector<Vec>|";
				for(unsigned int i = 0; i < position.size(); ++i) {
					(*ofstr) << position[i];
					if(i < position.size() - 1) {
						(*ofstr) << ";";
					}
				}
				(*ofstr) << " ";

				(*ofstr) << "tx_angle_1|float|" << tx_angle_1 << " ";
				(*ofstr) << "tx_angle_2|float|" << tx_angle_2 << " ";

				(*ofstr) << "tx_gm1_val|float|" << tx_gm1_val << " ";
				(*ofstr) << "tx_gm2_val|float|" << tx_gm2_val;

				(*ofstr) << std::endl;
			}

			//*********************************** Updates data for next iteration. *****************************************************
			for(int i = history - 1; i > 0; --i) {
				acc[i] = acc[i - 1];
				velocity[i] = velocity[i - 1];
				position[i] = position[i - 1];
			}
		}
	}
	signal(SIGINT, SIG_DFL);

	if(tx_fso != nullptr) {
		delete tx_fso;
	}

	if(ofstr != nullptr) {
		ofstr->close();
		delete ofstr;
	}
}

