
#include <imu/imu.h>

#include <lpsensor/ImuData.h>

#include <math.h>
#include <unistd.h>

#define pi 3.14159265

#define DEFAULT_ADDRESS  "00:04:3E:9B:A3:7E"

int main() {
	IMU imu(DEFAULT_ADDRESS);
	sleep(5);

	const float distance = 10; // TxRx distance in meters.

	const int num_samples = 10;
	

	// 1)calibrate at no movement condition, collect 1024 sample, average the value as offset value: sstateX // Only perform once before tracking begin
	float sstateX = 0;
	int calibration_count = 1024;
	int max_num_failure = 1024, num_failure = 0;
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
			
		sstateX += output.second.a[0];
	}
	sstateX = sstateX / float(calibration_count);

	if(num_failure >= max_num_failure) {
		std::cerr << "Couldn't calibrate data" << std::endl;
		exit(1);
	}


	float accX[2] = {0.0, 0.0}; // 1X2 array to hold accelerometer [1] is t [2] is t+1, initialize
	float positionX[2] = {0.0, 0.0}; // 1X2 array to hold position, initialize 
	float velocityX[2] = {0.0, 0.0}; // 1X2 array to hold velocity, initialize
	float t = float(num_samples) / 512.0; // the unit integration time intervel, eg: 512hz t=1/512 second, will change based on number of data took for average.

	int zero_count = 0;
	const float zero_eps = 0.01;
	const int zero_count_reset = 25;

	while(true) {
	  auto output = imu.getData(num_samples);
		if(output.first) {

			// See ImuData.h in LpSensor/include/ for more details.
			ImuData d = output.second;

			// Accelerometer Data
			std::cout << "Acc(" << d.a[0] << ", " << d.a[1] << ", " << d.a[2] << "), ";

			// Quaternion orientation data
			std::cout << "Q(" << d.q[0] << ", " << d.q[1] << ", " << d.q[2] << ", " << d.q[3] << ")" << std::endl;

			float theta = 0;

			// ******************************angle response calaulate start*************************************************/

			// 2) check if the movement ends, look up acceleration continuously, if detect straight 25 point Acceleration ==0, set velocity to zero
			// unsigned int countx;
			// countx = 0;
			// if(accX[1]==0){
			// 	countx++;
			// } else {
			// 	countx = 0;
			// }
			
			// if(countx >= 25) {
			// 	velocityX[0]=0;
			// 	velocityX[1]=0;
			// }
			
			// 3) reduce data noise by take 64 sample average. (imu.getData(num_samples) automatically gets the requested number of samples and averages them)
			accX[1] = d.a[0];
			// accX[1] = accX[1] - sstateX; // apply the zero offset and get ready for filter and integrate
			
			// filter out minimum noise using mechanical filter
			if((accX[1]<=0.005) && (accX[1]>=-0.005)) {
			    accX[1] = 0.0;
			}

			if(fabs(accX[1]) < zero_eps) {
				zero_count++;

			} else {
				zero_count = 0;
			}

			// 4) perform the velocity and position integration in time domain
			if(zero_count < zero_count_reset) {
				velocityX[1] = velocityX[0] + (accX[0] + (accX[1] - accX[0]) / 2.0) * t; //velocity
			} else {
				velocityX[0] = 0.0;
				velocityX[1] = 0.0;
			}
			positionX[1] = positionX[0] + (velocityX[1] + (velocityX[1] - velocityX[0]) / 2.0) * t; // position

			//current data become the initial data of next step
			accX[0] = accX[1];
			velocityX[0] = velocityX[1];
			positionX[0] = positionX[1];

			// 5) get current Euler angle and calculate angle resposne with current integrated position
			float EurZ = d.r[3];// read current eular angle                                                            !!! need grab IMU data!!!
			if (positionX[1] > 0) {
				theta = -(( 180 / pi * atan(distance/positionX[1]))+EurZ-90)/2;
			} else if (positionX[1] < 0) {
				theta = -((180/pi*atan(distance/positionX[1]))+EurZ-90)/2-90;
			} else {
				theta = 0;
			}
			

			// ******************************angle response calculate end****************************************************

			std::cout << "acceleration: " << accX[1] << std::endl;
			std::cout << "velocity:     " << velocityX[1] << std::endl;
			std::cout << "position:     " << positionX[1] << std::endl;
			// std::cout << "r[3]: " << d.r[3] << std::endl; 
			// std::cout << "Theta: " << theta << std::endl;
		}
	}
}

