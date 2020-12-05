/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/




#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>

#include <System.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/image_t.hpp"
#include "lcmtypes/mbot_imu_t.hpp"

using namespace std;

vector<ORB_SLAM3::IMU::Point> IMU_Data;
double start_time;

class Image_Handler {

private:
	ORB_SLAM3::System SLAM;
		
public:
	Image_Handler(const char* vocab_path, const char* settings_path) : SLAM(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true) {}
	void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string &chan, const image_t* msg) {
		if (start_time == -1) start_time = (double)msg->utime / 1.0e9;
		cv::Mat data_mat(msg->data, true);
		cv::Mat im(cv::imdecode(data_mat,0));
		SLAM.TrackMonocular(im, (double)msg->utime / 1.0e9 - start_time, IMU_Data);
		IMU_Data.clear();
	}

	~Image_Handler() {
		SLAM.Shutdown();
	}
};

class IMU_Handler {
	private:
		// Window Size
		unsigned int n = 10;
		// Accel Data - Z is known at 9.8 by assumption
		vector<float> accel_data_x;
		vector<float> accel_data_y;
		// IMU Data - x and y are 0 by assumption
		vector<float> gyro_data_z;
		// Noise Data
		float accel_noise[2] = {0.0, 0.0};
		float gyro_noise = 0.0;

		float calc_stdev(vector<float> &data)
		{
			// Check that the size of both windows are the same
			assert(data.size() == n);
			//Mean Variables
			float mean_sing = 0;
			float mean_sq = 0;

			for(unsigned int i = 0; i < n; i++)
			{
				mean_sing += (data[i] * data[i])/float(n);
				mean_sq += data[i]/float(n);
			}

			// sqrt(E(X^2) - E(X)^2)
			return sqrt(mean_sq - (mean_sing * mean_sing));
		}

		void update_data(float accel_x, float accel_y, float gyro_z)
		{
			// Push accel data into window
			accel_data_x.push_back(accel_x);
			accel_data_y.push_back(accel_y);
			// Push imu data into window
			gyro_data_z.push_back(gyro_z);

			// If the window contains more than the desired amount, remove from the beginning 
			// Accel Data
			while(accel_data_x.size() > n)
				accel_data_x.erase(accel_data_x.begin());
			while(accel_data_y.size() > n)
				accel_data_y.erase(accel_data_y.begin());
			// IMU Data
			while(gyro_data_z.size() > n)
				gyro_data_z.erase(gyro_data_z.begin());

			// Check if size within bounds
			assert(accel_data_x.size() <= n);
			assert(accel_data_y.size() <= n);
			assert(gyro_data_z.size() <= n);
		}

		void update_noise()
		{
			accel_noise[0] = calc_stdev(accel_data_x);
			accel_noise[1] = calc_stdev(accel_data_y);
			gyro_noise = calc_stdev(gyro_data_z);
		}

		vector<float> odo_iner_kf(
			float odo_ax, float odo_ay, float odo_gz, 
			float odo_nax, float odo_nay, float odo_ngz
		){
			vector<float> data;
			
			//Calculate Kalman Gains - K
			float k_ax = odo_nax / (odo_nax + accel_noise[0]);
			float k_ay = odo_nay / (odo_nay + accel_noise[1]);
			float k_gz = odo_ngz / (odo_ngz + gyro_noise);
			// Apply Kalman Filter
			data.push_back(odo_ax + k_ax * (accel_data_x.back() - odo_ax));
			data.push_back(odo_ay + k_ay * (accel_data_y.back() - odo_ay));
			data.push_back(odo_gz + k_gz * (gyro_data_z.back() - odo_gz));

			return data;
		}

	public:
		~IMU_Handler() {}
		
		void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string &chan, const mbot_imu_t* msg) {
			if (start_time == -1) 
				start_time = (double)msg->utime / 1.0e9;

			update_data(msg->accel[0], msg->accel[1], msg->gyro[2]);
			update_noise();
			vector<float> data = odo_iner_kf(msg->odometry_accel[0], msg->odometry_accel[1], msg->odometry_gyro[2], 
									msg->odometry_accel_uncertainty[0], msg->odometry_accel_uncertainty[1], msg->odometry_gyro_uncertainty[2]);
									
			IMU_Data.push_back(ORB_SLAM3::IMU::Point(data[0], data[1], 9.81, 0.0, 0.0, data[2], (double)msg->utime / 1.0e9 - start_time));
		}
};

int main(int argc, char **argv)
{  
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_mbot_stream path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    lcm::LCM lcm;

    if (!lcm.good()) {
		return 1;
    }

    start_time = -1;
    Image_Handler imhandle(argv[1],argv[2]);
    IMU_Handler imuhandle;

	lcm.subscribe("MBOT_IMAGE_STREAM", &Image_Handler::handleMessage, &imhandle);
	lcm.subscribe("MBOT_IMU", &IMU_Handler::handleMessage, &imuhandle);

	while(lcm.handle() == 0);

    return 0;
}
