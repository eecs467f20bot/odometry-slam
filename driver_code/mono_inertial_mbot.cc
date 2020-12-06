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
#include <string>
#include <cassert>

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
		vector<float> kf_accel_data_x;
		vector<float> kf_accel_data_y;
		// Gyro Data - x and y are 0 by assumption
		vector<float> gyro_data_z;
		vector<float> kf_gyro_data_z;
		// Noise Data
		float accel_noise[4] = {0.0, 0.0, 0.0, 0.0};
		float gyro_noise[2] = {0.0, 0.0};
		// Log file
		ofstream log_file;
		//Counter
		int data_count = 0;

		float calc_stdev(vector<float> &data)
		{
			// Check that the size of both windows are the same
			assert(data.size() <= n);
			//Mean Variables
			float mean_sing = 0;
			float mean_sq = 0;

			for(unsigned int i = 0; i < data.size(); i++)
			{
				mean_sq += (data[i] * data[i])/float(n);
				mean_sing += data[i]/float(n);
			}

			// sqrt(E(X^2) - E(X)^2)
			return sqrt(mean_sq - (mean_sing * mean_sing));
		}

		void update_data(float data_pt, vector<float> &data_vec)
		{
			// Push accel data into window
			data_vec.push_back(data_pt);
			// If the window contains more than the desired amount, remove from the beginning 
			while(data_vec.size() > n)
				data_vec.erase(data_vec.begin());

			// Check if size within bounds
			assert(data_vec.size() <= n);
		}

		void update_noise()
		{
			accel_noise[0] = calc_stdev(accel_data_x);
			accel_noise[1] = calc_stdev(accel_data_y);

			gyro_noise[0] = calc_stdev(gyro_data_z);
		}

		void update_kf_noise()
		{
			accel_noise[2] = calc_stdev(kf_accel_data_x);
			accel_noise[3] = calc_stdev(kf_accel_data_y);

			gyro_noise[1] = calc_stdev(kf_gyro_data_z);
		}

		vector<float> odo_iner_kf(
			float odo_ax, float odo_ay, float odo_gz, 
			float odo_nax, float odo_nay, float odo_ngz
		){
			//return variable
			vector<float> data;
			
			//Calculate Kalman Gains - K
			float k_ax = odo_nax / (odo_nax + accel_noise[0]);
			float k_ay = odo_nay / (odo_nay + accel_noise[1]);
			float k_gz = odo_ngz / (odo_ngz + gyro_noise[0]);
			// Apply Kalman Filter
			float kf_ax = odo_ax + k_ax * (accel_data_x.back() - odo_ax);
			float kf_ay = odo_ay + k_ay * (accel_data_y.back() - odo_ay);
			float kf_gz = odo_gz + k_gz * (gyro_data_z.back() - odo_gz);

			//Update kf data windows
			update_data(kf_ax, kf_accel_data_x);
			update_data(kf_ay, kf_accel_data_y);
			update_data(kf_gz, kf_gyro_data_z);

			update_kf_noise();

			//Log Noise to log file - count, i_ax, i_ay, i_gz, o_ax, o_ay, o_gz, kf_ax, kf_ay, kf_gz
			log_file << data_count << " "
					 << accel_noise[0] << " " 
					 << accel_noise[1] << " " 
					 << gyro_noise[0] << " " 
					 << odo_nax << " " 
					 << odo_nay << " " 
					 << odo_ngz << " "
					 << accel_noise[2] << " "
					 << accel_noise[3] << " "
					 << gyro_noise[1] << " "
					 << endl;

			//Increment data count
			data_count++;

			// Populate and return answer
			data.push_back(kf_ax);
			data.push_back(kf_ay);
			data.push_back(kf_gz);

			return data;
		}

	public:
		IMU_Handler()
		{
			log_file.open("noise_log.txt");
		}
		~IMU_Handler() 
		{
			log_file.close();
		}
		
		void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string &chan, const mbot_imu_t* msg) {
			if (start_time == -1) 
				start_time = (double)msg->utime / 1.0e9;

			update_data(msg->accel[0], accel_data_x);
			update_data(msg->accel[1], accel_data_y);
			update_data(msg->gyro[2], gyro_data_z);
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
