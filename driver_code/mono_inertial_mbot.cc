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



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

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

public:
	~IMU_Handler() {}
	void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string &chan, const mbot_imu_t* msg) {
		if (start_time == -1) start_time = (double)msg->utime / 1.0e9;
		IMU_Data.push_back(ORB_SLAM3::IMU::Point(msg->accel[0], msg->accel[1], msg->accel[2], msg->gyro[0], msg->gyro[1], msg->gyro[2], (double)msg->utime / 1.0e9 - start_time));
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
