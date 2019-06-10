
#include <cstring>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <memory>

#include <assert.h>
#include <unistd.h>
#include "memsync.h"

using namespace std;
using namespace cv;

bool flag_viewer = false;
bool flag_testonly = false;
bool flag_running = true;

struct IMUData
{
	double accel_x;
	double accel_y;
	double accel_z;
	double gyro_x;
	double gyro_y;
	double gyro_z;
};


int main(int argc, char** argv) try 
{
    if(argv != nullptr) {
        for(int i=1; i<argc; i++) {
            if(!std::strncmp(argv[i], "--help", 6)) {
                cout << "Usage: " << argv[0] << " [--viewer true/*false] [--test true/*false]" << endl;
                return 0;
            }
            else if(!std::strncmp(argv[i], "--viewer", 8)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_viewer = true;
            }
            else if(!std::strncmp(argv[i], "--test", 6)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_testonly = true;
            }
        }
    }

    signal(SIGINT, [](int){ flag_running = false; });
    signal(SIGABRT, [](int){ flag_running = false; });
    signal(SIGKILL, [](int){ flag_running = false; });
    signal(SIGTERM, [](int){ flag_running = false; });

	unsigned int rs_width = 640;	
	unsigned int rs_height = 480;	
	unsigned int rs_framerate = 60;	

	rs2::colorizer color_map;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, rs_width, rs_height, RS2_FORMAT_BGR8, rs_framerate);
	cfg.enable_stream(RS2_STREAM_DEPTH, rs_width, rs_height, RS2_FORMAT_Z16, rs_framerate);
	cfg.enable_stream(RS2_STREAM_INFRARED, 1, rs_width, rs_height, RS2_FORMAT_Y8, rs_framerate);
	cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
	cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	pipe.start(cfg);
    if(flag_testonly == true) cout << "rs2 pipeline start" << endl;

    rs2::frameset frames;
    for(int i=0; i<30; i++) frames = pipe.wait_for_frames();  

    rs2::frame color = frames.get_color_frame();
    rs2::frame depth = frames.get_depth_frame(); 
    rs2::frame ir = frames.get_infrared_frame();

	struct IMUData imu_data; 	
	rs2_vector accel, gyro;
	auto motion = frames.as<rs2::motion_frame>();
	if(motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
		motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) 
	{
		gyro = motion.get_motion_data();
		imu_data.gyro_x = gyro.x;
		imu_data.gyro_y = gyro.y;
		imu_data.gyro_z = gyro.z;
	}
	if(motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
		motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
	{
		accel = motion.get_motion_data();
		imu_data.accel_x = accel.x;
		imu_data.accel_y = accel.y;
		imu_data.accel_z = accel.z;
	}
    if(flag_testonly == true) cout << "get frames" << endl;

    int w, h;
    w = color.as<rs2::video_frame>().get_width();
    h = color.as<rs2::video_frame>().get_height();
    Mat imgColor(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    if(flag_testonly == true) cout << "create color mat (" << w << ", " << h << ", " << imgColor.elemSize() << ")";
    std::size_t color_bufsize = imgColor.total() * imgColor.elemSize();
    if(flag_testonly == true) cout << ", size: " << color_bufsize << endl;

    w = depth.as<rs2::video_frame>().get_width();
    h = depth.as<rs2::video_frame>().get_height();
    Mat imgDepth(Size(w, h), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);
    if(flag_testonly == true) cout << "create depth mat (" << w << ", " << h << ", " << imgDepth.elemSize() << ")";
    std::size_t depth_bufsize = imgDepth.total() * imgDepth.elemSize();
    if(flag_testonly == true) cout << ", size: " << depth_bufsize << endl;

    w = ir.as<rs2::video_frame>().get_width();
    h = ir.as<rs2::video_frame>().get_height();
    Mat imgIR(Size(w, h), CV_8UC1, (void*)ir.get_data(), Mat::AUTO_STEP);
    if(flag_testonly == true) cout << "create ir mat (" << w << ", " << h << ", " << imgIR.elemSize() << ")";
    std::size_t ir_bufsize = imgIR.total() * imgIR.elemSize();
    if(flag_testonly == true) cout << ", size: " << ir_bufsize << endl;

    if(flag_viewer == true) {
        namedWindow("Color", WINDOW_AUTOSIZE);
        imshow("Color", imgColor);
    }

    void* pHandle = nullptr;
    int retCode = 0;

    pHandle = MemSync_GetHandle();
    if(flag_testonly == true)
        if(pHandle == nullptr) cout << "MemSync handle is nullptr" << endl;

    assert(pHandle != nullptr && "pHandle is nullptr");

    if(pHandle != nullptr && MemSync_ValidateHandle(pHandle) == true) {
        MemSync_SetServerInfo(pHandle, "localhost", 11211);
    }

    unsigned long timestamp = 0;

    while(flag_running == true)
    {
        frames = pipe.wait_for_frames();
        color = frames.get_color_frame();
        depth = frames.get_depth_frame(); 
        ir = frames.get_infrared_frame();

		gyro.x = gyro.y = gyro.z = 0.0;
		accel.x = accel.y = accel.z = 0.0;		

		motion = frames.as<rs2::motion_frame>();
		if(motion && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
			if(motion.get_profile().stream_type() == RS2_STREAM_GYRO) {
				gyro = motion.get_motion_data();
				imu_data.gyro_x = gyro.x;
				imu_data.gyro_y = gyro.y;
				imu_data.gyro_z = gyro.z;
			}
			if(motion.get_profile().stream_type() == RS2_STREAM_ACCEL) {
				accel = motion.get_motion_data();
				imu_data.accel_x = accel.x;
				imu_data.accel_y = accel.y;
				imu_data.accel_z = accel.z;
			}
		}

		assert((void*)imgColor.data != nullptr && "imgColor.data is nullptr");
		assert((void*)imgDepth.data != nullptr && "imgDepth.data is nullptr");
		assert((void*)imgIR.data != nullptr && "imgIR.data is nullptr");

        std::memcpy((void*)imgColor.data, (const void*)color.get_data(), (std::size_t)color_bufsize);
        std::memcpy((void*)imgDepth.data, (const void*)depth.get_data(), (std::size_t)depth_bufsize);
        std::memcpy((void*)imgIR.data, (const void*)ir.get_data(), (std::size_t)ir_bufsize);

        if(flag_testonly == false) {
            if(pHandle != nullptr && MemSync_ValidateHandle(pHandle) == true) {             
                MemSync_SetID(pHandle, "RealSense_Color");
                timestamp = MemSync_CurrentTimestamp();                
                retCode = MemSync_Write(pHandle, (const char*)imgColor.data, color_bufsize, timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Write failed - mutex locked" << endl;
                }

                MemSync_SetID(pHandle, "RealSense_Depth");
                timestamp = MemSync_CurrentTimestamp();
                retCode = MemSync_Write(pHandle, (const char*)imgDepth.data, depth_bufsize, timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Write failed - mutex locked" << endl;
                }

				MemSync_SetID(pHandle, "RealSense_IR");
				timestamp = MemSync_CurrentTimestamp();
				retCode = MemSync_Write(pHandle, (const char*)imgIR.data, ir_bufsize, timestamp);
				if(retCode == MEMSYNC_MUTEX_LOCKED) {
					std::cerr << "MemSync_Write failed - mutex locked" << endl;
				}

				MemSync_SetID(pHandle, "RealSense_IMU");
				timestamp = MemSync_CurrentTimestamp();
				retCode = MemSync_Write(pHandle, (const char*)&imu_data, sizeof(struct IMUData), timestamp);
				if(retCode == MEMSYNC_MUTEX_LOCKED) {
					std::cerr << "MemSync_Write failed - mutex locked" << endl;
				}
            }
        }

        if(flag_viewer == true) {
            imshow("Color", imgColor);
            cvWaitKey(100);
        }
        else {
            usleep(100);
        }
    }

    if(flag_testonly == true) cout << "release memory..." << endl;
    imgColor.release();
    imgDepth.release();
    imgIR.release();

	if(flag_viewer == true) destroyAllWindows();

    if(pHandle != nullptr) MemSync_ReleaseHandle(pHandle); pHandle = nullptr;

    return 0;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return -1;    
}
catch (const std::exception& e)
{
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
}

