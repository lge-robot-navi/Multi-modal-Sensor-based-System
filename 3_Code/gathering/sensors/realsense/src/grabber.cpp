
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
bool flag_show_color = true;
bool flag_show_ir = false;
bool flag_show_depth = false, flag_show_aligned_depth = false;
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

struct CameraInfo
{
    unsigned int height;
    unsigned int width;
    char distortion_model[9];
    float D[5];
    float K[9];
    float R[9];
    float P[12];
};



int main(int argc, char** argv) try 
{
    // check arguments
    if(argv != nullptr) {
        for(int i=1; i<argc; i++) {
            if(!std::strncmp(argv[i], "--help", 6)) {
                cout << "Usage: " << argv[0] << " [--viewer true/*false] [--test true/*false] [--show_color true/*false] [--show_depth true/*false] [--show_aligned_depth true/*depth] [--show_ir true/*false] [--show_all true/*false]" << endl;
                return 0;
            }
            else if(!std::strncmp(argv[i], "--viewer", 8)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_viewer = true;
                else flag_viewer = false;
            }
            else if(!std::strncmp(argv[i], "--show_color", 12)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_show_color = true;
                else flag_show_color = false;
            }
            else if(!std::strncmp(argv[i], "--show_depth", 12)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_show_depth = true;
                else flag_show_depth = false;
            }
            else if(!std::strncmp(argv[i], "--show_aligned_depth", 20)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_show_aligned_depth = true;
                else flag_show_aligned_depth = false;
            }
            else if(!std::strncmp(argv[i], "--show_ir", 9)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_show_ir = true;
                else flag_show_ir = false;
            }
            else if(!std::strncmp(argv[i], "--show_all", 10)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) { 
                    flag_show_color = true; flag_show_depth = true; flag_show_aligned_depth = true; flag_show_ir = true;
                }
            }
            else if(!std::strncmp(argv[i], "--test", 6)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_testonly = true;
                else flag_testonly = false;
            }
        }
    }

    // set signal handler
    signal(SIGINT, [](int){ flag_running = false; });
    signal(SIGABRT, [](int){ flag_running = false; });
    signal(SIGKILL, [](int){ flag_running = false; });
    signal(SIGTERM, [](int){ flag_running = false; });

    // init. realsense camera interface
	unsigned int rs_width = 640;	// 1280
	unsigned int rs_height = 480;	// 720
	unsigned int rs_framerate = 60;	// 30

	rs2::colorizer color_map;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, rs_width, rs_height, RS2_FORMAT_BGR8, rs_framerate);
	cfg.enable_stream(RS2_STREAM_DEPTH, rs_width, rs_height, RS2_FORMAT_Z16, rs_framerate);
	cfg.enable_stream(RS2_STREAM_INFRARED, 1, rs_width, rs_height, RS2_FORMAT_Y8, rs_framerate); // 2
	cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
	cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	pipe.start(cfg);
    if(flag_testonly == true) cout << "rs2 pipeline start" << endl;

    // gathering image
    rs2::frameset frames;
    for(int i=0; i<30; i++) frames = pipe.wait_for_frames();    // dropping frames to let auto-exposure
    if(flag_testonly == true) cout << "dropped frames to let auto-exposure" << endl;

    rs2::video_frame color = frames.get_color_frame();
    rs2::depth_frame depth = frames.get_depth_frame();        //color_map(frames.get_depth_frame());
    rs2::video_frame ir = frames.get_infrared_frame();
    rs2::video_frame colorized_depth = color_map.colorize(depth);

    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::frameset processed = align_to_color.process(frames);
    rs2::depth_frame aligned_depth = processed.get_depth_frame();
    rs2::video_frame colorized_aligned_depth = color_map.colorize(aligned_depth);
    
    // gathering sensor value
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

    // creating openCV Matrix from a image
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

    w = colorized_depth.as<rs2::video_frame>().get_width();
    h = colorized_depth.as<rs2::video_frame>().get_height();
    Mat imgColorizedDepth(Size(w, h), CV_8UC3, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);
    if(flag_testonly == true) cout << "create colorized_depth mat (" << w << ", " << h << ", " << imgColorizedDepth.elemSize() <<")";
    std::size_t colorized_depth_bufsize = imgColorizedDepth.total() * imgColorizedDepth.elemSize();
    if(flag_testonly == true) cout << ", size: " << colorized_depth_bufsize << endl;

    w = aligned_depth.as<rs2::video_frame>().get_width();
    h = aligned_depth.as<rs2::video_frame>().get_height();
    Mat imgAlignedDepth(Size(w, h), CV_16UC1, (void*)aligned_depth.get_data(), Mat::AUTO_STEP);
    if(flag_testonly == true) cout << "create aligned_depth mat (" << w << ", " << h << ", " << imgAlignedDepth.elemSize() << ")";
    std::size_t aligned_depth_bufsize = imgAlignedDepth.total() * imgAlignedDepth.elemSize();
    if(flag_testonly == true) cout << ", size: " << aligned_depth_bufsize << endl;

    w = colorized_aligned_depth.as<rs2::video_frame>().get_width();
    h = colorized_aligned_depth.as<rs2::video_frame>().get_height();
    Mat imgColorizedAlignedDepth(Size(w, h), CV_8UC3, (void*)colorized_aligned_depth.get_data(), Mat::AUTO_STEP);
    if(flag_testonly == true) cout << "create colorized_aligned_depth mat (" << w << ", " << h << ", " << imgColorizedAlignedDepth.elemSize() << ")";
    std::size_t colorized_aligned_depth_bufsize = imgColorizedAlignedDepth.total() * imgColorizedAlignedDepth.elemSize();
    if(flag_testonly == true) cout << ", size: " << colorized_aligned_depth_bufsize << endl;


    ///// gathering camera_info : refer 'realsense-ros/base_realsense_node.cpp/updateStreamCalibData()
    struct CameraInfo cameraInfo_Color;
    //rs2::video_stream_profile color_profile = color.get_profile().as<rs2::video_stream_profile>();
    //auto color_intrinsic = color_profile.get_intrinsics();
    auto color_intrinsic = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    cameraInfo_Color.width = color_intrinsic.width;
    cameraInfo_Color.height = color_intrinsic.height;
    std::strncpy(cameraInfo_Color.distortion_model, "plumb_bob", 9);

    cameraInfo_Color.K[0] = color_intrinsic.fx;   cameraInfo_Color.K[1] = 0;                    cameraInfo_Color.K[2] = color_intrinsic.ppx;
    cameraInfo_Color.K[3] = 0;                    cameraInfo_Color.K[4] = color_intrinsic.fy;   cameraInfo_Color.K[5] = color_intrinsic.ppy;
    cameraInfo_Color.K[6] = 0;                    cameraInfo_Color.K[7] = 0;                    cameraInfo_Color.K[8] = 1;

    cameraInfo_Color.P[0] = cameraInfo_Color.K[0];cameraInfo_Color.P[1] = 0;                    cameraInfo_Color.P[2] = cameraInfo_Color.K[2];
    cameraInfo_Color.P[3] = 0;                    cameraInfo_Color.P[4] = 0;                    cameraInfo_Color.P[5] = cameraInfo_Color.K[4];
    cameraInfo_Color.P[6] = cameraInfo_Color.K[5];cameraInfo_Color.P[7] = 0;                    cameraInfo_Color.P[8] = 0;
    cameraInfo_Color.P[9] = 0;                    cameraInfo_Color.P[10] = 1;                   cameraInfo_Color.P[11] = 0;

    cameraInfo_Color.R[0] = 1.0;                  cameraInfo_Color.R[1] = 0.0;                  cameraInfo_Color.R[2] = 0.0;
    cameraInfo_Color.R[3] = 0.0;                  cameraInfo_Color.R[4] = 1.0;                  cameraInfo_Color.R[5] = 0.0;
    cameraInfo_Color.R[6] = 0.0;                  cameraInfo_Color.R[7] = 0.0;                  cameraInfo_Color.R[8] = 1.0;

    cameraInfo_Color.D[0] = color_intrinsic.coeffs[0];
    cameraInfo_Color.D[1] = color_intrinsic.coeffs[1];
    cameraInfo_Color.D[2] = color_intrinsic.coeffs[2];
    cameraInfo_Color.D[3] = color_intrinsic.coeffs[3];
    cameraInfo_Color.D[4] = color_intrinsic.coeffs[4];

    struct CameraInfo cameraInfo_IR;
    auto ir_intrinsic = pipe.get_active_profile().get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>().get_intrinsics();
    cameraInfo_IR.width = ir_intrinsic.width;
    cameraInfo_IR.height = ir_intrinsic.height;
    std::strncpy(cameraInfo_IR.distortion_model, "plumb_bob", 9);

    cameraInfo_IR.K[0] = ir_intrinsic.fx;       cameraInfo_IR.K[1] = 0;                         cameraInfo_IR.K[2] = ir_intrinsic.ppx;
    cameraInfo_IR.K[3] = 0;                     cameraInfo_IR.K[4] = ir_intrinsic.fy;           cameraInfo_IR.K[5] = ir_intrinsic.ppy;
    cameraInfo_IR.K[6] = 0;                     cameraInfo_IR.K[7] = 0;                         cameraInfo_IR.K[8] = 1;

    cameraInfo_IR.P[0] = cameraInfo_IR.K[0];    cameraInfo_IR.P[1] = 0;                         cameraInfo_IR.P[2] = cameraInfo_IR.K[2];
    cameraInfo_IR.P[3] = 0;                     cameraInfo_IR.P[4] = 0;                         cameraInfo_IR.P[5] = cameraInfo_IR.K[4];
    cameraInfo_IR.P[6] = cameraInfo_IR.K[5];    cameraInfo_IR.P[7] = 0;                         cameraInfo_IR.P[8] = 0;
    cameraInfo_IR.P[9] = 0;                     cameraInfo_IR.P[10] = 1;                        cameraInfo_IR.P[11] = 0;

    cameraInfo_IR.R[0] = 1.0;                   cameraInfo_IR.R[1] = 0.0;                       cameraInfo_IR.R[2] = 0.0;
    cameraInfo_IR.R[3] = 0.0;                   cameraInfo_IR.R[4] = 1.0;                       cameraInfo_IR.R[5] = 0.0;
    cameraInfo_IR.R[6] = 0.0;                   cameraInfo_IR.R[7] = 0.0;                       cameraInfo_IR.R[8] = 1.0;

    cameraInfo_IR.D[0] = ir_intrinsic.coeffs[0];
    cameraInfo_IR.D[1] = ir_intrinsic.coeffs[1];
    cameraInfo_IR.D[2] = ir_intrinsic.coeffs[2];
    cameraInfo_IR.D[3] = ir_intrinsic.coeffs[3];
    cameraInfo_IR.D[4] = ir_intrinsic.coeffs[4];


    if(flag_viewer == true) {
        // show image
        if(flag_show_color) {
            namedWindow("RS_Color", WINDOW_AUTOSIZE);
            imshow("RS_Color", imgColor);
        }
        if(flag_show_depth) {
            namedWindow("RS_Depth", WINDOW_AUTOSIZE);
            colorized_depth = color_map.colorize(depth);
            std::memcpy((void*)imgColorizedDepth.data, (const void*)colorized_depth.get_data(), colorized_depth_bufsize);
            imshow("RS_Depth", imgColorizedDepth);
        }
        if(flag_show_aligned_depth) {
            namedWindow("RS_AlignedDepth", WINDOW_AUTOSIZE);
            colorized_aligned_depth = color_map.colorize(aligned_depth);
            std::memcpy((void*)imgColorizedAlignedDepth.data, (const void*)colorized_aligned_depth.get_data(), colorized_aligned_depth_bufsize);
            imshow("RS_AlignedDepth", imgColorizedAlignedDepth);
        }
        if(flag_show_ir) {
            namedWindow("RS_IR", WINDOW_AUTOSIZE);
            imshow("RS_IR", imgIR);
        }
    }

    ///// init. memsync interface
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

    ///// polling ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    while(flag_running == true)
    {
        // get current frame & sensor values
        frames = pipe.wait_for_frames();
        color = frames.get_color_frame();
        depth = frames.get_depth_frame();       //color_map(frames.get_depth_frame());
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

        frames = align_to_color.process(frames);
        aligned_depth = frames.get_depth_frame();

		assert((void*)imgColor.data != nullptr && "imgColor.data is nullptr");
		assert((void*)imgDepth.data != nullptr && "imgDepth.data is nullptr");
		assert((void*)imgIR.data != nullptr && "imgIR.data is nullptr");
        assert((void*)imgAlignedDepth.data != nullptr && "imgAlignedDepth.data is nullptr");

        assert((void*)color.get_data() != nullptr && "color.get_data() is nullptr");
        assert((void*)depth.get_data() != nullptr && "depth.get_data() is nullptr");
        assert((void*)ir.get_data() != nullptr && "ir.get_data() is nullptr");
        assert((void*)aligned_depth.get_data() != nullptr && "aligned_depth.get_data() is nullptr");

        try
        {
            if(imgColor.size().width == color.get_width() && 
                imgColor.size().height == color.as<rs2::video_frame>().get_height() &&
                color.as<rs2::video_frame>().get_data() != nullptr) {
                std::memcpy((void*)imgColor.data, (const void*)color.as<rs2::video_frame>().get_data(), (std::size_t)color_bufsize);
            }
            else {
                printf("get failed color frame...\n");
            }

            if(imgDepth.size().width == depth.as<rs2::video_frame>().get_width() && 
                imgDepth.size().height == depth.as<rs2::video_frame>().get_height() &&
                depth.as<rs2::video_frame>().get_data() != nullptr) {
                std::memcpy((void*)imgDepth.data, (const void*)depth.as<rs2::video_frame>().get_data(), (std::size_t)depth_bufsize);
            }
            else {
                printf("get failed depth frame...\n");
            }

            if(imgIR.size().width == ir.as<rs2::video_frame>().get_width() && 
                imgIR.size().height == ir.as<rs2::video_frame>().get_height() &&
                ir.as<rs2::video_frame>().get_data() != nullptr) {
                std::memcpy((void*)imgIR.data, (const void*)ir.as<rs2::video_frame>().get_data(), (std::size_t)ir_bufsize);
            }
            else {
                printf("get failed ir frame...\n");
            }

            if(imgAlignedDepth.size().width == aligned_depth.as<rs2::depth_frame>().get_width() && 
                imgAlignedDepth.size().height == aligned_depth.as<rs2::depth_frame>().get_height() &&
                aligned_depth.as<rs2::depth_frame>().get_data() != nullptr) {
                std::memcpy((void*)imgAlignedDepth.data, (const void*)aligned_depth.get_data(), (std::size_t)aligned_depth_bufsize);
            }
            else {
                printf("get failed aligned_depth frame...\n");
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        // memsync. uploading
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

                MemSync_SetID(pHandle, "RealSense_AlignedDepth");
                timestamp = MemSync_CurrentTimestamp();
                retCode = MemSync_Write(pHandle, (const char*)imgAlignedDepth.data, aligned_depth_bufsize, timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Write failed - mutex locked" << endl;
                }

				MemSync_SetID(pHandle, "RealSense_IMU");
				timestamp = MemSync_CurrentTimestamp();
				retCode = MemSync_Write(pHandle, (const char*)&imu_data, sizeof(struct IMUData), timestamp);
				if(retCode == MEMSYNC_MUTEX_LOCKED) {
					std::cerr << "MemSync_Write failed - mutex locked" << endl;
				}

                MemSync_SetID(pHandle, "RealSense_ColorCameraInfo");
                timestamp = MemSync_CurrentTimestamp();
                retCode = MemSync_Write(pHandle, (const char*)&cameraInfo_Color, sizeof(struct CameraInfo), timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
					std::cerr << "MemSync_Write failed - mutex locked" << endl;
				}

                MemSync_SetID(pHandle, "RealSense_IRCameraInfo");
                timestamp = MemSync_CurrentTimestamp();
                retCode = MemSync_Write(pHandle, (const char*)&cameraInfo_IR, sizeof(struct CameraInfo), timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Write failed - mutex locked" << endl;
                }
            }
        }

        if(flag_viewer == true) {
            // show image
            if(flag_show_color) {
                namedWindow("RS_Color", WINDOW_AUTOSIZE);
                imshow("RS_Color", imgColor);
            }
            if(flag_show_depth) {
                namedWindow("RS_Depth", WINDOW_AUTOSIZE);
                colorized_depth = color_map.colorize(depth);
                std::memcpy((void*)imgColorizedDepth.data, (const void*)colorized_depth.get_data(), colorized_depth_bufsize);
                imshow("RS_Depth", imgColorizedDepth);
            }
            if(flag_show_aligned_depth) {
                namedWindow("RS_AlignedDepth", WINDOW_AUTOSIZE);
                colorized_aligned_depth = color_map.colorize(aligned_depth);
                std::memcpy((void*)imgColorizedAlignedDepth.data, (const void*)colorized_aligned_depth.get_data(), colorized_aligned_depth_bufsize);
                imshow("RS_AlignedDepth", imgColorizedAlignedDepth);
            }
            if(flag_show_ir) {
                namedWindow("RS_IR", WINDOW_AUTOSIZE);
                imshow("RS_IR", imgIR);
            }
            
	        cv::waitKey(50);
        }
        else {
            usleep(150);
        }
    }

    if(flag_testonly == true) cout << "release memory..." << endl;
    imgColor.release();
    imgDepth.release();
    imgIR.release();
    imgAlignedDepth.release();
    imgColorizedDepth.release();
    imgColorizedAlignedDepth.release();

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

