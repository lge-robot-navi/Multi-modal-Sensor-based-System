
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <memory>
#include <chrono>
#include <iomanip>
#include <vector>
#include <experimental/filesystem>

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include "memsync.h"

using namespace std;
using namespace cv;
namespace fs = std::experimental::filesystem::v1;

bool flag_viewer = false;
bool flag_testonly = false;
bool flag_running = true;
int saving_rate = 10;

fs::path root_path;

#define OUTPATHFILE_LENGTH  256
char output_filename[OUTPATHFILE_LENGTH] = {0, };

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
    auto starttime = std::chrono::system_clock::now();
    root_path = "/work/data";
	char* prefix_filename = "FX01";
    bool output_type_png = true, output_type_raw = false, output_type_both = false;

    // check arguments
    if(argv != nullptr) {
        for(int i=1; i<argc; i++) {
            if(!std::strncmp(argv[i], "--help", 6)) {
                cout << "Usage: " << argv[0] << " [--viewer true/*false] [--test true/*false] [--starttime <HHMMSS>] [--output <root_path>] [--prefix <prefix_filename>] [--rate <saving_rate(Hz)>] [--type <PNG*/RAW/BOTH>]" << endl;
                return 0;
            }
            else if(!std::strncmp(argv[i], "--viewer", 8)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_viewer = true;
            }
            else if(!std::strncmp(argv[i], "--test", 6)) {
                if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_testonly = true;
            }
            else if(!std::strncmp(argv[i], "--starttime", 11)) {
                if(i+1 < argc) {
                    time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    std::tm tm = *localtime(&tt);
                    std::stringstream ss(argv[++i]);
                    ss >> std::get_time(&tm, "%H%M%S");
                    starttime = std::chrono::system_clock::from_time_t(std::mktime(&tm));
                }
            }
            else if(!std::strncmp(argv[i], "--output", 8)) {
                if(i+1 < argc && argv[++i] != nullptr) {
                    root_path = argv[i];
                }
            }
			else if(!std::strncmp(argv[i], "--prefix", 8)) {
				if(i+1 < argc && argv[++i] != nullptr) {
					prefix_filename = argv[i];
				}
			}
			else if(!std::strncmp(argv[i], "--rate", 6)) {
				if(i+1 < argc && argv[++i] != nullptr) {
					saving_rate = (int)std::stoi(argv[i]);
				}
			}
            else if(!std::strncmp(argv[i], "--type", 6)) {
                if(i+1 < argc && argv[++i] != nullptr) {
                    if(!std::strncmp(argv[i], "raw", 3) || !std::strncmp(argv[i], "RAW", 3)) {
                        output_type_both = false; output_type_png = false; output_type_raw = true;
                    }
                    else if(!std::strncmp(argv[i], "both", 4) || !std::strncmp(argv[i], "BOTH", 4)) {
                        output_type_both = true; output_type_png = true; output_type_raw = true;
                    }
                }
            }
        } // for loop
    }

    // set signal handler
    signal(SIGINT, [](int){ flag_running = false; });
    signal(SIGABRT, [](int){ flag_running = false; });
    signal(SIGKILL, [](int){ flag_running = false; });
    signal(SIGTERM, [](int){ flag_running = false; });

    // creating openCV Matrix
    int w = 640, h = 480;
    std::shared_ptr<char> color_data(new char[w * h * 3]); 
    std::memset((void*)color_data.get(), 0, (std::size_t)(w * h * 3));
    Mat imgColor(Size(w, h), CV_8UC3, (void*)color_data.get(), Mat::AUTO_STEP);
    std::size_t color_bufsize = imgColor.total() * imgColor.elemSize();
    if(flag_testonly == true) cout << "create color mat (size: " << color_bufsize << ")" << endl;

    std::shared_ptr<char> depth_data(new char[w * h * 2]);
    std::memset((void*)depth_data.get(), 0, (std::size_t)(w * h * 2));
    Mat imgDepth(Size(w, h), CV_16UC1, (void*)depth_data.get(), Mat::AUTO_STEP);
    std::size_t depth_bufsize = imgDepth.total() * imgDepth.elemSize();
    if(flag_testonly == true) cout << "create depth mat (size: " << depth_bufsize << ")" << endl;

	std::shared_ptr<char> ir_data(new char[w * h * 1]);
	std::memset((void*)ir_data.get(), 0, (std::size_t)(w * h * 1));
	Mat imgIR(Size(w, h), CV_8UC1, (void*)ir_data.get(), Mat::AUTO_STEP);
	std::size_t ir_bufsize = imgIR.total() * imgIR.elemSize();
	if(flag_testonly == true) cout << "create ir mat (size: " << ir_bufsize << ")" << endl; 

    std::shared_ptr<char> aligned_depth_data(new char[w * h * 2]);
    std::memset((void*)aligned_depth_data.get(), 0, (std::size_t)(w * h * 2));
    Mat imgAlignedDepth(Size(w, h), CV_16UC1, (void*)aligned_depth_data.get(), Mat::AUTO_STEP);
    std::size_t aligned_depth_bufsize = imgAlignedDepth.total() * imgAlignedDepth.elemSize();
    if(flag_testonly == true) cout << "create aligned_depth mat(size: " << aligned_depth_bufsize << ")" << endl;

    if(flag_viewer == true) {
        // show image
        namedWindow("Color_Saver", WINDOW_AUTOSIZE);
        imshow("Color_Saver", imgColor);
    }

    // init. memsync interface
    void* pHandle = nullptr;
    int retCode = 0;

    pHandle = MemSync_GetHandle();
    assert(pHandle != nullptr && "MemSync's pHandle is nullptr");
	if(flag_testonly == true)
        if(pHandle == nullptr) cout << "MemSync handle is nullptr" << endl;

    if(pHandle != nullptr && MemSync_ValidateHandle(pHandle) == true) {
        MemSync_SetServerInfo(pHandle, "localhost", 11211);
    }

    // check output directories
    if(!fs::exists(root_path)) fs::create_directory(root_path);
    if(!fs::exists(root_path / "rgbdepth")) fs::create_directory(root_path / "rgbdepth");
    if(!fs::exists(root_path / "rgbdepth" / "color")) fs::create_directory(root_path / "rgbdepth" / "color");
    if(!fs::exists(root_path / "rgbdepth" / "depth")) fs::create_directory(root_path / "rgbdepth" / "depth");
	if(!fs::exists(root_path / "rgbdepth" / "aligned_depth")) fs::create_directory(root_path / "rgbdepth" / "aligned_depth");
    if(!fs::exists(root_path / "rgbdepth" / "ir")) fs::create_directory(root_path / "rgbdepth" / "ir");
	if(!fs::exists(root_path / "rgbdepth" / "imu")) fs::create_directory(root_path / "rgbdepth" / "imu");

	// get imu csv file handle
	struct IMUData imu_data;
	unsigned long imu_data_size = 0;

	time_t today_tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::tm today_tm = *localtime(&today_tt);
	char output_imu_filename[256] = {0, };
	snprintf(output_imu_filename, 256, "%s/%s_IMU_%02d%02d%02d_%02d%02d%02d.csv",
				(root_path / "rgbdepth" / "imu").c_str(),
				prefix_filename,
				(today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
				today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec);
	std::ofstream output_csv;
	output_csv.open(output_imu_filename);

    unsigned long timestamp = 0;
    unsigned long color_data_size = 0;
    unsigned long depth_data_size = 0;
	unsigned long ir_data_size = 0;
    unsigned long aligned_depth_data_size = 0;

    vector<int> compresstion_params;
    compresstion_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compresstion_params.push_back(0);

    // wait starttime
    while(flag_running == true)
    {
        auto currenttime = std::chrono::system_clock::now();
        auto diff_time = starttime - currenttime;
        if(diff_time.count() > 0) {
            if(flag_testonly == true) {
                std::time_t start = std::chrono::system_clock::to_time_t(starttime);
                std::time_t current = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                cout << "\r wait to starttime: " << std::ctime(&start) << endl;
                sleep(1);
            }
            else {
                usleep(100000);     // 100msec
            }
        }
        else {
            break;
        }
    }

    if(flag_testonly == true) {
        cout << "start saving loop" << endl;
    }

    // polling
    today_tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    today_tm = *localtime(&today_tt);

    //auto epoch = std::chrono::system_clock::now().time_since_epoch();
    //auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    int p_time_hook = (int)((epoch_ms.count() % 1000) / (1000 / saving_rate));
    int c_time_hook = 0;

    while(flag_running == true)
    {
        // save every 100msec (10Hz)
        epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        c_time_hook = (int)((epoch_ms.count() % 1000) / (1000 / saving_rate));
        if(c_time_hook != p_time_hook) {
            if(flag_testonly == false) {
                if(pHandle != nullptr && MemSync_ValidateHandle(pHandle) == true) {             
                    MemSync_SetID(pHandle, "RealSense_Color");
                    retCode = MemSync_Read(pHandle, color_data.get(), color_data_size, timestamp);
                    if(retCode == MEMSYNC_MUTEX_LOCKED) {
                        std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                    }

                    MemSync_SetID(pHandle, "RealSense_Depth");
                    retCode = MemSync_Read(pHandle, depth_data.get(), depth_data_size, timestamp);
                    if(retCode == MEMSYNC_MUTEX_LOCKED) {
                        std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                    }

		    		MemSync_SetID(pHandle, "RealSense_IR");
		    		retCode = MemSync_Read(pHandle, ir_data.get(), ir_data_size, timestamp);
		    		if(retCode == MEMSYNC_MUTEX_LOCKED) {
		    			std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
		    		}

                    MemSync_SetID(pHandle, "RealSense_AlignedDepth");
                    retCode = MemSync_Read(pHandle, aligned_depth_data.get(), aligned_depth_data_size, timestamp);
                    if(retCode == MEMSYNC_MUTEX_LOCKED) {
                        std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                    }

					MemSync_SetID(pHandle, "RealSense_IMU");
					retCode = MemSync_Read(pHandle, (char*)&imu_data, imu_data_size, timestamp);
					if(retCode == MEMSYNC_MUTEX_LOCKED) {
						std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
					} 
                }

                std::memcpy((void*)imgColor.data, (const void*)color_data.get(), (std::size_t)color_data_size);
                std::memcpy((void*)imgDepth.data, (const void*)depth_data.get(), (std::size_t)depth_data_size);
				std::memcpy((void*)imgIR.data, (const void*)ir_data.get(), (std::size_t)ir_data_size);
                std::memcpy((void*)imgAlignedDepth.data, (const void*)aligned_depth_data.get(), (std::size_t)aligned_depth_data_size);

                today_tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                today_tm = *localtime(&today_tt);
                
                memset(output_filename, 0, OUTPATHFILE_LENGTH);
                snprintf(output_filename, OUTPATHFILE_LENGTH, "%s/%s_RGB_%02d%02d%02d_%02d%02d%02d_%02d.png",
                            (root_path / "rgbdepth" / "color").c_str(),
							prefix_filename,
                            (today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
                            today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec,
                            c_time_hook);
                imwrite(output_filename, imgColor, compresstion_params);

				memset(output_filename, 0, OUTPATHFILE_LENGTH);
				snprintf(output_filename, OUTPATHFILE_LENGTH, "%s/%s_IR_%02d%02d%02d_%02d%02d%02d_%02d.png",
							(root_path / "rgbdepth" / "ir").c_str(),
							prefix_filename,
							(today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
							today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec,
							c_time_hook);
				imwrite(output_filename, imgIR, compresstion_params);

                if(output_type_png == true || output_type_both == true) {
                    memset(output_filename, 0, OUTPATHFILE_LENGTH);
                    snprintf(output_filename, OUTPATHFILE_LENGTH, "%s/%s_DEPTH_%02d%02d%02d_%02d%02d%02d_%02d.png",
                                (root_path / "rgbdepth" / "depth").c_str(),
                                prefix_filename,
                                (today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
                                today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec,
                                c_time_hook);
                    imwrite(output_filename, imgDepth, compresstion_params);

                    memset(output_filename, 0, OUTPATHFILE_LENGTH);
                    snprintf(output_filename, OUTPATHFILE_LENGTH, "%s/%s_ALIGNEDDEPTH_%02d%02d%02d_%02d%02d%02d_%02d.png",
                                (root_path / "rgbdepth" / "aligned_depth").c_str(),
                                prefix_filename,
                                (today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
                                today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec,
                                c_time_hook);
                    imwrite(output_filename, imgAlignedDepth, compresstion_params);
                }
                else if(output_type_raw == true || output_type_both == true) {
                    memset(output_filename, 0, OUTPATHFILE_LENGTH);
                    snprintf(output_filename, OUTPATHFILE_LENGTH, "%s/%s_DEPTH_%02d%02d%02d_%02d%02d%02d_%02d.raw",
                                (root_path / "rgbdepth" / "depth").c_str(),
                                prefix_filename,
                                (today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
                                today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec,
                                c_time_hook);
                    ofstream out_file(output_filename, ios::out | ios::binary);
                    out_file.write((const char*)depth_data.get(), (int)depth_data_size);
                    out_file.close();

                    memset(output_filename, 0, OUTPATHFILE_LENGTH);
                    snprintf(output_filename, OUTPATHFILE_LENGTH, "%s/%s_ALIGNEDDEPTH_%02d%02d%02d_%02d%02d%02d_%02d.raw",
                                (root_path / "rgbdepth" / "aligned_depth").c_str(),
                                prefix_filename,
                                (today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
                                today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec,
                                c_time_hook);
                    ofstream out_file2(output_filename, ios::out | ios::binary);
                    out_file2.write((const char*)aligned_depth_data.get(), (int)aligned_depth_data_size);
                    out_file2.close();
                }

				output_csv << timestamp << ",";
				output_csv << imu_data.accel_x << "," << imu_data.accel_y << "," << imu_data.accel_z << ",";
				output_csv << imu_data.gyro_x << "," << imu_data.gyro_y << "," << imu_data.gyro_z << "\n";
				output_csv.flush();
            }

            if(flag_viewer == true) {
				imshow("Color_Saver", imgColor);
				cv::waitKey(50);
            }

            p_time_hook = c_time_hook;
        }
        else {
            usleep(33333);
        }
    } // end of while

	output_csv.close();

    if(flag_testonly == true) cout << "release memory..." << endl;
    imgColor.release();
    imgDepth.release();
	imgIR.release();
    imgAlignedDepth.release();

	if(flag_viewer == true) destroyAllWindows();

    color_data.reset();
    depth_data.reset();
	ir_data.reset();
    aligned_depth_data.reset();

    if(pHandle != nullptr) MemSync_ReleaseHandle(pHandle); pHandle = nullptr;

    return 0;
}
catch (const std::exception& e)
{
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
}

