
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <memory>
#include <chrono>
#include <iomanip>
#include <experimental/filesystem>

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


int main(int argc, char** argv) try 
{
    auto starttime = std::chrono::system_clock::now();
    root_path = "/work/data";
	char* prefix_filename = "FX01"; 

    // check arguments
    if(argv != nullptr) {
        for(int i=1; i<argc; i++) {
            if(!std::strncmp(argv[i], "--help", 6)) {
                cout << "Usage: " << argv[0] << " [--viewer true/*false] [--test true/*false] [--starttime <HHMMSS>] [--output <root_path>] [--prefix <prefix_filename>] [--rate <saving_rate(Hz)>]" << endl;
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
        } // for loop
    }

    // set signal handler
    signal(SIGINT, [](int){ flag_running = false; });
    signal(SIGABRT, [](int){ flag_running = false; });
    signal(SIGKILL, [](int){ flag_running = false; });
    signal(SIGTERM, [](int){ flag_running = false; });

    // creating openCV Matrix
    int w = 640, h = 480;
    std::shared_ptr<char> rgbgs_data(new char[w * h * 3]); 
    std::memset((void*)rgbgs_data.get(), 0, (std::size_t)(w * h * 3));
    Mat imgRGBGS(Size(w, h), CV_8UC3, (void*)rgbgs_data.get(), Mat::AUTO_STEP);
    std::size_t rgbgs_bufsize = imgRGBGS.total() * imgRGBGS.elemSize();
    if(flag_testonly == true) cout << "create color mat (size: " << rgbgs_bufsize << ")" << endl;

    if(flag_viewer == true) {
        // show image
        namedWindow("RGBGS_Saver", WINDOW_AUTOSIZE);
        imshow("RGBGS_Saver", imgRGBGS);
    }

    // init. memsync interface
    void* pHandle = nullptr;
    int retCode = 0;

    pHandle = MemSync_GetHandle();
    if(flag_testonly == true)
        if(pHandle == nullptr) cout << "MemSync handle is nullptr" << endl;

    if(MemSync_ValidateHandle(pHandle) == true) {
        MemSync_SetServerInfo(pHandle, "localhost", 11211);
        MemSync_SetID(pHandle, "RGBGS");
    }

    // check output directories
    if(!fs::exists(root_path)) fs::create_directory(root_path);
    if(!fs::exists(root_path / "rgbgs")) fs::create_directory(root_path / "rgbgs");

    unsigned long timestamp = 0;
    unsigned long rgbgs_data_size = 0;

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
    time_t today_tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm today_tm = *localtime(&today_tt);

    auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    int p_time_hook = (int)((epoch_ms.count() % 1000) / (1000 / saving_rate));
    int c_time_hook = 0;

    while(flag_running == true)
    {
        // save every 100msec (default: 10Hz)
        epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        c_time_hook = (int)((epoch_ms.count() % 1000) / (1000 / saving_rate));
        if(c_time_hook != p_time_hook) {
            if(flag_testonly == false) {                
                rgbgs_data_size = 0;

                if(MemSync_ValidateHandle(pHandle) == true) {             
                    retCode = MemSync_Read(pHandle, rgbgs_data.get(), rgbgs_data_size, timestamp);
                    if(retCode == MEMSYNC_MUTEX_LOCKED) {
                        std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                    }
                }

                std::memcpy((void*)imgRGBGS.data, (const void*)rgbgs_data.get(), (std::size_t)rgbgs_data_size);
		cv::cvtColor(imgRGBGS, imgRGBGS, cv::COLOR_RGB2BGR);

                today_tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                today_tm = *localtime(&today_tt);
                
                memset(output_filename, 0, OUTPATHFILE_LENGTH);
                snprintf(output_filename, OUTPATHFILE_LENGTH, "%s/%s_RGBGS_%02d%02d%02d_%02d%02d%02d_%02d.png",
                            (root_path / "rgbgs").c_str(),
			    prefix_filename,
                            (today_tm.tm_year % 100), (today_tm.tm_mon+1), today_tm.tm_mday,
                            today_tm.tm_hour, today_tm.tm_min, today_tm.tm_sec,
                            c_time_hook);
                imwrite(output_filename, imgRGBGS);
            }

            if(flag_viewer == true) {
                imshow("RGBGS_Saver", imgRGBGS);
            }

            p_time_hook = c_time_hook;
        }
        else {
            usleep(33333);
        }
    } // end of while

    if(flag_testonly == true) cout << "release memory..." << endl;
    imgRGBGS.release();

	if(flag_viewer == true) destroyAllWindows();

    rgbgs_data.reset();

    if(pHandle != nullptr) MemSync_ReleaseHandle(pHandle); pHandle = nullptr;

    return 0;
}
catch (const std::exception& e)
{
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
}

