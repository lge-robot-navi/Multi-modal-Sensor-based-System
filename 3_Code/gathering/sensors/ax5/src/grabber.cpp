
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include "gev_camera.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>
#include "memsync.h"

using namespace std;

std::atomic<bool> flag_viewer(false);
std::atomic<bool> flag_testonly(false);
std::atomic<bool> flag_running(true);


int main(int argc, char** argv) try
{
	std::string deviceMacAddress = "00:11:1c:02:b7:40";

	if(argv != nullptr) {
		for(int i=1; i<argc; i++)
		{
			if(!std::strncmp(argv[i], "--help", 6)) {
				cout << "Usage: " << argv[0] << "[--viewer true/*false] [--test true/*false] [--device 00:11:1c:02:b7:40]" << endl;
				return 0;
			}
			else if(!std::strncmp(argv[i], "--viewer", 8)) {
				if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_viewer = true;
			}
			else if(!std::strncmp(argv[i], "--test", 6)) {
				if(i+1 < argc && !std::strncmp(argv[++i], "true", 4)) flag_testonly = true;
			}
			else if(!std::strncmp(argv[i], "--device", 8)) {
				if(i+1 < argc && std::strlen(argv[++i]) >= 17) deviceMacAddress = std::string(argv[i]);
				cout << "ThermalGrabber:: Device Mac is " << deviceMacAddress << "\n" << endl;
			}			
		}
	}

	// set signal handler
	signal(SIGINT, [](int){ flag_running = false; });
	signal(SIGABRT, [](int){ flag_running = false; });
	signal(SIGKILL, [](int){ flag_running = false; });
	signal(SIGTERM, [](int){ flag_running = false; });

	void* pHandle = nullptr;
	int retCode = 0;

	pHandle = MemSync_GetHandle();
	if(flag_testonly == true)
		if(pHandle == nullptr) cout << "MemSync handle is nullptr" << endl;

	if(MemSync_ValidateHandle(pHandle) == true) {
		MemSync_SetServerInfo(pHandle, "localhost", 11211);
		MemSync_SetID(pHandle, "Thermal_Image");
	}

	unsigned long timestamp = 0;
	
	uint32_t width = 0;
	uint32_t height = 0;
	uint8_t* pData = nullptr;

	// init. thermal camera using ebus sdk
	GEVCamera camera(deviceMacAddress); 
	camera.connect();
	if(flag_testonly == true) cout << "connected camera" << endl;

	if(flag_viewer == true) {
		cv::namedWindow("Thermal", CV_WINDOW_AUTOSIZE);
	}

	while(flag_running == true)
	{
		camera.getRawData(width, height, pData);
		if(width > 0 && height > 0 && pData != nullptr) {	
			if(flag_testonly == false) {
				if(MemSync_ValidateHandle(pHandle) == true) {
					timestamp = MemSync_CurrentTimestamp();
					retCode = MemSync_Write(pHandle, (const char*)pData, (width * height), timestamp);
					if(retCode == MEMSYNC_MUTEX_LOCKED) {
						throw "MemSync_Write failed - mutex locked";
					}
				}
				else {
					throw "MemSync handle is invalidate";
				}
			}
			
			if(flag_viewer == true) {
				cv::Mat1b img = cv::Mat1b(height, width, pData);
				cv::imshow("Thermal", img);
				img.release();
				cvWaitKey(50);
				continue;
			}
		}
		else {
			throw "camera.getRawData has failure! pData is nullptr";
		}

		usleep(50);
	}

	if(flag_testonly == true) cout << "disconnect camera and release memsync" << endl;
	if(flag_viewer == true) cv::destroyAllWindows();

	camera.disconnect();

	if(pHandle != nullptr) MemSync_ReleaseHandle(pHandle); pHandle = nullptr;

	return 0;
}
catch (const std::exception& e)
{
	std::cerr << "Exception: " << e.what() << std::endl;
	return -1;
}
catch (const char* e)
{
	std::cerr << "Exception: " << e << std::endl;
	return -1;
}
catch (std::string& e)
{
	std::cerr << "Exception: " << e << std::endl;
	return -1;
}
catch (...)
{
	std::cerr << "Exception occurred" << endl;
	return -1;
}


