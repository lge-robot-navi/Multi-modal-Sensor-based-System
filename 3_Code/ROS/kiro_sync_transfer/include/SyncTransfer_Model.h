
#ifndef _SYNC_TRANSFER_MODEL_H_489298098034278923764824235
#define _SYNC_TRANSFER_MODEL_H_489298098034278923764824235

#include <mutex>
#include <thread>
#include <atomic>

#include "CommonStruct.h"

using namespace std;

class KIROSyncTransferModel
{
	public:
		KIROSyncTransferModel();
		~KIROSyncTransferModel();

	public:		
		int Initialize();
		int Finalize();

		int StartThreadLoop();
		int StopThreadLoop();
		bool isThreadRunning() { return threadProc_KIROSyncTransfer_RunFlag; }

		void SetSimulationMode(bool mode) { _bSimulationMode = mode; }
		bool isSimulationMode() { return _bSimulationMode; }

	public:
		// Intel RealSense Camera
		unsigned int GetImageRSWidth() { return _imageRSWidth; }
		unsigned int GetImageRSHeight() { return _imageRSHeight; }
		unsigned int GetRSColorPixelBytes() { return _imageRSColorPixelBytes; }
		unsigned int GetRSDepthPixelBytes() { return _imageRSDepthPixelBytes; }
		unsigned int GetRSAlignedDepthPixelBytes() { return _imageRSAlignedDepthPixelBytes; }
		unsigned int GetRSIRPixelBytes() { return _imageRSIRPixelBytes; }

		unsigned int GetImageRSColorBufsize() { return _imageRSColorBufsize; }
		unsigned int GetImageRSDepthBufsize() { return _imageRSDepthBufsize; }
		unsigned int GetImageRSAlignedDepthBufsize() { return _imageRSAlignedDepthBufsize; }
		unsigned int GetImageRSIRBufsize() { return _imageRSIRBufsize; }

		// RunCam NightVision Camera
		unsigned int GetImageNVWidth() { return _imageNVWidth; }
		unsigned int GetImageNVHeight() { return _imageNVHeight; }
		unsigned int GetImageNVPixelBytes() { return _imageNVPixelBytes; }

		unsigned int GetImageNVBufsize() { return _imageNVBufsize; }

		// Thermal Camera
		unsigned int GetImageThermalWidth() { return _imageThermalWidth; }
		unsigned int GetImageThermalHeight() { return _imageThermalHeight; }
		unsigned int GetImageThermalPixelBytes() { return _imageThermalPixelBytes; }

		unsigned int GetImageThermalBufsize() { return _imageThermalBufsize; }

        // Global Shutter Camera
        unsigned int GetImageGSWidth() { return _imageGSWidth; }
        unsigned int GetImageGSHeight() { return _imageGSHeight; }
        unsigned int GetImageGSPixelBytes() { return _imageGSPixelBytes; }

        unsigned int GetImageGSBufsize() { return _imageGSBufsize; }

		// Resize Image
		unsigned int GetImageResizeWidth() { return _imageResizeWidth; }
		unsigned int GetImageResizeHeight() { return _imageResizeHeight; }

		// LiDAR
		unsigned int GetNumPointCloud() { return _numPointCloud; }
		unsigned int GetPointCloudBufsize() { return _pointCloudBufsize; }

		// CameraInfo
		unsigned int GetCameraInfoBufsize() { return sizeof(struct CameraInfo); }

		// OdometryInfo
		unsigned int GetOdometryInfoBufsize() { return sizeof(struct OdometryInfo); }

	public:
		int GetData_ImageRSColor(char*& data, unsigned long long &timestamp);
		int GetData_ImageRSDepth(char*& data, unsigned long long &timestamp);
		int GetData_ImageRSAlignedDepth(char *& data, unsigned long long &timestamp);
		int GetData_ImageRSIR(char*& data, unsigned long long &timestamp);
		int GetData_ImageRSCameraInfo_Color(struct CameraInfo& data, unsigned long long &timestamp);
		int GetData_ImageRSCameraInfo_IR(struct CameraInfo& data, unsigned long long &timestamp);
		int GetData_ImageNV1(char*& data, unsigned long long &timestamp);
		int GetData_ImageNV2(char*& data, unsigned long long &timestamp);
		int GetData_ImageThermal(char*& data, unsigned long long &timestamp);
        int GetData_ImageGS(char*& data, unsigned long long &timestamp);
		int GetData_PointCloud(ptcloud_t* data, unsigned long long &timestamp);
		int GetData_OdometryInfo(struct OdometryInfo& data, unsigned long long &timestamp);

		int GetData_Synchronized(char*& RSColorData, char*& RSDepthData, char*& RSAlignedDepthData, 
								 char*& RSIRData, struct CameraInfo& RSCameraInfo_ColorData, struct CameraInfo& RSCameraInfo_IRData,
								 char*& NV1Data, char*& ThermalData, ptcloud_t* LiDARData, unsigned long long &timestamp);

		int SetData_ImageRSColor(const char*& data, unsigned long long timestamp);
		int SetData_ImageRSDepth(const char*& data, unsigned long long timestamp);
		int SetData_ImageRSAlignedDepth(const char*& data, unsigned long long timestamp);
		int SetData_ImageRSIR(const char*& data, unsigned long long timestamp);
		int SetData_ImageRSCameraInfo_Color(struct CameraInfo& data, unsigned long long timestamp);
		int SetData_ImageRSCameraInfo_IR(struct CameraInfo& data, unsigned long long timestamp);
		int SetData_ImageNV1(const char*& data, unsigned long long timestamp);
		int SetData_ImageNV2(const char*& data, unsigned long long timestamp);
		int SetData_ImageThermal(const char*& data, unsigned long long timestamp);
        int SetData_ImageGS(const char*& data, unsigned long long timestamp);
		int SetData_PointCloud(const ptcloud_t* data, unsigned long long timestamp);
		int SetData_OdometryInfo(struct OdometryInfo& data, unsigned long long timestamp);

		int SetData_Synchronized(const char*& RSColorData, const char*& RSDepthData, const char*& RSAlignedDepthData, 
								 const char*& RSIRData, struct CameraInfo& RSCameraInfo_ColorData, struct CameraInfo& RSCameraInfo_IRData,
								 const char*& NV1Data, const char*& ThermalData, const ptcloud_t* LiDARData, unsigned long long timestamp);

	private:
		std::atomic<bool> _bSimulationMode;

		// Intel RealSense Camera
		const static unsigned int _imageRSWidth = 640;
		const static unsigned int _imageRSHeight = 480;
		const static unsigned int _imageRSColorPixelBytes = 3;	// CV_8UC3
		const static unsigned int _imageRSDepthPixelBytes = 2;	// CV_16UC1
		const static unsigned int _imageRSAlignedDepthPixelBytes = 2;	// CV_16UC1
		const static unsigned int _imageRSIRPixelBytes = 1;		// CV_8UC1

		// RunCam NightVision Camera
		const static unsigned int _imageNVWidth = 720;
		const static unsigned int _imageNVHeight = 480;
		const static unsigned int _imageNVPixelBytes = 3;		// CV_8UC3

		// Thermal Camera
		const static unsigned int _imageThermalWidth = 640;
		const static unsigned int _imageThermalHeight = 512;
		const static unsigned int _imageThermalPixelBytes = 2;	// CV_16UC1

        // Global Shutter Camera
        const static unsigned int _imageGSWidth = 640;
        const static unsigned int _imageGSHeight = 480;
        const static unsigned int _imageGSPixelBytes = 3;       // CV_8UC3

		// Resize Image
		const static unsigned int _imageResizeWidth = 640;
		const static unsigned int _imageResizeHeight = 480;

		// LiDAR
		const static unsigned int _numPointCloud = 30000;

		std::shared_ptr<uint8_t> _imageRSColorData;
		std::shared_ptr<uint8_t> _imageRSDepthData;
		std::shared_ptr<uint8_t> _imageRSAlignedDepthData;
		std::shared_ptr<uint8_t> _imageRSIRData;
		std::shared_ptr<uint8_t> _imageNV1Data, _imageNV2Data;
		std::shared_ptr<uint8_t> _imageThermalData;
        std::shared_ptr<uint8_t> _imageGSData;
		std::shared_ptr<ptcloud_t> _pointCloudData;

		unsigned int _imageRSColorBufsize;
		unsigned int _imageRSDepthBufsize;
		unsigned int _imageRSAlignedDepthBufsize;
		unsigned int _imageRSIRBufsize;		
		unsigned int _imageNVBufsize;
		unsigned int _imageThermalBufsize;
        unsigned int _imageGSBufsize;
		unsigned int _pointCloudBufsize;

		unsigned long long _curImageRSColorData_Timestamp;
		unsigned long long _curImageRSDepthData_Timestamp;
		unsigned long long _curImageRSAlignedDepthData_Timestamp;
		unsigned long long _curImageRSIRData_Timestamp;
		unsigned long long _curImageNV1Data_Timestamp, _curImageNV2Data_Timestamp;
		unsigned long long _curImageThermalData_Timestamp;
        unsigned long long _curImageGSData_Timestamp;
		unsigned long long _curPointCloudData_Timestamp;

		struct CameraInfo _RSCameraInfo_Color, _RSCameraInfo_IR;

		unsigned long long _curRSCameraInfo_Color_Timestamp;
		unsigned long long _curRSCameraInfo_IR_Timestamp;

		struct OdometryInfo _OdometryInfo;

		unsigned long long _curOdometryInfo_Timestamp;

		unsigned long long _curSynchronized_Timestamp;

	private:
		mutable std::mutex _mtx; 

		std::unique_ptr<std::thread> _thKIROSyncTransfer;

		std::atomic<bool> threadProc_KIROSyncTransfer_RunFlag;
		static void ThreadProc_KIROSyncTransfer(void *arg);

};


#endif


