
#include <cstring>
#include <chrono>
#include <memory>
#include <random>
#include "math.h"
#include <assert.h>

#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "memsync.h"

#include "SyncTransfer_Model.h" 

using namespace cv;

KIROSyncTransferModel::KIROSyncTransferModel()
{
    threadProc_KIROSyncTransfer_RunFlag = false;

    _bSimulationMode = false;

    Initialize();
}

KIROSyncTransferModel::~KIROSyncTransferModel()
{
    Finalize();
}

int KIROSyncTransferModel::Initialize()
{
    // init. variable
    _imageRSColorBufsize = _imageRSWidth * _imageRSHeight * _imageRSColorPixelBytes;
    _imageRSDepthBufsize = _imageRSWidth * _imageRSHeight * _imageRSDepthPixelBytes;
    _imageRSAlignedDepthBufsize = _imageRSWidth * _imageRSHeight * _imageRSAlignedDepthPixelBytes;
    _imageRSIRBufsize = _imageRSWidth * _imageRSHeight * _imageRSIRPixelBytes;
    _imageNVBufsize = _imageNVWidth * _imageNVHeight * _imageNVPixelBytes;
    _imageThermalBufsize = _imageThermalWidth * _imageThermalHeight * _imageThermalPixelBytes;
    _imageGSBufsize = _imageGSWidth * _imageGSHeight *_imageGSPixelBytes;
    _pointCloudBufsize = _numPointCloud * sizeof(ptcloud_t);

    _imageRSColorData = std::shared_ptr<uint8_t>(new uint8_t[_imageRSColorBufsize]);
    _imageRSDepthData = std::shared_ptr<uint8_t>(new uint8_t[_imageRSDepthBufsize]);
    _imageRSAlignedDepthData = std::shared_ptr<uint8_t>(new uint8_t[_imageRSAlignedDepthBufsize]);
    _imageRSIRData = std::shared_ptr<uint8_t>(new uint8_t[_imageRSIRBufsize]);
    _imageNV1Data = std::shared_ptr<uint8_t>(new uint8_t[_imageNVBufsize]);
    _imageNV2Data = std::shared_ptr<uint8_t>(new uint8_t[_imageNVBufsize]);
    _imageThermalData = std::shared_ptr<uint8_t>(new uint8_t[_imageThermalBufsize]);
    _imageGSData = std::shared_ptr<uint8_t>(new uint8_t[_imageGSBufsize]);
    _pointCloudData = std::shared_ptr<ptcloud_t>(new ptcloud_t[_numPointCloud]);

    std::memset((void*)_imageRSColorData.get(), 0, (std::size_t)(_imageRSColorBufsize));
    std::memset((void*)_imageRSDepthData.get(), 0, (std::size_t)(_imageRSDepthBufsize));
    std::memset((void*)_imageRSAlignedDepthData.get(), 0, (std::size_t)(_imageRSAlignedDepthBufsize));
    std::memset((void*)_imageRSIRData.get(), 0, (std::size_t)(_imageRSIRBufsize));
    std::memset((void*)_imageNV1Data.get(), 0, (std::size_t)(_imageNVBufsize));
    std::memset((void*)_imageNV2Data.get(), 0, (std::size_t)(_imageNVBufsize));
    std::memset((void*)_imageThermalData.get(), 0, (std::size_t)(_imageThermalBufsize));
    std::memset((void*)_imageGSData.get(), 0, (std::size_t)(_imageGSBufsize));
    std::memset((void*)_pointCloudData.get(), 0, (std::size_t)(_pointCloudBufsize));

    _curImageRSColorData_Timestamp = 0;
    _curImageRSDepthData_Timestamp = 0;
    _curImageRSAlignedDepthData_Timestamp = 0;
    _curImageRSIRData_Timestamp = 0;
    _curImageNV1Data_Timestamp = 0;
    _curImageNV2Data_Timestamp = 0;
    _curImageThermalData_Timestamp = 0;
    _curImageGSData_Timestamp = 0;
    _curPointCloudData_Timestamp = 0;
    _curSynchronized_Timestamp = 0;

    _RSCameraInfo_Color.height = 0; _RSCameraInfo_IR.height = 0;
    _RSCameraInfo_Color.width = 0;  _RSCameraInfo_IR.width = 0;
    for(int i=0; i<5; i++) { _RSCameraInfo_Color.D[i] = 0.0f; _RSCameraInfo_IR.D[i] = 0.0f; }
    for(int i=0; i<9; i++) { _RSCameraInfo_Color.K[i] = 0.0f; _RSCameraInfo_IR.K[i] = 0.0f; }
    for(int i=0; i<9; i++) { _RSCameraInfo_Color.R[i] = 0.0f; _RSCameraInfo_IR.R[i] = 0.0f; }
    for(int i=0; i<12; i++) { _RSCameraInfo_Color.P[i] = 0.0f; _RSCameraInfo_IR.P[i] = 0.0f; }

    if(threadProc_KIROSyncTransfer_RunFlag != false) {
        StopThreadLoop();
    }

    assert(true && "KIROSyncTransferModel::Initialize\n");

    return 0;
}

int KIROSyncTransferModel::Finalize()
{
    if(threadProc_KIROSyncTransfer_RunFlag != false) {
        StopThreadLoop();
    }

    if(_imageRSColorData.get() != nullptr) _imageRSColorData.reset(); _imageRSColorData = nullptr;
    if(_imageRSDepthData.get() != nullptr) _imageRSDepthData.reset(); _imageRSDepthData = nullptr;
    if(_imageRSAlignedDepthData.get() != nullptr) _imageRSAlignedDepthData.reset(); _imageRSAlignedDepthData = nullptr;
    if(_imageRSIRData.get() != nullptr) _imageRSIRData.reset(); _imageRSIRData = nullptr;
    if(_imageNV1Data.get() != nullptr) _imageNV1Data.reset(); _imageNV1Data = nullptr;
    if(_imageNV2Data.get() != nullptr) _imageNV2Data.reset(); _imageNV2Data = nullptr;
    if(_imageThermalData.get() != nullptr) _imageThermalData.reset(); _imageThermalData = nullptr;
    if(_imageGSData.get() != nullptr) _imageGSData.reset(); _imageGSData = nullptr;
    if(_pointCloudData.get() != nullptr) _pointCloudData.reset(); _pointCloudData = nullptr;

    assert(true && "RKIROSyncTransferModel::Finalize\n");

    return 0;
}

int KIROSyncTransferModel::StartThreadLoop()
{
    std::lock_guard<std::mutex> lock(_mtx);

    if(threadProc_KIROSyncTransfer_RunFlag == false) {
        threadProc_KIROSyncTransfer_RunFlag = true;
        _thKIROSyncTransfer = std::unique_ptr<std::thread>(new std::thread(ThreadProc_KIROSyncTransfer, this));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    assert(true && "KIROSyncTransferModel::StartThreadLoop\n");

    return 0;
}

int KIROSyncTransferModel::StopThreadLoop()
{
    std::lock_guard<std::mutex> lock(_mtx);

    if(threadProc_KIROSyncTransfer_RunFlag != false) {
        threadProc_KIROSyncTransfer_RunFlag = false;

        if(_thKIROSyncTransfer.get() != nullptr && _thKIROSyncTransfer->joinable() == true) _thKIROSyncTransfer->join();
        if(_thKIROSyncTransfer.get() != nullptr) _thKIROSyncTransfer.reset();
    }
	
    assert(true && "KIROSyncTransferModel::StopThreadLoop\n");

    return 0;
}

///////////////////////////////////////////////////////////////
// Get
int KIROSyncTransferModel::GetData_ImageRSColor(char*& data, unsigned long long &timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageRSColor::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageRSColorData.get(), (std::size_t)_imageRSColorBufsize);
    }
	timestamp = _curImageRSColorData_Timestamp;
	return 0;
}

int KIROSyncTransferModel::GetData_ImageRSDepth(char*& data, unsigned long long &timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageRSDepth::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageRSDepthData.get(), (std::size_t)_imageRSDepthBufsize);
    }
    timestamp = _curImageRSDepthData_Timestamp;
	return 0;
}

int KIROSyncTransferModel::GetData_ImageRSAlignedDepth(char*& data, unsigned long long &timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageRSAlignedDepth::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageRSAlignedDepthData.get(), (std::size_t)_imageRSAlignedDepthBufsize);
    }
    timestamp = _curImageRSAlignedDepthData_Timestamp;
    return 0;
}

int KIROSyncTransferModel::GetData_ImageRSIR(char*& data, unsigned long long &timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageRSIR::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageRSIRData.get(), (std::size_t)_imageRSIRBufsize);
    }
    timestamp = _curImageRSIRData_Timestamp;
	return 0;
}

int KIROSyncTransferModel::GetData_ImageRSCameraInfo_Color(struct CameraInfo& data, unsigned long long &timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    std::memcpy((void*)&data, (const void*)&_RSCameraInfo_Color, sizeof(struct CameraInfo));
    timestamp = _curRSCameraInfo_Color_Timestamp;
    return 0;
}

int KIROSyncTransferModel::GetData_ImageRSCameraInfo_IR(struct CameraInfo& data, unsigned long long &timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    std::memcpy((void*)&data, (const void*)&_RSCameraInfo_IR, sizeof(struct CameraInfo));
    timestamp = _curRSCameraInfo_IR_Timestamp;
    return 0;
}

int KIROSyncTransferModel::GetData_ImageNV1(char*& data, unsigned long long &timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageNV1::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageNV1Data.get(), (std::size_t)_imageNVBufsize);
    }
    timestamp = _curImageNV1Data_Timestamp;
	return 0;
}

int KIROSyncTransferModel::GetData_ImageNV2(char*& data, unsigned long long &timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageNV2::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageNV2Data.get(), (std::size_t)_imageNVBufsize);
    }
    timestamp = _curImageNV2Data_Timestamp;
	return 0;
}

int KIROSyncTransferModel::GetData_ImageThermal(char*& data, unsigned long long &timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageThermal::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageThermalData.get(), (std::size_t)_imageThermalBufsize);
    }
    timestamp = _curImageThermalData_Timestamp;
	return 0;
}

int KIROSyncTransferModel::GetData_ImageGS(char*& data, unsigned long long &timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_ImageGS::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)data, (const void*)_imageGSData.get(), (std::size_t)_imageGSBufsize);
    }
    timestamp = _curImageGSData_Timestamp;
    return 0;
}

int KIROSyncTransferModel::GetData_PointCloud(ptcloud_t* data, unsigned long long &timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "GetData_PointCloud:: data is nullptr");

    if(data != nullptr) { 
        std::memcpy((void*)data, (const void*)_pointCloudData.get(), (std::size_t)_pointCloudBufsize);
    }
    timestamp = _curPointCloudData_Timestamp;
    return 0;
}

int KIROSyncTransferModel::GetData_OdometryInfo(struct OdometryInfo& data, unsigned long long &timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    std::memcpy((void*)&data, (const void*)&_OdometryInfo, sizeof(struct OdometryInfo));
    timestamp = _curOdometryInfo_Timestamp;
    return 0;
}

int KIROSyncTransferModel::GetData_Synchronized(char*& RSColorData, char*& RSDepthData, char*& RSAlignedDepthData, 
                            char*& RSIRData, struct CameraInfo& RSCameraInfo_ColorData, struct CameraInfo& RSCameraInfo_IRData,
                            char*& NV1Data, char*& ThermalData, ptcloud_t* LiDARData, unsigned long long &timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    assert(RSColorData != nullptr && "GetData_Synchronized::RSColorData is nullptr");
    assert(RSDepthData != nullptr && "GetData_Synchronized::RSDepthData is nullptr");
    assert(RSAlignedDepthData != nullptr && "GetData_Synchronized::RSAlignedDepthData is nullptr");
    assert(RSIRData != nullptr && "GetData_Synchronized::RSIRData is nullptr");
    assert(NV1Data != nullptr && "GetData_Synchronized::NV1Data is nullptr");
    assert(ThermalData != nullptr && "GetData_Synchronized::ThermalData is nullptr");
    assert(LiDARData != nullptr && "GetData_Synchronized::LiDARData is nullptr");

    if(RSColorData != nullptr) {
        std::memcpy((void*)RSColorData, (const void*)_imageRSColorData.get(), (std::size_t)_imageRSColorBufsize);
    }
	
    if(RSDepthData != nullptr) {
        std::memcpy((void*)RSDepthData, (const void*)_imageRSDepthData.get(), (std::size_t)_imageRSDepthBufsize);
    }

    if(RSAlignedDepthData != nullptr) {
        std::memcpy((void*)RSAlignedDepthData, (const void*)_imageRSAlignedDepthData.get(), (std::size_t)_imageRSAlignedDepthBufsize);
    }
    
    if(RSIRData != nullptr) {
        std::memcpy((void*)RSIRData, (const void*)_imageRSIRData.get(), (std::size_t)_imageRSIRBufsize);
    }

    if(&RSCameraInfo_ColorData != nullptr) {
        std::memcpy((void*)&RSCameraInfo_ColorData, (const void*)&_RSCameraInfo_Color, sizeof(struct CameraInfo));
    }

    if(&RSCameraInfo_IRData != nullptr) {
        std::memcpy((void*)&RSCameraInfo_IRData, (const void*)&_RSCameraInfo_IR, sizeof(struct CameraInfo));
    }

    if(NV1Data != nullptr) {
        std::memcpy((void*)NV1Data, (const void*)_imageNV1Data.get(), (std::size_t)_imageNVBufsize);
    }

    if(ThermalData != nullptr) {
        std::memcpy((void*)ThermalData, (const void*)_imageThermalData.get(), (std::size_t)_imageThermalBufsize);
    }

    if(LiDARData != nullptr) { 
        std::memcpy((void*)LiDARData, (const void*)_pointCloudData.get(), (std::size_t)_pointCloudBufsize);
    }

	timestamp = _curImageRSColorData_Timestamp;
    return 0;
}


///// Set  
int KIROSyncTransferModel::SetData_ImageRSColor(const char*& data, unsigned long long timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageRSColor::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageRSColorData.get(), (const void*)data, _imageRSColorBufsize);
    }
	_curImageRSColorData_Timestamp = timestamp; 
	return 0;
}
 
int KIROSyncTransferModel::SetData_ImageRSDepth(const char*& data, unsigned long long timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageRSDepth::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageRSDepthData.get(), (const void*)data, _imageRSDepthBufsize);
    }
    _curImageRSDepthData_Timestamp = timestamp;
	return 0;
}

int KIROSyncTransferModel::SetData_ImageRSAlignedDepth(const char*& data, unsigned long long timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageRSAlignedDepth::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageRSAlignedDepthData.get(), (const void*)data, _imageRSAlignedDepthBufsize);
    }
    _curImageRSAlignedDepthData_Timestamp = timestamp;
    return 0;
}

int KIROSyncTransferModel::SetData_ImageRSIR(const char*& data, unsigned long long timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageRSIR::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageRSIRData.get(), (const void*)data, _imageRSIRBufsize);
    }
    _curImageRSIRData_Timestamp = timestamp;
	return 0;
}

int KIROSyncTransferModel::SetData_ImageRSCameraInfo_Color(struct CameraInfo& data, unsigned long long timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    std::memcpy((void*)&_RSCameraInfo_Color, (const void*)&data, sizeof(struct CameraInfo));
    _curRSCameraInfo_Color_Timestamp = timestamp;
    return 0;
}

int KIROSyncTransferModel::SetData_ImageRSCameraInfo_IR(struct CameraInfo& data, unsigned long long timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    std::memcpy((void*)&_RSCameraInfo_IR, (const void*)&data, sizeof(struct CameraInfo));
    _curRSCameraInfo_IR_Timestamp = timestamp;
    return 0;
}

int KIROSyncTransferModel::SetData_ImageNV1(const char*& data, unsigned long long timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageNV1::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageNV1Data.get(), (const void*)data, _imageNVBufsize);
    }
    _curImageNV1Data_Timestamp = timestamp;
	return 0;
}

int KIROSyncTransferModel::SetData_ImageNV2(const char*& data, unsigned long long timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageNV2::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageNV2Data.get(), (const void*)data, _imageNVBufsize);
    }
    _curImageNV2Data_Timestamp = timestamp;
	return 0;
}

int KIROSyncTransferModel::SetData_ImageThermal(const char*& data, unsigned long long timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageThermal::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageThermalData.get(), (const void*)data, _imageThermalBufsize);
    }
    _curImageThermalData_Timestamp = timestamp;
	return 0;
}

int KIROSyncTransferModel::SetData_ImageGS(const char*& data, unsigned long long timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_ImageGS::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_imageGSData.get(), (const void*)data, _imageGSBufsize);
    }
    _curImageGSData_Timestamp = timestamp;
    return 0;
}

int KIROSyncTransferModel::SetData_PointCloud(const ptcloud_t* data, unsigned long long timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    assert(data != nullptr && "SetData_PointCloud::data is nullptr");

    if(data != nullptr) {
        std::memcpy((void*)_pointCloudData.get(), (const void*)data, _pointCloudBufsize);
    }
    _curPointCloudData_Timestamp = timestamp;
    return 0;
}

int KIROSyncTransferModel::SetData_OdometryInfo(struct OdometryInfo& data, unsigned long long timestamp)
{
    std::lock_guard<std::mutex> lock(_mtx);

    std::memcpy((void*)&_OdometryInfo, (const void*)&data, sizeof(struct OdometryInfo));
    _curOdometryInfo_Timestamp = timestamp;
    return 0;
}

int KIROSyncTransferModel::SetData_Synchronized(const char*& RSColorData, const char*& RSDepthData, const char*& RSAlignedDepthData, 
                            const char*& RSIRData, struct CameraInfo& RSCameraInfo_ColorData, struct CameraInfo& RSCameraInfo_IRData,
                            const char*& NV1Data, const char*& ThermalData, const ptcloud_t* LiDARData, unsigned long long timestamp)
{
	std::lock_guard<std::mutex> lock(_mtx);

    assert(RSColorData != nullptr && "SetData_Synchronized::RSColorData is nullptr");
    assert(RSDepthData != nullptr && "SetData_Synchronized::RSDepthData is nullptr");
    assert(RSAlignedDepthData != nullptr && "SetData_Synchronized::RSAlignedDepthData is nullptr");
    assert(RSIRData != nullptr && "SetData_Synchronized::RSIRData is nullptr");
    assert(NV1Data != nullptr && "SetData_Synchronized::RSColorData is nullptr");
    assert(ThermalData != nullptr && "SetData_Synchronized::ThermalData is nullptr");
    assert(LiDARData != nullptr && "SetData_Synchronized::LiDARData is nullptr");

    if(RSColorData != nullptr) {
        std::memcpy((void*)_imageRSColorData.get(), (const void*)RSColorData, _imageRSColorBufsize);
    }

    if(RSDepthData != nullptr) {
        std::memcpy((void*)_imageRSDepthData.get(), (const void*)RSDepthData, _imageRSDepthBufsize);
    }

    if(RSAlignedDepthData != nullptr) {
        std::memcpy((void*)_imageRSAlignedDepthData.get(), (const void*)RSAlignedDepthData, _imageRSAlignedDepthBufsize);
    }

    if(RSIRData != nullptr) {
        std::memcpy((void*)_imageRSIRData.get(), (const void*)RSIRData, _imageRSIRBufsize);
    }

    if(&RSCameraInfo_ColorData != nullptr) {
        std::memcpy((void*)&_RSCameraInfo_Color, (const void*)&RSCameraInfo_ColorData, sizeof(struct CameraInfo));
    }

    if(&RSCameraInfo_IRData != nullptr) {
        std::memcpy((void*)&_RSCameraInfo_IR, (const void*)&RSCameraInfo_IRData, sizeof(struct CameraInfo));
    }

    if(NV1Data != nullptr) {
        std::memcpy((void*)_imageNV1Data.get(), (const void*)NV1Data, _imageNVBufsize);
    }

    if(ThermalData != nullptr) {
        std::memcpy((void*)_imageThermalData.get(), (const void*)ThermalData, _imageThermalBufsize);
    }

    if(LiDARData != nullptr) {
        std::memcpy((void*)_pointCloudData.get(), (const void*)LiDARData, _pointCloudBufsize);
    }

	_curSynchronized_Timestamp = timestamp; 
	return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////
void KIROSyncTransferModel::ThreadProc_KIROSyncTransfer(void *arg)
{
    #define CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP   if(pKIROSyncTransferModel == nullptr || pKIROSyncTransferModel->threadProc_KIROSyncTransfer_RunFlag != true) break;
    #define CHECK_KIROSYNCTRANSFERMODEL_CONDITION    (pKIROSyncTransferModel != nullptr && pKIROSyncTransferModel->threadProc_KIROSyncTransfer_RunFlag == true)

    KIROSyncTransferModel *pKIROSyncTransferModel = (KIROSyncTransferModel*)arg;

    fprintf(stderr, "KIROSyncTransferModel::ThreadProc_RGBDepthCamera: start threadloop\n");

    // create OpenCV Mat & PointCloud
    cv::Mat imgRSColor(cv::Size((int)pKIROSyncTransferModel->GetImageRSWidth(), (int)pKIROSyncTransferModel->GetImageRSHeight()), CV_8UC3);
    cv::Mat imgRSDepth(cv::Size((int)pKIROSyncTransferModel->GetImageRSWidth(), (int)pKIROSyncTransferModel->GetImageRSHeight()), CV_16UC1);
    cv::Mat imgRSAlignedDepth(cv::Size((int)pKIROSyncTransferModel->GetImageRSWidth(), (int)pKIROSyncTransferModel->GetImageRSHeight()), CV_16UC1);
    cv::Mat imgRSIR(cv::Size((int)pKIROSyncTransferModel->GetImageRSWidth(), (int)pKIROSyncTransferModel->GetImageRSHeight()), CV_8UC1);    
    cv::Mat imgNV1(cv::Size((int)pKIROSyncTransferModel->GetImageNVWidth(), (int)pKIROSyncTransferModel->GetImageNVHeight()), CV_8UC3);
    cv::Mat imgNV2(cv::Size((int)pKIROSyncTransferModel->GetImageNVWidth(), (int)pKIROSyncTransferModel->GetImageNVHeight()), CV_8UC3);
    cv::Mat imgThermal(cv::Size((int)pKIROSyncTransferModel->GetImageThermalWidth(), (int)pKIROSyncTransferModel->GetImageThermalHeight()), CV_16UC1);
    cv::Mat imgGS(cv::Size((int)pKIROSyncTransferModel->GetImageGSWidth(), (int)pKIROSyncTransferModel->GetImageGSHeight()), CV_8UC3);

    auto pointcloudData = std::shared_ptr<ptcloud_t>(new ptcloud_t[_numPointCloud]);

    // intrinsic parameters
    struct CameraInfo RSCameraInfo_Color, RSCameraInfo_IR;

    // init. memsync interface
    void* pHandle = nullptr;
    int retCode = 0;

    pHandle = MemSync_GetHandle();
    assert(pHandle != nullptr && "MemSync handle is nullptr");

    if(pHandle != nullptr && MemSync_ValidateHandle(pHandle) == true) {
        MemSync_SetServerInfo(pHandle, "localhost", 11211);
    }

    // thread-loop
    unsigned long color_data_timestamp = 0;
    unsigned long depth_data_timestamp = 0;
    unsigned long aligned_depth_data_timestamp = 0;
    unsigned long ir_data_timestamp = 0;
    unsigned long intrinsic_data_timestamp = 0;
    unsigned long nv1_data_timestamp = 0;
    unsigned long nv2_data_timestamp = 0;
    unsigned long thermal_data_timestamp = 0;
    unsigned long gs_data_timestamp = 0;
    unsigned long pointcloud_data_timestamp = 0;

    unsigned long color_data_size = 0;
    unsigned long depth_data_size = 0;
    unsigned long aligned_depth_data_size = 0;
    unsigned long ir_data_size = 0;
    unsigned long intrinsic_data_size = 0;
    unsigned long nv1_data_size = 0;
    unsigned long nv2_data_size = 0;
    unsigned long thermal_data_size = 0;
    unsigned long gs_data_size = 0;
    unsigned long pointcloud_data_size = 0;

    while(CHECK_KIROSYNCTRANSFERMODEL_CONDITION)
    {
        if(pKIROSyncTransferModel != nullptr && pKIROSyncTransferModel->isSimulationMode() == false)
        {
            if(pHandle != nullptr && MemSync_ValidateHandle(pHandle) == true) {
                // get data from MemSync server
                MemSync_SetID(pHandle, "RealSense_Color");
                retCode = MemSync_Read(pHandle, (char*)(imgRSColor.data), color_data_size, color_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageRSColor((const char*&)imgRSColor.data, color_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "RealSense_Depth");
                retCode = MemSync_Read(pHandle, (char*)(imgRSDepth.data), depth_data_size, depth_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageRSDepth((const char*&)imgRSDepth.data, depth_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "RealSense_IR");
                retCode = MemSync_Read(pHandle, (char*)(imgRSIR.data), ir_data_size, ir_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageRSIR((const char*&)imgRSIR.data, ir_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "RealSense_AlignedDepth");
                retCode = MemSync_Read(pHandle, (char*)(imgRSAlignedDepth.data), aligned_depth_data_size, aligned_depth_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageRSAlignedDepth((const char*&)imgRSAlignedDepth.data, aligned_depth_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "RealSense_ColorCameraInfo");
                retCode = MemSync_Read(pHandle, (char*)&RSCameraInfo_Color, intrinsic_data_size, intrinsic_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageRSCameraInfo_Color(RSCameraInfo_Color, intrinsic_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "RealSense_IRCameraInfo");
                retCode = MemSync_Read(pHandle, (char*)&RSCameraInfo_IR, intrinsic_data_size, intrinsic_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageRSCameraInfo_IR(RSCameraInfo_IR, intrinsic_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "NightVision1_Image");
                retCode = MemSync_Read(pHandle, (char*)(imgNV1.data), nv1_data_size, nv1_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageNV1((const char*&)imgNV1.data, nv1_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "Thermal_Image");
                retCode = MemSync_Read(pHandle, (char*)(imgThermal.data), thermal_data_size, thermal_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;
                }
                pKIROSyncTransferModel->SetData_ImageThermal((const char*&)imgThermal.data, thermal_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "RGBGS");
                retCode = MemSync_Read(pHandle, (char*)(imgGS.data), gs_data_size, gs_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;                
                }
                pKIROSyncTransferModel->SetData_ImageGS((const char*&)imgGS.data, gs_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                MemSync_SetID(pHandle, "LiDAR_PointCloud");
                retCode = MemSync_Read(pHandle, (char*)pointcloudData.get(), pointcloud_data_size, pointcloud_data_timestamp);
                if(retCode == MEMSYNC_MUTEX_LOCKED) {
                    std::cerr << "MemSync_Read failed - mutex locked" << std::endl;                
                }
                pKIROSyncTransferModel->SetData_PointCloud((const ptcloud_t*)(pointcloudData.get()), pointcloud_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;

                // Set synchronized data
                pKIROSyncTransferModel->SetData_Synchronized((const char*&)imgRSColor.data, (const char*&)imgRSDepth.data, (const char*&)imgRSAlignedDepth.data,
                                                            (const char*&)imgRSIR.data, (struct CameraInfo&)RSCameraInfo_Color, (struct CameraInfo&)RSCameraInfo_IR,
                                                            (const char*&)imgNV1.data, (const char*&)imgThermal.data, (const ptcloud_t*)(pointcloudData.get()),
                                                            color_data_timestamp);
                CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;
            }
        }
        else { // Simulation Mode,
 
        }

        CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
        CHECK_KIROSYNCTRANSFERMODEL_THREADLOOP;   
    }

    imgRSColor.release();
    imgRSDepth.release();
    imgRSAlignedDepth.release();
    imgRSIR.release();
    imgNV1.release();
    imgNV2.release();
    imgThermal.release();
    imgGS.release();
    
    if(pointcloudData.get() != nullptr) pointcloudData.reset(); pointcloudData = nullptr;
    if(pHandle != nullptr) MemSync_ReleaseHandle(pHandle); pHandle = nullptr;

    if(pKIROSyncTransferModel != nullptr) pKIROSyncTransferModel->threadProc_KIROSyncTransfer_RunFlag = false;

    return;
}


