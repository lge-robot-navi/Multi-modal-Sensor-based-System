#ifndef _GEV_CAMERA_H_849743274928573892753485638543875348534
#define _GEV_CAMERA_H_849743274928573892753485638543875348534

#include <PvSystem.h>
#include <PvInterface.h> 
#include <PvDevice.h>
#include <PvStream.h>
#include <PvBuffer.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvDeviceGEV.h>
#include <PvPixelType.h>
#include <PvBufferWriter.h>
#include <PvBufferConverterRGBFilter.h>

#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <list>
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>

typedef std::list<PvBuffer*> BufferList;
#define BUFFER_COUNT (16)

class GEVCamera
{
    public:
        GEVCamera(std::string macAddress);
        ~GEVCamera();

    public:
        void connect();
        void disconnect();

        PvStream* openStream(const PvDeviceInfo* aDeviceInfo);
        void configureStream(PvDevice* aDevice, PvStream* aStream);
        void createStreamBuffers(PvDevice* aDevice, PvStream* aStream, BufferList* aBufferList);
        void acquireImages(PvDevice* aDevice, PvStream* aStream);
        cv::Mat1b getImage();
        void getRawData(uint32_t& width, uint32_t& height, uint8_t*& bufferPtr);

        std::atomic<bool> running;

        std::mutex imgReadyMutex;
        std::condition_variable imgReadyConditionVariable;

        PvString macAddress;
        PvSystem system;
        PvStream* stream;
        PvDevice* device;
        BufferList bufferList;

        uint32_t imgWidth;
        uint32_t imgHeight;
        uint32_t imgPixelSize;

        uint8_t* imgBufferPtr;

        const PvDeviceInfo* deviceInfo;

        std::shared_ptr<std::thread> threadPtr;

};

#endif
