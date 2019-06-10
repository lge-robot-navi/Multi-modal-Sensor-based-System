
#include "gev_camera.h"

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <iomanip>

using namespace std;

GEVCamera::GEVCamera(std::string macAddress)
    : macAddress(macAddress.c_str())
    , running(true)
    , system()
    , stream(nullptr)
    , device(nullptr)
    , bufferList()
    , deviceInfo()
    , imgWidth()
    , imgHeight()
    , imgBufferPtr(nullptr)
    , threadPtr()
    , imgReadyConditionVariable()
    , imgReadyMutex()
{

}

GEVCamera::~GEVCamera(void)
{
    if(imgBufferPtr != nullptr) free(imgBufferPtr); imgBufferPtr = nullptr;
}

void GEVCamera::connect()
{
    if(threadPtr != nullptr) {
        cerr << "GEVCamera::connect::error: there is a connection running." << endl;
        return;
    }

    PvResult result = system.FindDevice(macAddress, &deviceInfo);
    device = PvDevice::CreateAndConnect(deviceInfo, &result);
    stream = openStream(deviceInfo);
    configureStream(device, stream);
    createStreamBuffers(device, stream, &bufferList);
    running = true;
    threadPtr = std::make_shared<std::thread>(&GEVCamera::acquireImages, this, this->device, this->stream);
}

PvStream* GEVCamera::openStream(const PvDeviceInfo* aDeviceInfo)
{
    PvStream* lStream;
    PvResult lResult;

    cout << "Opening stream to device." << endl;
    lStream = PvStream::CreateAndOpen(aDeviceInfo->GetConnectionID(), &lResult);
    if(lStream == nullptr) {
        cout << "Unable to stream from" << aDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
    }

    return lStream;
}


void GEVCamera::configureStream(PvDevice* aDevice, PvStream* aStream)
{
    PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV*>(aDevice);
    if(lDeviceGEV != nullptr) {
        PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV*>(aStream);
        lDeviceGEV->NegotiatePacketSize();
        lDeviceGEV->SetStreamDestination(lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort());
    }
}

void GEVCamera::createStreamBuffers(PvDevice* aDevice, PvStream *aStream, BufferList* aBufferList)
{
    uint32_t lSize = aDevice->GetPayloadSize();

    uint32_t lBufferCount = (aStream->GetQueuedBufferMaximum() < BUFFER_COUNT) ? 
                                aStream->GetQueuedBufferMaximum() : BUFFER_COUNT;
    
    for(uint32_t i=0; i<lBufferCount; i++) 
    {
        PvBuffer *lBuffer = new PvBuffer;
        lBuffer->Alloc(static_cast<uint32_t>(lSize));
        aBufferList->push_back(lBuffer);
    }

    BufferList::iterator lIt = aBufferList->begin();
    while(lIt != aBufferList->end())
    {
        aStream->QueueBuffer(*lIt);
        lIt++;
    }
}

cv::Mat1b GEVCamera::getImage()
{
    std::unique_lock<std::mutex> lock(this->imgReadyMutex);
    imgReadyConditionVariable.wait(lock);
    return cv::Mat1b(imgHeight, imgWidth, imgBufferPtr);
}

void GEVCamera::getRawData(uint32_t& width, uint32_t& height, uint8_t*& bufferPtr)
{
    std::unique_lock<std::mutex> lock(this->imgReadyMutex);
    imgReadyConditionVariable.wait(lock);

    width = imgWidth;
    height = imgHeight;
    bufferPtr = imgBufferPtr;
}

void GEVCamera::acquireImages(PvDevice* aDevice, PvStream* aStream)
{
    PvGenParameterArray *lDeviceParams = aDevice->GetParameters();
    PvGenCommand *lStart = dynamic_cast<PvGenCommand*>(lDeviceParams->Get("AcquisitionStart"));
    
    PvGenParameterArray *lStreamParams = aStream->GetParameters();
    PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat*>(lStreamParams->Get("AcquisitionRate"));
    PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat*>(lStreamParams->Get("Bandwidth"));

    cout << "Enabling streaming and sending AcquisitionStart command." << endl;
    aDevice->StreamEnable();
    lStart->Execute();

    double lFrameRateVal = 0.0;
    double lBandwidthVal = 0.0;

    while(this->running)
    {
        PvBuffer *lBuffer = nullptr;
        PvResult lOperationResult;

        PvResult lResult = aStream->RetrieveBuffer(&lBuffer, &lOperationResult, 1000);
        if(lResult.IsOK()) {
            if(lOperationResult.IsOK()) {
                PvPayloadType lType;

                lFrameRate->GetValue(lFrameRateVal);
                lBandwidth->GetValue(lBandwidthVal);

                lType = lBuffer->GetPayloadType();

                if(lType == PvPayloadTypeImage) {
                    PvImage *lImage = lBuffer->GetImage();
                    imgWidth = lImage->GetWidth();
                    imgHeight = lImage->GetHeight();
                    
                    uint32_t imgSize = imgHeight * imgWidth;
                    if(imgBufferPtr == nullptr) {
                        imgBufferPtr = (uint8_t*)malloc(imgSize);
                    }

                    memcpy(imgBufferPtr, lImage->GetDataPointer(), imgSize);
                    imgReadyConditionVariable.notify_one();
                }
                else {
                    cout << "(buffer does not contain image)" << endl;
                }
            }

            aStream->QueueBuffer(lBuffer);
        }
        else {
            cout << "GEVCamera::acquireImages::error: " << lResult.GetCodeString().GetAscii() << endl;
        }
    }

}

void GEVCamera::disconnect()
{
    running = false;
    threadPtr.get()->join();

    PvGenParameterArray *lDeviceParams = device->GetParameters();
    PvGenCommand *lStop = dynamic_cast<PvGenCommand*>(lDeviceParams->Get("AcquisitionStop"));

    cout << "Sending AcquitionStop command to the device" << endl;
    lStop->Execute();

    cout << "Disable streaming on the controller" << endl;
    device->StreamDisable();

    cout << "Aborting buffers still in stream" << endl;
    stream->AbortQueuedBuffers();

    while(stream->GetQueuedBufferCount() > 0)
    {
        PvBuffer *lBuffer = nullptr;
        PvResult lOperationResult;

        stream->RetrieveBuffer(&lBuffer, &lOperationResult);
    }

    threadPtr.reset();

    if(imgBufferPtr != nullptr) free(imgBufferPtr); imgBufferPtr = nullptr;
}


