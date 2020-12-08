
#ifndef _SYNC_TRANSFER_H_8032789423794872712786748127138912
#define _SYNC_TRANSFER_H_8032789423794872712786748127138912

#include <memory>

#include "SyncTransfer_Model.h"
#include "SyncTransfer_Controller_ROS.h"
#include "SyncTransfer_Controller_XMLRPC.h"

class KIROSyncTransfer
{
    // singleton pattern
    public:
        static KIROSyncTransfer& getInstance()
        {
            static KIROSyncTransfer instance;
            return instance;
        }

        static const std::unique_ptr<KIROSyncTransferModel>& getModel()
        {
            return (KIROSyncTransfer::getInstance().uptrKIROSyncTransferModel);
        }

        static const std::unique_ptr<KIROSyncTransferControllerROS>& getControllerROS()
        {
            return (KIROSyncTransfer::getInstance().uptrKIROSyncTransferControllerROS);
        }

        static const std::unique_ptr<KIROSyncTransferControllerXMLRPC>& getControllerXMLRPC()
        {
            return (KIROSyncTransfer::getInstance().uptrKIROSyncTransferControllerXMLRPC);
        }

        KIROSyncTransfer(KIROSyncTransfer const&) = delete;
        KIROSyncTransfer(KIROSyncTransfer &&) = delete;
        KIROSyncTransfer& operator=(KIROSyncTransfer const&) = delete;
        KIROSyncTransfer& operator=(KIROSyncTransfer &&) = delete;

    private:
        KIROSyncTransfer()
        {
            uptrKIROSyncTransferModel = std::unique_ptr<KIROSyncTransferModel>(new KIROSyncTransferModel());
            uptrKIROSyncTransferControllerROS = std::unique_ptr<KIROSyncTransferControllerROS>(new KIROSyncTransferControllerROS());
            uptrKIROSyncTransferControllerXMLRPC = std::unique_ptr<KIROSyncTransferControllerXMLRPC>(new KIROSyncTransferControllerXMLRPC());
        }

        ~KIROSyncTransfer()
        {
            if(uptrKIROSyncTransferControllerXMLRPC.get() != nullptr) uptrKIROSyncTransferControllerXMLRPC.reset();
            if(uptrKIROSyncTransferControllerROS.get() != nullptr) uptrKIROSyncTransferControllerROS.reset();
            if(uptrKIROSyncTransferModel.get() != nullptr) uptrKIROSyncTransferModel.reset();
        }

    private:
        std::unique_ptr<KIROSyncTransferModel> uptrKIROSyncTransferModel;
        std::unique_ptr<KIROSyncTransferControllerROS> uptrKIROSyncTransferControllerROS;
        std::unique_ptr<KIROSyncTransferControllerXMLRPC> uptrKIROSyncTransferControllerXMLRPC;

};

#endif

