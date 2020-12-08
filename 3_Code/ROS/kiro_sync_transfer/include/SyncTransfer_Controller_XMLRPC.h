
#ifndef _SYNC_TRANSFER_CONTROLLER_XMLRPC_H_89329789041327892373771234718432132
#define _SYNC_TRANSFER_CONTROLLER_XMLRPC_H_89329789041327892373771234718432132

#include <mutex>
#include <memory>
#include <thread>
#include <atomic>
#include <functional>

#include <xmlrpc-c/base.h>
#include <xmlrpc-c/server.h>
#include <xmlrpc-c/server_abyss.h>

#include "CommonStruct.h"
#include "CommonConfig.h"

class KIROSyncTransferControllerXMLRPC
{
    public:
        KIROSyncTransferControllerXMLRPC();
        ~KIROSyncTransferControllerXMLRPC();

    public:
        int Initialize();
        int Finalize();
        
        int StartThreadLoop();
        int StopThreadLoop();
        bool isThreadRunning() { return threadProc_KIROSyncTransferControllerXMLRPC_RunFlag; }

	///// XMLRPC Server
	public:
		void InitXMLRPCServer();
		void FiniXMLRPCServer();

		std::string GetXMLRPCServer_Address() { return dataSync_Address; }
		int GetXMLRPCServer_Port() { return dataSync_Port; }

		void SetXMLRPCServer_Address(std::string addr) { dataSync_Address = addr; }
		void SetXMLRPCServer_Port(int port) { dataSync_Port = port; }

    ///// ROS function handle
    private:
        std::function<bool(void)> fhROSInitialize;
        std::function<bool(void)> fhROSFinalize;
        std::function<bool(void)> fhROSRun;
        std::function<bool(void)> fhROSStop;

    public:
        void SetFhROSInitialize(std::function<bool(void)> func) { fhROSInitialize = std::move(func); }
        void SetFhROSFinalize(std::function<bool(void)> func) { fhROSFinalize = std::move(func); }
        void SetFhROSRun(std::function<bool(void)> func) { fhROSRun = std::move(func); }
        void SetFhROSStop(std::function<bool(void)> func) { fhROSStop = std::move(func); }

	private:
        mutable std::mutex _mtx;

		std::string dataSync_Address;
		int dataSync_Port;

		xmlrpc_env env;
		xmlrpc_server_abyss_t* _xmlrpc_serverhandle;

		std::unique_ptr<std::thread> _thKIROSyncTransferControllerXMLRPC;
		std::atomic<bool> threadProc_KIROSyncTransferControllerXMLRPC_RunFlag;
		static void ThreadProc_KIROSyncTransferControllerXMLRPC(void *arg);

    private:
        static xmlrpc_value* RequestHandler_PullData(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo);
        static xmlrpc_value* RequestHandler_ROSInitialize(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo);
        static xmlrpc_value* RequestHandler_ROSFinalize(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo);
        static xmlrpc_value* RequestHandler_ROSStartThreadLoop(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo);
        static xmlrpc_value* RequestHandler_ROSStopThreadLoop(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo);

};

#endif  


