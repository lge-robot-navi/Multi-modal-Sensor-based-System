
#include <cstring>
#include <chrono>

#include "SyncTransfer.h"
#include "SyncTransfer_Controller_XMLRPC.h"

KIROSyncTransferControllerXMLRPC::KIROSyncTransferControllerXMLRPC()
{
    threadProc_KIROSyncTransferControllerXMLRPC_RunFlag = false;

	fhROSInitialize = nullptr;
	fhROSFinalize = nullptr;
	fhROSRun = nullptr;
	fhROSStop = nullptr;

    Initialize();

    StartThreadLoop();  // 외부 인터페이스 제공을 위해 자체적으로 스레드 실행됨
}

KIROSyncTransferControllerXMLRPC::~KIROSyncTransferControllerXMLRPC()
{
    Finalize();
}

int KIROSyncTransferControllerXMLRPC::Initialize()
{
	if(threadProc_KIROSyncTransferControllerXMLRPC_RunFlag != false) {
        StopThreadLoop();
        threadProc_KIROSyncTransferControllerXMLRPC_RunFlag = false;
    }

	fprintf(stderr, "KIROSyncTransferControllerXMLRPC::Initialize: initialize\n");

    return 0;
}

int KIROSyncTransferControllerXMLRPC::Finalize()
{
    if(threadProc_KIROSyncTransferControllerXMLRPC_RunFlag != false) {
        StopThreadLoop();
    }

	fprintf(stderr, "KIROSyncTransferControllerXMLRPC::Finalize: finalize\n");

    return 0;
}

int KIROSyncTransferControllerXMLRPC::StartThreadLoop()
{
    std::lock_guard<std::mutex> lock(_mtx);

    InitXMLRPCServer();

    if(threadProc_KIROSyncTransferControllerXMLRPC_RunFlag == false) {
    	threadProc_KIROSyncTransferControllerXMLRPC_RunFlag = true;
	    _thKIROSyncTransferControllerXMLRPC = std::unique_ptr<std::thread>(new std::thread(ThreadProc_KIROSyncTransferControllerXMLRPC, this));
    }
	std::this_thread::sleep_for(std::chrono::milliseconds(20));

	fprintf(stderr, "KIROSyncTransferControllerXMLRPC::StartThreadLoop\n");

    return 0;
}

int KIROSyncTransferControllerXMLRPC::StopThreadLoop()
{
    std::lock_guard<std::mutex> lock(_mtx);

    FiniXMLRPCServer();

	if(threadProc_KIROSyncTransferControllerXMLRPC_RunFlag != false) {
		threadProc_KIROSyncTransferControllerXMLRPC_RunFlag = false;
		
		if(_thKIROSyncTransferControllerXMLRPC.get() != nullptr && _thKIROSyncTransferControllerXMLRPC->joinable() == true) _thKIROSyncTransferControllerXMLRPC->join();
		if(_thKIROSyncTransferControllerXMLRPC.get() != nullptr) _thKIROSyncTransferControllerXMLRPC.reset(); _thKIROSyncTransferControllerXMLRPC = nullptr;
	}

	fprintf(stderr, "KIROSyncTransferControllerXMLRPC::StopThreadLoop\n");

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
// XMLRPC Server 
void KIROSyncTransferControllerXMLRPC::InitXMLRPCServer()
{
	SetXMLRPCServer_Address(KIRO_SYNC_TRANSFER_DATASYNC_ADDRESS);
	SetXMLRPCServer_Port(KIRO_SYNC_TRANSFER_DATASYNC_PORTNUMBER);
}

void KIROSyncTransferControllerXMLRPC::FiniXMLRPCServer()
{
    xmlrpc_server_abyss_terminate(&env, _xmlrpc_serverhandle);
}

// Request Handler 
xmlrpc_value* KIROSyncTransferControllerXMLRPC::RequestHandler_PullData(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo)
{
	char *mainCmd = nullptr, *subCmd = nullptr;
	xmlrpc_value *retVal = nullptr;

	KIROSyncTransferControllerXMLRPC *ptrKIROSyncTransferControllerXMLRPC = (KIROSyncTransferControllerXMLRPC*)serverInfo;

	xmlrpc_decompose_value(env, recv_data, "(ss)", &mainCmd, &subCmd); // array of mainCmd and subCmd
	if(env->fault_occurred) {
		fprintf(stderr, "KIROSyncTransferControllerXMLRPC::RequestHandler_PullData - xmlrpc_decompose_value failed\n");
		goto REQUESTHANDLER_PULLDATA_CLEANUP;
	}

	if(mainCmd == nullptr || subCmd == nullptr) {
		fprintf(stderr, "KIROSyncTransferControllerXMLRPC::RequestHandler_PullData - parameter is null\n");
		goto REQUESTHANDLER_PULLDATA_CLEANUP;
	}

	if(std::strncmp(mainCmd, "PullData", 8) == 0)
	{
		retVal = xmlrpc_array_new(env);

		if(strncmp(subCmd, "None", 4) == 0 || strncmp(subCmd, "Watchdog", 8) == 0)
		{
			xmlrpc_value *iVal = xmlrpc_int_new(env, 1);
			xmlrpc_array_append_item(env, retVal, iVal);
			xmlrpc_DECREF(iVal); iVal = nullptr;
		}
		else {
			xmlrpc_value *iVal = xmlrpc_int_new(env, 0);
			xmlrpc_array_append_item(env, retVal, iVal);
			xmlrpc_DECREF(iVal); iVal = nullptr;
		}
	}

REQUESTHANDLER_PULLDATA_CLEANUP:
	if(mainCmd != nullptr) free(mainCmd); mainCmd = nullptr;
	if(subCmd != nullptr) free(subCmd); subCmd = nullptr;

	return retVal;
}

xmlrpc_value* KIROSyncTransferControllerXMLRPC::RequestHandler_ROSInitialize(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo)
{
	xmlrpc_value *retVal = nullptr;
	int flag = 0, result = 0;

	KIROSyncTransferControllerXMLRPC *ptrKIROSyncTransferControllerXMLRPC = (KIROSyncTransferControllerXMLRPC*)serverInfo;

	xmlrpc_decompose_value(env, recv_data, "(i)", &flag);
	if(env->fault_occurred) {
		fprintf(stderr, "KIROSyncTransferControllerXMLRPC::RequestHandler_Initialize - xmlrpc_decompose_value failed\n");
	}
	else {
		if(flag == 1) {
            // call Controller ROS's Call_Initialize()
			result = (ptrKIROSyncTransferControllerXMLRPC != nullptr &&
					  ptrKIROSyncTransferControllerXMLRPC->fhROSInitialize != nullptr && 
					  ptrKIROSyncTransferControllerXMLRPC->fhROSInitialize() == true) ? 1 : 0;

			if(result == 0) {
				result = KIROSyncTransfer::getControllerROS()->Call_Initialize();
			}
		}
	}

	retVal = xmlrpc_array_new(env);
	xmlrpc_value *iVal = xmlrpc_int_new(env, result);
	xmlrpc_array_append_item(env, retVal, iVal);
	xmlrpc_DECREF(iVal); iVal = nullptr;

	return retVal;
}

xmlrpc_value* KIROSyncTransferControllerXMLRPC::RequestHandler_ROSStartThreadLoop(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo)
{
	xmlrpc_value *retVal = nullptr;
	int flag = 0, result = 0;

	KIROSyncTransferControllerXMLRPC *ptrKIROSyncTransferControllerXMLRPC = (KIROSyncTransferControllerXMLRPC*)serverInfo;

	xmlrpc_decompose_value(env, recv_data, "(i)", &flag);
	if(env->fault_occurred) {
		fprintf(stderr, "KIROSyncTransferControllerXMLRPC::RequestHandler_StartThreadLoop - xmlrpc_decompose_value failed\n");
	}
	else {
		if(flag == 1) {
            // call Controller ROS's Call_Run()
			result = (ptrKIROSyncTransferControllerXMLRPC != nullptr &&
					  ptrKIROSyncTransferControllerXMLRPC->fhROSRun != nullptr && 
					  ptrKIROSyncTransferControllerXMLRPC->fhROSRun() == true) ? 1 : 0;

			if(result == 0) {
				result = KIROSyncTransfer::getControllerROS()->Call_Run();
			}
		}
	}

	retVal = xmlrpc_array_new(env);
	xmlrpc_value *iVal = xmlrpc_int_new(env, result);
	xmlrpc_array_append_item(env, retVal, iVal);
	xmlrpc_DECREF(iVal); iVal = nullptr;

	return retVal;
}

xmlrpc_value* KIROSyncTransferControllerXMLRPC::RequestHandler_ROSStopThreadLoop(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo)
{
    xmlrpc_value *retVal = nullptr;
    int flag = 0, result = 0;

	KIROSyncTransferControllerXMLRPC *ptrKIROSyncTransferControllerXMLRPC = (KIROSyncTransferControllerXMLRPC*)serverInfo;

    xmlrpc_decompose_value(env, recv_data, "(i)", &flag);
    if(env->fault_occurred) {
        fprintf(stderr, "KIROSyncTransferControllerXMLRPC::RequestHandler_Stop - xmlrpc_decompose_value failed\n");
    }
    else {
        if(flag == 1) {
            // call ControllerROS's Call_Stop()
			result = (ptrKIROSyncTransferControllerXMLRPC != nullptr &&
					  ptrKIROSyncTransferControllerXMLRPC->fhROSStop != nullptr && 
					  ptrKIROSyncTransferControllerXMLRPC->fhROSStop() == true) ? 1 : 0;

			if(result == 0) {
				result = KIROSyncTransfer::getControllerROS()->Call_Stop();
			}
        }
    }

    retVal = xmlrpc_array_new(env);
    xmlrpc_value *iVal = xmlrpc_int_new(env, result);
    xmlrpc_array_append_item(env, retVal, iVal);
    xmlrpc_DECREF(iVal); iVal = nullptr;

    return retVal;
}

xmlrpc_value* KIROSyncTransferControllerXMLRPC::RequestHandler_ROSFinalize(xmlrpc_env* env, xmlrpc_value* const recv_data, void* const serverInfo, void* const callInfo)
{
	xmlrpc_value *retVal = nullptr;
	int flag = 0, result = 0;

	KIROSyncTransferControllerXMLRPC *ptrKIROSyncTransferControllerXMLRPC = (KIROSyncTransferControllerXMLRPC*)serverInfo;

	xmlrpc_decompose_value(env, recv_data, "(i)", &flag);
	if(env->fault_occurred) {
		fprintf(stderr, "KIROSyncTransferControllerXMLRPC::RequestHandler_Finalize - xmlrpc_decompose_value failed\n");
	}
	else {
		if(flag == 1) {
            // call ControllerROS's Call_Finalize()
			result = (ptrKIROSyncTransferControllerXMLRPC != nullptr &&
					  ptrKIROSyncTransferControllerXMLRPC->fhROSFinalize != nullptr && 
					  ptrKIROSyncTransferControllerXMLRPC->fhROSFinalize() == true) ? 1 : 0;

			if(result == 0) {
				result = KIROSyncTransfer::getControllerROS()->Call_Finalize();
			}
		}
	}

	retVal = xmlrpc_array_new(env);
	xmlrpc_value *iVal = xmlrpc_int_new(env, result);
	xmlrpc_array_append_item(env, retVal, iVal);
	xmlrpc_DECREF(iVal); iVal = nullptr;

	return retVal;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Threadloop
void KIROSyncTransferControllerXMLRPC::ThreadProc_KIROSyncTransferControllerXMLRPC(void *arg)
{
	KIROSyncTransferControllerXMLRPC *ptrKIROSyncTransferControllerXMLRPC = (KIROSyncTransferControllerXMLRPC*)arg;
	if(ptrKIROSyncTransferControllerXMLRPC == nullptr) { fprintf(stderr, "ThreadProc_XMLRPCServer:: arg is nullptr\n"); return; }

	// start xmlrpc server
	xmlrpc_server_abyss_parms serverparm;
	xmlrpc_registry *registryP;

	struct xmlrpc_method_info3 const methodInfo_PullData = {
		"KIRO_Camera.PullData", &RequestHandler_PullData, (void*)ptrKIROSyncTransferControllerXMLRPC, 4194304, NULL, NULL };

	struct xmlrpc_method_info3 const methodInfo_ROSInitialize = {
		"KIRO_Camera.ROSInitialize", &RequestHandler_ROSInitialize, (void*)ptrKIROSyncTransferControllerXMLRPC, };

	struct xmlrpc_method_info3 const methodInfo_ROSFinalize = {
		"KIRO_Camera.ROSFinalize", &RequestHandler_ROSFinalize, (void*)ptrKIROSyncTransferControllerXMLRPC, };

	struct xmlrpc_method_info3 const methodInfo_ROSStartThreadLoop = {
		"KIRO_Camera.ROSStartThreadLoop", &RequestHandler_ROSStartThreadLoop, (void*)ptrKIROSyncTransferControllerXMLRPC, };

	struct xmlrpc_method_info3 const methodInfo_ROSStopThreadLoop = {
		"KIRO_Camera.ROSStopThreadLoop", &RequestHandler_ROSStopThreadLoop, (void*)ptrKIROSyncTransferControllerXMLRPC, };

	xmlrpc_env_init(&(ptrKIROSyncTransferControllerXMLRPC->env));
	xmlrpc_limit_set(XMLRPC_XML_SIZE_LIMIT_ID, 4194304);	// set max. 4MByte

	registryP = xmlrpc_registry_new(&(ptrKIROSyncTransferControllerXMLRPC->env));

	xmlrpc_registry_add_method3(&(ptrKIROSyncTransferControllerXMLRPC->env), registryP, &methodInfo_PullData);
	xmlrpc_registry_add_method3(&(ptrKIROSyncTransferControllerXMLRPC->env), registryP, &methodInfo_ROSInitialize);
	xmlrpc_registry_add_method3(&(ptrKIROSyncTransferControllerXMLRPC->env), registryP, &methodInfo_ROSStartThreadLoop);
	xmlrpc_registry_add_method3(&(ptrKIROSyncTransferControllerXMLRPC->env), registryP, &methodInfo_ROSStopThreadLoop);
	xmlrpc_registry_add_method3(&(ptrKIROSyncTransferControllerXMLRPC->env), registryP, &methodInfo_ROSFinalize);

	serverparm.config_file_name = NULL;
	serverparm.registryP = registryP;
	serverparm.port_number = (ptrKIROSyncTransferControllerXMLRPC->GetXMLRPCServer_Port());
	serverparm.log_file_name = "xmlrpclog_KIRO_SyncTransfer";
	serverparm.keepalive_max_conn = 0;
	serverparm.keepalive_timeout = 0;
	serverparm.timeout = 0;
	serverparm.dont_advertise = false;
	serverparm.socket_bound = true;

	xmlrpc_server_abyss_global_init(&(ptrKIROSyncTransferControllerXMLRPC->env));
	xmlrpc_server_abyss_create(&(ptrKIROSyncTransferControllerXMLRPC->env), &serverparm, XMLRPC_APSIZE(log_file_name), 
								&(ptrKIROSyncTransferControllerXMLRPC->_xmlrpc_serverhandle));

	fprintf(stderr, "KIROSyncTransferControllerXMLRPC:: XMLRPC Server Start...\n");

	xmlrpc_server_abyss_run_server(&(ptrKIROSyncTransferControllerXMLRPC->env), (ptrKIROSyncTransferControllerXMLRPC->_xmlrpc_serverhandle));

	fprintf(stderr, "KIROSyncTransferControllerXMLRPC:: XMLRPC Server Stop...\n");

	xmlrpc_server_abyss_destroy((ptrKIROSyncTransferControllerXMLRPC->_xmlrpc_serverhandle));
	xmlrpc_server_abyss_global_term();

	if(ptrKIROSyncTransferControllerXMLRPC != nullptr) ptrKIROSyncTransferControllerXMLRPC->threadProc_KIROSyncTransferControllerXMLRPC_RunFlag = false;
	return;

}


