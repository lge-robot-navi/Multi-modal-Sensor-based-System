#ifndef _MEMSYNC_H_9039803289472343248794241234879
#define _MEMSYNC_H_9039803289472343248794241234879

#define	MEMSYNC_API	extern "C"

enum MEMSYNC_RetState
{
	MEMSYNC_SUCCESS,
	MEMSYNC_ARG_NULL,
	MEMSYNC_ARG_BAD,
	MEMSYNC_MUTEX_LOCKED,
	MEMSYNC_CALL_FAIL,
	MEMSYNC_ERROR
};

MEMSYNC_API void* MemSync_GetHandle(void);
MEMSYNC_API bool  MemSync_ValidateHandle(void* pHandle);
MEMSYNC_API void  MemSync_ReleaseHandle(void* pHandle);

MEMSYNC_API int MemSync_SetServerInfo(void* pHandle, const char* ip_address, int port_number);
MEMSYNC_API int MemSync_SetRemoteServerInfo(void* pHandle, const char* remote_ip_address, int remote_port_number);
MEMSYNC_API int MemSync_SetGroupID(void* pHandle, const char* group_id);
MEMSYNC_API int MemSync_SetID(void* pHandle, const char* id);

MEMSYNC_API int MemSync_Write(void* pHandle, const char* data, unsigned long data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_WriteWithID(void* pHandle, const char* id, const char* data, unsigned long data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_WriteWithGroupID(void* pHandle, const char* group_id, const char* id, const char* data, unsigned long data_length, unsigned long& timestamp);

MEMSYNC_API int MemSync_WriteRemote(void* pHandle, const char* data, unsigned long data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_WriteWithIDRemote(void* pHandle, const char* id, const char* data, unsigned long data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_WriteWithGroupIDRemote(void* pHandle, const char* group_id, const char* id, const char* data, unsigned long data_length, unsigned long& timestamp);

MEMSYNC_API int MemSync_WriteBlocksRemote(void* pHandle, const char* data, unsigned long data_length, unsigned long block_size, unsigned long& timestamp);
MEMSYNC_API int MemSync_WriteBlocksWithIDRemote(void* pHandle, const char* id, const char* data, unsigned long data_length, unsigned long block_size, unsigned long& timestamp);
MEMSYNC_API int MemSync_WriteBlocksWithGroupIDRemote(void* pHandle, const char* group_id, const char* id, const char* data, unsigned long data_length, unsigned long block_size, unsigned long& timestamp);

MEMSYNC_API int MemSync_Read(void* pHandle, char* data, unsigned long& data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_ReadWithID(void* pHandle, const char* id, char* data, unsigned long& data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_ReadWithGroupID(void* pHandle, const char* group_id, const char* id, char* data, unsigned long& data_length, unsigned long& timestamp);

MEMSYNC_API int MemSync_ReadRemote(void* pHandle, char* data, unsigned long& data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_ReadWithIDRemote(void* pHandle, const char* id, char* data, unsigned long& data_length, unsigned long& timestamp);
MEMSYNC_API int MemSync_ReadWithGroupIDRemote(void* pHandle, const char* group_id, const char* id, char* data, unsigned long& data_length, unsigned long& timestamp);

MEMSYNC_API int MemSync_Replace(void* pHandle, const char* data, unsigned long data_length, unsigned long& timestamp);	// TBD

MEMSYNC_API int MemSync_Add(void* pHandle, const char* data, unsigned long data_length, unsigned long& timestamp);	// TBD

MEMSYNC_API int MemSync_Delete(void* pHandle);
MEMSYNC_API int MemSync_DeleteWithID(void* pHandle, const char* id);
MEMSYNC_API int MemSync_DeleteWithGroupID(void* pHandle, const char* group_id, const char* id);

MEMSYNC_API int MemSync_DeleteRemote(void* pHandle);
MEMSYNC_API int MemSync_DeleteWithIDRemote(void* pHandle, const char* id);
MEMSYNC_API int MemSync_DeleteWithGroupIDRemote(void* pHandle, const char* group_id, const char* id);


// update related
MEMSYNC_API int MemSync_Replication(void* pHandle_Src, void* pHandle_Dst);

// utilities
MEMSYNC_API unsigned long MemSync_CurrentTimestamp(void);



#endif

