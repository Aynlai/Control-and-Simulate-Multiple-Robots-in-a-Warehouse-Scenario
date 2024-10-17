#ifndef robotController_cap_host_h__
#define robotController_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap_simtarget.h"
typedef struct { rtwCAPI_ModelMappingInfo mmi ; }
robotController_host_DataMapInfo_T ;
#ifdef __cplusplus
extern "C" {
#endif
void robotController_host_InitializeDataMapInfo (
robotController_host_DataMapInfo_T * dataMap , const char * path ) ;
#ifdef __cplusplus
}
#endif
#endif
#endif
