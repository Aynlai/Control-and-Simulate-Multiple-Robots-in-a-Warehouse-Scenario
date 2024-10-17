#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "robotController_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 21
#endif
#ifndef SS_INT64
#define SS_INT64 22
#endif
#else
#include "builtin_typeid_types.h"
#include "robotController.h"
#include "robotController_capi.h"
#include "robotController_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static rtwCAPI_States rtBlockStates [ ] = { { 0 , - 1
, TARGET_STRING ( "robotController/Unit Delay" ) , TARGET_STRING ( "DSTATE" )
, "" , 0 , 0 , 0 , 0 , 0 , 0 , - 1 , 0 } , { 0 , - 1 , ( NULL ) , ( NULL ) ,
( NULL ) , 0 , 0 , 0 , 0 , 0 , 0 , - 1 , 0 } } ; static int_T
rt_LoggedStateIdxList [ ] = { 7 } ;
#ifndef HOST_CAPI_BUILD
static void robotController_InitializeDataAddr ( void * dataAddr [ ] ,
lwnhcfk22o * localDW ) { dataAddr [ 0 ] = ( void * ) ( & localDW ->
avbny3pgeq ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void robotController_InitializeVarDimsAddr ( int32_T * vardimsAddr [ ]
) { vardimsAddr [ 0 ] = ( NULL ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void robotController_InitializeLoggingFunctions ( RTWLoggingFcnPtr
loggingPtrs [ ] ) { loggingPtrs [ 0 ] = ( NULL ) ; }
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { {
"unsigned char" , "boolean_T" , 0 , 0 , sizeof ( boolean_T ) , ( uint8_T )
SS_BOOLEAN , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } } ; static uint_T rtDimensionArray [ ] = { 1 , 1
} ; static const real_T rtcapiStoredFloats [ ] = { 0.5 , 0.0 } ; static
rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , (
int8_T ) 0 , ( uint8_T ) 0 } } ; static int_T rtContextSystems [ 5 ] ; static
rtwCAPI_LoggingMetaInfo loggingMetaInfo [ ] = { { 0 , 0 , "" , 0 } } ; static
rtwCAPI_ModelMapLoggingStaticInfo mmiStaticInfoLogging = { 5 ,
rtContextSystems , loggingMetaInfo , 0 , ( NULL ) , { 0 , ( NULL ) , ( NULL )
} , 0 , ( NULL ) } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { {
rtBlockSignals , 0 , ( NULL ) , 0 , ( NULL ) , 0 } , { ( NULL ) , 0 , ( NULL
) , 0 } , { rtBlockStates , 1 } , { rtDataTypeMap , rtDimensionMap ,
rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" ,
{ 2916236916U , 183438928U , 3100783094U , 512884005U } , &
mmiStaticInfoLogging , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const
rtwCAPI_ModelMappingStaticInfo * robotController_GetCAPIStaticMap ( void ) {
return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
static void robotController_InitializeSystemRan ( m1mtbmkkvs * const
bzf3tuqsn5 , sysRanDType * systemRan [ ] , lwnhcfk22o * localDW , int_T
systemTid [ ] , void * rootSysRanPtr , int rootTid ) { UNUSED_PARAMETER (
bzf3tuqsn5 ) ; UNUSED_PARAMETER ( localDW ) ; systemRan [ 0 ] = ( sysRanDType
* ) rootSysRanPtr ; systemRan [ 1 ] = ( NULL ) ; systemRan [ 2 ] = ( NULL ) ;
systemRan [ 3 ] = ( NULL ) ; systemRan [ 4 ] = ( NULL ) ; systemTid [ 1 ] =
bzf3tuqsn5 -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 2 ] = bzf3tuqsn5
-> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 3 ] = bzf3tuqsn5 -> Timing .
mdlref_GlobalTID [ 0 ] ; systemTid [ 4 ] = bzf3tuqsn5 -> Timing .
mdlref_GlobalTID [ 0 ] ; systemTid [ 0 ] = rootTid ; rtContextSystems [ 0 ] =
0 ; rtContextSystems [ 1 ] = 0 ; rtContextSystems [ 2 ] = 0 ;
rtContextSystems [ 3 ] = 0 ; rtContextSystems [ 4 ] = 0 ; }
#endif
#ifndef HOST_CAPI_BUILD
void robotController_InitializeDataMapInfo ( m1mtbmkkvs * const bzf3tuqsn5 ,
lwnhcfk22o * localDW , void * sysRanPtr , int contextTid ) {
rtwCAPI_SetVersion ( bzf3tuqsn5 -> DataMapInfo . mmi , 1 ) ;
rtwCAPI_SetStaticMap ( bzf3tuqsn5 -> DataMapInfo . mmi , & mmiStatic ) ;
rtwCAPI_SetLoggingStaticMap ( bzf3tuqsn5 -> DataMapInfo . mmi , &
mmiStaticInfoLogging ) ; robotController_InitializeDataAddr ( bzf3tuqsn5 ->
DataMapInfo . dataAddress , localDW ) ; rtwCAPI_SetDataAddressMap (
bzf3tuqsn5 -> DataMapInfo . mmi , bzf3tuqsn5 -> DataMapInfo . dataAddress ) ;
robotController_InitializeVarDimsAddr ( bzf3tuqsn5 -> DataMapInfo .
vardimsAddress ) ; rtwCAPI_SetVarDimsAddressMap ( bzf3tuqsn5 -> DataMapInfo .
mmi , bzf3tuqsn5 -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetPath (
bzf3tuqsn5 -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetFullPath (
bzf3tuqsn5 -> DataMapInfo . mmi , ( NULL ) ) ;
robotController_InitializeLoggingFunctions ( bzf3tuqsn5 -> DataMapInfo .
loggingPtrs ) ; rtwCAPI_SetLoggingPtrs ( bzf3tuqsn5 -> DataMapInfo . mmi ,
bzf3tuqsn5 -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetInstanceLoggingInfo (
bzf3tuqsn5 -> DataMapInfo . mmi , & bzf3tuqsn5 -> DataMapInfo .
mmiLogInstanceInfo ) ; rtwCAPI_SetChildMMIArray ( bzf3tuqsn5 -> DataMapInfo .
mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArrayLen ( bzf3tuqsn5 -> DataMapInfo .
mmi , 0 ) ; robotController_InitializeSystemRan ( bzf3tuqsn5 , bzf3tuqsn5 ->
DataMapInfo . systemRan , localDW , bzf3tuqsn5 -> DataMapInfo . systemTid ,
sysRanPtr , contextTid ) ; rtwCAPI_SetSystemRan ( bzf3tuqsn5 -> DataMapInfo .
mmi , bzf3tuqsn5 -> DataMapInfo . systemRan ) ; rtwCAPI_SetSystemTid (
bzf3tuqsn5 -> DataMapInfo . mmi , bzf3tuqsn5 -> DataMapInfo . systemTid ) ;
rtwCAPI_SetGlobalTIDMap ( bzf3tuqsn5 -> DataMapInfo . mmi , & bzf3tuqsn5 ->
Timing . mdlref_GlobalTID [ 0 ] ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void robotController_host_InitializeDataMapInfo (
robotController_host_DataMapInfo_T * dataMap , const char * path ) {
rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap ->
mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetPath ( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap ->
mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
