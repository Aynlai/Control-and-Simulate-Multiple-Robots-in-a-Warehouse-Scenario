#ifndef robotController_h_
#define robotController_h_
#ifndef robotController_COMMON_INCLUDES_
#define robotController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "sf_runtime/sfc_sdi.h"
#endif
#include "robotController_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rtw_modelmap_simtarget.h"
#include <string.h>
#include <stddef.h>
typedef struct { real_T a1ty03ixk2 ; real_T ocoypaore1 [ 2 ] ; real_T
aix3fm0ysj [ 2 ] ; RobotState p55quualew ; boolean_T cbioefby2x ; }
dheucvhz3u ; typedef struct { ipw3ftlyif ljuqtcenlo ; real_T i2zecotakp ;
real_T o0yda5zduz [ 200 ] ; real_T aapcuge3jl [ 2 ] ; real_T johlz4ktqs [ 2 ]
; uint32_T fyhxlfl0zl ; uint32_T pggahx0edn [ 625 ] ; boolean_T avbny3pgeq ;
uint8_T c0setyxykh ; uint8_T h032gcefu3 ; boolean_T ey3alnk1or ; boolean_T
ftjxsixfdm ; boolean_T ls4ddqxq54 ; boolean_T oafkuis4gg ; } lwnhcfk22o ;
struct i4xuulnjq50_ { real_T P_4 ; real_T P_5 ; real_T P_6 ; boolean_T P_7 ;
} ; struct k3jhwrd30z { struct SimStruct_tag * _mdlRefSfcnS ; struct {
rtwCAPI_ModelMappingInfo mmi ; rtwCAPI_ModelMapLoggingInstanceInfo
mmiLogInstanceInfo ; void * dataAddress [ 1 ] ; int32_T * vardimsAddress [ 1
] ; RTWLoggingFcnPtr loggingPtrs [ 1 ] ; sysRanDType * systemRan [ 5 ] ;
int_T systemTid [ 5 ] ; } DataMapInfo ; struct { int_T mdlref_GlobalTID [ 2 ]
; } Timing ; } ; typedef struct { dheucvhz3u rtb ; lwnhcfk22o rtdw ;
m1mtbmkkvs rtm ; } dx4drjuueew ; extern real_T rtP_awayFromGoalThresh ;
extern real_T rtP_loadingStation [ 2 ] ; extern real_T rtP_unloadingStations
[ 6 ] ; extern boolean_T rtP_logicalMap [ 3600 ] ; extern void f4xdwczleo (
SimStruct * _mdlRefSfcnS , int_T mdlref_TID0 , int_T mdlref_TID1 , m1mtbmkkvs
* const bzf3tuqsn5 , dheucvhz3u * localB , lwnhcfk22o * localDW , void *
sysRanPtr , int contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI , const
char_T * rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) ; extern
void mr_robotController_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T *
modelName , int_T * retVal ) ; extern mxArray * mr_robotController_GetDWork (
const dx4drjuueew * mdlrefDW ) ; extern void mr_robotController_SetDWork (
dx4drjuueew * mdlrefDW , const mxArray * ssDW ) ; extern void
mr_robotController_RegisterSimStateChecksum ( SimStruct * S ) ; extern
mxArray * mr_robotController_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * robotController_GetCAPIStaticMap ( void ) ;
extern void a3uz1fjcva ( RobotPackageStatus * ab3l4ewxjl , dheucvhz3u *
localB , lwnhcfk22o * localDW ) ; extern void mj0lr24tvd ( dheucvhz3u *
localB , lwnhcfk22o * localDW ) ; extern void fh441y4mj5 ( lwnhcfk22o *
localDW ) ; extern void bq3nks32bt ( dheucvhz3u * localB , lwnhcfk22o *
localDW ) ; extern void robotController ( const RobotDeliverCommand *
irwasht4mi , const real_T m2qskxe3uq [ 3 ] , real_T hy2oo0lvw3 [ 2 ] ,
RobotPackageStatus * ab3l4ewxjl , real_T kbkkhgksqy [ 200 ] , dheucvhz3u *
localB , lwnhcfk22o * localDW ) ; extern void pedophjfqw ( m1mtbmkkvs * const
bzf3tuqsn5 ) ;
#endif
