#ifndef robotController_types_h_
#define robotController_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_RobotDeliverCommand_
#define DEFINED_TYPEDEF_FOR_RobotDeliverCommand_
typedef struct { real_T packageId ; real_T package ; real_T givenPackage ;
real_T active ; } RobotDeliverCommand ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_RobotState_
#define DEFINED_TYPEDEF_FOR_RobotState_
typedef enum { RobotState_AtDock = 1 , RobotState_AtLoadingStn ,
RobotState_AtUnloadingStn } RobotState ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_RobotPackageStatus_
#define DEFINED_TYPEDEF_FOR_RobotPackageStatus_
typedef struct { real_T hasPackage ; RobotState planningState ; uint8_T
sl_padding0 [ 4 ] ; } RobotPackageStatus ;
#endif
#ifndef struct_tag_ZDN0pcTMm94iYYIBOxR1i
#define struct_tag_ZDN0pcTMm94iYYIBOxR1i
struct tag_ZDN0pcTMm94iYYIBOxR1i { int32_T isInitialized ; } ;
#endif
#ifndef typedef_l0ipmq05ws
#define typedef_l0ipmq05ws
typedef struct tag_ZDN0pcTMm94iYYIBOxR1i l0ipmq05ws ;
#endif
#ifndef struct_tag_OotASTM6Tsr1B9u7XCFVQC
#define struct_tag_OotASTM6Tsr1B9u7XCFVQC
struct tag_OotASTM6Tsr1B9u7XCFVQC { real_T GridOriginInLocal [ 2 ] ; real_T
LocalOriginInWorld [ 2 ] ; real_T LocalOriginInWorldInternal [ 2 ] ; } ;
#endif
#ifndef typedef_ii4nhp3skj
#define typedef_ii4nhp3skj
typedef struct tag_OotASTM6Tsr1B9u7XCFVQC ii4nhp3skj ;
#endif
#ifndef struct_tag_mBSc2NEPGaJg9QOy7ETp9
#define struct_tag_mBSc2NEPGaJg9QOy7ETp9
struct tag_mBSc2NEPGaJg9QOy7ETp9 { real_T Head [ 2 ] ; real_T NewRegions [ 4
] ; boolean_T DropEntireMap ; boolean_T DropTwoRegions [ 2 ] ; } ;
#endif
#ifndef typedef_kg3nyqw0nj
#define typedef_kg3nyqw0nj
typedef struct tag_mBSc2NEPGaJg9QOy7ETp9 kg3nyqw0nj ;
#endif
#ifndef struct_tag_z08SEh4XC57E5zUaRGkSL
#define struct_tag_z08SEh4XC57E5zUaRGkSL
struct tag_z08SEh4XC57E5zUaRGkSL { boolean_T ConstVal ; kg3nyqw0nj * Index ;
boolean_T Buffer [ 3600 ] ; } ;
#endif
#ifndef typedef_kcmtw0dk4c
#define typedef_kcmtw0dk4c
typedef struct tag_z08SEh4XC57E5zUaRGkSL kcmtw0dk4c ;
#endif
#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE
struct tag_BlgwLpgj2bjudmbmVKWwDE { uint32_T f1 [ 8 ] ; } ;
#endif
#ifndef typedef_gbqfevksrb
#define typedef_gbqfevksrb
typedef struct tag_BlgwLpgj2bjudmbmVKWwDE gbqfevksrb ;
#endif
#ifndef struct_tag_PROpmHpYOUzrgB7XEY7gMD
#define struct_tag_PROpmHpYOUzrgB7XEY7gMD
struct tag_PROpmHpYOUzrgB7XEY7gMD { int32_T isInitialized ; boolean_T
TunablePropsChanged ; gbqfevksrb inputVarSize [ 2 ] ; real_T
MaxAngularVelocity ; real_T LookaheadDistance ; real_T DesiredLinearVelocity
; real_T ProjectionPoint [ 2 ] ; real_T ProjectionLineIndex ; real_T
LookaheadPoint [ 2 ] ; real_T LastPose [ 3 ] ; real_T WaypointsInternal [ 200
] ; } ;
#endif
#ifndef typedef_ipw3ftlyif
#define typedef_ipw3ftlyif
typedef struct tag_PROpmHpYOUzrgB7XEY7gMD ipw3ftlyif ;
#endif
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T { real_T * data ; int32_T * size ; int32_T
allocatedSize ; int32_T numDimensions ; boolean_T canFreeData ; } ;
#endif
#ifndef typedef_k0shasdzrq
#define typedef_k0shasdzrq
typedef struct emxArray_real_T k0shasdzrq ;
#endif
#ifndef struct_tag_li1QfKvBeHrXnPByd9qO1D
#define struct_tag_li1QfKvBeHrXnPByd9qO1D
struct tag_li1QfKvBeHrXnPByd9qO1D { k0shasdzrq * AdjacencyMatrix ; boolean_T
IsEuclidean ; k0shasdzrq * Nodes ; real_T CurrentLabel ; k0shasdzrq *
NodeLabels ; k0shasdzrq * NodeLabelSet ; real_T NumComponents ; } ;
#endif
#ifndef typedef_oxn0khgesl
#define typedef_oxn0khgesl
typedef struct tag_li1QfKvBeHrXnPByd9qO1D oxn0khgesl ;
#endif
#ifndef struct_tag_rVkQr4wHadiMDxOnyvWqXH
#define struct_tag_rVkQr4wHadiMDxOnyvWqXH
struct tag_rVkQr4wHadiMDxOnyvWqXH { int32_T isInitialized ; oxn0khgesl
Roadmap ; } ;
#endif
#ifndef typedef_njffc2stdn
#define typedef_njffc2stdn
typedef struct tag_rVkQr4wHadiMDxOnyvWqXH njffc2stdn ;
#endif
#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T { boolean_T * data ; int32_T * size ; int32_T
allocatedSize ; int32_T numDimensions ; boolean_T canFreeData ; } ;
#endif
#ifndef typedef_b4axleo1v0
#define typedef_b4axleo1v0
typedef struct emxArray_boolean_T b4axleo1v0 ;
#endif
#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T { int32_T * data ; int32_T * size ; int32_T
allocatedSize ; int32_T numDimensions ; boolean_T canFreeData ; } ;
#endif
#ifndef typedef_dxycgom5nk
#define typedef_dxycgom5nk
typedef struct emxArray_int32_T dxycgom5nk ;
#endif
#ifndef struct_tag_4rGpRLkDGiE84gmaKIauPG
#define struct_tag_4rGpRLkDGiE84gmaKIauPG
struct tag_4rGpRLkDGiE84gmaKIauPG { ii4nhp3skj SharedProperties ; kcmtw0dk4c
* Buffer ; boolean_T DefaultValueInternal ; boolean_T HasParent ; } ;
#endif
#ifndef typedef_pwx2zp040p
#define typedef_pwx2zp040p
typedef struct tag_4rGpRLkDGiE84gmaKIauPG pwx2zp040p ;
#endif
#ifndef struct_tag_0qbrAHU2naEdhrklNafxKB
#define struct_tag_0qbrAHU2naEdhrklNafxKB
struct tag_0qbrAHU2naEdhrklNafxKB { real_T ConnectionDistance ; real_T
InternalNumNodes ; pwx2zp040p * InternalMap ; l0ipmq05ws PathFinder ;
boolean_T UpdateFlag ; njffc2stdn RoadmapBuilder ; kg3nyqw0nj _pobj0 ;
kcmtw0dk4c _pobj1 ; pwx2zp040p _pobj2 ; } ;
#endif
#ifndef typedef_mgywmw0rjk
#define typedef_mgywmw0rjk
typedef struct tag_0qbrAHU2naEdhrklNafxKB mgywmw0rjk ;
#endif
#ifndef SS_UINT64
#define SS_UINT64 21
#endif
#ifndef SS_INT64
#define SS_INT64 22
#endif
typedef struct i4xuulnjq50_ i4xuulnjq50 ; typedef struct k3jhwrd30z
m1mtbmkkvs ;
#endif
