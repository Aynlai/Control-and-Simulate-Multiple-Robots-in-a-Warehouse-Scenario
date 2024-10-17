#include "robotController.h"
#include "robotController_types.h"
#include "rtwtypes.h"
#include "robotController_private.h"
#include <string.h>
#include <emmintrin.h>
#include "mwmathutil.h"
#include "maximum_w0hsPrr7.h"
#include <math.h>
#include "norm_LxdYsY7z.h"
#include "getTruncatedIncrements_6j1H9bPB.h"
#include "eps_EMqudTFw.h"
#include <stdlib.h>
#include <stddef.h>
#include "robotController_capi.h"
#include "div_s32.h"
#define dwrx5ivjph ((uint8_T)3U)
#define jiscqp1m4h ((uint8_T)0U)
#define ltsv3qsxc4 ((uint8_T)1U)
#define oywowyxvde ((uint8_T)2U)
static RegMdlInfo rtMdlInfo_robotController [ 62 ] = { { "dx4drjuueew" ,
MDL_INFO_NAME_MDLREF_DWORK , 0 , - 1 , ( void * ) "robotController" } , {
"oac35lstt1" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "nd2pkfb15l" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "aszedvsh3h" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "n1byazndfi" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "kyvxlojbfd" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "djnc03zqgw" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "ggsewrsere" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "nrfo5ook2c" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "btgyhodm2h" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "epi4cozyds" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "k5rsdcofv4" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "lwnhcfk22o" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "dheucvhz3u" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "nd3lm2hagh" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "pedophjfqw" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "pg0y5at0tn" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "bq3nks32bt" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "mj0lr24tvd" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "a3uz1fjcva" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "f4xdwczleo" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "fh441y4mj5" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "ajrh5b3vn3" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "robotController" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT
, 0 , 0 , ( NULL ) } , { "grms1g2fze" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0
, - 1 , ( void * ) "robotController" } , { "i4xuulnjq50" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "k3jhwrd30z" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "m1mtbmkkvs" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "nav_slalgs_internal_PurePursuit1" ,
MDL_INFO_ID_DATA_TYPE , 0 , - 1 , ( NULL ) } , { "RobotPackageStatus" ,
MDL_INFO_ID_DATA_TYPE , 0 , - 1 , ( NULL ) } , { "RobotState_AtUnloadingStn"
, MDL_INFO_ID_ENUMTYPE_STRING , 0 , 3 , ( void * ) "RobotState" } , {
"RobotState_AtLoadingStn" , MDL_INFO_ID_ENUMTYPE_STRING , 0 , 2 , ( void * )
"RobotState" } , { "RobotState_AtDock" , MDL_INFO_ID_ENUMTYPE_STRING , 0 , 1
, ( void * ) "RobotState" } , { "RobotState" , MDL_INFO_ID_DATA_TYPE , 0 , -
1 , ( NULL ) } , { "RobotDeliverCommand" , MDL_INFO_ID_DATA_TYPE , 0 , - 1 ,
( NULL ) } , { "mr_robotController_GetSimStateDisallowedBlocks" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "robotController" } , {
"mr_robotController_extractBitFieldFromCellArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "robotController" } , {
"mr_robotController_cacheBitFieldToCellArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "robotController" } , {
"mr_robotController_restoreDataFromMxArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "robotController" } , {
"mr_robotController_cacheDataToMxArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "robotController" } , {
"mr_robotController_extractBitFieldFromMxArray" , MDL_INFO_ID_MODEL_FCN_NAME
, 0 , - 1 , ( void * ) "robotController" } , {
"mr_robotController_cacheBitFieldToMxArray" , MDL_INFO_ID_MODEL_FCN_NAME , 0
, - 1 , ( void * ) "robotController" } , {
"mr_robotController_restoreDataFromMxArray" , MDL_INFO_ID_MODEL_FCN_NAME , 0
, - 1 , ( void * ) "robotController" } , {
"mr_robotController_cacheDataAsMxArray" , MDL_INFO_ID_MODEL_FCN_NAME , 0 , -
1 , ( void * ) "robotController" } , {
"mr_robotController_RegisterSimStateChecksum" , MDL_INFO_ID_MODEL_FCN_NAME ,
0 , - 1 , ( void * ) "robotController" } , { "mr_robotController_SetDWork" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "robotController" } , {
"mr_robotController_GetDWork" , MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void
* ) "robotController" } , { "l0ipmq05ws" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "robotController" } , { "ii4nhp3skj" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "kg3nyqw0nj" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "kcmtw0dk4c" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "gbqfevksrb" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "ipw3ftlyif" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "k0shasdzrq" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "oxn0khgesl" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "njffc2stdn" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "b4axleo1v0" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "dxycgom5nk" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "robotController" } ,
{ "pwx2zp040p" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"robotController" } , { "mgywmw0rjk" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 ,
- 1 , ( void * ) "robotController" } , { "robotController.h" ,
MDL_INFO_MODEL_FILENAME , 0 , - 1 , ( NULL ) } , { "robotController.c" ,
MDL_INFO_MODEL_FILENAME , 0 , - 1 , ( void * ) "robotController" } } ;
i4xuulnjq50 i4xuulnjq5 = { 0.1 , 1.0 , 0.5 , false } ; static void k5xyyjjkd4
( k0shasdzrq * * pEmxArray , int32_T numDimensions ) ; static void
h4cfxmkat3k ( oxn0khgesl * pStruct ) ; static void h4cfxmkat3 ( njffc2stdn *
pStruct ) ; static void jkvo50kvdt ( mgywmw0rjk * pStruct ) ; static void
clokujid0y ( k0shasdzrq * emxArray , int32_T oldNumel ) ; static void
njqibisfwd ( k0shasdzrq * * pEmxArray ) ; static void czc125gt15 ( oxn0khgesl
* pStruct ) ; static void czc125gt15n ( njffc2stdn * pStruct ) ; static void
a0exojw1vz ( mgywmw0rjk * pStruct ) ; static real_T mdldozxqlt ( const real_T
x [ 2 ] ) ; static real_T fswlcxxr3a ( const real_T pt1 [ 2 ] , real_T pt2 [
2 ] , const real_T refPt [ 2 ] ) ; static void mkwt2qmyjw ( pwx2zp040p * obj
, boolean_T mat [ 3600 ] ) ; static void ib42mb0qrj ( b4axleo1v0 * *
pEmxArray , int32_T numDimensions ) ; static void ceougeaq11 ( dxycgom5nk * *
pEmxArray , int32_T numDimensions ) ; static void c2nfvjswuo ( dxycgom5nk * *
pEmxArray ) ; static void diadlcmegy ( real_T varargin_1 , k0shasdzrq * r ,
lwnhcfk22o * localDW ) ; static void fizihoqn3p ( b4axleo1v0 * * pEmxArray )
; static void dfntiycgzz ( dxycgom5nk * emxArray , int32_T oldNumel ) ;
static void mdiomdajqb ( b4axleo1v0 * emxArray , int32_T oldNumel ) ; static
void bgl1auxn4q ( b4axleo1v0 * in1 , const k0shasdzrq * in2 , const
k0shasdzrq * in3 ) ; static void it14h2ybk0 ( const b4axleo1v0 * x , int32_T
i_data [ ] , int32_T i_size [ 2 ] ) ; static void g1w3qbdma2 ( const real_T a
[ 2 ] , const k0shasdzrq * b , k0shasdzrq * c ) ; static real_T nzneokfgio (
const k0shasdzrq * x , int32_T col ) ; static void eangyuxugt ( const
oxn0khgesl * obj , const real_T pose1 [ 2 ] , const k0shasdzrq * pose2 ,
k0shasdzrq * dist ) ; static void ge40mv13fr ( dxycgom5nk * idx , k0shasdzrq
* x , int32_T offset , int32_T np , int32_T nq , dxycgom5nk * iwork ,
k0shasdzrq * xwork ) ; static void cumpzj3gf0 ( dxycgom5nk * idx , k0shasdzrq
* x , int32_T offset , int32_T n , int32_T preSortLevel , dxycgom5nk * iwork
, k0shasdzrq * xwork ) ; static void e0juex3elc ( k0shasdzrq * x , dxycgom5nk
* idx ) ; static void eb1nrnik44 ( k0shasdzrq * x , dxycgom5nk * idx ) ;
static boolean_T btdzbryr1o ( const real_T p1 [ 2 ] , const real_T p2 [ 2 ] ,
const boolean_T map [ 3600 ] , const real_T gridLocation [ 2 ] ) ; static
void p3g0upgfmv ( oxn0khgesl * obj , real_T node1 , real_T node2 , real_T w )
; static void oung1l0hdz ( mgywmw0rjk * obj , lwnhcfk22o * localDW ) ; static
real_T le2qxtov41 ( const real_T node [ 2 ] , const oxn0khgesl * roadmap ,
pwx2zp040p * map ) ; static void fsaynwsdln ( const oxn0khgesl * obj , real_T
nodes , k0shasdzrq * componentList ) ; static boolean_T hulg2uozzd ( const
k0shasdzrq * varargin_1 , const k0shasdzrq * varargin_2 ) ; static void
mbfnj03133 ( const oxn0khgesl * obj , k0shasdzrq * edgeList ) ; static void
g1w3qbdma2o ( const k0shasdzrq * a , const real_T b [ 2 ] , k0shasdzrq * c )
; static void eb1nrnik44z ( k0shasdzrq * x ) ; static void j2eqhook05 ( const
oxn0khgesl * obj , real_T startNode , real_T goalNode , k0shasdzrq * path ) ;
static void bej3nj0pvr ( mgywmw0rjk * obj , const real_T start [ 2 ] , const
real_T goal [ 2 ] , k0shasdzrq * path , lwnhcfk22o * localDW ) ; static void
k5xyyjjkd4 ( k0shasdzrq * * pEmxArray , int32_T numDimensions ) { k0shasdzrq
* emxArray ; int32_T i ; * pEmxArray = ( k0shasdzrq * ) malloc ( sizeof (
k0shasdzrq ) ) ; emxArray = * pEmxArray ; emxArray -> data = ( real_T * )
NULL ; emxArray -> numDimensions = numDimensions ; emxArray -> size = (
int32_T * ) malloc ( sizeof ( int32_T ) * ( uint32_T ) numDimensions ) ;
emxArray -> allocatedSize = 0 ; emxArray -> canFreeData = true ; for ( i = 0
; i < numDimensions ; i ++ ) { emxArray -> size [ i ] = 0 ; } } static void
h4cfxmkat3k ( oxn0khgesl * pStruct ) { k5xyyjjkd4 ( & pStruct ->
AdjacencyMatrix , 2 ) ; k5xyyjjkd4 ( & pStruct -> Nodes , 2 ) ; k5xyyjjkd4 (
& pStruct -> NodeLabels , 2 ) ; k5xyyjjkd4 ( & pStruct -> NodeLabelSet , 2 )
; } static void h4cfxmkat3 ( njffc2stdn * pStruct ) { h4cfxmkat3k ( & pStruct
-> Roadmap ) ; } static void jkvo50kvdt ( mgywmw0rjk * pStruct ) { h4cfxmkat3
( & pStruct -> RoadmapBuilder ) ; } static void clokujid0y ( k0shasdzrq *
emxArray , int32_T oldNumel ) { int32_T i ; int32_T newNumel ; void * newData
; if ( oldNumel < 0 ) { oldNumel = 0 ; } newNumel = 1 ; for ( i = 0 ; i <
emxArray -> numDimensions ; i ++ ) { newNumel *= emxArray -> size [ i ] ; }
if ( newNumel > emxArray -> allocatedSize ) { i = emxArray -> allocatedSize ;
if ( i < 16 ) { i = 16 ; } while ( i < newNumel ) { if ( i > 1073741823 ) { i
= MAX_int32_T ; } else { i <<= 1 ; } } newData = malloc ( ( uint32_T ) i *
sizeof ( real_T ) ) ; if ( emxArray -> data != NULL ) { memcpy ( newData ,
emxArray -> data , sizeof ( real_T ) * ( uint32_T ) oldNumel ) ; if (
emxArray -> canFreeData ) { free ( emxArray -> data ) ; } } emxArray -> data
= ( real_T * ) newData ; emxArray -> allocatedSize = i ; emxArray ->
canFreeData = true ; } } static void njqibisfwd ( k0shasdzrq * * pEmxArray )
{ if ( * pEmxArray != ( k0shasdzrq * ) NULL ) { if ( ( ( * pEmxArray ) ->
data != ( real_T * ) NULL ) && ( * pEmxArray ) -> canFreeData ) { free ( ( *
pEmxArray ) -> data ) ; } free ( ( * pEmxArray ) -> size ) ; free ( *
pEmxArray ) ; * pEmxArray = ( k0shasdzrq * ) NULL ; } } static void
czc125gt15 ( oxn0khgesl * pStruct ) { njqibisfwd ( & pStruct ->
AdjacencyMatrix ) ; njqibisfwd ( & pStruct -> Nodes ) ; njqibisfwd ( &
pStruct -> NodeLabels ) ; njqibisfwd ( & pStruct -> NodeLabelSet ) ; } static
void czc125gt15n ( njffc2stdn * pStruct ) { czc125gt15 ( & pStruct -> Roadmap
) ; } static void a0exojw1vz ( mgywmw0rjk * pStruct ) { czc125gt15n ( &
pStruct -> RoadmapBuilder ) ; } static real_T mdldozxqlt ( const real_T x [ 2
] ) { real_T absxk ; real_T scale ; real_T t ; real_T y ; scale =
3.3121686421112381E-170 ; absxk = muDoubleScalarAbs ( x [ 0 ] ) ; if ( absxk
> 3.3121686421112381E-170 ) { y = 1.0 ; scale = absxk ; } else { t = absxk /
3.3121686421112381E-170 ; y = t * t ; } absxk = muDoubleScalarAbs ( x [ 1 ] )
; if ( absxk > scale ) { t = scale / absxk ; y = y * t * t + 1.0 ; scale =
absxk ; } else { t = absxk / scale ; y += t * t ; } return scale *
muDoubleScalarSqrt ( y ) ; } static real_T fswlcxxr3a ( const real_T pt1 [ 2
] , real_T pt2 [ 2 ] , const real_T refPt [ 2 ] ) { __m128d tmp ; real_T
refPt_p [ 2 ] ; real_T alpha ; real_T distance ; real_T v12 ; real_T v12_p ;
int32_T b_k ; boolean_T exitg1 ; boolean_T p ; boolean_T p_p ; p = false ;
p_p = true ; b_k = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( b_k < 2 ) )
{ if ( ! ( pt1 [ b_k ] == pt2 [ b_k ] ) ) { p_p = false ; exitg1 = true ; }
else { b_k ++ ; } } if ( p_p ) { p = true ; } if ( p ) { pt2 [ 0 ] = pt1 [ 0
] ; refPt_p [ 0 ] = refPt [ 0 ] - pt1 [ 0 ] ; pt2 [ 1 ] = pt1 [ 1 ] ; refPt_p
[ 1 ] = refPt [ 1 ] - pt1 [ 1 ] ; distance = mdldozxqlt ( refPt_p ) ; } else
{ alpha = pt2 [ 0 ] - pt1 [ 0 ] ; v12 = ( pt2 [ 0 ] - refPt [ 0 ] ) * alpha ;
v12_p = alpha * alpha ; alpha = pt2 [ 1 ] - pt1 [ 1 ] ; v12 += ( pt2 [ 1 ] -
refPt [ 1 ] ) * alpha ; v12_p += alpha * alpha ; alpha = v12 / v12_p ; if (
alpha > 1.0 ) { pt2 [ 0 ] = pt1 [ 0 ] ; pt2 [ 1 ] = pt1 [ 1 ] ; } else if ( !
( alpha < 0.0 ) ) { tmp = _mm_set1_pd ( alpha ) ; tmp = _mm_add_pd (
_mm_mul_pd ( _mm_sub_pd ( _mm_set1_pd ( 1.0 ) , tmp ) , _mm_loadu_pd ( & pt2
[ 0 ] ) ) , _mm_mul_pd ( tmp , _mm_loadu_pd ( & pt1 [ 0 ] ) ) ) ;
_mm_storeu_pd ( & pt2 [ 0 ] , tmp ) ; } tmp = _mm_sub_pd ( _mm_loadu_pd ( &
refPt [ 0 ] ) , _mm_loadu_pd ( & pt2 [ 0 ] ) ) ; _mm_storeu_pd ( & refPt_p [
0 ] , tmp ) ; distance = mdldozxqlt ( refPt_p ) ; } return distance ; }
static void mkwt2qmyjw ( pwx2zp040p * obj , boolean_T mat [ 3600 ] ) {
kcmtw0dk4c * b_obj ; kg3nyqw0nj * c_index ; real_T p ; int32_T b ; int32_T
b_x ; int32_T d_k ; int32_T e ; int32_T e_tmp ; int32_T g_k ; int32_T i1 ;
int32_T j ; int32_T ns ; int32_T pageroot ; int32_T pagesize ; int32_T stride
; boolean_T buffer [ 30 ] ; boolean_T x [ 2 ] ; boolean_T exitg1 ; boolean_T
y ; c_index = obj -> Buffer -> Index ; x [ 0 ] = ( c_index -> Head [ 0 ] ==
1.0 ) ; x [ 1 ] = ( c_index -> Head [ 1 ] == 1.0 ) ; y = true ; g_k = 0 ;
exitg1 = false ; while ( ( ! exitg1 ) && ( g_k <= 1 ) ) { if ( ! x [ g_k ] )
{ y = false ; exitg1 = true ; } else { g_k ++ ; } } if ( y ) { for ( g_k = 0
; g_k < 3600 ; g_k ++ ) { mat [ g_k ] = obj -> Buffer -> Buffer [ g_k ] ; } }
else { c_index = obj -> Buffer -> Index ; for ( g_k = 0 ; g_k < 3600 ; g_k ++
) { mat [ g_k ] = obj -> Buffer -> Buffer [ g_k ] ; } stride = 1 ; for ( g_k
= 0 ; g_k < 2 ; g_k ++ ) { p = - ( c_index -> Head [ g_k ] - 1.0 ) ; if ( p <
0.0 ) { b_x = - ( int32_T ) p ; y = false ; } else { b_x = ( int32_T ) p ; y
= true ; } if ( b_x > 60 ) { b_x -= 60 * div_s32 ( b_x , 60 ) ; } if ( b_x >
30 ) { b_x = 60 - b_x ; y = ! y ; } x [ g_k ] = y ; ns = b_x ; pagesize =
stride * 60 ; if ( b_x > 0 ) { b = - 59 * g_k + 59 ; for ( b_x = 0 ; b_x <= b
; b_x ++ ) { pageroot = b_x * pagesize ; for ( j = 0 ; j < stride ; j ++ ) {
i1 = pageroot + j ; if ( x [ g_k ] ) { e_tmp = ( uint8_T ) ns ; for ( d_k = 0
; d_k < e_tmp ; d_k ++ ) { buffer [ d_k ] = mat [ ( ( d_k - ns ) + 60 ) *
stride + i1 ] ; } for ( d_k = 60 ; d_k >= ns + 1 ; d_k -- ) { mat [ i1 + (
d_k - 1 ) * stride ] = mat [ ( ( d_k - ns ) - 1 ) * stride + i1 ] ; } for (
d_k = 0 ; d_k < e_tmp ; d_k ++ ) { mat [ i1 + d_k * stride ] = buffer [ d_k ]
; } } else { e_tmp = ( uint8_T ) ns ; for ( d_k = 0 ; d_k < e_tmp ; d_k ++ )
{ buffer [ d_k ] = mat [ d_k * stride + i1 ] ; } e = ( uint8_T ) ( 60 - ns )
; for ( d_k = 0 ; d_k < e ; d_k ++ ) { mat [ i1 + d_k * stride ] = mat [ (
d_k + ns ) * stride + i1 ] ; } for ( d_k = 0 ; d_k < e_tmp ; d_k ++ ) { mat [
i1 + ( ( d_k - ns ) + 60 ) * stride ] = buffer [ d_k ] ; } } } } } stride =
pagesize ; } if ( ! obj -> HasParent ) { for ( g_k = 0 ; g_k < 3600 ; g_k ++
) { obj -> Buffer -> Buffer [ g_k ] = mat [ g_k ] ; } c_index = obj -> Buffer
-> Index ; c_index -> Head [ 0 ] = 1.0 ; c_index -> Head [ 1 ] = 1.0 ; b_obj
= obj -> Buffer ; b_obj -> Index = c_index ; } } } static void ib42mb0qrj (
b4axleo1v0 * * pEmxArray , int32_T numDimensions ) { b4axleo1v0 * emxArray ;
int32_T i ; * pEmxArray = ( b4axleo1v0 * ) malloc ( sizeof ( b4axleo1v0 ) ) ;
emxArray = * pEmxArray ; emxArray -> data = ( boolean_T * ) NULL ; emxArray
-> numDimensions = numDimensions ; emxArray -> size = ( int32_T * ) malloc (
sizeof ( int32_T ) * ( uint32_T ) numDimensions ) ; emxArray -> allocatedSize
= 0 ; emxArray -> canFreeData = true ; for ( i = 0 ; i < numDimensions ; i ++
) { emxArray -> size [ i ] = 0 ; } } static void ceougeaq11 ( dxycgom5nk * *
pEmxArray , int32_T numDimensions ) { dxycgom5nk * emxArray ; int32_T i ; *
pEmxArray = ( dxycgom5nk * ) malloc ( sizeof ( dxycgom5nk ) ) ; emxArray = *
pEmxArray ; emxArray -> data = ( int32_T * ) NULL ; emxArray -> numDimensions
= numDimensions ; emxArray -> size = ( int32_T * ) malloc ( sizeof ( int32_T
) * ( uint32_T ) numDimensions ) ; emxArray -> allocatedSize = 0 ; emxArray
-> canFreeData = true ; for ( i = 0 ; i < numDimensions ; i ++ ) { emxArray
-> size [ i ] = 0 ; } } static void c2nfvjswuo ( dxycgom5nk * * pEmxArray ) {
if ( * pEmxArray != ( dxycgom5nk * ) NULL ) { if ( ( ( * pEmxArray ) -> data
!= ( int32_T * ) NULL ) && ( * pEmxArray ) -> canFreeData ) { free ( ( *
pEmxArray ) -> data ) ; } free ( ( * pEmxArray ) -> size ) ; free ( *
pEmxArray ) ; * pEmxArray = ( dxycgom5nk * ) NULL ; } } static void
diadlcmegy ( real_T varargin_1 , k0shasdzrq * r , lwnhcfk22o * localDW ) {
real_T b_r ; int32_T b ; int32_T b_k ; int32_T exitg1 ; int32_T k ; int32_T
kk ; uint32_T u [ 2 ] ; uint32_T mti ; uint32_T y ; boolean_T b_isvalid ;
boolean_T exitg2 ; k = r -> size [ 0 ] * r -> size [ 1 ] ; r -> size [ 0 ] =
( int32_T ) varargin_1 ; r -> size [ 1 ] = 2 ; clokujid0y ( r , k ) ; b = (
int32_T ) varargin_1 << 1 ; for ( k = 0 ; k < b ; k ++ ) { do { exitg1 = 0 ;
for ( b_k = 0 ; b_k < 2 ; b_k ++ ) { mti = localDW -> pggahx0edn [ 624 ] + 1U
; if ( localDW -> pggahx0edn [ 624 ] + 1U >= 625U ) { for ( kk = 0 ; kk < 227
; kk ++ ) { mti = ( localDW -> pggahx0edn [ kk + 1 ] & 2147483647U ) | (
localDW -> pggahx0edn [ kk ] & 2147483648U ) ; if ( ( mti & 1U ) == 0U ) {
mti >>= 1U ; } else { mti = mti >> 1U ^ 2567483615U ; } localDW -> pggahx0edn
[ kk ] = localDW -> pggahx0edn [ kk + 397 ] ^ mti ; } for ( kk = 0 ; kk < 396
; kk ++ ) { mti = ( localDW -> pggahx0edn [ kk + 227 ] & 2147483648U ) | (
localDW -> pggahx0edn [ kk + 228 ] & 2147483647U ) ; if ( ( mti & 1U ) == 0U
) { mti >>= 1U ; } else { mti = mti >> 1U ^ 2567483615U ; } localDW ->
pggahx0edn [ kk + 227 ] = localDW -> pggahx0edn [ kk ] ^ mti ; } mti = (
localDW -> pggahx0edn [ 623 ] & 2147483648U ) | ( localDW -> pggahx0edn [ 0 ]
& 2147483647U ) ; if ( ( mti & 1U ) == 0U ) { mti >>= 1U ; } else { mti = mti
>> 1U ^ 2567483615U ; } localDW -> pggahx0edn [ 623 ] = localDW -> pggahx0edn
[ 396 ] ^ mti ; mti = 1U ; } y = localDW -> pggahx0edn [ ( int32_T ) mti - 1
] ; localDW -> pggahx0edn [ 624 ] = mti ; y ^= y >> 11U ; y ^= y << 7U &
2636928640U ; y ^= y << 15U & 4022730752U ; u [ b_k ] = y >> 18U ^ y ; } b_r
= ( ( real_T ) ( u [ 0 ] >> 5U ) * 6.7108864E+7 + ( real_T ) ( u [ 1 ] >> 6U
) ) * 1.1102230246251565E-16 ; if ( b_r == 0.0 ) { if ( ( localDW ->
pggahx0edn [ 624 ] >= 1U ) && ( localDW -> pggahx0edn [ 624 ] < 625U ) ) {
b_isvalid = false ; b_k = 1 ; exitg2 = false ; while ( ( ! exitg2 ) && ( b_k
< 625 ) ) { if ( localDW -> pggahx0edn [ b_k - 1 ] == 0U ) { b_k ++ ; } else
{ b_isvalid = true ; exitg2 = true ; } } } else { b_isvalid = false ; } if (
! b_isvalid ) { localDW -> pggahx0edn [ 0 ] = 5489U ; localDW -> pggahx0edn [
624 ] = 624U ; } } else { exitg1 = 1 ; } } while ( exitg1 == 0 ) ; r -> data
[ k ] = b_r ; } } static void fizihoqn3p ( b4axleo1v0 * * pEmxArray ) { if (
* pEmxArray != ( b4axleo1v0 * ) NULL ) { if ( ( ( * pEmxArray ) -> data != (
boolean_T * ) NULL ) && ( * pEmxArray ) -> canFreeData ) { free ( ( *
pEmxArray ) -> data ) ; } free ( ( * pEmxArray ) -> size ) ; free ( *
pEmxArray ) ; * pEmxArray = ( b4axleo1v0 * ) NULL ; } } static void
dfntiycgzz ( dxycgom5nk * emxArray , int32_T oldNumel ) { int32_T i ; int32_T
newNumel ; void * newData ; if ( oldNumel < 0 ) { oldNumel = 0 ; } newNumel =
1 ; for ( i = 0 ; i < emxArray -> numDimensions ; i ++ ) { newNumel *=
emxArray -> size [ i ] ; } if ( newNumel > emxArray -> allocatedSize ) { i =
emxArray -> allocatedSize ; if ( i < 16 ) { i = 16 ; } while ( i < newNumel )
{ if ( i > 1073741823 ) { i = MAX_int32_T ; } else { i <<= 1 ; } } newData =
malloc ( ( uint32_T ) i * sizeof ( int32_T ) ) ; if ( emxArray -> data !=
NULL ) { memcpy ( newData , emxArray -> data , sizeof ( int32_T ) * (
uint32_T ) oldNumel ) ; if ( emxArray -> canFreeData ) { free ( emxArray ->
data ) ; } } emxArray -> data = ( int32_T * ) newData ; emxArray ->
allocatedSize = i ; emxArray -> canFreeData = true ; } } static void
mdiomdajqb ( b4axleo1v0 * emxArray , int32_T oldNumel ) { int32_T i ; int32_T
newNumel ; void * newData ; if ( oldNumel < 0 ) { oldNumel = 0 ; } newNumel =
1 ; for ( i = 0 ; i < emxArray -> numDimensions ; i ++ ) { newNumel *=
emxArray -> size [ i ] ; } if ( newNumel > emxArray -> allocatedSize ) { i =
emxArray -> allocatedSize ; if ( i < 16 ) { i = 16 ; } while ( i < newNumel )
{ if ( i > 1073741823 ) { i = MAX_int32_T ; } else { i <<= 1 ; } } newData =
malloc ( ( uint32_T ) i * sizeof ( boolean_T ) ) ; if ( emxArray -> data !=
NULL ) { memcpy ( newData , emxArray -> data , sizeof ( boolean_T ) * (
uint32_T ) oldNumel ) ; if ( emxArray -> canFreeData ) { free ( emxArray ->
data ) ; } } emxArray -> data = ( boolean_T * ) newData ; emxArray ->
allocatedSize = i ; emxArray -> canFreeData = true ; } } static void
bgl1auxn4q ( b4axleo1v0 * in1 , const k0shasdzrq * in2 , const k0shasdzrq *
in3 ) { int32_T i ; int32_T loop_ub ; int32_T stride_0_1 ; int32_T stride_1_1
; i = in1 -> size [ 0 ] * in1 -> size [ 1 ] ; in1 -> size [ 0 ] = 1 ;
mdiomdajqb ( in1 , i ) ; if ( in3 -> size [ 1 ] == 1 ) { i = in1 -> size [ 0
] * in1 -> size [ 1 ] ; in1 -> size [ 1 ] = in2 -> size [ 1 ] ; mdiomdajqb (
in1 , i ) ; } else { i = in1 -> size [ 0 ] * in1 -> size [ 1 ] ; in1 -> size
[ 1 ] = in3 -> size [ 1 ] ; mdiomdajqb ( in1 , i ) ; } stride_0_1 = ( in2 ->
size [ 1 ] != 1 ) ; stride_1_1 = ( in3 -> size [ 1 ] != 1 ) ; if ( in3 ->
size [ 1 ] == 1 ) { loop_ub = in2 -> size [ 1 ] ; } else { loop_ub = in3 ->
size [ 1 ] ; } for ( i = 0 ; i < loop_ub ; i ++ ) { in1 -> data [ i ] = ( (
in2 -> data [ i * stride_0_1 ] < 2.2204460492503131E-16 ) && ( in3 -> data [
i * stride_1_1 ] < 2.2204460492503131E-16 ) ) ; } } static void it14h2ybk0 (
const b4axleo1v0 * x , int32_T i_data [ ] , int32_T i_size [ 2 ] ) { int32_T
idx ; int32_T ii ; int32_T k ; boolean_T exitg1 ; k = ( x -> size [ 1 ] >= 1
) ; idx = 0 ; i_size [ 0 ] = 1 ; i_size [ 1 ] = k ; ii = 0 ; exitg1 = false ;
while ( ( ! exitg1 ) && ( ii <= x -> size [ 1 ] - 1 ) ) { if ( x -> data [ ii
] ) { idx = 1 ; i_data [ 0 ] = ii + 1 ; exitg1 = true ; } else { ii ++ ; } }
if ( k == 1 ) { if ( idx == 0 ) { i_size [ 0 ] = 1 ; i_size [ 1 ] = 0 ; } }
else { i_size [ 1 ] = ( idx >= 1 ) ; } } static void g1w3qbdma2 ( const
real_T a [ 2 ] , const k0shasdzrq * b , k0shasdzrq * c ) { int32_T bcoef ;
int32_T d ; int32_T k ; k = c -> size [ 0 ] * c -> size [ 1 ] ; c -> size [ 0
] = 2 ; c -> size [ 1 ] = b -> size [ 1 ] ; clokujid0y ( c , k ) ; if ( b ->
size [ 1 ] != 0 ) { bcoef = ( b -> size [ 1 ] != 1 ) ; d = b -> size [ 1 ] -
1 ; for ( k = 0 ; k <= d ; k ++ ) { _mm_storeu_pd ( & c -> data [ k << 1 ] ,
_mm_sub_pd ( _mm_loadu_pd ( & a [ 0 ] ) , _mm_loadu_pd ( & b -> data [ (
bcoef * k ) << 1 ] ) ) ) ; } } } static real_T nzneokfgio ( const k0shasdzrq
* x , int32_T col ) { int32_T i0 ; i0 = ( col - 1 ) << 1 ; return x -> data [
i0 + 1 ] + x -> data [ i0 ] ; } static void eangyuxugt ( const oxn0khgesl *
obj , const real_T pose1 [ 2 ] , const k0shasdzrq * pose2 , k0shasdzrq * dist
) { __m128d tmp_p ; k0shasdzrq * tmp ; k0shasdzrq * x ; real_T varargin_1 ;
int32_T b_ncols ; int32_T b_nx ; int32_T scalarLB ; int32_T vectorUB ;
k5xyyjjkd4 ( & x , 2 ) ; k5xyyjjkd4 ( & tmp , 2 ) ; if ( obj -> IsEuclidean )
{ g1w3qbdma2 ( pose1 , pose2 , tmp ) ; scalarLB = x -> size [ 0 ] * x -> size
[ 1 ] ; x -> size [ 0 ] = 2 ; x -> size [ 1 ] = tmp -> size [ 1 ] ;
clokujid0y ( x , scalarLB ) ; b_nx = tmp -> size [ 1 ] << 1 ; scalarLB = (
b_nx / 2 ) << 1 ; vectorUB = scalarLB - 2 ; for ( b_ncols = 0 ; b_ncols <=
vectorUB ; b_ncols += 2 ) { tmp_p = _mm_loadu_pd ( & tmp -> data [ b_ncols ]
) ; _mm_storeu_pd ( & x -> data [ b_ncols ] , _mm_mul_pd ( tmp_p , tmp_p ) )
; } for ( b_ncols = scalarLB ; b_ncols < b_nx ; b_ncols ++ ) { varargin_1 =
tmp -> data [ b_ncols ] ; x -> data [ b_ncols ] = varargin_1 * varargin_1 ; }
if ( x -> size [ 1 ] == 0 ) { dist -> size [ 0 ] = 1 ; dist -> size [ 1 ] = 0
; } else { scalarLB = dist -> size [ 0 ] * dist -> size [ 1 ] ; dist -> size
[ 0 ] = 1 ; b_ncols = x -> size [ 1 ] ; dist -> size [ 1 ] = x -> size [ 1 ]
; clokujid0y ( dist , scalarLB ) ; for ( scalarLB = 0 ; scalarLB < b_ncols ;
scalarLB ++ ) { dist -> data [ scalarLB ] = nzneokfgio ( x , scalarLB + 1 ) ;
} } b_nx = dist -> size [ 1 ] ; scalarLB = ( dist -> size [ 1 ] / 2 ) << 1 ;
vectorUB = scalarLB - 2 ; for ( b_ncols = 0 ; b_ncols <= vectorUB ; b_ncols
+= 2 ) { tmp_p = _mm_loadu_pd ( & dist -> data [ b_ncols ] ) ; _mm_storeu_pd
( & dist -> data [ b_ncols ] , _mm_sqrt_pd ( tmp_p ) ) ; } for ( b_ncols =
scalarLB ; b_ncols < b_nx ; b_ncols ++ ) { dist -> data [ b_ncols ] =
muDoubleScalarSqrt ( dist -> data [ b_ncols ] ) ; } } else { g1w3qbdma2 (
pose1 , pose2 , tmp ) ; scalarLB = x -> size [ 0 ] * x -> size [ 1 ] ; x ->
size [ 0 ] = 2 ; x -> size [ 1 ] = tmp -> size [ 1 ] ; clokujid0y ( x ,
scalarLB ) ; b_nx = tmp -> size [ 1 ] << 1 ; scalarLB = ( b_nx / 2 ) << 1 ;
vectorUB = scalarLB - 2 ; for ( b_ncols = 0 ; b_ncols <= vectorUB ; b_ncols
+= 2 ) { tmp_p = _mm_loadu_pd ( & tmp -> data [ b_ncols ] ) ; _mm_storeu_pd (
& x -> data [ b_ncols ] , _mm_mul_pd ( tmp_p , tmp_p ) ) ; } for ( b_ncols =
scalarLB ; b_ncols < b_nx ; b_ncols ++ ) { varargin_1 = tmp -> data [ b_ncols
] ; x -> data [ b_ncols ] = varargin_1 * varargin_1 ; } if ( x -> size [ 1 ]
== 0 ) { dist -> size [ 0 ] = 1 ; dist -> size [ 1 ] = 0 ; } else { scalarLB
= dist -> size [ 0 ] * dist -> size [ 1 ] ; dist -> size [ 0 ] = 1 ; b_ncols
= x -> size [ 1 ] ; dist -> size [ 1 ] = x -> size [ 1 ] ; clokujid0y ( dist
, scalarLB ) ; for ( scalarLB = 0 ; scalarLB < b_ncols ; scalarLB ++ ) { dist
-> data [ scalarLB ] = nzneokfgio ( x , scalarLB + 1 ) ; } } b_nx = dist ->
size [ 1 ] ; scalarLB = ( dist -> size [ 1 ] / 2 ) << 1 ; vectorUB = scalarLB
- 2 ; for ( b_ncols = 0 ; b_ncols <= vectorUB ; b_ncols += 2 ) { tmp_p =
_mm_loadu_pd ( & dist -> data [ b_ncols ] ) ; _mm_storeu_pd ( & dist -> data
[ b_ncols ] , _mm_sqrt_pd ( tmp_p ) ) ; } for ( b_ncols = scalarLB ; b_ncols
< b_nx ; b_ncols ++ ) { dist -> data [ b_ncols ] = muDoubleScalarSqrt ( dist
-> data [ b_ncols ] ) ; } } njqibisfwd ( & tmp ) ; njqibisfwd ( & x ) ; }
static void ge40mv13fr ( dxycgom5nk * idx , k0shasdzrq * x , int32_T offset ,
int32_T np , int32_T nq , dxycgom5nk * iwork , k0shasdzrq * xwork ) { int32_T
exitg1 ; int32_T iout ; int32_T n ; int32_T q ; int32_T qend ; if ( nq != 0 )
{ qend = np + nq ; for ( q = 0 ; q < qend ; q ++ ) { iout = offset + q ;
iwork -> data [ q ] = idx -> data [ iout ] ; xwork -> data [ q ] = x -> data
[ iout ] ; } n = 0 ; q = np ; iout = offset - 1 ; do { exitg1 = 0 ; iout ++ ;
if ( xwork -> data [ n ] <= xwork -> data [ q ] ) { idx -> data [ iout ] =
iwork -> data [ n ] ; x -> data [ iout ] = xwork -> data [ n ] ; if ( n + 1 <
np ) { n ++ ; } else { exitg1 = 1 ; } } else { idx -> data [ iout ] = iwork
-> data [ q ] ; x -> data [ iout ] = xwork -> data [ q ] ; if ( q + 1 < qend
) { q ++ ; } else { qend = iout - n ; for ( q = n + 1 ; q <= np ; q ++ ) {
iout = qend + q ; idx -> data [ iout ] = iwork -> data [ q - 1 ] ; x -> data
[ iout ] = xwork -> data [ q - 1 ] ; } exitg1 = 1 ; } } } while ( exitg1 == 0
) ; } } static void cumpzj3gf0 ( dxycgom5nk * idx , k0shasdzrq * x , int32_T
offset , int32_T n , int32_T preSortLevel , dxycgom5nk * iwork , k0shasdzrq *
xwork ) { int32_T bLen ; int32_T nPairs ; int32_T nTail ; int32_T tailOffset
; nPairs = n >> preSortLevel ; bLen = 1 << preSortLevel ; while ( nPairs > 1
) { if ( ( ( uint32_T ) nPairs & 1U ) != 0U ) { nPairs -- ; tailOffset = bLen
* nPairs ; nTail = n - tailOffset ; if ( nTail > bLen ) { ge40mv13fr ( idx ,
x , offset + tailOffset , bLen , nTail - bLen , iwork , xwork ) ; } } nTail =
bLen << 1 ; nPairs >>= 1 ; for ( tailOffset = 0 ; tailOffset < nPairs ;
tailOffset ++ ) { ge40mv13fr ( idx , x , offset + tailOffset * nTail , bLen ,
bLen , iwork , xwork ) ; } bLen = nTail ; } if ( n > bLen ) { ge40mv13fr (
idx , x , offset , bLen , n - bLen , iwork , xwork ) ; } } static void
e0juex3elc ( k0shasdzrq * x , dxycgom5nk * idx ) { dxycgom5nk * iwork ;
k0shasdzrq * b_x ; k0shasdzrq * xwork ; real_T b_xwork [ 256 ] ; real_T x4 [
4 ] ; real_T tmp ; real_T tmp_p ; int32_T b_iwork [ 256 ] ; int32_T idx4 [ 4
] ; int32_T b_iwork_tmp ; int32_T d ; int32_T exitg1 ; int32_T i1 ; int32_T
i2 ; int32_T i3 ; int32_T i4 ; int32_T ib ; int32_T loop_ub ; int32_T n ;
int32_T nBlocks ; int32_T p ; int32_T perm_p ; int32_T q ; int32_T wOffset ;
int8_T perm [ 4 ] ; b_iwork_tmp = idx -> size [ 0 ] * idx -> size [ 1 ] ; idx
-> size [ 0 ] = 1 ; loop_ub = x -> size [ 1 ] ; idx -> size [ 1 ] = x -> size
[ 1 ] ; dfntiycgzz ( idx , b_iwork_tmp ) ; if ( loop_ub - 1 >= 0 ) { memset (
& idx -> data [ 0 ] , 0 , ( uint32_T ) loop_ub * sizeof ( int32_T ) ) ; } if
( x -> size [ 1 ] != 0 ) { k5xyyjjkd4 ( & xwork , 1 ) ; b_iwork_tmp = xwork
-> size [ 0 ] ; xwork -> size [ 0 ] = x -> size [ 1 ] ; clokujid0y ( xwork ,
b_iwork_tmp ) ; b_iwork_tmp = idx -> size [ 0 ] * idx -> size [ 1 ] ; idx ->
size [ 0 ] = 1 ; idx -> size [ 1 ] = x -> size [ 1 ] ; dfntiycgzz ( idx ,
b_iwork_tmp ) ; k5xyyjjkd4 ( & b_x , 2 ) ; b_iwork_tmp = b_x -> size [ 0 ] *
b_x -> size [ 1 ] ; b_x -> size [ 0 ] = 1 ; b_x -> size [ 1 ] = x -> size [ 1
] ; clokujid0y ( b_x , b_iwork_tmp ) ; for ( ib = 0 ; ib < loop_ub ; ib ++ )
{ idx -> data [ ib ] = 0 ; b_x -> data [ ib ] = x -> data [ ib ] ; } n = x ->
size [ 1 ] - 1 ; x4 [ 0 ] = 0.0 ; idx4 [ 0 ] = 0 ; x4 [ 1 ] = 0.0 ; idx4 [ 1
] = 0 ; x4 [ 2 ] = 0.0 ; idx4 [ 2 ] = 0 ; x4 [ 3 ] = 0.0 ; idx4 [ 3 ] = 0 ;
nBlocks = 0 ; ib = 0 ; for ( wOffset = 0 ; wOffset <= n ; wOffset ++ ) { if (
muDoubleScalarIsNaN ( b_x -> data [ wOffset ] ) ) { p = n - nBlocks ; idx ->
data [ p ] = wOffset + 1 ; xwork -> data [ p ] = b_x -> data [ wOffset ] ;
nBlocks ++ ; } else { ib ++ ; idx4 [ ib - 1 ] = wOffset + 1 ; x4 [ ib - 1 ] =
b_x -> data [ wOffset ] ; if ( ib == 4 ) { ib = wOffset - nBlocks ; if ( x4 [
0 ] <= x4 [ 1 ] ) { i1 = 1 ; i2 = 2 ; } else { i1 = 2 ; i2 = 1 ; } if ( x4 [
2 ] <= x4 [ 3 ] ) { i3 = 3 ; i4 = 4 ; } else { i3 = 4 ; i4 = 3 ; } tmp = x4 [
i3 - 1 ] ; tmp_p = x4 [ i1 - 1 ] ; if ( tmp_p <= tmp ) { tmp_p = x4 [ i2 - 1
] ; if ( tmp_p <= tmp ) { p = i1 ; perm_p = i2 ; i1 = i3 ; i2 = i4 ; } else
if ( tmp_p <= x4 [ i4 - 1 ] ) { p = i1 ; perm_p = i3 ; i1 = i2 ; i2 = i4 ; }
else { p = i1 ; perm_p = i3 ; i1 = i4 ; } } else { tmp = x4 [ i4 - 1 ] ; if (
tmp_p <= tmp ) { if ( x4 [ i2 - 1 ] <= tmp ) { p = i3 ; perm_p = i1 ; i1 = i2
; i2 = i4 ; } else { p = i3 ; perm_p = i1 ; i1 = i4 ; } } else { p = i3 ;
perm_p = i4 ; } } idx -> data [ ib - 3 ] = idx4 [ p - 1 ] ; idx -> data [ ib
- 2 ] = idx4 [ perm_p - 1 ] ; idx -> data [ ib - 1 ] = idx4 [ i1 - 1 ] ; idx
-> data [ ib ] = idx4 [ i2 - 1 ] ; b_x -> data [ ib - 3 ] = x4 [ p - 1 ] ;
b_x -> data [ ib - 2 ] = x4 [ perm_p - 1 ] ; b_x -> data [ ib - 1 ] = x4 [ i1
- 1 ] ; b_x -> data [ ib ] = x4 [ i2 - 1 ] ; ib = 0 ; } } } perm_p = x ->
size [ 1 ] - nBlocks ; if ( ib > 0 ) { perm [ 1 ] = 0 ; perm [ 2 ] = 0 ; perm
[ 3 ] = 0 ; if ( ib == 1 ) { perm [ 0 ] = 1 ; } else if ( ib == 2 ) { if ( x4
[ 0 ] <= x4 [ 1 ] ) { perm [ 0 ] = 1 ; perm [ 1 ] = 2 ; } else { perm [ 0 ] =
2 ; perm [ 1 ] = 1 ; } } else if ( x4 [ 0 ] <= x4 [ 1 ] ) { if ( x4 [ 1 ] <=
x4 [ 2 ] ) { perm [ 0 ] = 1 ; perm [ 1 ] = 2 ; perm [ 2 ] = 3 ; } else if (
x4 [ 0 ] <= x4 [ 2 ] ) { perm [ 0 ] = 1 ; perm [ 1 ] = 3 ; perm [ 2 ] = 2 ; }
else { perm [ 0 ] = 3 ; perm [ 1 ] = 1 ; perm [ 2 ] = 2 ; } } else if ( x4 [
0 ] <= x4 [ 2 ] ) { perm [ 0 ] = 2 ; perm [ 1 ] = 1 ; perm [ 2 ] = 3 ; } else
if ( x4 [ 1 ] <= x4 [ 2 ] ) { perm [ 0 ] = 2 ; perm [ 1 ] = 3 ; perm [ 2 ] =
1 ; } else { perm [ 0 ] = 3 ; perm [ 1 ] = 2 ; perm [ 2 ] = 1 ; } i2 = (
uint8_T ) ib ; for ( i1 = 0 ; i1 < i2 ; i1 ++ ) { p = perm [ i1 ] ;
b_iwork_tmp = ( perm_p - ib ) + i1 ; idx -> data [ b_iwork_tmp ] = idx4 [ p -
1 ] ; b_x -> data [ b_iwork_tmp ] = x4 [ p - 1 ] ; } } i1 = nBlocks >> 1 ;
for ( ib = 0 ; ib < i1 ; ib ++ ) { wOffset = perm_p + ib ; i2 = idx -> data [
wOffset ] ; p = n - ib ; idx -> data [ wOffset ] = idx -> data [ p ] ; idx ->
data [ p ] = i2 ; b_x -> data [ wOffset ] = xwork -> data [ p ] ; b_x -> data
[ p ] = xwork -> data [ wOffset ] ; } if ( ( ( uint32_T ) nBlocks & 1U ) !=
0U ) { nBlocks = perm_p + i1 ; b_x -> data [ nBlocks ] = xwork -> data [
nBlocks ] ; } ceougeaq11 ( & iwork , 1 ) ; b_iwork_tmp = iwork -> size [ 0 ]
; iwork -> size [ 0 ] = x -> size [ 1 ] ; dfntiycgzz ( iwork , b_iwork_tmp )
; if ( loop_ub - 1 >= 0 ) { memset ( & iwork -> data [ 0 ] , 0 , ( uint32_T )
loop_ub * sizeof ( int32_T ) ) ; } wOffset = 2 ; if ( perm_p > 1 ) { if ( x
-> size [ 1 ] >= 256 ) { nBlocks = perm_p >> 8 ; if ( nBlocks > 0 ) { for (
wOffset = 0 ; wOffset < nBlocks ; wOffset ++ ) { i2 = ( wOffset << 8 ) - 1 ;
for ( ib = 0 ; ib < 6 ; ib ++ ) { i3 = 1 << ( ib + 2 ) ; i4 = i3 << 1 ; d =
256 >> ( ib + 3 ) ; for ( i1 = 0 ; i1 < d ; i1 ++ ) { n = i1 * i4 + i2 ; for
( p = 0 ; p < i4 ; p ++ ) { b_iwork_tmp = ( n + p ) + 1 ; b_iwork [ p ] = idx
-> data [ b_iwork_tmp ] ; b_xwork [ p ] = b_x -> data [ b_iwork_tmp ] ; } p =
0 ; q = i3 ; do { exitg1 = 0 ; n ++ ; if ( b_xwork [ p ] <= b_xwork [ q ] ) {
idx -> data [ n ] = b_iwork [ p ] ; b_x -> data [ n ] = b_xwork [ p ] ; if (
p + 1 < i3 ) { p ++ ; } else { exitg1 = 1 ; } } else { idx -> data [ n ] =
b_iwork [ q ] ; b_x -> data [ n ] = b_xwork [ q ] ; if ( q + 1 < i4 ) { q ++
; } else { q = n - p ; for ( n = p + 1 ; n <= i3 ; n ++ ) { b_iwork_tmp = q +
n ; idx -> data [ b_iwork_tmp ] = b_iwork [ n - 1 ] ; b_x -> data [
b_iwork_tmp ] = b_xwork [ n - 1 ] ; } exitg1 = 1 ; } } } while ( exitg1 == 0
) ; } } } nBlocks <<= 8 ; wOffset = perm_p - nBlocks ; if ( wOffset > 0 ) {
b_iwork_tmp = iwork -> size [ 0 ] ; iwork -> size [ 0 ] = x -> size [ 1 ] ;
dfntiycgzz ( iwork , b_iwork_tmp ) ; if ( loop_ub - 1 >= 0 ) { memset ( &
iwork -> data [ 0 ] , 0 , ( uint32_T ) loop_ub * sizeof ( int32_T ) ) ; }
cumpzj3gf0 ( idx , b_x , nBlocks , wOffset , 2 , iwork , xwork ) ; } wOffset
= 8 ; } } cumpzj3gf0 ( idx , b_x , 0 , perm_p , wOffset , iwork , xwork ) ; }
njqibisfwd ( & xwork ) ; c2nfvjswuo ( & iwork ) ; b_iwork_tmp = x -> size [ 0
] * x -> size [ 1 ] ; x -> size [ 0 ] = 1 ; clokujid0y ( x , b_iwork_tmp ) ;
loop_ub = b_x -> size [ 1 ] ; b_iwork_tmp = x -> size [ 0 ] * x -> size [ 1 ]
; x -> size [ 1 ] = b_x -> size [ 1 ] ; clokujid0y ( x , b_iwork_tmp ) ; if (
loop_ub - 1 >= 0 ) { memcpy ( & x -> data [ 0 ] , & b_x -> data [ 0 ] , (
uint32_T ) loop_ub * sizeof ( real_T ) ) ; } njqibisfwd ( & b_x ) ; } }
static void eb1nrnik44 ( k0shasdzrq * x , dxycgom5nk * idx ) { e0juex3elc ( x
, idx ) ; } static boolean_T btdzbryr1o ( const real_T p1 [ 2 ] , const
real_T p2 [ 2 ] , const boolean_T map [ 3600 ] , const real_T gridLocation [
2 ] ) { __m128d tmp_p ; real_T xp_data [ 184 ] ; real_T yp_data [ 184 ] ;
real_T endPtX_data [ 4 ] ; real_T endPtY_data [ 4 ] ; real_T tmp [ 2 ] ;
real_T b_y0 ; real_T b_y_tmp_tmp ; real_T c_y_tmp_tmp ; real_T dtx ; real_T
dty ; real_T i ; real_T iter ; real_T n ; real_T txNext ; real_T tyNext ;
real_T x ; real_T x0 ; real_T xInc ; real_T y ; real_T yInc ; int32_T c_p ;
int32_T endPtX_size_idx_0 ; boolean_T endClipped ; boolean_T exitg1 ;
boolean_T p ; p = true ; tmp_p = _mm_loadu_pd ( & gridLocation [ 0 ] ) ;
_mm_storeu_pd ( & tmp [ 0 ] , _mm_sub_pd ( _mm_loadu_pd ( & p1 [ 0 ] ) ,
tmp_p ) ) ; x0 = tmp [ 0 ] ; b_y0 = tmp [ 1 ] ; b_y_tmp_tmp =
muDoubleScalarFloor ( tmp [ 0 ] ) ; c_y_tmp_tmp = muDoubleScalarFloor ( tmp [
1 ] ) ; x = b_y_tmp_tmp + 1.0 ; y = c_y_tmp_tmp + 1.0 ; _mm_storeu_pd ( & tmp
[ 0 ] , _mm_sub_pd ( _mm_loadu_pd ( & p2 [ 0 ] ) , tmp_p ) ) ;
getTruncatedIncrements_6j1H9bPB ( x0 , tmp [ 0 ] , b_y0 , tmp [ 1 ] , &
endClipped , & n , & xInc , & yInc , & dtx , & dty , & txNext , & tyNext ) ;
if ( n != 0.0 ) { for ( c_p = 0 ; c_p < 184 ; c_p ++ ) { xp_data [ c_p ] = (
rtNaN ) ; yp_data [ c_p ] = ( rtNaN ) ; } xp_data [ 0 ] = b_y_tmp_tmp + 1.0 ;
yp_data [ 0 ] = c_y_tmp_tmp + 1.0 ; i = 1.0 ; iter = 1.0 ; while ( iter <= n
) { if ( muDoubleScalarAbs ( tyNext - txNext ) < 1.0E-15 ) { x += xInc ;
txNext += dtx ; y += yInc ; tyNext += dty ; xp_data [ ( int32_T ) ( i + 1.0 )
- 1 ] = x ; yp_data [ ( int32_T ) ( i + 1.0 ) - 1 ] = y - yInc ; xp_data [ (
int32_T ) ( i + 2.0 ) - 1 ] = x - xInc ; yp_data [ ( int32_T ) ( i + 2.0 ) -
1 ] = y ; xp_data [ ( int32_T ) ( i + 3.0 ) - 1 ] = x ; yp_data [ ( int32_T )
( i + 3.0 ) - 1 ] = y ; i += 3.0 ; iter += 2.0 ; } else { if ( tyNext <
txNext ) { y += yInc ; tyNext += dty ; xp_data [ ( int32_T ) ( i + 1.0 ) - 1
] = x ; yp_data [ ( int32_T ) ( i + 1.0 ) - 1 ] = y ; i ++ ; if ( ( txNext >
1.0E+10 ) && ( muDoubleScalarAbs ( muDoubleScalarRound ( x0 ) - x0 ) <=
eps_EMqudTFw ( x0 ) ) ) { xp_data [ ( int32_T ) ( i + 1.0 ) - 1 ] = x + xInc
; yp_data [ ( int32_T ) ( i + 1.0 ) - 1 ] = y ; i ++ ; } } else if ( tyNext >
txNext ) { x += xInc ; txNext += dtx ; xp_data [ ( int32_T ) ( i + 1.0 ) - 1
] = x ; yp_data [ ( int32_T ) ( i + 1.0 ) - 1 ] = y ; i ++ ; if ( ( tyNext >
1.0E+10 ) && ( muDoubleScalarAbs ( muDoubleScalarRound ( b_y0 ) - b_y0 ) <=
eps_EMqudTFw ( b_y0 ) ) ) { xp_data [ ( int32_T ) ( i + 1.0 ) - 1 ] = x ;
yp_data [ ( int32_T ) ( i + 1.0 ) - 1 ] = y + yInc ; i ++ ; } } iter ++ ; } }
} else { i = 1.0 ; xp_data [ 0 ] = b_y_tmp_tmp + 1.0 ; yp_data [ 0 ] =
c_y_tmp_tmp + 1.0 ; } if ( endClipped ) { endPtX_size_idx_0 = 0 ; } else { n
= muDoubleScalarFloor ( tmp [ 0 ] ) ; x = 2.0 * eps_EMqudTFw ( tmp [ 0 ] ) ;
if ( muDoubleScalarAbs ( tmp [ 0 ] - n ) <= x ) { xInc = muDoubleScalarFloor
( tmp [ 1 ] ) ; x = 2.0 * eps_EMqudTFw ( tmp [ 1 ] ) ; if ( muDoubleScalarAbs
( tmp [ 1 ] - xInc ) <= x ) { endPtX_size_idx_0 = 4 ; endPtX_data [ 0 ] = n ;
endPtY_data [ 0 ] = xInc + 1.0 ; endPtX_data [ 1 ] = n + 1.0 ; endPtY_data [
1 ] = xInc ; endPtX_data [ 2 ] = n ; endPtY_data [ 2 ] = xInc ; endPtX_data [
3 ] = n + 1.0 ; endPtY_data [ 3 ] = xInc + 1.0 ; } else { y =
muDoubleScalarCeil ( tmp [ 1 ] ) ; if ( muDoubleScalarAbs ( tmp [ 1 ] - y )
<= x ) { endPtX_size_idx_0 = 4 ; endPtX_data [ 0 ] = n ; endPtY_data [ 0 ] =
y + 1.0 ; endPtX_data [ 1 ] = n + 1.0 ; endPtY_data [ 1 ] = y ; endPtX_data [
2 ] = n ; endPtY_data [ 2 ] = y ; endPtX_data [ 3 ] = n + 1.0 ; endPtY_data [
3 ] = y + 1.0 ; } else { endPtX_size_idx_0 = 1 ; endPtX_data [ 0 ] = n ;
endPtY_data [ 0 ] = xInc + 1.0 ; } } } else { xInc = muDoubleScalarCeil ( tmp
[ 0 ] ) ; if ( muDoubleScalarAbs ( tmp [ 0 ] - xInc ) <= x ) { x =
muDoubleScalarFloor ( tmp [ 1 ] ) ; y = eps_EMqudTFw ( tmp [ 1 ] ) ; if (
muDoubleScalarAbs ( tmp [ 1 ] - x ) <= y ) { endPtX_size_idx_0 = 4 ;
endPtX_data [ 0 ] = xInc ; endPtY_data [ 0 ] = x + 1.0 ; endPtX_data [ 1 ] =
xInc + 1.0 ; endPtY_data [ 1 ] = x ; endPtX_data [ 2 ] = xInc ; endPtY_data [
2 ] = x ; endPtX_data [ 3 ] = xInc + 1.0 ; endPtY_data [ 3 ] = x + 1.0 ; }
else { n = muDoubleScalarCeil ( tmp [ 1 ] ) ; if ( muDoubleScalarAbs ( tmp [
1 ] - n ) <= 2.0 * y ) { endPtX_size_idx_0 = 4 ; endPtX_data [ 0 ] = xInc ;
endPtY_data [ 0 ] = n + 1.0 ; endPtX_data [ 1 ] = xInc + 1.0 ; endPtY_data [
1 ] = n ; endPtX_data [ 2 ] = xInc ; endPtY_data [ 2 ] = n ; endPtX_data [ 3
] = xInc + 1.0 ; endPtY_data [ 3 ] = n + 1.0 ; } else { endPtX_size_idx_0 = 1
; endPtX_data [ 0 ] = xInc ; endPtY_data [ 0 ] = x + 1.0 ; } } } else { x =
muDoubleScalarFloor ( tmp [ 1 ] ) ; y = 2.0 * eps_EMqudTFw ( tmp [ 1 ] ) ; if
( muDoubleScalarAbs ( tmp [ 1 ] - x ) <= y ) { endPtX_size_idx_0 = 1 ;
endPtX_data [ 0 ] = n + 1.0 ; endPtY_data [ 0 ] = x ; } else { x =
muDoubleScalarCeil ( tmp [ 1 ] ) ; if ( muDoubleScalarAbs ( tmp [ 1 ] - x )
<= y ) { endPtX_size_idx_0 = 1 ; endPtX_data [ 0 ] = n + 1.0 ; endPtY_data [
0 ] = x + 1.0 ; } else { endPtX_size_idx_0 = 1 ; endPtX_data [ 0 ] = xp_data
[ ( int32_T ) i - 1 ] ; endPtY_data [ 0 ] = yp_data [ ( int32_T ) i - 1 ] ; }
} } } } c_p = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( c_p <= ( int32_T
) i - 1 ) ) { if ( ( xp_data [ c_p ] >= 1.0 ) && ( xp_data [ c_p ] <= 60.0 )
&& ( yp_data [ c_p ] >= 1.0 ) && ( yp_data [ c_p ] <= 60.0 ) && map [ (
int32_T ) ( ( xp_data [ c_p ] - 1.0 ) * 60.0 + ( 61.0 - yp_data [ c_p ] ) ) -
1 ] ) { p = false ; exitg1 = true ; } else { c_p ++ ; } } if ( p ) { c_p = 0
; exitg1 = false ; while ( ( ! exitg1 ) && ( c_p <= endPtX_size_idx_0 - 1 ) )
{ if ( ( endPtX_data [ c_p ] >= 1.0 ) && ( endPtX_data [ c_p ] <= 60.0 ) && (
endPtY_data [ c_p ] >= 1.0 ) && ( endPtY_data [ c_p ] <= 60.0 ) && map [ (
int32_T ) ( ( endPtX_data [ c_p ] - 1.0 ) * 60.0 + ( 61.0 - endPtY_data [ c_p
] ) ) - 1 ] ) { p = false ; exitg1 = true ; } else { c_p ++ ; } } } if ( p )
{ x = 2.0 * eps_EMqudTFw ( x0 ) ; if ( muDoubleScalarAbs ( x0 - b_y_tmp_tmp )
<= x ) { x = 2.0 * eps_EMqudTFw ( b_y0 ) ; if ( muDoubleScalarAbs ( b_y0 -
c_y_tmp_tmp ) <= x ) { endPtX_size_idx_0 = 4 ; endPtY_data [ 0 ] =
b_y_tmp_tmp ; endPtX_data [ 0 ] = c_y_tmp_tmp + 1.0 ; endPtY_data [ 1 ] =
b_y_tmp_tmp + 1.0 ; endPtX_data [ 1 ] = c_y_tmp_tmp ; endPtY_data [ 2 ] =
b_y_tmp_tmp ; endPtX_data [ 2 ] = c_y_tmp_tmp ; endPtY_data [ 3 ] =
b_y_tmp_tmp + 1.0 ; endPtX_data [ 3 ] = c_y_tmp_tmp + 1.0 ; } else { x0 =
muDoubleScalarCeil ( b_y0 ) ; if ( muDoubleScalarAbs ( b_y0 - x0 ) <= x ) {
endPtX_size_idx_0 = 4 ; endPtY_data [ 0 ] = b_y_tmp_tmp ; endPtX_data [ 0 ] =
x0 + 1.0 ; endPtY_data [ 1 ] = b_y_tmp_tmp + 1.0 ; endPtX_data [ 1 ] = x0 ;
endPtY_data [ 2 ] = b_y_tmp_tmp ; endPtX_data [ 2 ] = x0 ; endPtY_data [ 3 ]
= b_y_tmp_tmp + 1.0 ; endPtX_data [ 3 ] = x0 + 1.0 ; } else {
endPtX_size_idx_0 = 1 ; endPtY_data [ 0 ] = b_y_tmp_tmp ; endPtX_data [ 0 ] =
c_y_tmp_tmp + 1.0 ; } } } else { i = muDoubleScalarCeil ( x0 ) ; if (
muDoubleScalarAbs ( x0 - i ) <= x ) { x = eps_EMqudTFw ( b_y0 ) ; if (
muDoubleScalarAbs ( b_y0 - c_y_tmp_tmp ) <= x ) { endPtX_size_idx_0 = 4 ;
endPtY_data [ 0 ] = i ; endPtX_data [ 0 ] = c_y_tmp_tmp + 1.0 ; endPtY_data [
1 ] = i + 1.0 ; endPtX_data [ 1 ] = c_y_tmp_tmp ; endPtY_data [ 2 ] = i ;
endPtX_data [ 2 ] = c_y_tmp_tmp ; endPtY_data [ 3 ] = i + 1.0 ; endPtX_data [
3 ] = c_y_tmp_tmp + 1.0 ; } else { y = muDoubleScalarCeil ( b_y0 ) ; if (
muDoubleScalarAbs ( b_y0 - y ) <= 2.0 * x ) { endPtX_size_idx_0 = 4 ;
endPtY_data [ 0 ] = i ; endPtX_data [ 0 ] = y + 1.0 ; endPtY_data [ 1 ] = i +
1.0 ; endPtX_data [ 1 ] = y ; endPtY_data [ 2 ] = i ; endPtX_data [ 2 ] = y ;
endPtY_data [ 3 ] = i + 1.0 ; endPtX_data [ 3 ] = y + 1.0 ; } else {
endPtX_size_idx_0 = 1 ; endPtY_data [ 0 ] = i ; endPtX_data [ 0 ] =
c_y_tmp_tmp + 1.0 ; } } } else { x = 2.0 * eps_EMqudTFw ( b_y0 ) ; if (
muDoubleScalarAbs ( b_y0 - c_y_tmp_tmp ) <= x ) { endPtX_size_idx_0 = 1 ;
endPtY_data [ 0 ] = b_y_tmp_tmp + 1.0 ; endPtX_data [ 0 ] = c_y_tmp_tmp ; }
else { y = muDoubleScalarCeil ( b_y0 ) ; if ( muDoubleScalarAbs ( b_y0 - y )
<= x ) { endPtX_size_idx_0 = 1 ; endPtY_data [ 0 ] = b_y_tmp_tmp + 1.0 ;
endPtX_data [ 0 ] = y + 1.0 ; } else { endPtX_size_idx_0 = 0 ; } } } } c_p =
0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( c_p <= endPtX_size_idx_0 - 1 )
) { if ( ( endPtY_data [ c_p ] >= 1.0 ) && ( endPtY_data [ c_p ] <= 60.0 ) &&
( endPtX_data [ c_p ] >= 1.0 ) && ( endPtX_data [ c_p ] <= 60.0 ) && map [ (
int32_T ) ( ( endPtY_data [ c_p ] - 1.0 ) * 60.0 + ( 61.0 - endPtX_data [ c_p
] ) ) - 1 ] ) { p = false ; exitg1 = true ; } else { c_p ++ ; } } } return p
; } static void p3g0upgfmv ( oxn0khgesl * obj , real_T node1 , real_T node2 ,
real_T w ) { dxycgom5nk * idx ; dxycgom5nk * iwork ; dxycgom5nk * tmp ;
k0shasdzrq * subNodeLabelSet ; real_T ldom ; real_T lsub ; real_T tmp_p ;
int32_T b_j ; int32_T d_k ; int32_T i ; int32_T i2 ; int32_T kEnd ; int32_T n
; int32_T na ; int32_T na_tmp ; int32_T p ; int32_T pEnd ; int32_T q ;
int32_T qEnd ; boolean_T exitg1 ; obj -> AdjacencyMatrix -> data [ ( (
int32_T ) node1 + obj -> AdjacencyMatrix -> size [ 0 ] * ( ( int32_T ) node2
- 1 ) ) - 1 ] = w ; obj -> AdjacencyMatrix -> data [ ( ( int32_T ) node2 +
obj -> AdjacencyMatrix -> size [ 0 ] * ( ( int32_T ) node1 - 1 ) ) - 1 ] = w
; lsub = obj -> NodeLabels -> data [ ( int32_T ) node2 - 1 ] ; tmp_p = obj ->
NodeLabels -> data [ ( int32_T ) node1 - 1 ] ; if ( lsub != tmp_p ) { ldom =
muDoubleScalarMin ( lsub , tmp_p ) ; lsub = muDoubleScalarMax ( lsub , tmp_p
) ; i2 = obj -> NodeLabels -> size [ 1 ] - 1 ; na = 0 ; for ( i = 0 ; i <= i2
; i ++ ) { if ( obj -> NodeLabels -> data [ i ] == lsub ) { na ++ ; } }
ceougeaq11 ( & idx , 2 ) ; pEnd = idx -> size [ 0 ] * idx -> size [ 1 ] ; idx
-> size [ 0 ] = 1 ; idx -> size [ 1 ] = na ; dfntiycgzz ( idx , pEnd ) ; na =
0 ; for ( i = 0 ; i <= i2 ; i ++ ) { if ( obj -> NodeLabels -> data [ i ] ==
lsub ) { idx -> data [ na ] = i ; na ++ ; } } i = idx -> size [ 1 ] - 1 ; for
( na = 0 ; na <= i ; na ++ ) { obj -> NodeLabels -> data [ idx -> data [ na ]
] = ldom ; } na_tmp = obj -> NodeLabels -> size [ 1 ] ; n = obj -> NodeLabels
-> size [ 1 ] + 1 ; pEnd = idx -> size [ 0 ] * idx -> size [ 1 ] ; idx ->
size [ 0 ] = 1 ; idx -> size [ 1 ] = obj -> NodeLabels -> size [ 1 ] ;
dfntiycgzz ( idx , pEnd ) ; if ( na_tmp - 1 >= 0 ) { memset ( & idx -> data [
0 ] , 0 , ( uint32_T ) na_tmp * sizeof ( int32_T ) ) ; } if ( obj ->
NodeLabels -> size [ 1 ] != 0 ) { pEnd = idx -> size [ 0 ] * idx -> size [ 1
] ; idx -> size [ 0 ] = 1 ; idx -> size [ 1 ] = obj -> NodeLabels -> size [ 1
] ; dfntiycgzz ( idx , pEnd ) ; if ( na_tmp - 1 >= 0 ) { memset ( & idx ->
data [ 0 ] , 0 , ( uint32_T ) na_tmp * sizeof ( int32_T ) ) ; } ceougeaq11 (
& iwork , 1 ) ; pEnd = iwork -> size [ 0 ] ; iwork -> size [ 0 ] = obj ->
NodeLabels -> size [ 1 ] ; dfntiycgzz ( iwork , pEnd ) ; for ( i = 1 ; i <=
i2 ; i += 2 ) { ldom = obj -> NodeLabels -> data [ i ] ; if ( ( obj ->
NodeLabels -> data [ i - 1 ] <= ldom ) || muDoubleScalarIsNaN ( ldom ) ) {
idx -> data [ i - 1 ] = i ; idx -> data [ i ] = i + 1 ; } else { idx -> data
[ i - 1 ] = i + 1 ; idx -> data [ i ] = i ; } } if ( ( ( uint32_T ) obj ->
NodeLabels -> size [ 1 ] & 1U ) != 0U ) { idx -> data [ obj -> NodeLabels ->
size [ 1 ] - 1 ] = obj -> NodeLabels -> size [ 1 ] ; } i = 2 ; while ( i < n
- 1 ) { i2 = i << 1 ; b_j = 1 ; pEnd = i + 1 ; while ( pEnd < n ) { p = b_j ;
q = pEnd - 1 ; qEnd = b_j + i2 ; if ( qEnd > n ) { qEnd = n ; } d_k = 0 ;
kEnd = qEnd - b_j ; while ( d_k + 1 <= kEnd ) { lsub = obj -> NodeLabels ->
data [ idx -> data [ q ] - 1 ] ; na = idx -> data [ p - 1 ] ; if ( ( obj ->
NodeLabels -> data [ na - 1 ] <= lsub ) || muDoubleScalarIsNaN ( lsub ) ) {
iwork -> data [ d_k ] = na ; p ++ ; if ( p == pEnd ) { while ( q + 1 < qEnd )
{ d_k ++ ; iwork -> data [ d_k ] = idx -> data [ q ] ; q ++ ; } } } else {
iwork -> data [ d_k ] = idx -> data [ q ] ; q ++ ; if ( q + 1 == qEnd ) {
while ( p < pEnd ) { d_k ++ ; iwork -> data [ d_k ] = idx -> data [ p - 1 ] ;
p ++ ; } } } d_k ++ ; } for ( pEnd = 0 ; pEnd < kEnd ; pEnd ++ ) { idx ->
data [ ( b_j + pEnd ) - 1 ] = iwork -> data [ pEnd ] ; } b_j = qEnd ; pEnd =
qEnd + i ; } i = i2 ; } c2nfvjswuo ( & iwork ) ; } k5xyyjjkd4 ( &
subNodeLabelSet , 2 ) ; pEnd = subNodeLabelSet -> size [ 0 ] *
subNodeLabelSet -> size [ 1 ] ; subNodeLabelSet -> size [ 0 ] = 1 ;
subNodeLabelSet -> size [ 1 ] = obj -> NodeLabels -> size [ 1 ] ; clokujid0y
( subNodeLabelSet , pEnd ) ; for ( i = 0 ; i < na_tmp ; i ++ ) {
subNodeLabelSet -> data [ i ] = obj -> NodeLabels -> data [ idx -> data [ i ]
- 1 ] ; } c2nfvjswuo ( & idx ) ; i = 0 ; while ( ( i + 1 <= na_tmp ) &&
muDoubleScalarIsInf ( subNodeLabelSet -> data [ i ] ) && ( subNodeLabelSet ->
data [ i ] < 0.0 ) ) { i ++ ; } na = i ; i = obj -> NodeLabels -> size [ 1 ]
; while ( ( i >= 1 ) && muDoubleScalarIsNaN ( subNodeLabelSet -> data [ i - 1
] ) ) { i -- ; } i2 = obj -> NodeLabels -> size [ 1 ] - i ; exitg1 = false ;
while ( ( ! exitg1 ) && ( i >= 1 ) ) { lsub = subNodeLabelSet -> data [ i - 1
] ; if ( muDoubleScalarIsInf ( lsub ) && ( lsub > 0.0 ) ) { i -- ; } else {
exitg1 = true ; } } qEnd = ( obj -> NodeLabels -> size [ 1 ] - i ) - i2 ; b_j
= - 1 ; if ( na > 0 ) { b_j = 0 ; } while ( na + 1 <= i ) { ldom =
subNodeLabelSet -> data [ na ] ; do { na ++ ; } while ( ! ( ( na + 1 > i ) ||
( subNodeLabelSet -> data [ na ] != ldom ) ) ) ; b_j ++ ; subNodeLabelSet ->
data [ b_j ] = ldom ; } if ( qEnd > 0 ) { b_j ++ ; subNodeLabelSet -> data [
b_j ] = subNodeLabelSet -> data [ i ] ; } na = i + qEnd ; for ( i = 0 ; i <
i2 ; i ++ ) { subNodeLabelSet -> data [ ( b_j + i ) + 1 ] = subNodeLabelSet
-> data [ na + i ] ; } if ( i2 - 1 >= 0 ) { b_j += i2 ; } if ( b_j + 1 < 1 )
{ na = 0 ; subNodeLabelSet -> size [ 1 ] = 0 ; } else { na = b_j + 1 ; pEnd =
subNodeLabelSet -> size [ 0 ] * subNodeLabelSet -> size [ 1 ] ;
subNodeLabelSet -> size [ 1 ] = b_j + 1 ; clokujid0y ( subNodeLabelSet , pEnd
) ; } i2 = na - 1 ; na = 0 ; for ( i = 0 ; i <= i2 ; i ++ ) { if ( !
muDoubleScalarIsNaN ( subNodeLabelSet -> data [ i ] ) ) { na ++ ; } } obj ->
NumComponents = na ; i = obj -> AdjacencyMatrix -> size [ 0 ] ; pEnd = obj ->
NodeLabelSet -> size [ 0 ] * obj -> NodeLabelSet -> size [ 1 ] ; obj ->
NodeLabelSet -> size [ 0 ] = 1 ; obj -> NodeLabelSet -> size [ 1 ] = obj ->
AdjacencyMatrix -> size [ 0 ] ; clokujid0y ( obj -> NodeLabelSet , pEnd ) ;
for ( na = 0 ; na < i ; na ++ ) { obj -> NodeLabelSet -> data [ na ] = (
rtNaN ) ; } if ( obj -> NumComponents < 1.0 ) { n = 0 ; } else { n = (
int32_T ) obj -> NumComponents ; } na = 0 ; for ( i = 0 ; i <= i2 ; i ++ ) {
if ( ! muDoubleScalarIsNaN ( subNodeLabelSet -> data [ i ] ) ) { na ++ ; } }
ceougeaq11 ( & tmp , 2 ) ; pEnd = tmp -> size [ 0 ] * tmp -> size [ 1 ] ; tmp
-> size [ 0 ] = 1 ; tmp -> size [ 1 ] = na ; dfntiycgzz ( tmp , pEnd ) ; na =
0 ; for ( i = 0 ; i <= i2 ; i ++ ) { if ( ! muDoubleScalarIsNaN (
subNodeLabelSet -> data [ i ] ) ) { tmp -> data [ na ] = i ; na ++ ; } } for
( na = 0 ; na < n ; na ++ ) { obj -> NodeLabelSet -> data [ na ] =
subNodeLabelSet -> data [ tmp -> data [ na ] ] ; } c2nfvjswuo ( & tmp ) ;
njqibisfwd ( & subNodeLabelSet ) ; } } static void oung1l0hdz ( mgywmw0rjk *
obj , lwnhcfk22o * localDW ) { __m128d tmp_f ; b4axleo1v0 * idx ; b4axleo1v0
* originIdx ; b4axleo1v0 * sameNode ; dxycgom5nk * iidx ; dxycgom5nk * tmp ;
dxycgom5nk * tmp_e ; dxycgom5nk * tmp_g ; dxycgom5nk * tmp_i ; dxycgom5nk *
tmp_m ; dxycgom5nk * tmp_p ; k0shasdzrq * b_x ; k0shasdzrq * b_y ; k0shasdzrq
* c_y ; k0shasdzrq * e_y ; k0shasdzrq * gridXY ; k0shasdzrq * nodeList ;
k0shasdzrq * x ; k0shasdzrq * xrand ; k0shasdzrq * xyrand ; k0shasdzrq * y ;
k0shasdzrq * yrand ; pwx2zp040p * varargin_1 ; real_T a [ 4 ] ; real_T
location [ 2 ] ; real_T nearbyNode [ 2 ] ; real_T newNode [ 2 ] ; real_T
tmp_j [ 2 ] ; real_T absx ; real_T connectionDistance ; real_T newNodeID ;
real_T toFill ; real_T varargin_1_p ; real_T xl ; real_T xlims ; real_T ylims
; int32_T q_size [ 2 ] ; int32_T c_nx ; int32_T ii_data_idx_0 ; int32_T j ;
int32_T lastind ; int32_T loop_ub ; int32_T loop_ub_tmp ; int32_T
loop_ub_tmp_p ; int32_T obj_p ; int32_T q_data ; int32_T trueCount ; uint32_T
temp_data_idx_0 ; boolean_T grid [ 3600 ] ; boolean_T d_y ; boolean_T exitg1
; varargin_1 = obj -> InternalMap ; newNodeID = obj -> InternalNumNodes ;
connectionDistance = obj -> ConnectionDistance ; if ( obj -> RoadmapBuilder .
isInitialized != 1 ) { obj -> RoadmapBuilder . isInitialized = 1 ; } c_nx =
obj -> RoadmapBuilder . Roadmap . Nodes -> size [ 0 ] * obj -> RoadmapBuilder
. Roadmap . Nodes -> size [ 1 ] ; obj -> RoadmapBuilder . Roadmap . Nodes ->
size [ 0 ] = 2 ; clokujid0y ( obj -> RoadmapBuilder . Roadmap . Nodes , c_nx
) ; loop_ub_tmp_p = ( int32_T ) newNodeID ; c_nx = obj -> RoadmapBuilder .
Roadmap . Nodes -> size [ 0 ] * obj -> RoadmapBuilder . Roadmap . Nodes ->
size [ 1 ] ; obj -> RoadmapBuilder . Roadmap . Nodes -> size [ 1 ] = (
int32_T ) newNodeID ; clokujid0y ( obj -> RoadmapBuilder . Roadmap . Nodes ,
c_nx ) ; loop_ub = ( ( int32_T ) newNodeID << 1 ) - 1 ; for ( c_nx = 0 ; c_nx
<= loop_ub ; c_nx ++ ) { obj -> RoadmapBuilder . Roadmap . Nodes -> data [
c_nx ] = ( rtNaN ) ; } c_nx = obj -> RoadmapBuilder . Roadmap .
AdjacencyMatrix -> size [ 0 ] * obj -> RoadmapBuilder . Roadmap .
AdjacencyMatrix -> size [ 1 ] ; obj -> RoadmapBuilder . Roadmap .
AdjacencyMatrix -> size [ 0 ] = ( int32_T ) newNodeID ; obj -> RoadmapBuilder
. Roadmap . AdjacencyMatrix -> size [ 1 ] = ( int32_T ) newNodeID ;
clokujid0y ( obj -> RoadmapBuilder . Roadmap . AdjacencyMatrix , c_nx ) ;
loop_ub = ( int32_T ) newNodeID * ( int32_T ) newNodeID - 1 ; for ( c_nx = 0
; c_nx <= loop_ub ; c_nx ++ ) { obj -> RoadmapBuilder . Roadmap .
AdjacencyMatrix -> data [ c_nx ] = 0.0 ; } c_nx = obj -> RoadmapBuilder .
Roadmap . NodeLabelSet -> size [ 0 ] * obj -> RoadmapBuilder . Roadmap .
NodeLabelSet -> size [ 1 ] ; obj -> RoadmapBuilder . Roadmap . NodeLabelSet
-> size [ 0 ] = 1 ; obj -> RoadmapBuilder . Roadmap . NodeLabelSet -> size [
1 ] = ( int32_T ) newNodeID ; clokujid0y ( obj -> RoadmapBuilder . Roadmap .
NodeLabelSet , c_nx ) ; loop_ub_tmp = ( int32_T ) newNodeID - 1 ; for ( c_nx
= 0 ; c_nx <= loop_ub_tmp ; c_nx ++ ) { obj -> RoadmapBuilder . Roadmap .
NodeLabelSet -> data [ c_nx ] = ( rtNaN ) ; } c_nx = obj -> RoadmapBuilder .
Roadmap . NodeLabels -> size [ 0 ] * obj -> RoadmapBuilder . Roadmap .
NodeLabels -> size [ 1 ] ; obj -> RoadmapBuilder . Roadmap . NodeLabels ->
size [ 0 ] = 1 ; obj -> RoadmapBuilder . Roadmap . NodeLabels -> size [ 1 ] =
( int32_T ) newNodeID ; clokujid0y ( obj -> RoadmapBuilder . Roadmap .
NodeLabels , c_nx ) ; for ( c_nx = 0 ; c_nx <= loop_ub_tmp ; c_nx ++ ) { obj
-> RoadmapBuilder . Roadmap . NodeLabels -> data [ c_nx ] = ( rtNaN ) ; } obj
-> RoadmapBuilder . Roadmap . CurrentLabel = 0.0 ; obj -> RoadmapBuilder .
Roadmap . NumComponents = 0.0 ; obj -> RoadmapBuilder . Roadmap . IsEuclidean
= true ; mkwt2qmyjw ( varargin_1 , grid ) ; tmp_f = _mm_add_pd ( _mm_set1_pd
( varargin_1 -> SharedProperties . LocalOriginInWorld [ 0 ] ) , _mm_set1_pd (
varargin_1 -> SharedProperties . GridOriginInLocal [ 0 ] ) ) ; _mm_storeu_pd
( & tmp_j [ 0 ] , tmp_f ) ; xl = tmp_j [ 0 ] ; absx = tmp_j [ 1 ] ; tmp_f =
_mm_add_pd ( _mm_set1_pd ( varargin_1 -> SharedProperties .
LocalOriginInWorld [ 1 ] ) , _mm_set1_pd ( varargin_1 -> SharedProperties .
GridOriginInLocal [ 1 ] ) ) ; _mm_storeu_pd ( & tmp_j [ 0 ] , tmp_f ) ;
varargin_1_p = tmp_j [ 0 ] ; k5xyyjjkd4 ( & x , 1 ) ; c_nx = x -> size [ 0 ]
; x -> size [ 0 ] = ( int32_T ) newNodeID ; clokujid0y ( x , c_nx ) ;
k5xyyjjkd4 ( & y , 1 ) ; c_nx = y -> size [ 0 ] ; y -> size [ 0 ] = ( int32_T
) newNodeID ; clokujid0y ( y , c_nx ) ; for ( c_nx = 0 ; c_nx < loop_ub_tmp_p
; c_nx ++ ) { x -> data [ c_nx ] = 0.0 ; y -> data [ c_nx ] = 0.0 ; } lastind
= 0 ; if ( newNodeID > 0.0 ) { xlims = ( absx + 60.0 ) - xl ; ylims = ( tmp_j
[ 1 ] + 60.0 ) - tmp_j [ 0 ] ; } k5xyyjjkd4 ( & xyrand , 2 ) ; k5xyyjjkd4 ( &
xrand , 1 ) ; k5xyyjjkd4 ( & yrand , 1 ) ; ib42mb0qrj ( & idx , 1 ) ;
k5xyyjjkd4 ( & gridXY , 2 ) ; ib42mb0qrj ( & originIdx , 2 ) ; k5xyyjjkd4 ( &
e_y , 2 ) ; ceougeaq11 ( & tmp_m , 1 ) ; ceougeaq11 ( & tmp_g , 1 ) ; while (
lastind < newNodeID ) { diadlcmegy ( newNodeID , xyrand , localDW ) ; loop_ub
= xyrand -> size [ 0 ] ; c_nx = xrand -> size [ 0 ] ; xrand -> size [ 0 ] =
xyrand -> size [ 0 ] ; clokujid0y ( xrand , c_nx ) ; c_nx = yrand -> size [ 0
] ; yrand -> size [ 0 ] = xyrand -> size [ 0 ] ; clokujid0y ( yrand , c_nx )
; lastind = ( xyrand -> size [ 0 ] / 2 ) << 1 ; loop_ub_tmp = lastind - 2 ;
for ( c_nx = 0 ; c_nx <= loop_ub_tmp ; c_nx += 2 ) { tmp_f = _mm_loadu_pd ( &
xyrand -> data [ c_nx ] ) ; _mm_storeu_pd ( & xrand -> data [ c_nx ] ,
_mm_add_pd ( _mm_mul_pd ( _mm_set1_pd ( xlims ) , tmp_f ) , _mm_set1_pd ( xl
) ) ) ; tmp_f = _mm_loadu_pd ( & xyrand -> data [ c_nx + xyrand -> size [ 0 ]
] ) ; _mm_storeu_pd ( & yrand -> data [ c_nx ] , _mm_add_pd ( _mm_mul_pd (
_mm_set1_pd ( ylims ) , tmp_f ) , _mm_set1_pd ( varargin_1_p ) ) ) ; } for (
c_nx = lastind ; c_nx < loop_ub ; c_nx ++ ) { _mm_storeu_pd ( & tmp_j [ 0 ] ,
_mm_add_pd ( _mm_mul_pd ( _mm_set_pd ( ylims , xlims ) , _mm_set_pd ( xyrand
-> data [ c_nx + xyrand -> size [ 0 ] ] , xyrand -> data [ c_nx ] ) ) ,
_mm_set_pd ( varargin_1_p , xl ) ) ) ; xrand -> data [ c_nx ] = tmp_j [ 0 ] ;
yrand -> data [ c_nx ] = tmp_j [ 1 ] ; } location [ 0 ] = varargin_1 ->
SharedProperties . LocalOriginInWorld [ 0 ] ; location [ 1 ] = varargin_1 ->
SharedProperties . LocalOriginInWorld [ 1 ] ; loop_ub = xrand -> size [ 0 ] ;
c_nx = gridXY -> size [ 0 ] * gridXY -> size [ 1 ] ; gridXY -> size [ 0 ] =
xrand -> size [ 0 ] ; gridXY -> size [ 1 ] = 2 ; clokujid0y ( gridXY , c_nx )
; if ( loop_ub - 1 >= 0 ) { memcpy ( & gridXY -> data [ 0 ] , & xrand -> data
[ 0 ] , ( uint32_T ) loop_ub * sizeof ( real_T ) ) ; } trueCount = yrand ->
size [ 0 ] ; for ( c_nx = 0 ; c_nx < trueCount ; c_nx ++ ) { gridXY -> data [
c_nx + xrand -> size [ 0 ] ] = yrand -> data [ c_nx ] ; } c_nx = xyrand ->
size [ 0 ] * xyrand -> size [ 1 ] ; xyrand -> size [ 0 ] = xrand -> size [ 0
] ; xyrand -> size [ 1 ] = 2 ; clokujid0y ( xyrand , c_nx ) ; for ( c_nx = 0
; c_nx < 2 ; c_nx ++ ) { lastind = ( loop_ub / 2 ) << 1 ; loop_ub_tmp =
lastind - 2 ; for ( obj_p = 0 ; obj_p <= loop_ub_tmp ; obj_p += 2 ) { tmp_f =
_mm_loadu_pd ( & gridXY -> data [ gridXY -> size [ 0 ] * c_nx + obj_p ] ) ;
_mm_storeu_pd ( & xyrand -> data [ obj_p + xyrand -> size [ 0 ] * c_nx ] ,
_mm_sub_pd ( tmp_f , _mm_set1_pd ( location [ c_nx ] ) ) ) ; } for ( obj_p =
lastind ; obj_p < loop_ub ; obj_p ++ ) { xyrand -> data [ obj_p + xyrand ->
size [ 0 ] * c_nx ] = gridXY -> data [ gridXY -> size [ 0 ] * c_nx + obj_p ]
- location [ c_nx ] ; } } location [ 0 ] = varargin_1 -> SharedProperties .
GridOriginInLocal [ 0 ] ; location [ 1 ] = varargin_1 -> SharedProperties .
GridOriginInLocal [ 1 ] ; toFill = - location [ 1 ] ; absx = - location [ 0 ]
; loop_ub = xyrand -> size [ 0 ] ; c_nx = gridXY -> size [ 0 ] * gridXY ->
size [ 1 ] ; gridXY -> size [ 0 ] = xyrand -> size [ 0 ] ; gridXY -> size [ 1
] = 2 ; clokujid0y ( gridXY , c_nx ) ; lastind = ( xyrand -> size [ 0 ] / 2 )
<< 1 ; loop_ub_tmp = lastind - 2 ; for ( c_nx = 0 ; c_nx <= loop_ub_tmp ;
c_nx += 2 ) { tmp_f = _mm_loadu_pd ( & xyrand -> data [ c_nx + xyrand -> size
[ 0 ] ] ) ; _mm_storeu_pd ( & gridXY -> data [ c_nx ] , _mm_add_pd (
_mm_set1_pd ( toFill ) , tmp_f ) ) ; tmp_f = _mm_loadu_pd ( & xyrand -> data
[ c_nx ] ) ; _mm_storeu_pd ( & gridXY -> data [ c_nx + gridXY -> size [ 0 ] ]
, _mm_add_pd ( _mm_set1_pd ( absx ) , tmp_f ) ) ; } for ( c_nx = lastind ;
c_nx < loop_ub ; c_nx ++ ) { _mm_storeu_pd ( & tmp_j [ 0 ] , _mm_add_pd (
_mm_set_pd ( absx , toFill ) , _mm_set_pd ( xyrand -> data [ c_nx ] , xyrand
-> data [ c_nx + xyrand -> size [ 0 ] ] ) ) ) ; gridXY -> data [ c_nx ] =
tmp_j [ 0 ] ; gridXY -> data [ c_nx + gridXY -> size [ 0 ] ] = tmp_j [ 1 ] ;
} loop_ub = gridXY -> size [ 0 ] ; c_nx = xyrand -> size [ 0 ] * xyrand ->
size [ 1 ] ; xyrand -> size [ 0 ] = gridXY -> size [ 0 ] ; xyrand -> size [ 1
] = 2 ; clokujid0y ( xyrand , c_nx ) ; loop_ub_tmp = gridXY -> size [ 0 ] <<
1 ; if ( loop_ub_tmp - 1 >= 0 ) { memcpy ( & xyrand -> data [ 0 ] , & gridXY
-> data [ 0 ] , ( uint32_T ) loop_ub_tmp * sizeof ( real_T ) ) ; } a [ 0 ] =
muDoubleScalarAbs ( location [ 0 ] ) ; a [ 1 ] = muDoubleScalarAbs ( location
[ 0 ] + 60.0 ) ; a [ 2 ] = muDoubleScalarAbs ( location [ 1 ] ) ; a [ 3 ] =
muDoubleScalarAbs ( location [ 1 ] + 60.0 ) ; c_nx = e_y -> size [ 0 ] * e_y
-> size [ 1 ] ; e_y -> size [ 0 ] = gridXY -> size [ 0 ] ; e_y -> size [ 1 ]
= 2 ; clokujid0y ( e_y , c_nx ) ; for ( obj_p = 0 ; obj_p < loop_ub_tmp ;
obj_p ++ ) { xyrand -> data [ obj_p ] = muDoubleScalarCeil ( xyrand -> data [
obj_p ] ) ; e_y -> data [ obj_p ] = muDoubleScalarAbs ( gridXY -> data [
obj_p ] ) ; } absx = muDoubleScalarAbs ( maximum_w0hsPrr7 ( a ) ) ; if (
muDoubleScalarIsInf ( absx ) || muDoubleScalarIsNaN ( absx ) ) { absx = (
rtNaN ) ; } else if ( absx < 4.4501477170144028E-308 ) { absx =
4.94065645841247E-324 ; } else { frexp ( absx , & j ) ; absx = ldexp ( 1.0 ,
j - 53 ) ; } c_nx = originIdx -> size [ 0 ] * originIdx -> size [ 1 ] ;
originIdx -> size [ 0 ] = gridXY -> size [ 0 ] ; originIdx -> size [ 1 ] = 2
; mdiomdajqb ( originIdx , c_nx ) ; toFill = absx * 2.0 ; for ( c_nx = 0 ;
c_nx < loop_ub_tmp ; c_nx ++ ) { originIdx -> data [ c_nx ] = ( e_y -> data [
c_nx ] < toFill ) ; } d_y = false ; lastind = 1 ; exitg1 = false ; while ( (
! exitg1 ) && ( lastind <= loop_ub_tmp ) ) { if ( e_y -> data [ lastind - 1 ]
< absx * 2.0 ) { d_y = true ; exitg1 = true ; } else { lastind ++ ; } } if (
d_y ) { obj_p = loop_ub_tmp - 1 ; for ( c_nx = 0 ; c_nx <= obj_p ; c_nx ++ )
{ if ( originIdx -> data [ c_nx ] ) { xyrand -> data [ c_nx ] = 1.0 ; } } }
lastind = ( gridXY -> size [ 0 ] / 2 ) << 1 ; loop_ub_tmp = lastind - 2 ; for
( c_nx = 0 ; c_nx <= loop_ub_tmp ; c_nx += 2 ) { tmp_f = _mm_loadu_pd ( &
xyrand -> data [ c_nx ] ) ; _mm_storeu_pd ( & xyrand -> data [ c_nx ] ,
_mm_sub_pd ( _mm_set1_pd ( 61.0 ) , tmp_f ) ) ; } for ( c_nx = lastind ; c_nx
< loop_ub ; c_nx ++ ) { xyrand -> data [ c_nx ] = 61.0 - xyrand -> data [
c_nx ] ; } loop_ub = xyrand -> size [ 0 ] ; c_nx = idx -> size [ 0 ] ; idx ->
size [ 0 ] = xyrand -> size [ 0 ] ; mdiomdajqb ( idx , c_nx ) ; for ( c_nx =
0 ; c_nx < loop_ub ; c_nx ++ ) { idx -> data [ c_nx ] = ! grid [ ( ( (
int32_T ) xyrand -> data [ c_nx + xyrand -> size [ 0 ] ] - 1 ) * 60 + (
int32_T ) xyrand -> data [ c_nx ] ) - 1 ] ; } lastind = ( x -> size [ 0 ] >=
1 ) ; obj_p = ( int32_T ) newNodeID ; c_nx = 0 ; exitg1 = false ; while ( ( !
exitg1 ) && ( obj_p > 0 ) ) { if ( x -> data [ obj_p - 1 ] != 0.0 ) { c_nx =
1 ; ii_data_idx_0 = obj_p ; exitg1 = true ; } else { obj_p -- ; } } if (
lastind == 1 ) { if ( c_nx == 0 ) { lastind = 0 ; } } else { lastind = ( c_nx
>= 1 ) ; } if ( lastind - 1 >= 0 ) { temp_data_idx_0 = ( uint32_T )
ii_data_idx_0 ; } if ( lastind == 0 ) { lastind = 0 ; } else { lastind = (
int32_T ) temp_data_idx_0 ; } if ( idx -> size [ 0 ] == 0 ) { c_nx = 0 ; }
else { c_nx = idx -> data [ 0 ] ; for ( obj_p = 2 ; obj_p <= loop_ub ; obj_p
++ ) { c_nx += idx -> data [ obj_p - 1 ] ; } } if ( lastind < newNodeID ) {
absx = muDoubleScalarMin ( newNodeID , ( uint32_T ) lastind + ( uint32_T )
c_nx ) ; toFill = absx - ( real_T ) lastind ; if ( ( real_T ) lastind + 1.0 >
absx ) { obj_p = 0 ; } else { obj_p = lastind ; } loop_ub_tmp = idx -> size [
0 ] - 1 ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <= loop_ub_tmp ; c_nx ++ ) {
if ( idx -> data [ c_nx ] ) { trueCount ++ ; } } c_nx = tmp_m -> size [ 0 ] ;
tmp_m -> size [ 0 ] = trueCount ; dfntiycgzz ( tmp_m , c_nx ) ; trueCount = 0
; for ( c_nx = 0 ; c_nx <= loop_ub_tmp ; c_nx ++ ) { if ( idx -> data [ c_nx
] ) { tmp_m -> data [ trueCount ] = c_nx ; trueCount ++ ; } } if ( toFill <
1.0 ) { loop_ub = - 1 ; } else { loop_ub = ( int32_T ) toFill - 1 ; } for (
c_nx = 0 ; c_nx <= loop_ub ; c_nx ++ ) { x -> data [ obj_p + c_nx ] = xrand
-> data [ tmp_m -> data [ c_nx ] ] ; } if ( ( real_T ) lastind + 1.0 > absx )
{ obj_p = 0 ; } else { obj_p = lastind ; } trueCount = 0 ; for ( c_nx = 0 ;
c_nx <= loop_ub_tmp ; c_nx ++ ) { if ( idx -> data [ c_nx ] ) { trueCount ++
; } } c_nx = tmp_g -> size [ 0 ] ; tmp_g -> size [ 0 ] = trueCount ;
dfntiycgzz ( tmp_g , c_nx ) ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <=
loop_ub_tmp ; c_nx ++ ) { if ( idx -> data [ c_nx ] ) { tmp_g -> data [
trueCount ] = c_nx ; trueCount ++ ; } } if ( toFill < 1.0 ) { loop_ub = - 1 ;
} else { loop_ub = ( int32_T ) toFill - 1 ; } for ( c_nx = 0 ; c_nx <=
loop_ub ; c_nx ++ ) { y -> data [ obj_p + c_nx ] = yrand -> data [ tmp_g ->
data [ c_nx ] ] ; } } } c2nfvjswuo ( & tmp_g ) ; c2nfvjswuo ( & tmp_m ) ;
njqibisfwd ( & e_y ) ; fizihoqn3p ( & originIdx ) ; njqibisfwd ( & gridXY ) ;
fizihoqn3p ( & idx ) ; njqibisfwd ( & yrand ) ; njqibisfwd ( & xrand ) ;
njqibisfwd ( & xyrand ) ; tmp_f = _mm_add_pd ( _mm_loadu_pd ( & varargin_1 ->
SharedProperties . LocalOriginInWorld [ 0 ] ) , _mm_loadu_pd ( & varargin_1
-> SharedProperties . GridOriginInLocal [ 0 ] ) ) ; _mm_storeu_pd ( &
location [ 0 ] , tmp_f ) ; ib42mb0qrj ( & sameNode , 2 ) ; ceougeaq11 ( &
iidx , 2 ) ; k5xyyjjkd4 ( & b_y , 2 ) ; k5xyyjjkd4 ( & c_y , 2 ) ; k5xyyjjkd4
( & nodeList , 2 ) ; k5xyyjjkd4 ( & b_x , 2 ) ; ceougeaq11 ( & tmp_p , 2 ) ;
ceougeaq11 ( & tmp_e , 2 ) ; ceougeaq11 ( & tmp_i , 2 ) ; for ( j = 0 ; j <
loop_ub_tmp_p ; j ++ ) { newNode [ 0 ] = x -> data [ j ] ; newNode [ 1 ] = y
-> data [ j ] ; c_nx = b_x -> size [ 0 ] * b_x -> size [ 1 ] ; b_x -> size [
0 ] = 1 ; b_x -> size [ 1 ] = obj -> RoadmapBuilder . Roadmap . Nodes -> size
[ 1 ] ; clokujid0y ( b_x , c_nx ) ; loop_ub = obj -> RoadmapBuilder . Roadmap
. Nodes -> size [ 1 ] ; for ( c_nx = 0 ; c_nx < loop_ub ; c_nx ++ ) { b_x ->
data [ c_nx ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ c_nx << 1 ]
; } obj_p = b_x -> size [ 1 ] - 1 ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <=
obj_p ; c_nx ++ ) { if ( ! muDoubleScalarIsNaN ( b_x -> data [ c_nx ] ) ) {
trueCount ++ ; } } c_nx = tmp_p -> size [ 0 ] * tmp_p -> size [ 1 ] ; tmp_p
-> size [ 0 ] = 1 ; tmp_p -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_p ,
c_nx ) ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <= obj_p ; c_nx ++ ) { if ( !
muDoubleScalarIsNaN ( b_x -> data [ c_nx ] ) ) { tmp_p -> data [ trueCount ]
= c_nx ; trueCount ++ ; } } c_nx = nodeList -> size [ 0 ] * nodeList -> size
[ 1 ] ; nodeList -> size [ 0 ] = 2 ; loop_ub = tmp_p -> size [ 1 ] ; nodeList
-> size [ 1 ] = tmp_p -> size [ 1 ] ; clokujid0y ( nodeList , c_nx ) ; for (
c_nx = 0 ; c_nx < loop_ub ; c_nx ++ ) { obj_p = tmp_p -> data [ c_nx ] ;
nodeList -> data [ c_nx << 1 ] = obj -> RoadmapBuilder . Roadmap . Nodes ->
data [ obj_p << 1 ] ; nodeList -> data [ 1 + ( c_nx << 1 ) ] = obj ->
RoadmapBuilder . Roadmap . Nodes -> data [ ( obj_p << 1 ) + 1 ] ; } absx =
newNode [ 0 ] ; c_nx = b_x -> size [ 0 ] * b_x -> size [ 1 ] ; b_x -> size [
0 ] = 1 ; b_x -> size [ 1 ] = tmp_p -> size [ 1 ] ; clokujid0y ( b_x , c_nx )
; lastind = ( tmp_p -> size [ 1 ] / 2 ) << 1 ; loop_ub_tmp = lastind - 2 ;
for ( c_nx = 0 ; c_nx <= loop_ub_tmp ; c_nx += 2 ) { _mm_storeu_pd ( & tmp_j
[ 0 ] , _mm_sub_pd ( _mm_set_pd ( nodeList -> data [ ( c_nx + 1 ) << 1 ] ,
nodeList -> data [ c_nx << 1 ] ) , _mm_set1_pd ( absx ) ) ) ; b_x -> data [
c_nx ] = tmp_j [ 0 ] ; b_x -> data [ c_nx + 1 ] = tmp_j [ 1 ] ; } for ( c_nx
= lastind ; c_nx < loop_ub ; c_nx ++ ) { b_x -> data [ c_nx ] = nodeList ->
data [ c_nx << 1 ] - absx ; } c_nx = b_y -> size [ 0 ] * b_y -> size [ 1 ] ;
b_y -> size [ 0 ] = 1 ; b_y -> size [ 1 ] = tmp_p -> size [ 1 ] ; clokujid0y
( b_y , c_nx ) ; for ( c_nx = 0 ; c_nx < loop_ub ; c_nx ++ ) { b_y -> data [
c_nx ] = muDoubleScalarAbs ( b_x -> data [ c_nx ] ) ; } c_nx = b_x -> size [
0 ] * b_x -> size [ 1 ] ; b_x -> size [ 0 ] = 1 ; b_x -> size [ 1 ] = obj ->
RoadmapBuilder . Roadmap . Nodes -> size [ 1 ] ; clokujid0y ( b_x , c_nx ) ;
trueCount = obj -> RoadmapBuilder . Roadmap . Nodes -> size [ 1 ] ; for (
c_nx = 0 ; c_nx < trueCount ; c_nx ++ ) { b_x -> data [ c_nx ] = obj ->
RoadmapBuilder . Roadmap . Nodes -> data [ c_nx << 1 ] ; } obj_p = b_x ->
size [ 1 ] - 1 ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <= obj_p ; c_nx ++ ) {
if ( ! muDoubleScalarIsNaN ( b_x -> data [ c_nx ] ) ) { trueCount ++ ; } }
c_nx = tmp_e -> size [ 0 ] * tmp_e -> size [ 1 ] ; tmp_e -> size [ 0 ] = 1 ;
tmp_e -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_e , c_nx ) ; trueCount = 0
; for ( c_nx = 0 ; c_nx <= obj_p ; c_nx ++ ) { if ( ! muDoubleScalarIsNaN (
b_x -> data [ c_nx ] ) ) { tmp_e -> data [ trueCount ] = c_nx ; trueCount ++
; } } c_nx = nodeList -> size [ 0 ] * nodeList -> size [ 1 ] ; nodeList ->
size [ 0 ] = 2 ; trueCount = tmp_e -> size [ 1 ] ; nodeList -> size [ 1 ] =
tmp_e -> size [ 1 ] ; clokujid0y ( nodeList , c_nx ) ; for ( c_nx = 0 ; c_nx
< trueCount ; c_nx ++ ) { obj_p = tmp_e -> data [ c_nx ] ; nodeList -> data [
c_nx << 1 ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ obj_p << 1 ]
; nodeList -> data [ 1 + ( c_nx << 1 ) ] = obj -> RoadmapBuilder . Roadmap .
Nodes -> data [ ( obj_p << 1 ) + 1 ] ; } absx = newNode [ 1 ] ; c_nx = b_x ->
size [ 0 ] * b_x -> size [ 1 ] ; b_x -> size [ 0 ] = 1 ; b_x -> size [ 1 ] =
tmp_e -> size [ 1 ] ; clokujid0y ( b_x , c_nx ) ; lastind = ( tmp_e -> size [
1 ] / 2 ) << 1 ; loop_ub_tmp = lastind - 2 ; for ( c_nx = 0 ; c_nx <=
loop_ub_tmp ; c_nx += 2 ) { _mm_storeu_pd ( & tmp_j [ 0 ] , _mm_sub_pd (
_mm_set_pd ( nodeList -> data [ 1 + ( ( c_nx + 1 ) << 1 ) ] , nodeList ->
data [ 1 + ( c_nx << 1 ) ] ) , _mm_set1_pd ( absx ) ) ) ; b_x -> data [ c_nx
] = tmp_j [ 0 ] ; b_x -> data [ c_nx + 1 ] = tmp_j [ 1 ] ; } for ( c_nx =
lastind ; c_nx < trueCount ; c_nx ++ ) { b_x -> data [ c_nx ] = nodeList ->
data [ ( c_nx << 1 ) + 1 ] - absx ; } c_nx = c_y -> size [ 0 ] * c_y -> size
[ 1 ] ; c_y -> size [ 0 ] = 1 ; c_y -> size [ 1 ] = tmp_e -> size [ 1 ] ;
clokujid0y ( c_y , c_nx ) ; for ( c_nx = 0 ; c_nx < trueCount ; c_nx ++ ) {
c_y -> data [ c_nx ] = muDoubleScalarAbs ( b_x -> data [ c_nx ] ) ; } if (
b_y -> size [ 1 ] == c_y -> size [ 1 ] ) { c_nx = sameNode -> size [ 0 ] *
sameNode -> size [ 1 ] ; sameNode -> size [ 0 ] = 1 ; sameNode -> size [ 1 ]
= tmp_p -> size [ 1 ] ; mdiomdajqb ( sameNode , c_nx ) ; for ( c_nx = 0 ;
c_nx < loop_ub ; c_nx ++ ) { sameNode -> data [ c_nx ] = ( ( b_y -> data [
c_nx ] < 2.2204460492503131E-16 ) && ( c_y -> data [ c_nx ] <
2.2204460492503131E-16 ) ) ; } } else { bgl1auxn4q ( sameNode , b_y , c_y ) ;
} d_y = false ; lastind = 1 ; exitg1 = false ; while ( ( ! exitg1 ) && (
lastind <= sameNode -> size [ 1 ] ) ) { if ( sameNode -> data [ lastind - 1 ]
) { d_y = true ; exitg1 = true ; } else { lastind ++ ; } } if ( d_y ) {
it14h2ybk0 ( sameNode , & q_data , q_size ) ; if ( q_size [ 1 ] != 0 ) {
newNodeID = q_data ; } else { newNodeID = 0.0 ; } } else { obj_p = ( int32_T
) ( obj -> RoadmapBuilder . Roadmap . CurrentLabel + 1.0 ) ; c_nx = ( obj_p -
1 ) << 1 ; obj -> RoadmapBuilder . Roadmap . Nodes -> data [ c_nx ] = newNode
[ 0 ] ; obj -> RoadmapBuilder . Roadmap . Nodes -> data [ 1 + c_nx ] =
newNode [ 1 ] ; newNodeID = obj -> RoadmapBuilder . Roadmap . CurrentLabel +
1.0 ; obj -> RoadmapBuilder . Roadmap . CurrentLabel ++ ; xl = obj ->
RoadmapBuilder . Roadmap . CurrentLabel ; obj -> RoadmapBuilder . Roadmap .
NumComponents ++ ; obj -> RoadmapBuilder . Roadmap . NodeLabelSet -> data [ (
int32_T ) obj -> RoadmapBuilder . Roadmap . NumComponents - 1 ] = xl ; obj ->
RoadmapBuilder . Roadmap . NodeLabels -> data [ ( int32_T ) newNodeID - 1 ] =
xl ; } c_nx = b_x -> size [ 0 ] * b_x -> size [ 1 ] ; b_x -> size [ 0 ] = 1 ;
b_x -> size [ 1 ] = obj -> RoadmapBuilder . Roadmap . Nodes -> size [ 1 ] ;
clokujid0y ( b_x , c_nx ) ; loop_ub = obj -> RoadmapBuilder . Roadmap . Nodes
-> size [ 1 ] ; for ( c_nx = 0 ; c_nx < loop_ub ; c_nx ++ ) { b_x -> data [
c_nx ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ c_nx << 1 ] ; }
obj_p = b_x -> size [ 1 ] - 1 ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <=
obj_p ; c_nx ++ ) { if ( ! muDoubleScalarIsNaN ( b_x -> data [ c_nx ] ) ) {
trueCount ++ ; } } c_nx = tmp_i -> size [ 0 ] * tmp_i -> size [ 1 ] ; tmp_i
-> size [ 0 ] = 1 ; tmp_i -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_i ,
c_nx ) ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <= obj_p ; c_nx ++ ) { if ( !
muDoubleScalarIsNaN ( b_x -> data [ c_nx ] ) ) { tmp_i -> data [ trueCount ]
= c_nx ; trueCount ++ ; } } c_nx = nodeList -> size [ 0 ] * nodeList -> size
[ 1 ] ; nodeList -> size [ 0 ] = 2 ; loop_ub = tmp_i -> size [ 1 ] ; nodeList
-> size [ 1 ] = tmp_i -> size [ 1 ] ; clokujid0y ( nodeList , c_nx ) ; for (
c_nx = 0 ; c_nx < loop_ub ; c_nx ++ ) { obj_p = tmp_i -> data [ c_nx ] ;
nodeList -> data [ c_nx << 1 ] = obj -> RoadmapBuilder . Roadmap . Nodes ->
data [ obj_p << 1 ] ; nodeList -> data [ 1 + ( c_nx << 1 ) ] = obj ->
RoadmapBuilder . Roadmap . Nodes -> data [ ( obj_p << 1 ) + 1 ] ; }
eangyuxugt ( & obj -> RoadmapBuilder . Roadmap , newNode , nodeList , b_x ) ;
eb1nrnik44 ( b_x , iidx ) ; c_nx = b_y -> size [ 0 ] * b_y -> size [ 1 ] ;
b_y -> size [ 0 ] = 1 ; loop_ub = iidx -> size [ 1 ] ; b_y -> size [ 1 ] =
iidx -> size [ 1 ] ; clokujid0y ( b_y , c_nx ) ; for ( c_nx = 0 ; c_nx <
loop_ub ; c_nx ++ ) { b_y -> data [ c_nx ] = iidx -> data [ c_nx ] ; } c_nx =
0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( c_nx <= b_x -> size [ 1 ] - 1
) ) { if ( b_y -> data [ c_nx ] == newNodeID ) { c_nx ++ ; } else if ( b_x ->
data [ c_nx ] > connectionDistance ) { exitg1 = true ; } else { loop_ub = (
int32_T ) b_y -> data [ c_nx ] ; obj_p = ( loop_ub - 1 ) << 1 ; nearbyNode [
0 ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ obj_p ] ; nearbyNode
[ 1 ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ 1 + obj_p ] ; if (
btdzbryr1o ( newNode , nearbyNode , grid , location ) ) { p3g0upgfmv ( & obj
-> RoadmapBuilder . Roadmap , newNodeID , b_y -> data [ c_nx ] , b_x -> data
[ c_nx ] ) ; } c_nx ++ ; } } } c2nfvjswuo ( & tmp_i ) ; c2nfvjswuo ( & tmp_e
) ; c2nfvjswuo ( & tmp_p ) ; njqibisfwd ( & c_y ) ; njqibisfwd ( & b_y ) ;
c2nfvjswuo ( & iidx ) ; fizihoqn3p ( & sameNode ) ; njqibisfwd ( & y ) ;
njqibisfwd ( & x ) ; c_nx = b_x -> size [ 0 ] * b_x -> size [ 1 ] ; b_x ->
size [ 0 ] = 1 ; b_x -> size [ 1 ] = obj -> RoadmapBuilder . Roadmap . Nodes
-> size [ 1 ] ; clokujid0y ( b_x , c_nx ) ; loop_ub = obj -> RoadmapBuilder .
Roadmap . Nodes -> size [ 1 ] ; for ( c_nx = 0 ; c_nx < loop_ub ; c_nx ++ ) {
b_x -> data [ c_nx ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ c_nx
<< 1 ] ; } obj_p = b_x -> size [ 1 ] - 1 ; trueCount = 0 ; for ( c_nx = 0 ;
c_nx <= obj_p ; c_nx ++ ) { if ( ! muDoubleScalarIsNaN ( b_x -> data [ c_nx ]
) ) { trueCount ++ ; } } ceougeaq11 ( & tmp , 2 ) ; c_nx = tmp -> size [ 0 ]
* tmp -> size [ 1 ] ; tmp -> size [ 0 ] = 1 ; tmp -> size [ 1 ] = trueCount ;
dfntiycgzz ( tmp , c_nx ) ; trueCount = 0 ; for ( c_nx = 0 ; c_nx <= obj_p ;
c_nx ++ ) { if ( ! muDoubleScalarIsNaN ( b_x -> data [ c_nx ] ) ) { tmp ->
data [ trueCount ] = c_nx ; trueCount ++ ; } } njqibisfwd ( & b_x ) ; c_nx =
nodeList -> size [ 0 ] * nodeList -> size [ 1 ] ; nodeList -> size [ 0 ] = 2
; loop_ub = tmp -> size [ 1 ] ; nodeList -> size [ 1 ] = tmp -> size [ 1 ] ;
clokujid0y ( nodeList , c_nx ) ; for ( c_nx = 0 ; c_nx < loop_ub ; c_nx ++ )
{ obj_p = tmp -> data [ c_nx ] ; nodeList -> data [ c_nx << 1 ] = obj ->
RoadmapBuilder . Roadmap . Nodes -> data [ obj_p << 1 ] ; nodeList -> data [
1 + ( c_nx << 1 ) ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ (
obj_p << 1 ) + 1 ] ; } c2nfvjswuo ( & tmp ) ; obj -> InternalNumNodes =
nodeList -> size [ 1 ] ; njqibisfwd ( & nodeList ) ; obj -> UpdateFlag =
false ; } static real_T le2qxtov41 ( const real_T node [ 2 ] , const
oxn0khgesl * roadmap , pwx2zp040p * map ) { __m128d tmp_p ; b4axleo1v0 * tmp
; dxycgom5nk * iidx ; k0shasdzrq * roadmap_p ; k0shasdzrq * x ; real_T node_p
[ 2 ] ; real_T closestNode ; int32_T i ; int32_T loop_ub ; int32_T trueCount
; boolean_T grid [ 3600 ] ; boolean_T exitg1 ; closestNode = ( rtNaN ) ;
ib42mb0qrj ( & tmp , 2 ) ; i = tmp -> size [ 0 ] * tmp -> size [ 1 ] ; tmp ->
size [ 0 ] = 1 ; loop_ub = roadmap -> Nodes -> size [ 1 ] ; tmp -> size [ 1 ]
= roadmap -> Nodes -> size [ 1 ] ; mdiomdajqb ( tmp , i ) ; for ( i = 0 ; i <
loop_ub ; i ++ ) { tmp -> data [ i ] = ! muDoubleScalarIsNaN ( roadmap ->
Nodes -> data [ i << 1 ] ) ; } loop_ub = tmp -> size [ 1 ] - 1 ; trueCount =
0 ; for ( i = 0 ; i <= loop_ub ; i ++ ) { if ( tmp -> data [ i ] ) {
trueCount ++ ; } } ceougeaq11 ( & iidx , 2 ) ; i = iidx -> size [ 0 ] * iidx
-> size [ 1 ] ; iidx -> size [ 0 ] = 1 ; iidx -> size [ 1 ] = trueCount ;
dfntiycgzz ( iidx , i ) ; trueCount = 0 ; for ( i = 0 ; i <= loop_ub ; i ++ )
{ if ( tmp -> data [ i ] ) { iidx -> data [ trueCount ] = i ; trueCount ++ ;
} } fizihoqn3p ( & tmp ) ; k5xyyjjkd4 ( & roadmap_p , 2 ) ; i = roadmap_p ->
size [ 0 ] * roadmap_p -> size [ 1 ] ; roadmap_p -> size [ 0 ] = 2 ; loop_ub
= iidx -> size [ 1 ] ; roadmap_p -> size [ 1 ] = iidx -> size [ 1 ] ;
clokujid0y ( roadmap_p , i ) ; for ( i = 0 ; i < loop_ub ; i ++ ) { trueCount
= iidx -> data [ i ] ; roadmap_p -> data [ i << 1 ] = roadmap -> Nodes ->
data [ trueCount << 1 ] ; roadmap_p -> data [ 1 + ( i << 1 ) ] = roadmap ->
Nodes -> data [ ( trueCount << 1 ) + 1 ] ; } k5xyyjjkd4 ( & x , 2 ) ;
eangyuxugt ( roadmap , node , roadmap_p , x ) ; njqibisfwd ( & roadmap_p ) ;
eb1nrnik44 ( x , iidx ) ; i = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && (
i <= x -> size [ 1 ] - 1 ) ) { _mm_storeu_pd ( & node_p [ 0 ] , _mm_sub_pd (
_mm_loadu_pd ( & node [ 0 ] ) , _mm_set1_pd ( iidx -> data [ i ] ) ) ) ; if (
norm_LxdYsY7z ( node_p ) < 2.2204460492503131E-16 ) { i ++ ; } else {
mkwt2qmyjw ( map , grid ) ; tmp_p = _mm_add_pd ( _mm_loadu_pd ( & map ->
SharedProperties . LocalOriginInWorld [ 0 ] ) , _mm_loadu_pd ( & map ->
SharedProperties . GridOriginInLocal [ 0 ] ) ) ; _mm_storeu_pd ( & node_p [ 0
] , tmp_p ) ; if ( btdzbryr1o ( node , & roadmap -> Nodes -> data [ ( iidx ->
data [ i ] - 1 ) << 1 ] , grid , node_p ) ) { closestNode = iidx -> data [ i
] ; exitg1 = true ; } else { i ++ ; } } } njqibisfwd ( & x ) ; c2nfvjswuo ( &
iidx ) ; return closestNode ; } static void fsaynwsdln ( const oxn0khgesl *
obj , real_T nodes , k0shasdzrq * componentList ) { b4axleo1v0 * logicalArray
; dxycgom5nk * ii ; int32_T i ; int32_T na ; int32_T nx ; int32_T obj_tmp ;
int32_T tmp ; int32_T trueCount ; boolean_T exitg1 ; componentList -> size [
0 ] = 1 ; componentList -> size [ 1 ] = 0 ; trueCount = 0 ; if ( !
muDoubleScalarIsNaN ( nodes ) ) { for ( i = 0 ; i < 1 ; i ++ ) { trueCount ++
; } } ib42mb0qrj ( & logicalArray , 2 ) ; ceougeaq11 ( & ii , 2 ) ; for ( i =
0 ; i < trueCount ; i ++ ) { obj_tmp = obj -> NodeLabelSet -> size [ 1 ] ;
tmp = logicalArray -> size [ 0 ] * logicalArray -> size [ 1 ] ; logicalArray
-> size [ 0 ] = 1 ; logicalArray -> size [ 1 ] = obj -> NodeLabelSet -> size
[ 1 ] ; mdiomdajqb ( logicalArray , tmp ) ; for ( nx = 0 ; nx < obj_tmp ; nx
++ ) { logicalArray -> data [ nx ] = false ; if ( obj -> NodeLabels -> data [
( int32_T ) nodes - 1 ] == obj -> NodeLabelSet -> data [ nx ] ) {
logicalArray -> data [ nx ] = true ; } } na = 0 ; tmp = ii -> size [ 0 ] * ii
-> size [ 1 ] ; ii -> size [ 0 ] = 1 ; ii -> size [ 1 ] = obj -> NodeLabelSet
-> size [ 1 ] ; dfntiycgzz ( ii , tmp ) ; nx = 0 ; exitg1 = false ; while ( (
! exitg1 ) && ( nx <= obj_tmp - 1 ) ) { if ( logicalArray -> data [ nx ] ) {
na ++ ; ii -> data [ na - 1 ] = nx + 1 ; if ( na >= obj_tmp ) { exitg1 = true
; } else { nx ++ ; } } else { nx ++ ; } } if ( logicalArray -> size [ 1 ] ==
1 ) { if ( na == 0 ) { ii -> size [ 0 ] = 1 ; ii -> size [ 1 ] = 0 ; } } else
if ( na < 1 ) { ii -> size [ 1 ] = 0 ; } else { tmp = ii -> size [ 0 ] * ii
-> size [ 1 ] ; ii -> size [ 1 ] = na ; dfntiycgzz ( ii , tmp ) ; } nx =
componentList -> size [ 1 ] ; na = ii -> size [ 1 ] ; tmp = componentList ->
size [ 0 ] * componentList -> size [ 1 ] ; componentList -> size [ 1 ] += ii
-> size [ 1 ] ; clokujid0y ( componentList , tmp ) ; for ( obj_tmp = 0 ;
obj_tmp < na ; obj_tmp ++ ) { componentList -> data [ nx + obj_tmp ] = ii ->
data [ obj_tmp ] ; } } c2nfvjswuo ( & ii ) ; fizihoqn3p ( & logicalArray ) ;
} static boolean_T hulg2uozzd ( const k0shasdzrq * varargin_1 , const
k0shasdzrq * varargin_2 ) { int32_T k ; boolean_T d_p ; boolean_T exitg1 ;
boolean_T p ; p = false ; d_p = false ; if ( varargin_1 -> size [ 1 ] ==
varargin_2 -> size [ 1 ] ) { d_p = true ; } if ( d_p && ( varargin_1 -> size
[ 1 ] != 0 ) && ( varargin_2 -> size [ 1 ] != 0 ) ) { k = 0 ; exitg1 = false
; while ( ( ! exitg1 ) && ( k <= varargin_2 -> size [ 1 ] - 1 ) ) { if ( ! (
varargin_1 -> data [ k ] == varargin_2 -> data [ k ] ) ) { d_p = false ;
exitg1 = true ; } else { k ++ ; } } } if ( d_p ) { p = true ; } return p ; }
static void mbfnj03133 ( const oxn0khgesl * obj , k0shasdzrq * edgeList ) {
dxycgom5nk * b_i ; dxycgom5nk * b_j ; k0shasdzrq * x ; int32_T idx ; int32_T
iend ; int32_T loop_ub_tmp ; int32_T n ; int32_T nx ; boolean_T exitg1 ;
boolean_T guard1 ; k5xyyjjkd4 ( & x , 2 ) ; nx = x -> size [ 0 ] * x -> size
[ 1 ] ; x -> size [ 0 ] = obj -> AdjacencyMatrix -> size [ 0 ] ; n = obj ->
AdjacencyMatrix -> size [ 1 ] ; x -> size [ 1 ] = obj -> AdjacencyMatrix ->
size [ 1 ] ; clokujid0y ( x , nx ) ; loop_ub_tmp = obj -> AdjacencyMatrix ->
size [ 0 ] * obj -> AdjacencyMatrix -> size [ 1 ] ; for ( idx = 0 ; idx <
loop_ub_tmp ; idx ++ ) { x -> data [ idx ] = obj -> AdjacencyMatrix -> data [
idx ] ; } if ( ( obj -> AdjacencyMatrix -> size [ 0 ] != 0 ) && ( obj ->
AdjacencyMatrix -> size [ 1 ] != 0 ) && ( obj -> AdjacencyMatrix -> size [ 1
] > 1 ) ) { iend = 1 ; for ( nx = 2 ; nx <= n ; nx ++ ) { for ( idx = 0 ; idx
< iend ; idx ++ ) { x -> data [ idx + x -> size [ 0 ] * ( nx - 1 ) ] = 0.0 ;
} if ( iend < x -> size [ 0 ] ) { iend ++ ; } } } ceougeaq11 ( & b_i , 1 ) ;
ceougeaq11 ( & b_j , 1 ) ; if ( loop_ub_tmp == 0 ) { b_i -> size [ 0 ] = 0 ;
b_j -> size [ 0 ] = 0 ; } else { idx = 0 ; nx = b_i -> size [ 0 ] ; b_i ->
size [ 0 ] = loop_ub_tmp ; dfntiycgzz ( b_i , nx ) ; nx = b_j -> size [ 0 ] ;
b_j -> size [ 0 ] = loop_ub_tmp ; dfntiycgzz ( b_j , nx ) ; n = 1 ; iend = 1
; exitg1 = false ; while ( ( ! exitg1 ) && ( iend <= x -> size [ 1 ] ) ) {
guard1 = false ; if ( x -> data [ ( ( iend - 1 ) * x -> size [ 0 ] + n ) - 1
] != 0.0 ) { idx ++ ; b_i -> data [ idx - 1 ] = n ; b_j -> data [ idx - 1 ] =
iend ; if ( idx >= loop_ub_tmp ) { exitg1 = true ; } else { guard1 = true ; }
} else { guard1 = true ; } if ( guard1 ) { n ++ ; if ( n > x -> size [ 0 ] )
{ n = 1 ; iend ++ ; } } } if ( loop_ub_tmp == 1 ) { if ( idx == 0 ) { b_i ->
size [ 0 ] = 0 ; b_j -> size [ 0 ] = 0 ; } } else if ( idx < 1 ) { b_i ->
size [ 0 ] = 0 ; b_j -> size [ 0 ] = 0 ; } else { nx = b_i -> size [ 0 ] ;
b_i -> size [ 0 ] = idx ; dfntiycgzz ( b_i , nx ) ; nx = b_j -> size [ 0 ] ;
b_j -> size [ 0 ] = idx ; dfntiycgzz ( b_j , nx ) ; } } njqibisfwd ( & x ) ;
nx = edgeList -> size [ 0 ] * edgeList -> size [ 1 ] ; edgeList -> size [ 0 ]
= 2 ; loop_ub_tmp = b_i -> size [ 0 ] ; edgeList -> size [ 1 ] = b_i -> size
[ 0 ] ; clokujid0y ( edgeList , nx ) ; for ( idx = 0 ; idx < loop_ub_tmp ;
idx ++ ) { edgeList -> data [ idx << 1 ] = b_i -> data [ idx ] ; edgeList ->
data [ 1 + ( idx << 1 ) ] = b_j -> data [ idx ] ; } c2nfvjswuo ( & b_j ) ;
c2nfvjswuo ( & b_i ) ; } static void g1w3qbdma2o ( const k0shasdzrq * a ,
const real_T b [ 2 ] , k0shasdzrq * c ) { int32_T acoef ; int32_T d ; int32_T
k ; k = c -> size [ 0 ] * c -> size [ 1 ] ; c -> size [ 0 ] = 2 ; c -> size [
1 ] = a -> size [ 1 ] ; clokujid0y ( c , k ) ; if ( a -> size [ 1 ] != 0 ) {
acoef = ( a -> size [ 1 ] != 1 ) ; d = a -> size [ 1 ] - 1 ; for ( k = 0 ; k
<= d ; k ++ ) { _mm_storeu_pd ( & c -> data [ k << 1 ] , _mm_sub_pd (
_mm_loadu_pd ( & a -> data [ ( acoef * k ) << 1 ] ) , _mm_loadu_pd ( & b [ 0
] ) ) ) ; } } } static void eb1nrnik44z ( k0shasdzrq * x ) { dxycgom5nk * x_p
; ceougeaq11 ( & x_p , 2 ) ; e0juex3elc ( x , x_p ) ; c2nfvjswuo ( & x_p ) ;
} static void j2eqhook05 ( const oxn0khgesl * obj , real_T startNode , real_T
goalNode , k0shasdzrq * path ) { __m128d tmp_fs ; b4axleo1v0 * b_x_tmp_tmp ;
b4axleo1v0 * d ; b4axleo1v0 * tmp ; b4axleo1v0 * tmp_p ; dxycgom5nk * g_tmp ;
dxycgom5nk * tmp_b ; dxycgom5nk * tmp_c ; dxycgom5nk * tmp_d ; dxycgom5nk *
tmp_dz ; dxycgom5nk * tmp_e ; dxycgom5nk * tmp_f ; dxycgom5nk * tmp_g ;
dxycgom5nk * tmp_i ; dxycgom5nk * tmp_j ; dxycgom5nk * tmp_k ; dxycgom5nk *
tmp_l ; dxycgom5nk * tmp_m ; dxycgom5nk * tmp_n ; dxycgom5nk * tmp_o ;
dxycgom5nk * x_tmp ; k0shasdzrq * adjacencyMatrix ; k0shasdzrq * b ;
k0shasdzrq * bb ; k0shasdzrq * cameFrom ; k0shasdzrq * closedSet ; k0shasdzrq
* componentGoal ; k0shasdzrq * componentStart ; k0shasdzrq * edgeList ;
k0shasdzrq * fScore ; k0shasdzrq * gScore ; k0shasdzrq * hScore ; k0shasdzrq
* nodeList ; k0shasdzrq * openSet ; real_T sz [ 2 ] ; real_T componentStart_p
; real_T currentNode ; real_T tentativeGScore ; int32_T cb_size [ 2 ] ;
int32_T adjacencyMatrix_e ; int32_T adjacencyMatrix_p ; int32_T b_nx ;
int32_T cb_data ; int32_T edgeList_tmp ; int32_T exitg1 ; int32_T last ;
int32_T loop_ub ; int32_T loop_ub_p ; int32_T partialTrueCount ; int32_T
trueCount ; int32_T vectorUB ; boolean_T exitg2 ; boolean_T tentativeIsBetter
; k5xyyjjkd4 ( & componentStart , 2 ) ; fsaynwsdln ( obj , startNode ,
componentStart ) ; k5xyyjjkd4 ( & componentGoal , 2 ) ; fsaynwsdln ( obj ,
goalNode , componentGoal ) ; if ( ! hulg2uozzd ( componentStart ,
componentGoal ) ) { path -> size [ 0 ] = 0 ; path -> size [ 1 ] = 0 ; } else
{ k5xyyjjkd4 ( & b , 2 ) ; mbfnj03133 ( obj , b ) ; k5xyyjjkd4 ( & nodeList ,
2 ) ; mbfnj03133 ( obj , nodeList ) ; k5xyyjjkd4 ( & bb , 2 ) ; mbfnj03133 (
obj , bb ) ; k5xyyjjkd4 ( & edgeList , 2 ) ; partialTrueCount = edgeList ->
size [ 0 ] * edgeList -> size [ 1 ] ; edgeList -> size [ 0 ] = 2 ; loop_ub_p
= bb -> size [ 1 ] + b -> size [ 1 ] ; edgeList -> size [ 1 ] = loop_ub_p ;
clokujid0y ( edgeList , partialTrueCount ) ; loop_ub = bb -> size [ 1 ] ; for
( b_nx = 0 ; b_nx < loop_ub ; b_nx ++ ) { partialTrueCount = b_nx << 1 ;
edgeList -> data [ partialTrueCount ] = bb -> data [ partialTrueCount ] ;
partialTrueCount = ( b_nx << 1 ) + 1 ; edgeList -> data [ partialTrueCount ]
= bb -> data [ partialTrueCount ] ; } loop_ub = b -> size [ 1 ] ; for ( b_nx
= 0 ; b_nx < loop_ub ; b_nx ++ ) { edgeList -> data [ ( b_nx + bb -> size [ 1
] ) << 1 ] = b -> data [ ( b_nx << 1 ) + 1 ] ; } loop_ub = nodeList -> size [
1 ] ; for ( b_nx = 0 ; b_nx < loop_ub ; b_nx ++ ) { edgeList -> data [ 1 + (
( b_nx + bb -> size [ 1 ] ) << 1 ) ] = nodeList -> data [ b_nx << 1 ] ; }
njqibisfwd ( & bb ) ; k5xyyjjkd4 ( & adjacencyMatrix , 2 ) ;
adjacencyMatrix_p = obj -> AdjacencyMatrix -> size [ 0 ] ; partialTrueCount =
adjacencyMatrix -> size [ 0 ] * adjacencyMatrix -> size [ 1 ] ;
adjacencyMatrix -> size [ 0 ] = obj -> AdjacencyMatrix -> size [ 0 ] ;
adjacencyMatrix_e = obj -> AdjacencyMatrix -> size [ 1 ] ; adjacencyMatrix ->
size [ 1 ] = obj -> AdjacencyMatrix -> size [ 1 ] ; clokujid0y (
adjacencyMatrix , partialTrueCount ) ; loop_ub = obj -> AdjacencyMatrix ->
size [ 0 ] * obj -> AdjacencyMatrix -> size [ 1 ] ; for ( b_nx = 0 ; b_nx <
loop_ub ; b_nx ++ ) { adjacencyMatrix -> data [ b_nx ] = obj ->
AdjacencyMatrix -> data [ b_nx ] ; } ib42mb0qrj ( & b_x_tmp_tmp , 2 ) ;
partialTrueCount = b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ;
b_x_tmp_tmp -> size [ 0 ] = 1 ; loop_ub = obj -> Nodes -> size [ 1 ] ;
b_x_tmp_tmp -> size [ 1 ] = obj -> Nodes -> size [ 1 ] ; mdiomdajqb (
b_x_tmp_tmp , partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub ; b_nx ++
) { b_x_tmp_tmp -> data [ b_nx ] = ! muDoubleScalarIsNaN ( obj -> Nodes ->
data [ b_nx << 1 ] ) ; } last = b_x_tmp_tmp -> size [ 1 ] - 1 ; loop_ub = 0 ;
for ( b_nx = 0 ; b_nx <= last ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ]
) { loop_ub ++ ; } } ceougeaq11 ( & tmp_e , 2 ) ; partialTrueCount = tmp_e ->
size [ 0 ] * tmp_e -> size [ 1 ] ; tmp_e -> size [ 0 ] = 1 ; tmp_e -> size [
1 ] = loop_ub ; dfntiycgzz ( tmp_e , partialTrueCount ) ; partialTrueCount =
0 ; for ( b_nx = 0 ; b_nx <= last ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [
b_nx ] ) { tmp_e -> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ;
} } partialTrueCount = nodeList -> size [ 0 ] * nodeList -> size [ 1 ] ;
nodeList -> size [ 0 ] = 2 ; loop_ub = tmp_e -> size [ 1 ] ; nodeList -> size
[ 1 ] = tmp_e -> size [ 1 ] ; clokujid0y ( nodeList , partialTrueCount ) ;
for ( b_nx = 0 ; b_nx < loop_ub ; b_nx ++ ) { last = tmp_e -> data [ b_nx ] ;
nodeList -> data [ b_nx << 1 ] = obj -> Nodes -> data [ last << 1 ] ;
nodeList -> data [ 1 + ( b_nx << 1 ) ] = obj -> Nodes -> data [ ( last << 1 )
+ 1 ] ; } c2nfvjswuo ( & tmp_e ) ; eangyuxugt ( obj , & nodeList -> data [ (
( int32_T ) startNode - 1 ) << 1 ] , nodeList , componentStart ) ; b_nx = ( (
int32_T ) goalNode - 1 ) << 1 ; sz [ 0 ] = nodeList -> data [ b_nx ] ; sz [ 1
] = nodeList -> data [ 1 + b_nx ] ; if ( obj -> IsEuclidean ) { g1w3qbdma2o (
nodeList , sz , b ) ; partialTrueCount = nodeList -> size [ 0 ] * nodeList ->
size [ 1 ] ; nodeList -> size [ 0 ] = 2 ; nodeList -> size [ 1 ] = b -> size
[ 1 ] ; clokujid0y ( nodeList , partialTrueCount ) ; loop_ub = b -> size [ 1
] << 1 ; partialTrueCount = ( loop_ub / 2 ) << 1 ; vectorUB =
partialTrueCount - 2 ; for ( b_nx = 0 ; b_nx <= vectorUB ; b_nx += 2 ) {
tmp_fs = _mm_loadu_pd ( & b -> data [ b_nx ] ) ; _mm_storeu_pd ( & nodeList
-> data [ b_nx ] , _mm_mul_pd ( tmp_fs , tmp_fs ) ) ; } for ( b_nx =
partialTrueCount ; b_nx < loop_ub ; b_nx ++ ) { currentNode = b -> data [
b_nx ] ; nodeList -> data [ b_nx ] = currentNode * currentNode ; } if (
nodeList -> size [ 1 ] == 0 ) { componentGoal -> size [ 0 ] = 1 ;
componentGoal -> size [ 1 ] = 0 ; } else { partialTrueCount = componentGoal
-> size [ 0 ] * componentGoal -> size [ 1 ] ; componentGoal -> size [ 0 ] = 1
; b_nx = nodeList -> size [ 1 ] ; componentGoal -> size [ 1 ] = nodeList ->
size [ 1 ] ; clokujid0y ( componentGoal , partialTrueCount ) ; for ( last = 0
; last < b_nx ; last ++ ) { componentGoal -> data [ last ] = nzneokfgio (
nodeList , last + 1 ) ; } } b_nx = componentGoal -> size [ 1 ] ;
partialTrueCount = ( componentGoal -> size [ 1 ] / 2 ) << 1 ; vectorUB =
partialTrueCount - 2 ; for ( last = 0 ; last <= vectorUB ; last += 2 ) {
tmp_fs = _mm_loadu_pd ( & componentGoal -> data [ last ] ) ; _mm_storeu_pd (
& componentGoal -> data [ last ] , _mm_sqrt_pd ( tmp_fs ) ) ; } for ( last =
partialTrueCount ; last < b_nx ; last ++ ) { componentGoal -> data [ last ] =
muDoubleScalarSqrt ( componentGoal -> data [ last ] ) ; } } else {
g1w3qbdma2o ( nodeList , sz , b ) ; partialTrueCount = nodeList -> size [ 0 ]
* nodeList -> size [ 1 ] ; nodeList -> size [ 0 ] = 2 ; nodeList -> size [ 1
] = b -> size [ 1 ] ; clokujid0y ( nodeList , partialTrueCount ) ; loop_ub =
b -> size [ 1 ] << 1 ; partialTrueCount = ( loop_ub / 2 ) << 1 ; vectorUB =
partialTrueCount - 2 ; for ( b_nx = 0 ; b_nx <= vectorUB ; b_nx += 2 ) {
tmp_fs = _mm_loadu_pd ( & b -> data [ b_nx ] ) ; _mm_storeu_pd ( & nodeList
-> data [ b_nx ] , _mm_mul_pd ( tmp_fs , tmp_fs ) ) ; } for ( b_nx =
partialTrueCount ; b_nx < loop_ub ; b_nx ++ ) { currentNode = b -> data [
b_nx ] ; nodeList -> data [ b_nx ] = currentNode * currentNode ; } if (
nodeList -> size [ 1 ] == 0 ) { componentGoal -> size [ 0 ] = 1 ;
componentGoal -> size [ 1 ] = 0 ; } else { partialTrueCount = componentGoal
-> size [ 0 ] * componentGoal -> size [ 1 ] ; componentGoal -> size [ 0 ] = 1
; b_nx = nodeList -> size [ 1 ] ; componentGoal -> size [ 1 ] = nodeList ->
size [ 1 ] ; clokujid0y ( componentGoal , partialTrueCount ) ; for ( last = 0
; last < b_nx ; last ++ ) { componentGoal -> data [ last ] = nzneokfgio (
nodeList , last + 1 ) ; } } b_nx = componentGoal -> size [ 1 ] ;
partialTrueCount = ( componentGoal -> size [ 1 ] / 2 ) << 1 ; vectorUB =
partialTrueCount - 2 ; for ( last = 0 ; last <= vectorUB ; last += 2 ) {
tmp_fs = _mm_loadu_pd ( & componentGoal -> data [ last ] ) ; _mm_storeu_pd (
& componentGoal -> data [ last ] , _mm_sqrt_pd ( tmp_fs ) ) ; } for ( last =
partialTrueCount ; last < b_nx ; last ++ ) { componentGoal -> data [ last ] =
muDoubleScalarSqrt ( componentGoal -> data [ last ] ) ; } } njqibisfwd ( & b
) ; for ( b_nx = 0 ; b_nx < adjacencyMatrix_e ; b_nx ++ ) { adjacencyMatrix
-> data [ ( ( int32_T ) startNode + adjacencyMatrix -> size [ 0 ] * b_nx ) -
1 ] = componentStart -> data [ b_nx ] ; } k5xyyjjkd4 ( & closedSet , 2 ) ;
partialTrueCount = closedSet -> size [ 0 ] * closedSet -> size [ 1 ] ;
closedSet -> size [ 0 ] = 1 ; closedSet -> size [ 1 ] = obj ->
AdjacencyMatrix -> size [ 0 ] ; clokujid0y ( closedSet , partialTrueCount ) ;
k5xyyjjkd4 ( & gScore , 2 ) ; partialTrueCount = gScore -> size [ 0 ] *
gScore -> size [ 1 ] ; gScore -> size [ 0 ] = 1 ; gScore -> size [ 1 ] = obj
-> AdjacencyMatrix -> size [ 0 ] ; clokujid0y ( gScore , partialTrueCount ) ;
k5xyyjjkd4 ( & hScore , 2 ) ; partialTrueCount = hScore -> size [ 0 ] *
hScore -> size [ 1 ] ; hScore -> size [ 0 ] = 1 ; hScore -> size [ 1 ] = obj
-> AdjacencyMatrix -> size [ 0 ] ; clokujid0y ( hScore , partialTrueCount ) ;
k5xyyjjkd4 ( & fScore , 2 ) ; partialTrueCount = fScore -> size [ 0 ] *
fScore -> size [ 1 ] ; fScore -> size [ 0 ] = 1 ; fScore -> size [ 1 ] = obj
-> AdjacencyMatrix -> size [ 0 ] ; clokujid0y ( fScore , partialTrueCount ) ;
k5xyyjjkd4 ( & openSet , 2 ) ; partialTrueCount = openSet -> size [ 0 ] *
openSet -> size [ 1 ] ; openSet -> size [ 0 ] = 1 ; openSet -> size [ 1 ] =
obj -> AdjacencyMatrix -> size [ 0 ] ; clokujid0y ( openSet ,
partialTrueCount ) ; k5xyyjjkd4 ( & cameFrom , 2 ) ; partialTrueCount =
cameFrom -> size [ 0 ] * cameFrom -> size [ 1 ] ; cameFrom -> size [ 0 ] = 1
; cameFrom -> size [ 1 ] = obj -> AdjacencyMatrix -> size [ 0 ] ; clokujid0y
( cameFrom , partialTrueCount ) ; for ( b_nx = 0 ; b_nx < adjacencyMatrix_p ;
b_nx ++ ) { adjacencyMatrix -> data [ b_nx + adjacencyMatrix -> size [ 0 ] *
( ( int32_T ) goalNode - 1 ) ] = componentGoal -> data [ b_nx ] ; closedSet
-> data [ b_nx ] = 0.0 ; gScore -> data [ b_nx ] = ( rtInf ) ; hScore -> data
[ b_nx ] = ( rtInf ) ; fScore -> data [ b_nx ] = ( rtInf ) ; openSet -> data
[ b_nx ] = 0.0 ; cameFrom -> data [ b_nx ] = 0.0 ; } gScore -> data [ (
int32_T ) startNode - 1 ] = 0.0 ; openSet -> data [ 0 ] = startNode ; hScore
-> data [ ( int32_T ) startNode - 1 ] = adjacencyMatrix -> data [ ( ( (
int32_T ) goalNode - 1 ) * adjacencyMatrix -> size [ 0 ] + ( int32_T )
startNode ) - 1 ] ; fScore -> data [ ( int32_T ) startNode - 1 ] = gScore ->
data [ ( int32_T ) startNode - 1 ] + hScore -> data [ ( int32_T ) startNode -
1 ] ; ib42mb0qrj ( & d , 2 ) ; ceougeaq11 ( & x_tmp , 2 ) ; ceougeaq11 ( &
g_tmp , 1 ) ; ib42mb0qrj ( & tmp , 2 ) ; ib42mb0qrj ( & tmp_p , 2 ) ;
ceougeaq11 ( & tmp_i , 2 ) ; ceougeaq11 ( & tmp_m , 2 ) ; ceougeaq11 ( &
tmp_g , 2 ) ; ceougeaq11 ( & tmp_j , 2 ) ; ceougeaq11 ( & tmp_f , 2 ) ;
ceougeaq11 ( & tmp_c , 2 ) ; ceougeaq11 ( & tmp_k , 2 ) ; ceougeaq11 ( &
tmp_b , 2 ) ; ceougeaq11 ( & tmp_n , 2 ) ; ceougeaq11 ( & tmp_l , 2 ) ;
ceougeaq11 ( & tmp_d , 2 ) ; ceougeaq11 ( & tmp_o , 2 ) ; ceougeaq11 ( &
tmp_dz , 2 ) ; do { exitg1 = 0 ; partialTrueCount = b_x_tmp_tmp -> size [ 0 ]
* b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp ->
size [ 1 ] = adjacencyMatrix_p ; mdiomdajqb ( b_x_tmp_tmp , partialTrueCount
) ; for ( b_nx = 0 ; b_nx < adjacencyMatrix_p ; b_nx ++ ) { b_x_tmp_tmp ->
data [ b_nx ] = ( openSet -> data [ b_nx ] == 0.0 ) ; } tentativeIsBetter =
true ; last = 1 ; exitg2 = false ; while ( ( ! exitg2 ) && ( last <=
b_x_tmp_tmp -> size [ 1 ] ) ) { if ( ! b_x_tmp_tmp -> data [ last - 1 ] ) {
tentativeIsBetter = false ; exitg2 = true ; } else { last ++ ; } } if ( !
tentativeIsBetter ) { adjacencyMatrix_e = openSet -> size [ 1 ] - 1 ; loop_ub
= 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( openSet
-> data [ b_nx ] > 0.0 ) { loop_ub ++ ; } } partialTrueCount = x_tmp -> size
[ 0 ] * x_tmp -> size [ 1 ] ; x_tmp -> size [ 0 ] = 1 ; x_tmp -> size [ 1 ] =
loop_ub ; dfntiycgzz ( x_tmp , partialTrueCount ) ; partialTrueCount = 0 ;
for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( openSet -> data
[ b_nx ] > 0.0 ) { x_tmp -> data [ partialTrueCount ] = b_nx ;
partialTrueCount ++ ; } } partialTrueCount = componentStart -> size [ 0 ] *
componentStart -> size [ 1 ] ; componentStart -> size [ 0 ] = 1 ; loop_ub =
x_tmp -> size [ 1 ] ; componentStart -> size [ 1 ] = x_tmp -> size [ 1 ] ;
clokujid0y ( componentStart , partialTrueCount ) ; for ( b_nx = 0 ; b_nx <
loop_ub ; b_nx ++ ) { componentStart -> data [ b_nx ] = fScore -> data [ (
int32_T ) openSet -> data [ x_tmp -> data [ b_nx ] ] - 1 ] ; } if ( x_tmp ->
size [ 1 ] <= 2 ) { if ( x_tmp -> size [ 1 ] == 1 ) { vectorUB = 0 ; } else {
currentNode = fScore -> data [ ( int32_T ) openSet -> data [ x_tmp -> data [
0 ] ] - 1 ] ; componentStart_p = fScore -> data [ ( int32_T ) openSet -> data
[ x_tmp -> data [ x_tmp -> size [ 1 ] - 1 ] ] - 1 ] ; if ( ( currentNode >
componentStart_p ) || ( muDoubleScalarIsNaN ( currentNode ) && ( !
muDoubleScalarIsNaN ( componentStart_p ) ) ) ) { vectorUB = x_tmp -> size [ 1
] - 1 ; } else { vectorUB = 0 ; } } } else { if ( ! muDoubleScalarIsNaN (
fScore -> data [ ( int32_T ) openSet -> data [ x_tmp -> data [ 0 ] ] - 1 ] )
) { last = 0 ; } else { last = - 1 ; b_nx = 2 ; exitg2 = false ; while ( ( !
exitg2 ) && ( b_nx <= loop_ub ) ) { if ( ! muDoubleScalarIsNaN (
componentStart -> data [ b_nx - 1 ] ) ) { last = b_nx - 1 ; exitg2 = true ; }
else { b_nx ++ ; } } } if ( last + 1 == 0 ) { vectorUB = 0 ; } else {
currentNode = fScore -> data [ ( int32_T ) openSet -> data [ x_tmp -> data [
last ] ] - 1 ] ; vectorUB = last ; for ( b_nx = last + 2 ; b_nx <= loop_ub ;
b_nx ++ ) { componentStart_p = componentStart -> data [ b_nx - 1 ] ; if (
currentNode > componentStart_p ) { currentNode = componentStart_p ; vectorUB
= b_nx - 1 ; } } } } partialTrueCount = componentStart -> size [ 0 ] *
componentStart -> size [ 1 ] ; componentStart -> size [ 0 ] = 1 ;
componentStart -> size [ 1 ] = x_tmp -> size [ 1 ] ; clokujid0y (
componentStart , partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub ; b_nx
++ ) { componentStart -> data [ b_nx ] = openSet -> data [ x_tmp -> data [
b_nx ] ] ; } currentNode = openSet -> data [ x_tmp -> data [ vectorUB ] ] ;
if ( currentNode == goalNode ) { currentNode = goalNode ; componentStart ->
size [ 0 ] = 1 ; componentStart -> size [ 1 ] = 0 ; do { partialTrueCount =
componentGoal -> size [ 0 ] * componentGoal -> size [ 1 ] ; componentGoal ->
size [ 0 ] = 1 ; loop_ub_p = componentStart -> size [ 1 ] + 1 ; componentGoal
-> size [ 1 ] = componentStart -> size [ 1 ] + 1 ; clokujid0y ( componentGoal
, partialTrueCount ) ; componentGoal -> data [ 0 ] = currentNode ; loop_ub =
componentStart -> size [ 1 ] ; if ( loop_ub - 1 >= 0 ) { memcpy ( &
componentGoal -> data [ 1 ] , & componentStart -> data [ 0 ] , ( uint32_T )
loop_ub * sizeof ( real_T ) ) ; } partialTrueCount = componentStart -> size [
0 ] * componentStart -> size [ 1 ] ; componentStart -> size [ 0 ] = 1 ;
componentStart -> size [ 1 ] = loop_ub_p ; clokujid0y ( componentStart ,
partialTrueCount ) ; if ( loop_ub_p - 1 >= 0 ) { memcpy ( & componentStart ->
data [ 0 ] , & componentGoal -> data [ 0 ] , ( uint32_T ) loop_ub_p * sizeof
( real_T ) ) ; } currentNode = cameFrom -> data [ ( int32_T ) currentNode - 1
] ; } while ( ! ( currentNode == 0.0 ) ) ; exitg1 = 1 ; } else { for ( b_nx =
0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( openSet -> data [ b_nx ] ==
componentStart -> data [ vectorUB ] ) { openSet -> data [ b_nx ] = 0.0 ; } }
partialTrueCount = b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ;
b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = adjacencyMatrix_p
; mdiomdajqb ( b_x_tmp_tmp , partialTrueCount ) ; for ( b_nx = 0 ; b_nx <
adjacencyMatrix_p ; b_nx ++ ) { b_x_tmp_tmp -> data [ b_nx ] = ( closedSet ->
data [ b_nx ] == 0.0 ) ; } it14h2ybk0 ( b_x_tmp_tmp , & cb_data , cb_size ) ;
if ( cb_size [ 1 ] - 1 >= 0 ) { closedSet -> data [ cb_data - 1 ] =
componentStart -> data [ vectorUB ] ; } partialTrueCount = componentGoal ->
size [ 0 ] * componentGoal -> size [ 1 ] ; componentGoal -> size [ 0 ] = 1 ;
componentGoal -> size [ 1 ] = cb_data ; clokujid0y ( componentGoal ,
partialTrueCount ) ; if ( cb_data - 1 >= 0 ) { memcpy ( & componentGoal ->
data [ 0 ] , & closedSet -> data [ 0 ] , ( uint32_T ) cb_data * sizeof (
real_T ) ) ; } eb1nrnik44z ( componentGoal ) ; loop_ub = componentGoal ->
size [ 1 ] ; if ( loop_ub - 1 >= 0 ) { memcpy ( & closedSet -> data [ 0 ] , &
componentGoal -> data [ 0 ] , ( uint32_T ) loop_ub * sizeof ( real_T ) ) ; }
partialTrueCount = b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ;
b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ;
mdiomdajqb ( b_x_tmp_tmp , partialTrueCount ) ; for ( b_nx = 0 ; b_nx <
loop_ub_p ; b_nx ++ ) { b_x_tmp_tmp -> data [ b_nx ] = ( edgeList -> data [
b_nx << 1 ] == currentNode ) ; } adjacencyMatrix_e = b_x_tmp_tmp -> size [ 1
] - 1 ; loop_ub = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ )
{ if ( b_x_tmp_tmp -> data [ b_nx ] ) { loop_ub ++ ; } } partialTrueCount =
tmp_i -> size [ 0 ] * tmp_i -> size [ 1 ] ; tmp_i -> size [ 0 ] = 1 ; tmp_i
-> size [ 1 ] = loop_ub ; dfntiycgzz ( tmp_i , partialTrueCount ) ;
partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ )
{ if ( b_x_tmp_tmp -> data [ b_nx ] ) { tmp_i -> data [ partialTrueCount ] =
b_nx ; partialTrueCount ++ ; } } partialTrueCount = nodeList -> size [ 0 ] *
nodeList -> size [ 1 ] ; nodeList -> size [ 0 ] = 2 ; loop_ub = tmp_i -> size
[ 1 ] ; nodeList -> size [ 1 ] = tmp_i -> size [ 1 ] ; clokujid0y ( nodeList
, partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub ; b_nx ++ ) { last =
tmp_i -> data [ b_nx ] ; nodeList -> data [ b_nx << 1 ] = edgeList -> data [
last << 1 ] ; nodeList -> data [ 1 + ( b_nx << 1 ) ] = edgeList -> data [ (
last << 1 ) + 1 ] ; } partialTrueCount = d -> size [ 0 ] * d -> size [ 1 ] ;
d -> size [ 0 ] = 2 ; d -> size [ 1 ] = tmp_i -> size [ 1 ] ; mdiomdajqb ( d
, partialTrueCount ) ; componentStart_p = componentStart -> data [ vectorUB ]
; last = nodeList -> size [ 1 ] << 1 ; for ( b_nx = 0 ; b_nx < last ; b_nx ++
) { d -> data [ b_nx ] = ( nodeList -> data [ b_nx ] != componentStart_p ) ;
} vectorUB = last - 1 ; loop_ub = 0 ; for ( b_nx = 0 ; b_nx <= vectorUB ;
b_nx ++ ) { if ( nodeList -> data [ b_nx ] != currentNode ) { loop_ub ++ ; }
} for ( last = 0 ; last < loop_ub ; last ++ ) { trueCount = 0 ; for ( b_nx =
0 ; b_nx <= vectorUB ; b_nx ++ ) { if ( d -> data [ b_nx ] ) { trueCount ++ ;
} } partialTrueCount = g_tmp -> size [ 0 ] ; g_tmp -> size [ 0 ] = trueCount
; dfntiycgzz ( g_tmp , partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx
= 0 ; b_nx <= vectorUB ; b_nx ++ ) { if ( d -> data [ b_nx ] ) { g_tmp ->
data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } } partialTrueCount
= b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size
[ 0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp
, partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ ) {
b_x_tmp_tmp -> data [ b_nx ] = ( edgeList -> data [ b_nx << 1 ] ==
currentNode ) ; } trueCount = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e
; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { trueCount ++ ; } }
partialTrueCount = tmp_m -> size [ 0 ] * tmp_m -> size [ 1 ] ; tmp_m -> size
[ 0 ] = 1 ; tmp_m -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_m ,
partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { tmp_m
-> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } } edgeList_tmp
= g_tmp -> data [ last ] % 2 ; componentStart_p = edgeList -> data [ ( tmp_m
-> data [ g_tmp -> data [ last ] / 2 ] << 1 ) + edgeList_tmp ] ;
partialTrueCount = b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ;
b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = adjacencyMatrix_p
; mdiomdajqb ( b_x_tmp_tmp , partialTrueCount ) ; for ( b_nx = 0 ; b_nx <
adjacencyMatrix_p ; b_nx ++ ) { b_x_tmp_tmp -> data [ b_nx ] = ( closedSet ->
data [ b_nx ] == componentStart_p ) ; } it14h2ybk0 ( b_x_tmp_tmp , & cb_data
, cb_size ) ; if ( cb_size [ 1 ] == 0 ) { partialTrueCount = b_x_tmp_tmp ->
size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [ 0 ] = 1 ;
b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp ,
partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ ) {
b_x_tmp_tmp -> data [ b_nx ] = ( edgeList -> data [ b_nx << 1 ] ==
currentNode ) ; } trueCount = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e
; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { trueCount ++ ; } }
partialTrueCount = tmp_g -> size [ 0 ] * tmp_g -> size [ 1 ] ; tmp_g -> size
[ 0 ] = 1 ; tmp_g -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_g ,
partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { tmp_g
-> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } }
tentativeGScore = adjacencyMatrix -> data [ ( ( ( int32_T ) edgeList -> data
[ ( tmp_g -> data [ g_tmp -> data [ last ] / 2 ] << 1 ) + edgeList_tmp ] - 1
) * adjacencyMatrix -> size [ 0 ] + ( int32_T ) currentNode ) - 1 ] + gScore
-> data [ ( int32_T ) currentNode - 1 ] ; partialTrueCount = b_x_tmp_tmp ->
size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [ 0 ] = 1 ;
b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp ,
partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ ) {
b_x_tmp_tmp -> data [ b_nx ] = ( edgeList -> data [ b_nx << 1 ] ==
currentNode ) ; } trueCount = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e
; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { trueCount ++ ; } }
partialTrueCount = tmp_j -> size [ 0 ] * tmp_j -> size [ 1 ] ; tmp_j -> size
[ 0 ] = 1 ; tmp_j -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_j ,
partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { tmp_j
-> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } }
componentStart_p = edgeList -> data [ ( tmp_j -> data [ g_tmp -> data [ last
] / 2 ] << 1 ) + edgeList_tmp ] ; partialTrueCount = b_x_tmp_tmp -> size [ 0
] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp
-> size [ 1 ] = adjacencyMatrix_p ; mdiomdajqb ( b_x_tmp_tmp ,
partialTrueCount ) ; for ( b_nx = 0 ; b_nx < adjacencyMatrix_p ; b_nx ++ ) {
b_x_tmp_tmp -> data [ b_nx ] = ( openSet -> data [ b_nx ] == componentStart_p
) ; } it14h2ybk0 ( b_x_tmp_tmp , & cb_data , cb_size ) ; if ( cb_size [ 1 ]
== 0 ) { partialTrueCount = b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [
1 ] ; b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] =
adjacencyMatrix_p ; mdiomdajqb ( b_x_tmp_tmp , partialTrueCount ) ; for (
b_nx = 0 ; b_nx < adjacencyMatrix_p ; b_nx ++ ) { b_x_tmp_tmp -> data [ b_nx
] = ( openSet -> data [ b_nx ] == 0.0 ) ; } it14h2ybk0 ( b_x_tmp_tmp , &
cb_data , cb_size ) ; partialTrueCount = b_x_tmp_tmp -> size [ 0 ] *
b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp ->
size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp , partialTrueCount ) ; for
( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ ) { b_x_tmp_tmp -> data [ b_nx ] = (
edgeList -> data [ b_nx << 1 ] == currentNode ) ; } trueCount = 0 ; for (
b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [
b_nx ] ) { trueCount ++ ; } } partialTrueCount = tmp_c -> size [ 0 ] * tmp_c
-> size [ 1 ] ; tmp_c -> size [ 0 ] = 1 ; tmp_c -> size [ 1 ] = trueCount ;
dfntiycgzz ( tmp_c , partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx =
0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ]
) { tmp_c -> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } } if
( cb_size [ 1 ] - 1 >= 0 ) { openSet -> data [ cb_data - 1 ] = edgeList ->
data [ ( tmp_c -> data [ g_tmp -> data [ last ] / 2 ] << 1 ) + edgeList_tmp ]
; } partialTrueCount = b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ]
; b_x_tmp_tmp -> size [ 0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ;
mdiomdajqb ( b_x_tmp_tmp , partialTrueCount ) ; partialTrueCount = tmp ->
size [ 0 ] * tmp -> size [ 1 ] ; tmp -> size [ 0 ] = 1 ; tmp -> size [ 1 ] =
loop_ub_p ; mdiomdajqb ( tmp , partialTrueCount ) ; for ( b_nx = 0 ; b_nx <
loop_ub_p ; b_nx ++ ) { componentStart_p = edgeList -> data [ b_nx << 1 ] ;
b_x_tmp_tmp -> data [ b_nx ] = ( componentStart_p == currentNode ) ; tmp ->
data [ b_nx ] = ( componentStart_p == currentNode ) ; } trueCount = 0 ; for (
b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [
b_nx ] ) { trueCount ++ ; } } partialTrueCount = tmp_b -> size [ 0 ] * tmp_b
-> size [ 1 ] ; tmp_b -> size [ 0 ] = 1 ; tmp_b -> size [ 1 ] = trueCount ;
dfntiycgzz ( tmp_b , partialTrueCount ) ; partialTrueCount = 0 ; trueCount =
0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp
-> data [ b_nx ] ) { tmp_b -> data [ partialTrueCount ] = b_nx ;
partialTrueCount ++ ; } if ( tmp -> data [ b_nx ] ) { trueCount ++ ; } }
partialTrueCount = tmp_n -> size [ 0 ] * tmp_n -> size [ 1 ] ; tmp_n -> size
[ 0 ] = 1 ; tmp_n -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_n ,
partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( tmp -> data [ b_nx ] ) { tmp_n -> data [
partialTrueCount ] = b_nx ; partialTrueCount ++ ; } } hScore -> data [ (
int32_T ) edgeList -> data [ edgeList_tmp + ( tmp_b -> data [ g_tmp -> data [
last ] / 2 ] << 1 ) ] - 1 ] = adjacencyMatrix -> data [ ( ( int32_T )
edgeList -> data [ ( tmp_n -> data [ g_tmp -> data [ last ] / 2 ] << 1 ) +
edgeList_tmp ] + ( ( int32_T ) goalNode - 1 ) * adjacencyMatrix -> size [ 0 ]
) - 1 ] ; tentativeIsBetter = true ; } else { partialTrueCount = b_x_tmp_tmp
-> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [ 0 ] = 1 ;
b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp ,
partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ ) {
b_x_tmp_tmp -> data [ b_nx ] = ( edgeList -> data [ b_nx << 1 ] ==
currentNode ) ; } trueCount = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e
; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { trueCount ++ ; } }
partialTrueCount = tmp_f -> size [ 0 ] * tmp_f -> size [ 1 ] ; tmp_f -> size
[ 0 ] = 1 ; tmp_f -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_f ,
partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { tmp_f
-> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } }
tentativeIsBetter = ( tentativeGScore < gScore -> data [ ( int32_T ) edgeList
-> data [ ( tmp_f -> data [ g_tmp -> data [ last ] / 2 ] << 1 ) +
edgeList_tmp ] - 1 ] ) ; } if ( tentativeIsBetter ) { partialTrueCount =
b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [
0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp ,
partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ ) {
b_x_tmp_tmp -> data [ b_nx ] = ( edgeList -> data [ b_nx << 1 ] ==
currentNode ) ; } trueCount = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e
; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { trueCount ++ ; } }
partialTrueCount = tmp_k -> size [ 0 ] * tmp_k -> size [ 1 ] ; tmp_k -> size
[ 0 ] = 1 ; tmp_k -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_k ,
partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { tmp_k
-> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } } cameFrom ->
data [ ( int32_T ) edgeList -> data [ edgeList_tmp + ( tmp_k -> data [ g_tmp
-> data [ last ] / 2 ] << 1 ) ] - 1 ] = currentNode ; partialTrueCount =
b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [
0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp ,
partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ ) {
b_x_tmp_tmp -> data [ b_nx ] = ( edgeList -> data [ b_nx << 1 ] ==
currentNode ) ; } trueCount = 0 ; for ( b_nx = 0 ; b_nx <= adjacencyMatrix_e
; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { trueCount ++ ; } }
partialTrueCount = tmp_l -> size [ 0 ] * tmp_l -> size [ 1 ] ; tmp_l -> size
[ 0 ] = 1 ; tmp_l -> size [ 1 ] = trueCount ; dfntiycgzz ( tmp_l ,
partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) { tmp_l
-> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } } gScore ->
data [ ( int32_T ) edgeList -> data [ edgeList_tmp + ( tmp_l -> data [ g_tmp
-> data [ last ] / 2 ] << 1 ) ] - 1 ] = tentativeGScore ; partialTrueCount =
b_x_tmp_tmp -> size [ 0 ] * b_x_tmp_tmp -> size [ 1 ] ; b_x_tmp_tmp -> size [
0 ] = 1 ; b_x_tmp_tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( b_x_tmp_tmp ,
partialTrueCount ) ; partialTrueCount = tmp -> size [ 0 ] * tmp -> size [ 1 ]
; tmp -> size [ 0 ] = 1 ; tmp -> size [ 1 ] = loop_ub_p ; mdiomdajqb ( tmp ,
partialTrueCount ) ; partialTrueCount = tmp_p -> size [ 0 ] * tmp_p -> size [
1 ] ; tmp_p -> size [ 0 ] = 1 ; tmp_p -> size [ 1 ] = loop_ub_p ; mdiomdajqb
( tmp_p , partialTrueCount ) ; for ( b_nx = 0 ; b_nx < loop_ub_p ; b_nx ++ )
{ componentStart_p = edgeList -> data [ b_nx << 1 ] ; b_x_tmp_tmp -> data [
b_nx ] = ( componentStart_p == currentNode ) ; tmp -> data [ b_nx ] = (
componentStart_p == currentNode ) ; tmp_p -> data [ b_nx ] = (
componentStart_p == currentNode ) ; } trueCount = 0 ; for ( b_nx = 0 ; b_nx
<= adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [ b_nx ] ) {
trueCount ++ ; } } partialTrueCount = tmp_d -> size [ 0 ] * tmp_d -> size [ 1
] ; tmp_d -> size [ 0 ] = 1 ; tmp_d -> size [ 1 ] = trueCount ; dfntiycgzz (
tmp_d , partialTrueCount ) ; partialTrueCount = 0 ; trueCount = 0 ; for (
b_nx = 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( b_x_tmp_tmp -> data [
b_nx ] ) { tmp_d -> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ;
} if ( tmp -> data [ b_nx ] ) { trueCount ++ ; } } partialTrueCount = tmp_o
-> size [ 0 ] * tmp_o -> size [ 1 ] ; tmp_o -> size [ 0 ] = 1 ; tmp_o -> size
[ 1 ] = trueCount ; dfntiycgzz ( tmp_o , partialTrueCount ) ;
partialTrueCount = 0 ; trueCount = 0 ; for ( b_nx = 0 ; b_nx <=
adjacencyMatrix_e ; b_nx ++ ) { if ( tmp -> data [ b_nx ] ) { tmp_o -> data [
partialTrueCount ] = b_nx ; partialTrueCount ++ ; } if ( tmp_p -> data [ b_nx
] ) { trueCount ++ ; } } partialTrueCount = tmp_dz -> size [ 0 ] * tmp_dz ->
size [ 1 ] ; tmp_dz -> size [ 0 ] = 1 ; tmp_dz -> size [ 1 ] = trueCount ;
dfntiycgzz ( tmp_dz , partialTrueCount ) ; partialTrueCount = 0 ; for ( b_nx
= 0 ; b_nx <= adjacencyMatrix_e ; b_nx ++ ) { if ( tmp_p -> data [ b_nx ] ) {
tmp_dz -> data [ partialTrueCount ] = b_nx ; partialTrueCount ++ ; } } fScore
-> data [ ( int32_T ) edgeList -> data [ edgeList_tmp + ( tmp_d -> data [
g_tmp -> data [ last ] / 2 ] << 1 ) ] - 1 ] = gScore -> data [ ( int32_T )
edgeList -> data [ ( tmp_o -> data [ g_tmp -> data [ last ] / 2 ] << 1 ) +
edgeList_tmp ] - 1 ] + hScore -> data [ ( int32_T ) edgeList -> data [ (
tmp_dz -> data [ g_tmp -> data [ last ] / 2 ] << 1 ) + edgeList_tmp ] - 1 ] ;
} } } } } else { componentStart -> size [ 0 ] = 1 ; componentStart -> size [
1 ] = 0 ; exitg1 = 1 ; } } while ( exitg1 == 0 ) ; c2nfvjswuo ( & tmp_dz ) ;
c2nfvjswuo ( & tmp_o ) ; c2nfvjswuo ( & tmp_d ) ; c2nfvjswuo ( & tmp_l ) ;
c2nfvjswuo ( & tmp_n ) ; c2nfvjswuo ( & tmp_b ) ; c2nfvjswuo ( & tmp_k ) ;
c2nfvjswuo ( & tmp_c ) ; c2nfvjswuo ( & tmp_f ) ; c2nfvjswuo ( & tmp_j ) ;
c2nfvjswuo ( & tmp_g ) ; c2nfvjswuo ( & tmp_m ) ; c2nfvjswuo ( & tmp_i ) ;
fizihoqn3p ( & tmp_p ) ; fizihoqn3p ( & tmp ) ; fizihoqn3p ( & b_x_tmp_tmp )
; c2nfvjswuo ( & g_tmp ) ; c2nfvjswuo ( & x_tmp ) ; fizihoqn3p ( & d ) ;
njqibisfwd ( & cameFrom ) ; njqibisfwd ( & openSet ) ; njqibisfwd ( & fScore
) ; njqibisfwd ( & hScore ) ; njqibisfwd ( & gScore ) ; njqibisfwd ( &
closedSet ) ; njqibisfwd ( & edgeList ) ; njqibisfwd ( & nodeList ) ;
njqibisfwd ( & adjacencyMatrix ) ; partialTrueCount = path -> size [ 0 ] *
path -> size [ 1 ] ; path -> size [ 0 ] = 1 ; loop_ub_p = componentStart ->
size [ 1 ] ; path -> size [ 1 ] = componentStart -> size [ 1 ] ; clokujid0y (
path , partialTrueCount ) ; if ( loop_ub_p - 1 >= 0 ) { memcpy ( & path ->
data [ 0 ] , & componentStart -> data [ 0 ] , ( uint32_T ) loop_ub_p * sizeof
( real_T ) ) ; } } njqibisfwd ( & componentGoal ) ; njqibisfwd ( &
componentStart ) ; } static void bej3nj0pvr ( mgywmw0rjk * obj , const real_T
start [ 2 ] , const real_T goal [ 2 ] , k0shasdzrq * path , lwnhcfk22o *
localDW ) { __m128d tmp ; k0shasdzrq * b_varargin_2 ; k0shasdzrq *
c_varargin_2 ; k0shasdzrq * componentGoal ; k0shasdzrq * componentStart ;
k0shasdzrq * gpath ; k0shasdzrq * node ; k0shasdzrq * path_p ; pwx2zp040p *
b_obj ; real_T a [ 4 ] ; real_T start_p [ 2 ] ; real_T gOrig_idx_0 ; real_T
gOrig_idx_1 ; int32_T c_varargin_2_idx_0 ; int32_T exponent ; int32_T
gpath_tmp ; int32_T loop_ub ; int32_T node_tmp ; boolean_T unusedExpr [ 3600
] ; boolean_T b_p ; boolean_T exitg1 ; boolean_T p ; b_obj = obj ->
InternalMap ; gOrig_idx_0 = b_obj -> SharedProperties . GridOriginInLocal [ 0
] ; gOrig_idx_1 = b_obj -> SharedProperties . GridOriginInLocal [ 1 ] ; a [ 0
] = muDoubleScalarAbs ( gOrig_idx_0 ) ; a [ 1 ] = muDoubleScalarAbs (
gOrig_idx_0 + 60.0 ) ; a [ 2 ] = muDoubleScalarAbs ( gOrig_idx_1 ) ; a [ 3 ]
= muDoubleScalarAbs ( gOrig_idx_1 + 60.0 ) ; gOrig_idx_0 = muDoubleScalarAbs
( maximum_w0hsPrr7 ( a ) ) ; if ( ( ! muDoubleScalarIsInf ( gOrig_idx_0 ) )
&& ( ! muDoubleScalarIsNaN ( gOrig_idx_0 ) ) && ( ! ( gOrig_idx_0 <
4.4501477170144028E-308 ) ) ) { frexp ( gOrig_idx_0 , & exponent ) ; } b_obj
= obj -> InternalMap ; mkwt2qmyjw ( b_obj , unusedExpr ) ; b_obj = obj ->
InternalMap ; mkwt2qmyjw ( b_obj , unusedExpr ) ; if ( obj -> UpdateFlag ) {
oung1l0hdz ( obj , localDW ) ; } b_obj = obj -> InternalMap ; if ( obj ->
PathFinder . isInitialized != 1 ) { obj -> PathFinder . isInitialized = 1 ; }
path -> size [ 0 ] = 0 ; path -> size [ 1 ] = 0 ; p = false ; b_p = true ;
exponent = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( exponent < 2 ) ) {
if ( ! ( start [ exponent ] == goal [ exponent ] ) ) { b_p = false ; exitg1 =
true ; } else { exponent ++ ; } } if ( b_p ) { p = true ; } if ( p ) {
exponent = path -> size [ 0 ] * path -> size [ 1 ] ; path -> size [ 0 ] = 1 ;
path -> size [ 1 ] = 2 ; clokujid0y ( path , exponent ) ; path -> data [ 0 ]
= start [ 0 ] ; path -> data [ 1 ] = start [ 1 ] ; } else { gOrig_idx_0 =
le2qxtov41 ( goal , & obj -> RoadmapBuilder . Roadmap , b_obj ) ; gOrig_idx_1
= le2qxtov41 ( start , & obj -> RoadmapBuilder . Roadmap , b_obj ) ; if ( ( !
muDoubleScalarIsNaN ( gOrig_idx_0 ) ) && ( ! muDoubleScalarIsNaN (
gOrig_idx_1 ) ) ) { k5xyyjjkd4 ( & componentStart , 2 ) ; fsaynwsdln ( & obj
-> RoadmapBuilder . Roadmap , gOrig_idx_1 , componentStart ) ; k5xyyjjkd4 ( &
componentGoal , 2 ) ; fsaynwsdln ( & obj -> RoadmapBuilder . Roadmap ,
gOrig_idx_0 , componentGoal ) ; if ( hulg2uozzd ( componentStart ,
componentGoal ) ) { k5xyyjjkd4 ( & node , 2 ) ; j2eqhook05 ( & obj ->
RoadmapBuilder . Roadmap , gOrig_idx_1 , gOrig_idx_0 , node ) ; node_tmp =
node -> size [ 0 ] * node -> size [ 1 ] ; k5xyyjjkd4 ( & gpath , 2 ) ;
exponent = gpath -> size [ 0 ] * gpath -> size [ 1 ] ; gpath -> size [ 0 ] =
2 ; gpath -> size [ 1 ] = node_tmp ; clokujid0y ( gpath , exponent ) ; for (
exponent = 0 ; exponent < node_tmp ; exponent ++ ) { gpath_tmp = ( int32_T )
node -> data [ exponent ] - 1 ; gpath -> data [ exponent << 1 ] = obj ->
RoadmapBuilder . Roadmap . Nodes -> data [ gpath_tmp << 1 ] ; gpath -> data [
1 + ( exponent << 1 ) ] = obj -> RoadmapBuilder . Roadmap . Nodes -> data [ (
gpath_tmp << 1 ) + 1 ] ; } njqibisfwd ( & node ) ; tmp = _mm_sub_pd (
_mm_loadu_pd ( & start [ 0 ] ) , _mm_loadu_pd ( & gpath -> data [ 0 ] ) ) ;
_mm_storeu_pd ( & start_p [ 0 ] , tmp ) ; if ( norm_LxdYsY7z ( start_p ) <
2.2204460492503131E-16 ) { k5xyyjjkd4 ( & b_varargin_2 , 1 ) ; exponent =
b_varargin_2 -> size [ 0 ] ; b_varargin_2 -> size [ 0 ] = node_tmp ;
clokujid0y ( b_varargin_2 , exponent ) ; for ( exponent = 0 ; exponent <
node_tmp ; exponent ++ ) { b_varargin_2 -> data [ exponent ] = gpath -> data
[ ( exponent << 1 ) + 1 ] ; } if ( b_varargin_2 -> size [ 0 ] != 0 ) {
c_varargin_2_idx_0 = node_tmp ; } else { c_varargin_2_idx_0 = 0 ; } exponent
= path -> size [ 0 ] * path -> size [ 1 ] ; path -> size [ 0 ] =
c_varargin_2_idx_0 + 1 ; path -> size [ 1 ] = 2 ; clokujid0y ( path ,
exponent ) ; path -> data [ 0 ] = start [ 0 ] ; path -> data [ path -> size [
0 ] ] = start [ 1 ] ; for ( exponent = 0 ; exponent < 2 ; exponent ++ ) { for
( gpath_tmp = 0 ; gpath_tmp < c_varargin_2_idx_0 ; gpath_tmp ++ ) { path ->
data [ ( gpath_tmp + path -> size [ 0 ] * exponent ) + 1 ] = b_varargin_2 ->
data [ c_varargin_2_idx_0 * exponent + gpath_tmp ] ; } } njqibisfwd ( &
b_varargin_2 ) ; } else { k5xyyjjkd4 ( & c_varargin_2 , 2 ) ; exponent =
c_varargin_2 -> size [ 0 ] * c_varargin_2 -> size [ 1 ] ; c_varargin_2 ->
size [ 0 ] = node_tmp ; c_varargin_2 -> size [ 1 ] = 2 ; clokujid0y (
c_varargin_2 , exponent ) ; for ( exponent = 0 ; exponent < 2 ; exponent ++ )
{ for ( gpath_tmp = 0 ; gpath_tmp < node_tmp ; gpath_tmp ++ ) { c_varargin_2
-> data [ gpath_tmp + c_varargin_2 -> size [ 0 ] * exponent ] = gpath -> data
[ ( gpath_tmp << 1 ) + exponent ] ; } } if ( c_varargin_2 -> size [ 0 ] != 0
) { c_varargin_2_idx_0 = node_tmp ; } else { c_varargin_2_idx_0 = 0 ; }
exponent = path -> size [ 0 ] * path -> size [ 1 ] ; path -> size [ 0 ] =
c_varargin_2_idx_0 + 1 ; path -> size [ 1 ] = 2 ; clokujid0y ( path ,
exponent ) ; path -> data [ 0 ] = start [ 0 ] ; path -> data [ path -> size [
0 ] ] = start [ 1 ] ; for ( exponent = 0 ; exponent < 2 ; exponent ++ ) { for
( gpath_tmp = 0 ; gpath_tmp < c_varargin_2_idx_0 ; gpath_tmp ++ ) { path ->
data [ ( gpath_tmp + path -> size [ 0 ] * exponent ) + 1 ] = c_varargin_2 ->
data [ c_varargin_2_idx_0 * exponent + gpath_tmp ] ; } } njqibisfwd ( &
c_varargin_2 ) ; } tmp = _mm_sub_pd ( _mm_loadu_pd ( & goal [ 0 ] ) ,
_mm_loadu_pd ( & gpath -> data [ ( gpath -> size [ 1 ] - 1 ) << 1 ] ) ) ;
njqibisfwd ( & gpath ) ; _mm_storeu_pd ( & start_p [ 0 ] , tmp ) ; if (
norm_LxdYsY7z ( start_p ) > 2.2204460492503131E-16 ) { loop_ub = path -> size
[ 0 ] ; c_varargin_2_idx_0 = path -> size [ 0 ] ; k5xyyjjkd4 ( & path_p , 2 )
; node_tmp = path -> size [ 0 ] + 1 ; exponent = path_p -> size [ 0 ] *
path_p -> size [ 1 ] ; path_p -> size [ 0 ] = path -> size [ 0 ] + 1 ; path_p
-> size [ 1 ] = 2 ; clokujid0y ( path_p , exponent ) ; for ( exponent = 0 ;
exponent < 2 ; exponent ++ ) { for ( gpath_tmp = 0 ; gpath_tmp < loop_ub ;
gpath_tmp ++ ) { path_p -> data [ gpath_tmp + path_p -> size [ 0 ] * exponent
] = path -> data [ c_varargin_2_idx_0 * exponent + gpath_tmp ] ; } path_p ->
data [ loop_ub + path_p -> size [ 0 ] * exponent ] = goal [ exponent ] ; }
exponent = path -> size [ 0 ] * path -> size [ 1 ] ; path -> size [ 0 ] =
node_tmp ; path -> size [ 1 ] = 2 ; clokujid0y ( path , exponent ) ; loop_ub
= path_p -> size [ 0 ] << 1 ; if ( loop_ub - 1 >= 0 ) { memcpy ( & path ->
data [ 0 ] , & path_p -> data [ 0 ] , ( uint32_T ) loop_ub * sizeof ( real_T
) ) ; } njqibisfwd ( & path_p ) ; } } njqibisfwd ( & componentGoal ) ;
njqibisfwd ( & componentStart ) ; } } } void a3uz1fjcva ( RobotPackageStatus
* ab3l4ewxjl , dheucvhz3u * localB , lwnhcfk22o * localDW ) { __m128d tmp ;
__m128d tmp_p ; static const uint32_T tmp_e [ 625 ] = { 5489U , 1301868182U ,
2938499221U , 2950281878U , 1875628136U , 751856242U , 944701696U ,
2243192071U , 694061057U , 219885934U , 2066767472U , 3182869408U ,
485472502U , 2336857883U , 1071588843U , 3418470598U , 951210697U ,
3693558366U , 2923482051U , 1793174584U , 2982310801U , 1586906132U ,
1951078751U , 1808158765U , 1733897588U , 431328322U , 4202539044U ,
530658942U , 1714810322U , 3025256284U , 3342585396U , 1937033938U ,
2640572511U , 1654299090U , 3692403553U , 4233871309U , 3497650794U ,
862629010U , 2943236032U , 2426458545U , 1603307207U , 1133453895U ,
3099196360U , 2208657629U , 2747653927U , 931059398U , 761573964U ,
3157853227U , 785880413U , 730313442U , 124945756U , 2937117055U ,
3295982469U , 1724353043U , 3021675344U , 3884886417U , 4010150098U ,
4056961966U , 699635835U , 2681338818U , 1339167484U , 720757518U ,
2800161476U , 2376097373U , 1532957371U , 3902664099U , 1238982754U ,
3725394514U , 3449176889U , 3570962471U , 4287636090U , 4087307012U ,
3603343627U , 202242161U , 2995682783U , 1620962684U , 3704723357U ,
371613603U , 2814834333U , 2111005706U , 624778151U , 2094172212U ,
4284947003U , 1211977835U , 991917094U , 1570449747U , 2962370480U ,
1259410321U , 170182696U , 146300961U , 2836829791U , 619452428U ,
2723670296U , 1881399711U , 1161269684U , 1675188680U , 4132175277U ,
780088327U , 3409462821U , 1036518241U , 1834958505U , 3048448173U ,
161811569U , 618488316U , 44795092U , 3918322701U , 1924681712U , 3239478144U
, 383254043U , 4042306580U , 2146983041U , 3992780527U , 3518029708U ,
3545545436U , 3901231469U , 1896136409U , 2028528556U , 2339662006U ,
501326714U , 2060962201U , 2502746480U , 561575027U , 581893337U ,
3393774360U , 1778912547U , 3626131687U , 2175155826U , 319853231U ,
986875531U , 819755096U , 2915734330U , 2688355739U , 3482074849U , 2736559U
, 2296975761U , 1029741190U , 2876812646U , 690154749U , 579200347U ,
4027461746U , 1285330465U , 2701024045U , 4117700889U , 759495121U ,
3332270341U , 2313004527U , 2277067795U , 4131855432U , 2722057515U ,
1264804546U , 3848622725U , 2211267957U , 4100593547U , 959123777U ,
2130745407U , 3194437393U , 486673947U , 1377371204U , 17472727U , 352317554U
, 3955548058U , 159652094U , 1232063192U , 3835177280U , 49423123U ,
3083993636U , 733092U , 2120519771U , 2573409834U , 1112952433U , 3239502554U
, 761045320U , 1087580692U , 2540165110U , 641058802U , 1792435497U ,
2261799288U , 1579184083U , 627146892U , 2165744623U , 2200142389U ,
2167590760U , 2381418376U , 1793358889U , 3081659520U , 1663384067U ,
2009658756U , 2689600308U , 739136266U , 2304581039U , 3529067263U ,
591360555U , 525209271U , 3131882996U , 294230224U , 2076220115U ,
3113580446U , 1245621585U , 1386885462U , 3203270426U , 123512128U ,
12350217U , 354956375U , 4282398238U , 3356876605U , 3888857667U , 157639694U
, 2616064085U , 1563068963U , 2762125883U , 4045394511U , 4180452559U ,
3294769488U , 1684529556U , 1002945951U , 3181438866U , 22506664U ,
691783457U , 2685221343U , 171579916U , 3878728600U , 2475806724U ,
2030324028U , 3331164912U , 1708711359U , 1970023127U , 2859691344U ,
2588476477U , 2748146879U , 136111222U , 2967685492U , 909517429U ,
2835297809U , 3206906216U , 3186870716U , 341264097U , 2542035121U ,
3353277068U , 548223577U , 3170936588U , 1678403446U , 297435620U ,
2337555430U , 466603495U , 1132321815U , 1208589219U , 696392160U ,
894244439U , 2562678859U , 470224582U , 3306867480U , 201364898U ,
2075966438U , 1767227936U , 2929737987U , 3674877796U , 2654196643U ,
3692734598U , 3528895099U , 2796780123U , 3048728353U , 842329300U ,
191554730U , 2922459673U , 3489020079U , 3979110629U , 1022523848U ,
2202932467U , 3583655201U , 3565113719U , 587085778U , 4176046313U ,
3013713762U , 950944241U , 396426791U , 3784844662U , 3477431613U ,
3594592395U , 2782043838U , 3392093507U , 3106564952U , 2829419931U ,
1358665591U , 2206918825U , 3170783123U , 31522386U , 2988194168U ,
1782249537U , 1105080928U , 843500134U , 1225290080U , 1521001832U ,
3605886097U , 2802786495U , 2728923319U , 3996284304U , 903417639U ,
1171249804U , 1020374987U , 2824535874U , 423621996U , 1988534473U ,
2493544470U , 1008604435U , 1756003503U , 1488867287U , 1386808992U ,
732088248U , 1780630732U , 2482101014U , 976561178U , 1543448953U ,
2602866064U , 2021139923U , 1952599828U , 2360242564U , 2117959962U ,
2753061860U , 2388623612U , 4138193781U , 2962920654U , 2284970429U ,
766920861U , 3457264692U , 2879611383U , 815055854U , 2332929068U ,
1254853997U , 3740375268U , 3799380844U , 4091048725U , 2006331129U ,
1982546212U , 686850534U , 1907447564U , 2682801776U , 2780821066U ,
998290361U , 1342433871U , 4195430425U , 607905174U , 3902331779U ,
2454067926U , 1708133115U , 1170874362U , 2008609376U , 3260320415U ,
2211196135U , 433538229U , 2728786374U , 2189520818U , 262554063U ,
1182318347U , 3710237267U , 1221022450U , 715966018U , 2417068910U ,
2591870721U , 2870691989U , 3418190842U , 4238214053U , 1540704231U ,
1575580968U , 2095917976U , 4078310857U , 2313532447U , 2110690783U ,
4056346629U , 4061784526U , 1123218514U , 551538993U , 597148360U ,
4120175196U , 3581618160U , 3181170517U , 422862282U , 3227524138U ,
1713114790U , 662317149U , 1230418732U , 928171837U , 1324564878U ,
1928816105U , 1786535431U , 2878099422U , 3290185549U , 539474248U ,
1657512683U , 552370646U , 1671741683U , 3655312128U , 1552739510U ,
2605208763U , 1441755014U , 181878989U , 3124053868U , 1447103986U ,
3183906156U , 1728556020U , 3502241336U , 3055466967U , 1013272474U ,
818402132U , 1715099063U , 2900113506U , 397254517U , 4194863039U ,
1009068739U , 232864647U , 2540223708U , 2608288560U , 2415367765U ,
478404847U , 3455100648U , 3182600021U , 2115988978U , 434269567U ,
4117179324U , 3461774077U , 887256537U , 3545801025U , 286388911U ,
3451742129U , 1981164769U , 786667016U , 3310123729U , 3097811076U ,
2224235657U , 2959658883U , 3370969234U , 2514770915U , 3345656436U ,
2677010851U , 2206236470U , 271648054U , 2342188545U , 4292848611U ,
3646533909U , 3754009956U , 3803931226U , 4160647125U , 1477814055U ,
4043852216U , 1876372354U , 3133294443U , 3871104810U , 3177020907U ,
2074304428U , 3479393793U , 759562891U , 164128153U , 1839069216U ,
2114162633U , 3989947309U , 3611054956U , 1333547922U , 835429831U ,
494987340U , 171987910U , 1252001001U , 370809172U , 3508925425U ,
2535703112U , 1276855041U , 1922855120U , 835673414U , 3030664304U ,
613287117U , 171219893U , 3423096126U , 3376881639U , 2287770315U ,
1658692645U , 1262815245U , 3957234326U , 1168096164U , 2968737525U ,
2655813712U , 2132313144U , 3976047964U , 326516571U , 353088456U ,
3679188938U , 3205649712U , 2654036126U , 1249024881U , 880166166U ,
691800469U , 2229503665U , 1673458056U , 4032208375U , 1851778863U ,
2563757330U , 376742205U , 1794655231U , 340247333U , 1505873033U ,
396524441U , 879666767U , 3335579166U , 3260764261U , 3335999539U ,
506221798U , 4214658741U , 975887814U , 2080536343U , 3360539560U ,
571586418U , 138896374U , 4234352651U , 2737620262U , 3928362291U ,
1516365296U , 38056726U , 3599462320U , 3585007266U , 3850961033U ,
471667319U , 1536883193U , 2310166751U , 1861637689U , 2530999841U ,
4139843801U , 2710569485U , 827578615U , 2012334720U , 2907369459U ,
3029312804U , 2820112398U , 1965028045U , 35518606U , 2478379033U ,
643747771U , 1924139484U , 4123405127U , 3811735531U , 3429660832U ,
3285177704U , 1948416081U , 1311525291U , 1183517742U , 1739192232U ,
3979815115U , 2567840007U , 4116821529U , 213304419U , 4125718577U ,
1473064925U , 2442436592U , 1893310111U , 4195361916U , 3747569474U ,
828465101U , 2991227658U , 750582866U , 1205170309U , 1409813056U ,
678418130U , 1171531016U , 3821236156U , 354504587U , 4202874632U ,
3882511497U , 1893248677U , 1903078632U , 26340130U , 2069166240U ,
3657122492U , 3725758099U , 831344905U , 811453383U , 3447711422U ,
2434543565U , 4166886888U , 3358210805U , 4142984013U , 2988152326U ,
3527824853U , 982082992U , 2809155763U , 190157081U , 3340214818U ,
2365432395U , 2548636180U , 2894533366U , 3474657421U , 2372634704U ,
2845748389U , 43024175U , 2774226648U , 1987702864U , 3186502468U ,
453610222U , 4204736567U , 1392892630U , 2471323686U , 2470534280U ,
3541393095U , 4269885866U , 3909911300U , 759132955U , 1482612480U ,
667715263U , 1795580598U , 2337923983U , 3390586366U , 581426223U ,
1515718634U , 476374295U , 705213300U , 363062054U , 2084697697U ,
2407503428U , 2292957699U , 2426213835U , 2199989172U , 1987356470U ,
4026755612U , 2147252133U , 270400031U , 1367820199U , 2369854699U ,
2844269403U , 79981964U , 624U } ; localDW -> avbny3pgeq = i4xuulnjq5 . P_7 ;
localDW -> fyhxlfl0zl = 0U ; localB -> a1ty03ixk2 = 0.0 ; localB ->
p55quualew = RobotState_AtDock ; localB -> ocoypaore1 [ 0 ] = 0.0 ; localB ->
aix3fm0ysj [ 0 ] = 0.0 ; localB -> ocoypaore1 [ 1 ] = 0.0 ; localB ->
aix3fm0ysj [ 1 ] = 0.0 ; localDW -> i2zecotakp = 500.0 ; localDW ->
c0setyxykh = 0U ; localDW -> h032gcefu3 = jiscqp1m4h ; ab3l4ewxjl ->
hasPackage = 0.0 ; ab3l4ewxjl -> planningState = RobotState_AtDock ; memcpy (
& localDW -> pggahx0edn [ 0 ] , & tmp_e [ 0 ] , 625U * sizeof ( uint32_T ) )
; localDW -> ftjxsixfdm = false ; localDW -> ls4ddqxq54 = false ; localDW ->
oafkuis4gg = false ; tmp_p = _mm_set1_pd ( 0.0 ) ; tmp = _mm_mul_pd ( tmp_p ,
_mm_loadu_pd ( & localDW -> ljuqtcenlo . LookaheadPoint [ 0 ] ) ) ;
_mm_storeu_pd ( & localDW -> ljuqtcenlo . LookaheadPoint [ 0 ] , tmp ) ;
tmp_p = _mm_mul_pd ( tmp_p , _mm_loadu_pd ( & localDW -> ljuqtcenlo .
LastPose [ 0 ] ) ) ; _mm_storeu_pd ( & localDW -> ljuqtcenlo . LastPose [ 0 ]
, tmp_p ) ; localDW -> ljuqtcenlo . LastPose [ 2 ] *= 0.0 ; localDW ->
ljuqtcenlo . ProjectionPoint [ 0 ] = ( rtNaN ) ; localDW -> ljuqtcenlo .
ProjectionPoint [ 1 ] = ( rtNaN ) ; localDW -> ljuqtcenlo .
ProjectionLineIndex *= 0.0 ; } void mj0lr24tvd ( dheucvhz3u * localB ,
lwnhcfk22o * localDW ) { __m128d tmp_p ; real_T tmp [ 2 ] ; static const
uint32_T tmp_e [ 625 ] = { 5489U , 1301868182U , 2938499221U , 2950281878U ,
1875628136U , 751856242U , 944701696U , 2243192071U , 694061057U , 219885934U
, 2066767472U , 3182869408U , 485472502U , 2336857883U , 1071588843U ,
3418470598U , 951210697U , 3693558366U , 2923482051U , 1793174584U ,
2982310801U , 1586906132U , 1951078751U , 1808158765U , 1733897588U ,
431328322U , 4202539044U , 530658942U , 1714810322U , 3025256284U ,
3342585396U , 1937033938U , 2640572511U , 1654299090U , 3692403553U ,
4233871309U , 3497650794U , 862629010U , 2943236032U , 2426458545U ,
1603307207U , 1133453895U , 3099196360U , 2208657629U , 2747653927U ,
931059398U , 761573964U , 3157853227U , 785880413U , 730313442U , 124945756U
, 2937117055U , 3295982469U , 1724353043U , 3021675344U , 3884886417U ,
4010150098U , 4056961966U , 699635835U , 2681338818U , 1339167484U ,
720757518U , 2800161476U , 2376097373U , 1532957371U , 3902664099U ,
1238982754U , 3725394514U , 3449176889U , 3570962471U , 4287636090U ,
4087307012U , 3603343627U , 202242161U , 2995682783U , 1620962684U ,
3704723357U , 371613603U , 2814834333U , 2111005706U , 624778151U ,
2094172212U , 4284947003U , 1211977835U , 991917094U , 1570449747U ,
2962370480U , 1259410321U , 170182696U , 146300961U , 2836829791U ,
619452428U , 2723670296U , 1881399711U , 1161269684U , 1675188680U ,
4132175277U , 780088327U , 3409462821U , 1036518241U , 1834958505U ,
3048448173U , 161811569U , 618488316U , 44795092U , 3918322701U , 1924681712U
, 3239478144U , 383254043U , 4042306580U , 2146983041U , 3992780527U ,
3518029708U , 3545545436U , 3901231469U , 1896136409U , 2028528556U ,
2339662006U , 501326714U , 2060962201U , 2502746480U , 561575027U ,
581893337U , 3393774360U , 1778912547U , 3626131687U , 2175155826U ,
319853231U , 986875531U , 819755096U , 2915734330U , 2688355739U ,
3482074849U , 2736559U , 2296975761U , 1029741190U , 2876812646U , 690154749U
, 579200347U , 4027461746U , 1285330465U , 2701024045U , 4117700889U ,
759495121U , 3332270341U , 2313004527U , 2277067795U , 4131855432U ,
2722057515U , 1264804546U , 3848622725U , 2211267957U , 4100593547U ,
959123777U , 2130745407U , 3194437393U , 486673947U , 1377371204U , 17472727U
, 352317554U , 3955548058U , 159652094U , 1232063192U , 3835177280U ,
49423123U , 3083993636U , 733092U , 2120519771U , 2573409834U , 1112952433U ,
3239502554U , 761045320U , 1087580692U , 2540165110U , 641058802U ,
1792435497U , 2261799288U , 1579184083U , 627146892U , 2165744623U ,
2200142389U , 2167590760U , 2381418376U , 1793358889U , 3081659520U ,
1663384067U , 2009658756U , 2689600308U , 739136266U , 2304581039U ,
3529067263U , 591360555U , 525209271U , 3131882996U , 294230224U ,
2076220115U , 3113580446U , 1245621585U , 1386885462U , 3203270426U ,
123512128U , 12350217U , 354956375U , 4282398238U , 3356876605U , 3888857667U
, 157639694U , 2616064085U , 1563068963U , 2762125883U , 4045394511U ,
4180452559U , 3294769488U , 1684529556U , 1002945951U , 3181438866U ,
22506664U , 691783457U , 2685221343U , 171579916U , 3878728600U , 2475806724U
, 2030324028U , 3331164912U , 1708711359U , 1970023127U , 2859691344U ,
2588476477U , 2748146879U , 136111222U , 2967685492U , 909517429U ,
2835297809U , 3206906216U , 3186870716U , 341264097U , 2542035121U ,
3353277068U , 548223577U , 3170936588U , 1678403446U , 297435620U ,
2337555430U , 466603495U , 1132321815U , 1208589219U , 696392160U ,
894244439U , 2562678859U , 470224582U , 3306867480U , 201364898U ,
2075966438U , 1767227936U , 2929737987U , 3674877796U , 2654196643U ,
3692734598U , 3528895099U , 2796780123U , 3048728353U , 842329300U ,
191554730U , 2922459673U , 3489020079U , 3979110629U , 1022523848U ,
2202932467U , 3583655201U , 3565113719U , 587085778U , 4176046313U ,
3013713762U , 950944241U , 396426791U , 3784844662U , 3477431613U ,
3594592395U , 2782043838U , 3392093507U , 3106564952U , 2829419931U ,
1358665591U , 2206918825U , 3170783123U , 31522386U , 2988194168U ,
1782249537U , 1105080928U , 843500134U , 1225290080U , 1521001832U ,
3605886097U , 2802786495U , 2728923319U , 3996284304U , 903417639U ,
1171249804U , 1020374987U , 2824535874U , 423621996U , 1988534473U ,
2493544470U , 1008604435U , 1756003503U , 1488867287U , 1386808992U ,
732088248U , 1780630732U , 2482101014U , 976561178U , 1543448953U ,
2602866064U , 2021139923U , 1952599828U , 2360242564U , 2117959962U ,
2753061860U , 2388623612U , 4138193781U , 2962920654U , 2284970429U ,
766920861U , 3457264692U , 2879611383U , 815055854U , 2332929068U ,
1254853997U , 3740375268U , 3799380844U , 4091048725U , 2006331129U ,
1982546212U , 686850534U , 1907447564U , 2682801776U , 2780821066U ,
998290361U , 1342433871U , 4195430425U , 607905174U , 3902331779U ,
2454067926U , 1708133115U , 1170874362U , 2008609376U , 3260320415U ,
2211196135U , 433538229U , 2728786374U , 2189520818U , 262554063U ,
1182318347U , 3710237267U , 1221022450U , 715966018U , 2417068910U ,
2591870721U , 2870691989U , 3418190842U , 4238214053U , 1540704231U ,
1575580968U , 2095917976U , 4078310857U , 2313532447U , 2110690783U ,
4056346629U , 4061784526U , 1123218514U , 551538993U , 597148360U ,
4120175196U , 3581618160U , 3181170517U , 422862282U , 3227524138U ,
1713114790U , 662317149U , 1230418732U , 928171837U , 1324564878U ,
1928816105U , 1786535431U , 2878099422U , 3290185549U , 539474248U ,
1657512683U , 552370646U , 1671741683U , 3655312128U , 1552739510U ,
2605208763U , 1441755014U , 181878989U , 3124053868U , 1447103986U ,
3183906156U , 1728556020U , 3502241336U , 3055466967U , 1013272474U ,
818402132U , 1715099063U , 2900113506U , 397254517U , 4194863039U ,
1009068739U , 232864647U , 2540223708U , 2608288560U , 2415367765U ,
478404847U , 3455100648U , 3182600021U , 2115988978U , 434269567U ,
4117179324U , 3461774077U , 887256537U , 3545801025U , 286388911U ,
3451742129U , 1981164769U , 786667016U , 3310123729U , 3097811076U ,
2224235657U , 2959658883U , 3370969234U , 2514770915U , 3345656436U ,
2677010851U , 2206236470U , 271648054U , 2342188545U , 4292848611U ,
3646533909U , 3754009956U , 3803931226U , 4160647125U , 1477814055U ,
4043852216U , 1876372354U , 3133294443U , 3871104810U , 3177020907U ,
2074304428U , 3479393793U , 759562891U , 164128153U , 1839069216U ,
2114162633U , 3989947309U , 3611054956U , 1333547922U , 835429831U ,
494987340U , 171987910U , 1252001001U , 370809172U , 3508925425U ,
2535703112U , 1276855041U , 1922855120U , 835673414U , 3030664304U ,
613287117U , 171219893U , 3423096126U , 3376881639U , 2287770315U ,
1658692645U , 1262815245U , 3957234326U , 1168096164U , 2968737525U ,
2655813712U , 2132313144U , 3976047964U , 326516571U , 353088456U ,
3679188938U , 3205649712U , 2654036126U , 1249024881U , 880166166U ,
691800469U , 2229503665U , 1673458056U , 4032208375U , 1851778863U ,
2563757330U , 376742205U , 1794655231U , 340247333U , 1505873033U ,
396524441U , 879666767U , 3335579166U , 3260764261U , 3335999539U ,
506221798U , 4214658741U , 975887814U , 2080536343U , 3360539560U ,
571586418U , 138896374U , 4234352651U , 2737620262U , 3928362291U ,
1516365296U , 38056726U , 3599462320U , 3585007266U , 3850961033U ,
471667319U , 1536883193U , 2310166751U , 1861637689U , 2530999841U ,
4139843801U , 2710569485U , 827578615U , 2012334720U , 2907369459U ,
3029312804U , 2820112398U , 1965028045U , 35518606U , 2478379033U ,
643747771U , 1924139484U , 4123405127U , 3811735531U , 3429660832U ,
3285177704U , 1948416081U , 1311525291U , 1183517742U , 1739192232U ,
3979815115U , 2567840007U , 4116821529U , 213304419U , 4125718577U ,
1473064925U , 2442436592U , 1893310111U , 4195361916U , 3747569474U ,
828465101U , 2991227658U , 750582866U , 1205170309U , 1409813056U ,
678418130U , 1171531016U , 3821236156U , 354504587U , 4202874632U ,
3882511497U , 1893248677U , 1903078632U , 26340130U , 2069166240U ,
3657122492U , 3725758099U , 831344905U , 811453383U , 3447711422U ,
2434543565U , 4166886888U , 3358210805U , 4142984013U , 2988152326U ,
3527824853U , 982082992U , 2809155763U , 190157081U , 3340214818U ,
2365432395U , 2548636180U , 2894533366U , 3474657421U , 2372634704U ,
2845748389U , 43024175U , 2774226648U , 1987702864U , 3186502468U ,
453610222U , 4204736567U , 1392892630U , 2471323686U , 2470534280U ,
3541393095U , 4269885866U , 3909911300U , 759132955U , 1482612480U ,
667715263U , 1795580598U , 2337923983U , 3390586366U , 581426223U ,
1515718634U , 476374295U , 705213300U , 363062054U , 2084697697U ,
2407503428U , 2292957699U , 2426213835U , 2199989172U , 1987356470U ,
4026755612U , 2147252133U , 270400031U , 1367820199U , 2369854699U ,
2844269403U , 79981964U , 624U } ; localDW -> avbny3pgeq = i4xuulnjq5 . P_7 ;
memcpy ( & localDW -> pggahx0edn [ 0 ] , & tmp_e [ 0 ] , 625U * sizeof (
uint32_T ) ) ; localDW -> ftjxsixfdm = false ; localDW -> ls4ddqxq54 = false
; localDW -> oafkuis4gg = false ; localDW -> fyhxlfl0zl = 0U ; localB ->
a1ty03ixk2 = 0.0 ; localB -> p55quualew = RobotState_AtDock ; localDW ->
i2zecotakp = 500.0 ; localDW -> c0setyxykh = 0U ; localDW -> h032gcefu3 =
jiscqp1m4h ; localB -> ocoypaore1 [ 0 ] = 0.0 ; localB -> aix3fm0ysj [ 0 ] =
0.0 ; localDW -> ljuqtcenlo . LookaheadPoint [ 0 ] *= 0.0 ; localB ->
ocoypaore1 [ 1 ] = 0.0 ; localB -> aix3fm0ysj [ 1 ] = 0.0 ; tmp_p =
_mm_set1_pd ( 0.0 ) ; _mm_storeu_pd ( & tmp [ 0 ] , _mm_mul_pd ( tmp_p ,
_mm_set_pd ( localDW -> ljuqtcenlo . LastPose [ 0 ] , localDW -> ljuqtcenlo .
LookaheadPoint [ 1 ] ) ) ) ; localDW -> ljuqtcenlo . LookaheadPoint [ 1 ] =
tmp [ 0 ] ; localDW -> ljuqtcenlo . LastPose [ 0 ] = tmp [ 1 ] ; tmp_p =
_mm_mul_pd ( tmp_p , _mm_loadu_pd ( & localDW -> ljuqtcenlo . LastPose [ 1 ]
) ) ; _mm_storeu_pd ( & localDW -> ljuqtcenlo . LastPose [ 1 ] , tmp_p ) ;
localDW -> ljuqtcenlo . ProjectionPoint [ 0 ] = ( rtNaN ) ; localDW ->
ljuqtcenlo . ProjectionPoint [ 1 ] = ( rtNaN ) ; localDW -> ljuqtcenlo .
ProjectionLineIndex *= 0.0 ; } void fh441y4mj5 ( lwnhcfk22o * localDW ) {
int32_T i ; static const int8_T tmp [ 8 ] = { 100 , 2 , 1 , 1 , 1 , 1 , 1 , 1
} ; localDW -> ey3alnk1or = true ; localDW -> ljuqtcenlo .
DesiredLinearVelocity = i4xuulnjq5 . P_4 ; localDW -> ljuqtcenlo .
MaxAngularVelocity = i4xuulnjq5 . P_5 ; localDW -> ljuqtcenlo .
LookaheadDistance = i4xuulnjq5 . P_6 ; localDW -> ljuqtcenlo . isInitialized
= 1 ; for ( i = 0 ; i < 8 ; i ++ ) { localDW -> ljuqtcenlo . inputVarSize [ 0
] . f1 [ i ] = 1U ; localDW -> ljuqtcenlo . inputVarSize [ 1 ] . f1 [ i ] = (
uint32_T ) tmp [ i ] ; } for ( i = 0 ; i < 200 ; i ++ ) { localDW ->
ljuqtcenlo . WaypointsInternal [ i ] = ( rtNaN ) ; } localDW -> ljuqtcenlo .
LookaheadPoint [ 0 ] = 0.0 ; localDW -> ljuqtcenlo . LookaheadPoint [ 1 ] =
0.0 ; localDW -> ljuqtcenlo . LastPose [ 0 ] = 0.0 ; localDW -> ljuqtcenlo .
LastPose [ 1 ] = 0.0 ; localDW -> ljuqtcenlo . LastPose [ 2 ] = 0.0 ; localDW
-> ljuqtcenlo . ProjectionPoint [ 0 ] = ( rtNaN ) ; localDW -> ljuqtcenlo .
ProjectionPoint [ 1 ] = ( rtNaN ) ; localDW -> ljuqtcenlo .
ProjectionLineIndex = 0.0 ; localDW -> ljuqtcenlo . TunablePropsChanged =
false ; } void robotController ( const RobotDeliverCommand * irwasht4mi ,
const real_T m2qskxe3uq [ 3 ] , real_T hy2oo0lvw3 [ 2 ] , RobotPackageStatus
* ab3l4ewxjl , real_T kbkkhgksqy [ 200 ] , dheucvhz3u * localB , lwnhcfk22o *
localDW ) { __m128d tmp_e ; __m128d tmp_i ; k0shasdzrq * xy ; kg3nyqw0nj *
b_index ; mgywmw0rjk prm ; real_T jfuphejcdb [ 2 ] ; real_T kbkkhgksqy_e [ 2
] ; real_T kbkkhgksqy_p [ 2 ] ; real_T tmp_p [ 2 ] ; real_T dist ; real_T
kbkkhgksqy_tmp ; real_T lookaheadEndPt_idx_0 ; real_T lookaheadEndPt_idx_1 ;
real_T lookaheadIdx ; real_T minDistance ; int32_T i ; int32_T ibmat ;
int32_T kbkkhgksqy_tmp_tmp ; int32_T prm_p ; int8_T tmp_data [ 100 ] ;
boolean_T b [ 200 ] ; boolean_T tmp [ 100 ] ; boolean_T b_p ; boolean_T
searchFlag ; static const int8_T inSize [ 8 ] = { 100 , 2 , 1 , 1 , 1 , 1 , 1
, 1 } ; int32_T tmp_size_idx_0 ; boolean_T exitg1 ; jkvo50kvdt ( & prm ) ; if
( localDW -> fyhxlfl0zl < MAX_uint32_T ) { localDW -> fyhxlfl0zl ++ ; } if (
localDW -> c0setyxykh == 0U ) { localDW -> c0setyxykh = 1U ; localB ->
a1ty03ixk2 = 0.0 ; localDW -> h032gcefu3 = ltsv3qsxc4 ; localB -> aix3fm0ysj
[ 0 ] = rtP_loadingStation [ 0 ] ; localB -> ocoypaore1 [ 0 ] = m2qskxe3uq [
0 ] ; localB -> aix3fm0ysj [ 1 ] = rtP_loadingStation [ 1 ] ; localB ->
ocoypaore1 [ 1 ] = m2qskxe3uq [ 1 ] ; } else { switch ( localDW -> h032gcefu3
) { case ltsv3qsxc4 : if ( ( localB -> a1ty03ixk2 != 0.0 ) && ( irwasht4mi ->
package != 0.0 ) ) { localDW -> fyhxlfl0zl = 0U ; localDW -> h032gcefu3 =
dwrx5ivjph ; } else if ( ! localDW -> avbny3pgeq ) { localB -> p55quualew =
RobotState_AtLoadingStn ; if ( irwasht4mi -> givenPackage != 0.0 ) { localB
-> a1ty03ixk2 = 1.0 ; } } break ; case oywowyxvde : if ( ! localDW ->
avbny3pgeq ) { localB -> p55quualew = RobotState_AtUnloadingStn ; localB ->
a1ty03ixk2 = 0.0 ; localDW -> fyhxlfl0zl = 0U ; localDW -> h032gcefu3 =
dwrx5ivjph ; } break ; default : if ( ( localB -> p55quualew ==
RobotState_AtLoadingStn ) && ( localDW -> fyhxlfl0zl >= ( uint32_T ) localDW
-> i2zecotakp ) ) { localDW -> h032gcefu3 = oywowyxvde ; localB -> aix3fm0ysj
[ 0 ] = rtP_unloadingStations [ ( int32_T ) irwasht4mi -> package - 1 ] ;
localB -> ocoypaore1 [ 0 ] = m2qskxe3uq [ 0 ] ; localB -> aix3fm0ysj [ 1 ] =
rtP_unloadingStations [ ( int32_T ) irwasht4mi -> package + 2 ] ; localB ->
ocoypaore1 [ 1 ] = m2qskxe3uq [ 1 ] ; } else if ( ( localB -> p55quualew ==
RobotState_AtUnloadingStn ) && ( localDW -> fyhxlfl0zl >= ( uint32_T )
localDW -> i2zecotakp ) ) { localDW -> h032gcefu3 = ltsv3qsxc4 ; localB ->
aix3fm0ysj [ 0 ] = rtP_loadingStation [ 0 ] ; localB -> ocoypaore1 [ 0 ] =
m2qskxe3uq [ 0 ] ; localB -> aix3fm0ysj [ 1 ] = rtP_loadingStation [ 1 ] ;
localB -> ocoypaore1 [ 1 ] = m2qskxe3uq [ 1 ] ; } break ; } } ab3l4ewxjl ->
hasPackage = localB -> a1ty03ixk2 ; ab3l4ewxjl -> planningState = localB ->
p55quualew ; dist = m2qskxe3uq [ 0 ] - localB -> aix3fm0ysj [ 0 ] ;
minDistance = dist * dist ; dist = m2qskxe3uq [ 1 ] - localB -> aix3fm0ysj [
1 ] ; minDistance += dist * dist ; localB -> cbioefby2x = (
muDoubleScalarSqrt ( minDistance ) > rtP_awayFromGoalThresh ) ; if ( !
localDW -> ls4ddqxq54 ) { localDW -> aapcuge3jl [ 0 ] = localB -> ocoypaore1
[ 0 ] ; localDW -> aapcuge3jl [ 1 ] = localB -> ocoypaore1 [ 1 ] ; localDW ->
ls4ddqxq54 = true ; } if ( ! localDW -> oafkuis4gg ) { localDW -> johlz4ktqs
[ 0 ] = localB -> aix3fm0ysj [ 0 ] ; localDW -> johlz4ktqs [ 1 ] = localB ->
aix3fm0ysj [ 1 ] ; localDW -> oafkuis4gg = true ; } searchFlag = false ; b_p
= true ; i = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( i < 2 ) ) { if (
! ( localDW -> aapcuge3jl [ i ] == localB -> ocoypaore1 [ i ] ) ) { b_p =
false ; exitg1 = true ; } else { i ++ ; } } if ( b_p ) { searchFlag = true ;
} if ( ! searchFlag ) { searchFlag = true ; } else { searchFlag = false ; b_p
= true ; i = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( i < 2 ) ) { if (
! ( localDW -> johlz4ktqs [ i ] == localB -> aix3fm0ysj [ i ] ) ) { b_p =
false ; exitg1 = true ; } else { i ++ ; } } if ( b_p ) { searchFlag = true ;
} if ( ! searchFlag ) { searchFlag = true ; } else { searchFlag = false ; } }
if ( ( ! localDW -> ftjxsixfdm ) || searchFlag ) { prm . ConnectionDistance =
( rtInf ) ; prm . _pobj2 . HasParent = false ; prm . _pobj2 .
SharedProperties . GridOriginInLocal [ 0 ] = 0.0 ; prm . _pobj2 .
SharedProperties . GridOriginInLocal [ 1 ] = 0.0 ; prm . _pobj2 .
SharedProperties . LocalOriginInWorld [ 0 ] = 0.0 ; prm . _pobj2 .
SharedProperties . LocalOriginInWorld [ 1 ] = 0.0 ; prm . _pobj2 .
SharedProperties . LocalOriginInWorldInternal [ 0 ] = - 0.0 ; prm . _pobj2 .
SharedProperties . LocalOriginInWorldInternal [ 1 ] = - 0.0 ; prm . _pobj2 .
DefaultValueInternal = false ; prm . _pobj0 . Head [ 0 ] = 1.0 ; prm . _pobj0
. Head [ 1 ] = 1.0 ; prm . _pobj0 . NewRegions [ 0 ] = 0.0 ; prm . _pobj0 .
NewRegions [ 1 ] = 0.0 ; prm . _pobj0 . NewRegions [ 2 ] = 0.0 ; prm . _pobj0
. NewRegions [ 3 ] = 0.0 ; prm . _pobj0 . DropEntireMap = false ; prm .
_pobj0 . DropTwoRegions [ 0 ] = false ; prm . _pobj0 . DropTwoRegions [ 1 ] =
false ; prm . _pobj1 . Index = & prm . _pobj0 ; prm . _pobj1 . ConstVal =
false ; memcpy ( & prm . _pobj1 . Buffer [ 0 ] , & rtP_logicalMap [ 0 ] ,
3600U * sizeof ( boolean_T ) ) ; prm . _pobj2 . Buffer = & prm . _pobj1 ;
b_index = prm . _pobj2 . Buffer -> Index ; prm . _pobj2 . Buffer -> Index =
b_index ; prm . InternalMap = & prm . _pobj2 ; prm . InternalNumNodes = 100.0
; prm . UpdateFlag = true ; prm . RoadmapBuilder . isInitialized = 0 ; i =
prm . RoadmapBuilder . Roadmap . Nodes -> size [ 0 ] * prm . RoadmapBuilder .
Roadmap . Nodes -> size [ 1 ] ; prm . RoadmapBuilder . Roadmap . Nodes ->
size [ 0 ] = 2 ; prm . RoadmapBuilder . Roadmap . Nodes -> size [ 1 ] = 2 ;
clokujid0y ( prm . RoadmapBuilder . Roadmap . Nodes , i ) ; prm .
RoadmapBuilder . Roadmap . Nodes -> data [ 0 ] = ( rtNaN ) ; prm .
RoadmapBuilder . Roadmap . Nodes -> data [ 1 ] = ( rtNaN ) ; prm .
RoadmapBuilder . Roadmap . Nodes -> data [ 2 ] = ( rtNaN ) ; prm .
RoadmapBuilder . Roadmap . Nodes -> data [ 3 ] = ( rtNaN ) ; i = prm .
RoadmapBuilder . Roadmap . AdjacencyMatrix -> size [ 0 ] * prm .
RoadmapBuilder . Roadmap . AdjacencyMatrix -> size [ 1 ] ; prm .
RoadmapBuilder . Roadmap . AdjacencyMatrix -> size [ 0 ] = 2 ; prm .
RoadmapBuilder . Roadmap . AdjacencyMatrix -> size [ 1 ] = 2 ; clokujid0y (
prm . RoadmapBuilder . Roadmap . AdjacencyMatrix , i ) ; prm . RoadmapBuilder
. Roadmap . AdjacencyMatrix -> data [ 0 ] = 0.0 ; prm . RoadmapBuilder .
Roadmap . AdjacencyMatrix -> data [ 1 ] = 0.0 ; prm . RoadmapBuilder .
Roadmap . AdjacencyMatrix -> data [ 2 ] = 0.0 ; prm . RoadmapBuilder .
Roadmap . AdjacencyMatrix -> data [ 3 ] = 0.0 ; i = prm . RoadmapBuilder .
Roadmap . NodeLabelSet -> size [ 0 ] * prm . RoadmapBuilder . Roadmap .
NodeLabelSet -> size [ 1 ] ; prm . RoadmapBuilder . Roadmap . NodeLabelSet ->
size [ 0 ] = 1 ; prm . RoadmapBuilder . Roadmap . NodeLabelSet -> size [ 1 ]
= 2 ; clokujid0y ( prm . RoadmapBuilder . Roadmap . NodeLabelSet , i ) ; prm
. RoadmapBuilder . Roadmap . NodeLabelSet -> data [ 0 ] = ( rtNaN ) ; prm .
RoadmapBuilder . Roadmap . NodeLabelSet -> data [ 1 ] = ( rtNaN ) ; prm_p =
prm . RoadmapBuilder . Roadmap . NodeLabels -> size [ 0 ] * prm .
RoadmapBuilder . Roadmap . NodeLabels -> size [ 1 ] ; prm . RoadmapBuilder .
Roadmap . NodeLabels -> size [ 0 ] = 1 ; prm . RoadmapBuilder . Roadmap .
NodeLabels -> size [ 1 ] = 2 ; clokujid0y ( prm . RoadmapBuilder . Roadmap .
NodeLabels , prm_p ) ; prm . RoadmapBuilder . Roadmap . NodeLabels -> data [
0 ] = ( rtNaN ) ; prm . RoadmapBuilder . Roadmap . NodeLabels -> data [ 1 ] =
( rtNaN ) ; prm . RoadmapBuilder . Roadmap . CurrentLabel = 0.0 ; prm .
RoadmapBuilder . Roadmap . NumComponents = 0.0 ; prm . RoadmapBuilder .
Roadmap . IsEuclidean = true ; prm . PathFinder . isInitialized = 0 ;
k5xyyjjkd4 ( & xy , 2 ) ; bej3nj0pvr ( & prm , localB -> ocoypaore1 , localB
-> aix3fm0ysj , xy , localDW ) ; for ( i = 0 ; i < 2 ; i ++ ) { ibmat = i *
100 ; for ( prm_p = 0 ; prm_p < 100 ; prm_p ++ ) { localDW -> o0yda5zduz [
ibmat + prm_p ] = localB -> aix3fm0ysj [ i ] ; } } if ( ( xy -> size [ 0 ] ==
0 ) || ( xy -> size [ 1 ] == 0 ) ) { i = 0 ; } else { prm_p = xy -> size [ 0
] ; ibmat = xy -> size [ 1 ] ; i = muIntScalarMax_sint32 ( prm_p , ibmat ) ;
} for ( prm_p = 0 ; prm_p < 2 ; prm_p ++ ) { for ( ibmat = 0 ; ibmat < i ;
ibmat ++ ) { localDW -> o0yda5zduz [ ibmat + 100 * prm_p ] = xy -> data [ i *
prm_p + ibmat ] ; } } njqibisfwd ( & xy ) ; localDW -> ftjxsixfdm = true ; }
memcpy ( & kbkkhgksqy [ 0 ] , & localDW -> o0yda5zduz [ 0 ] , 200U * sizeof (
real_T ) ) ; localDW -> aapcuge3jl [ 0 ] = localB -> ocoypaore1 [ 0 ] ;
localDW -> johlz4ktqs [ 0 ] = localB -> aix3fm0ysj [ 0 ] ; localDW ->
aapcuge3jl [ 1 ] = localB -> ocoypaore1 [ 1 ] ; localDW -> johlz4ktqs [ 1 ] =
localB -> aix3fm0ysj [ 1 ] ; if ( localDW -> ljuqtcenlo .
DesiredLinearVelocity != i4xuulnjq5 . P_4 ) { if ( localDW -> ljuqtcenlo .
isInitialized == 1 ) { localDW -> ljuqtcenlo . TunablePropsChanged = true ; }
localDW -> ljuqtcenlo . DesiredLinearVelocity = i4xuulnjq5 . P_4 ; } if (
localDW -> ljuqtcenlo . MaxAngularVelocity != i4xuulnjq5 . P_5 ) { if (
localDW -> ljuqtcenlo . isInitialized == 1 ) { localDW -> ljuqtcenlo .
TunablePropsChanged = true ; } localDW -> ljuqtcenlo . MaxAngularVelocity =
i4xuulnjq5 . P_5 ; } if ( localDW -> ljuqtcenlo . LookaheadDistance !=
i4xuulnjq5 . P_6 ) { if ( localDW -> ljuqtcenlo . isInitialized == 1 ) {
localDW -> ljuqtcenlo . TunablePropsChanged = true ; } localDW -> ljuqtcenlo
. LookaheadDistance = i4xuulnjq5 . P_6 ; } if ( localDW -> ljuqtcenlo .
TunablePropsChanged ) { localDW -> ljuqtcenlo . TunablePropsChanged = false ;
} i = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( i < 8 ) ) { if ( localDW
-> ljuqtcenlo . inputVarSize [ 1 ] . f1 [ i ] != ( uint32_T ) inSize [ i ] )
{ for ( prm_p = 0 ; prm_p < 8 ; prm_p ++ ) { localDW -> ljuqtcenlo .
inputVarSize [ 1 ] . f1 [ prm_p ] = ( uint32_T ) inSize [ prm_p ] ; } exitg1
= true ; } else { i ++ ; } } searchFlag = false ; b_p = true ; i = 0 ; exitg1
= false ; while ( ( ! exitg1 ) && ( i < 200 ) ) { if ( ( localDW ->
ljuqtcenlo . WaypointsInternal [ i ] == kbkkhgksqy [ i ] ) || (
muDoubleScalarIsNaN ( localDW -> ljuqtcenlo . WaypointsInternal [ i ] ) &&
muDoubleScalarIsNaN ( kbkkhgksqy [ i ] ) ) ) { i ++ ; } else { b_p = false ;
exitg1 = true ; } } if ( b_p ) { searchFlag = true ; } if ( ! searchFlag ) {
memcpy ( & localDW -> ljuqtcenlo . WaypointsInternal [ 0 ] , & kbkkhgksqy [ 0
] , 200U * sizeof ( real_T ) ) ; localDW -> ljuqtcenlo . ProjectionLineIndex
= 0.0 ; } for ( prm_p = 0 ; prm_p < 200 ; prm_p ++ ) { b [ prm_p ] = !
muDoubleScalarIsNaN ( kbkkhgksqy [ prm_p ] ) ; } prm_p = 0 ; for ( i = 0 ; i
< 100 ; i ++ ) { searchFlag = ( b [ i ] && b [ i + 100 ] ) ; tmp [ i ] =
searchFlag ; if ( searchFlag ) { prm_p ++ ; } } tmp_size_idx_0 = prm_p ;
prm_p = 0 ; for ( i = 0 ; i < 100 ; i ++ ) { if ( tmp [ i ] ) { tmp_data [
prm_p ] = ( int8_T ) i ; prm_p ++ ; } } if ( tmp_size_idx_0 == 0 ) { dist =
0.0 ; minDistance = 0.0 ; } else { searchFlag = false ; if ( localDW ->
ljuqtcenlo . ProjectionLineIndex == 0.0 ) { searchFlag = true ; localDW ->
ljuqtcenlo . ProjectionPoint [ 0 ] = kbkkhgksqy [ tmp_data [ 0 ] ] ; localDW
-> ljuqtcenlo . ProjectionPoint [ 1 ] = kbkkhgksqy [ tmp_data [ 0 ] + 100 ] ;
localDW -> ljuqtcenlo . ProjectionLineIndex = 1.0 ; } if ( tmp_size_idx_0 ==
1 ) { lookaheadEndPt_idx_0 = kbkkhgksqy [ tmp_data [ 0 ] ] ; localDW ->
ljuqtcenlo . ProjectionPoint [ 0 ] = lookaheadEndPt_idx_0 ;
lookaheadEndPt_idx_1 = kbkkhgksqy [ tmp_data [ 0 ] + 100 ] ; localDW ->
ljuqtcenlo . ProjectionPoint [ 1 ] = lookaheadEndPt_idx_1 ; } else {
jfuphejcdb [ 0 ] = kbkkhgksqy [ tmp_data [ ( int32_T ) ( localDW ->
ljuqtcenlo . ProjectionLineIndex + 1.0 ) - 1 ] ] ; jfuphejcdb [ 1 ] =
kbkkhgksqy [ tmp_data [ ( int32_T ) ( localDW -> ljuqtcenlo .
ProjectionLineIndex + 1.0 ) - 1 ] + 100 ] ; minDistance = fswlcxxr3a (
localDW -> ljuqtcenlo . ProjectionPoint , jfuphejcdb , & m2qskxe3uq [ 0 ] ) ;
localDW -> ljuqtcenlo . ProjectionPoint [ 0 ] = jfuphejcdb [ 0 ] ;
kbkkhgksqy_p [ 0 ] = jfuphejcdb [ 0 ] - kbkkhgksqy [ tmp_data [ ( int32_T ) (
localDW -> ljuqtcenlo . ProjectionLineIndex + 1.0 ) - 1 ] ] ; localDW ->
ljuqtcenlo . ProjectionPoint [ 1 ] = jfuphejcdb [ 1 ] ; kbkkhgksqy_p [ 1 ] =
jfuphejcdb [ 1 ] - kbkkhgksqy [ tmp_data [ ( int32_T ) ( localDW ->
ljuqtcenlo . ProjectionLineIndex + 1.0 ) - 1 ] + 100 ] ; dist = mdldozxqlt (
kbkkhgksqy_p ) ; lookaheadEndPt_idx_0 = localDW -> ljuqtcenlo .
ProjectionLineIndex + 1.0 ; i = ( int32_T ) ( ( 1.0 - ( localDW -> ljuqtcenlo
. ProjectionLineIndex + 1.0 ) ) + ( ( real_T ) tmp_size_idx_0 - 1.0 ) ) - 1 ;
prm_p = 0 ; exitg1 = false ; while ( ( ! exitg1 ) && ( prm_p <= i ) ) {
lookaheadEndPt_idx_1 = lookaheadEndPt_idx_0 + ( real_T ) prm_p ; if ( ( !
searchFlag ) && ( dist > localDW -> ljuqtcenlo . LookaheadDistance ) ) {
exitg1 = true ; } else { ibmat = tmp_data [ ( int32_T ) (
lookaheadEndPt_idx_1 + 1.0 ) - 1 ] ; lookaheadIdx = kbkkhgksqy [ ibmat ] ;
kbkkhgksqy_tmp_tmp = tmp_data [ ( int32_T ) lookaheadEndPt_idx_1 - 1 ] ;
kbkkhgksqy_tmp = kbkkhgksqy [ kbkkhgksqy_tmp_tmp ] ; kbkkhgksqy_p [ 0 ] =
kbkkhgksqy_tmp - lookaheadIdx ; jfuphejcdb [ 0 ] = lookaheadIdx ;
kbkkhgksqy_e [ 0 ] = kbkkhgksqy_tmp ; lookaheadIdx = kbkkhgksqy [ ibmat + 100
] ; kbkkhgksqy_tmp = kbkkhgksqy [ kbkkhgksqy_tmp_tmp + 100 ] ; kbkkhgksqy_p [
1 ] = kbkkhgksqy_tmp - lookaheadIdx ; jfuphejcdb [ 1 ] = lookaheadIdx ;
kbkkhgksqy_e [ 1 ] = kbkkhgksqy_tmp ; dist += mdldozxqlt ( kbkkhgksqy_p ) ;
lookaheadIdx = fswlcxxr3a ( kbkkhgksqy_e , jfuphejcdb , & m2qskxe3uq [ 0 ] )
; if ( lookaheadIdx < minDistance ) { minDistance = lookaheadIdx ; localDW ->
ljuqtcenlo . ProjectionPoint [ 0 ] = jfuphejcdb [ 0 ] ; localDW -> ljuqtcenlo
. ProjectionPoint [ 1 ] = jfuphejcdb [ 1 ] ; localDW -> ljuqtcenlo .
ProjectionLineIndex = lookaheadEndPt_idx_1 ; } prm_p ++ ; } } prm_p =
tmp_data [ ( int32_T ) ( localDW -> ljuqtcenlo . ProjectionLineIndex + 1.0 )
- 1 ] ; lookaheadEndPt_idx_0 = kbkkhgksqy [ prm_p ] ; kbkkhgksqy_p [ 0 ] =
localDW -> ljuqtcenlo . ProjectionPoint [ 0 ] - lookaheadEndPt_idx_0 ;
jfuphejcdb [ 0 ] = localDW -> ljuqtcenlo . ProjectionPoint [ 0 ] ;
lookaheadEndPt_idx_1 = kbkkhgksqy [ prm_p + 100 ] ; kbkkhgksqy_p [ 1 ] =
localDW -> ljuqtcenlo . ProjectionPoint [ 1 ] - lookaheadEndPt_idx_1 ;
jfuphejcdb [ 1 ] = localDW -> ljuqtcenlo . ProjectionPoint [ 1 ] ; dist =
mdldozxqlt ( kbkkhgksqy_p ) ; minDistance = dist - localDW -> ljuqtcenlo .
LookaheadDistance ; lookaheadIdx = localDW -> ljuqtcenlo .
ProjectionLineIndex ; while ( ( minDistance < 0.0 ) && ( lookaheadIdx < (
real_T ) tmp_size_idx_0 - 1.0 ) ) { lookaheadIdx ++ ; i = tmp_data [ (
int32_T ) lookaheadIdx - 1 ] ; minDistance = kbkkhgksqy [ i ] ; jfuphejcdb [
0 ] = minDistance ; prm_p = tmp_data [ ( int32_T ) ( lookaheadIdx + 1.0 ) - 1
] ; lookaheadEndPt_idx_0 = kbkkhgksqy [ prm_p ] ; kbkkhgksqy_p [ 0 ] =
minDistance - lookaheadEndPt_idx_0 ; minDistance = kbkkhgksqy [ i + 100 ] ;
jfuphejcdb [ 1 ] = minDistance ; lookaheadEndPt_idx_1 = kbkkhgksqy [ prm_p +
100 ] ; kbkkhgksqy_p [ 1 ] = minDistance - lookaheadEndPt_idx_1 ; dist +=
mdldozxqlt ( kbkkhgksqy_p ) ; minDistance = dist - localDW -> ljuqtcenlo .
LookaheadDistance ; } tmp_i = _mm_set_pd ( lookaheadEndPt_idx_1 ,
lookaheadEndPt_idx_0 ) ; tmp_e = _mm_sub_pd ( _mm_loadu_pd ( & jfuphejcdb [ 0
] ) , tmp_i ) ; _mm_storeu_pd ( & kbkkhgksqy_p [ 0 ] , tmp_e ) ; dist =
minDistance / mdldozxqlt ( kbkkhgksqy_p ) ; if ( dist > 0.0 ) { tmp_e =
_mm_set1_pd ( dist ) ; tmp_i = _mm_add_pd ( _mm_mul_pd ( _mm_sub_pd (
_mm_set1_pd ( 1.0 ) , tmp_e ) , tmp_i ) , _mm_mul_pd ( tmp_e , _mm_loadu_pd (
& jfuphejcdb [ 0 ] ) ) ) ; _mm_storeu_pd ( & tmp_p [ 0 ] , tmp_i ) ;
lookaheadEndPt_idx_0 = tmp_p [ 0 ] ; lookaheadEndPt_idx_1 = tmp_p [ 1 ] ; } }
localDW -> ljuqtcenlo . LookaheadPoint [ 0 ] = lookaheadEndPt_idx_0 ; localDW
-> ljuqtcenlo . LookaheadPoint [ 1 ] = lookaheadEndPt_idx_1 ; dist =
muDoubleScalarAtan2 ( localDW -> ljuqtcenlo . LookaheadPoint [ 1 ] -
m2qskxe3uq [ 1 ] , localDW -> ljuqtcenlo . LookaheadPoint [ 0 ] - m2qskxe3uq
[ 0 ] ) - m2qskxe3uq [ 2 ] ; if ( muDoubleScalarAbs ( dist ) >
3.1415926535897931 ) { if ( muDoubleScalarIsNaN ( dist + 3.1415926535897931 )
|| muDoubleScalarIsInf ( dist + 3.1415926535897931 ) ) { minDistance = (
rtNaN ) ; } else if ( dist + 3.1415926535897931 == 0.0 ) { minDistance = 0.0
; } else { minDistance = muDoubleScalarRem ( dist + 3.1415926535897931 ,
6.2831853071795862 ) ; searchFlag = ( minDistance == 0.0 ) ; if ( !
searchFlag ) { lookaheadIdx = muDoubleScalarAbs ( ( dist + 3.1415926535897931
) / 6.2831853071795862 ) ; searchFlag = ! ( muDoubleScalarAbs ( lookaheadIdx
- muDoubleScalarFloor ( lookaheadIdx + 0.5 ) ) > 2.2204460492503131E-16 *
lookaheadIdx ) ; } if ( searchFlag ) { minDistance = 0.0 ; } else if ( dist +
3.1415926535897931 < 0.0 ) { minDistance += 6.2831853071795862 ; } } if ( (
minDistance == 0.0 ) && ( dist + 3.1415926535897931 > 0.0 ) ) { minDistance =
6.2831853071795862 ; } dist = minDistance - 3.1415926535897931 ; }
minDistance = 2.0 * muDoubleScalarSin ( dist ) / localDW -> ljuqtcenlo .
LookaheadDistance ; if ( muDoubleScalarIsNaN ( minDistance ) ) { minDistance
= 0.0 ; } if ( muDoubleScalarAbs ( muDoubleScalarAbs ( dist ) -
3.1415926535897931 ) < 1.4901161193847656E-8 ) { minDistance =
muDoubleScalarSign ( minDistance ) ; } if ( muDoubleScalarAbs ( minDistance )
> localDW -> ljuqtcenlo . MaxAngularVelocity ) { minDistance =
muDoubleScalarSign ( minDistance ) * localDW -> ljuqtcenlo .
MaxAngularVelocity ; } dist = localDW -> ljuqtcenlo . DesiredLinearVelocity ;
localDW -> ljuqtcenlo . LastPose [ 0 ] = m2qskxe3uq [ 0 ] ; localDW ->
ljuqtcenlo . LastPose [ 1 ] = m2qskxe3uq [ 1 ] ; localDW -> ljuqtcenlo .
LastPose [ 2 ] = m2qskxe3uq [ 2 ] ; } hy2oo0lvw3 [ 0 ] = ( real_T ) localB ->
cbioefby2x * irwasht4mi -> active * dist ; hy2oo0lvw3 [ 1 ] = ( real_T )
localB -> cbioefby2x * irwasht4mi -> active * minDistance ; a0exojw1vz ( &
prm ) ; } void bq3nks32bt ( dheucvhz3u * localB , lwnhcfk22o * localDW ) {
localDW -> avbny3pgeq = localB -> cbioefby2x ; } void pedophjfqw ( m1mtbmkkvs
* const bzf3tuqsn5 ) { if ( ! slIsRapidAcceleratorSimulating ( ) ) {
slmrRunPluginEvent ( bzf3tuqsn5 -> _mdlRefSfcnS , "robotController" ,
"SIMSTATUS_TERMINATING_MODELREF_ACCEL_EVENT" ) ; } } void f4xdwczleo (
SimStruct * _mdlRefSfcnS , int_T mdlref_TID0 , int_T mdlref_TID1 , m1mtbmkkvs
* const bzf3tuqsn5 , dheucvhz3u * localB , lwnhcfk22o * localDW , void *
sysRanPtr , int contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI , const
char_T * rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) { ( void
) memset ( ( void * ) bzf3tuqsn5 , 0 , sizeof ( m1mtbmkkvs ) ) ; bzf3tuqsn5
-> Timing . mdlref_GlobalTID [ 0 ] = mdlref_TID0 ; bzf3tuqsn5 -> Timing .
mdlref_GlobalTID [ 1 ] = mdlref_TID1 ; bzf3tuqsn5 -> _mdlRefSfcnS = (
_mdlRefSfcnS ) ; if ( ! slIsRapidAcceleratorSimulating ( ) ) {
slmrRunPluginEvent ( bzf3tuqsn5 -> _mdlRefSfcnS , "robotController" ,
"START_OF_SIM_MODEL_MODELREF_ACCEL_EVENT" ) ; } ( void ) memset ( ( ( void *
) localB ) , 0 , sizeof ( dheucvhz3u ) ) ; { localB -> a1ty03ixk2 = 0.0 ;
localB -> ocoypaore1 [ 0 ] = 0.0 ; localB -> ocoypaore1 [ 1 ] = 0.0 ; localB
-> aix3fm0ysj [ 0 ] = 0.0 ; localB -> aix3fm0ysj [ 1 ] = 0.0 ; localB ->
p55quualew = RobotState_AtDock ; } ( void ) memset ( ( void * ) localDW , 0 ,
sizeof ( lwnhcfk22o ) ) ; localDW -> i2zecotakp = 0.0 ; { int32_T i ; for ( i
= 0 ; i < 200 ; i ++ ) { localDW -> o0yda5zduz [ i ] = 0.0 ; } } localDW ->
aapcuge3jl [ 0 ] = 0.0 ; localDW -> aapcuge3jl [ 1 ] = 0.0 ; localDW ->
johlz4ktqs [ 0 ] = 0.0 ; localDW -> johlz4ktqs [ 1 ] = 0.0 ;
robotController_InitializeDataMapInfo ( bzf3tuqsn5 , localDW , sysRanPtr ,
contextTid ) ; if ( ( rt_ParentMMI != ( NULL ) ) && ( rt_ChildPath != ( NULL
) ) ) { rtwCAPI_SetChildMMI ( * rt_ParentMMI , rt_ChildMMIIdx , & (
bzf3tuqsn5 -> DataMapInfo . mmi ) ) ; rtwCAPI_SetPath ( bzf3tuqsn5 ->
DataMapInfo . mmi , rt_ChildPath ) ; rtwCAPI_MMISetContStateStartIndex (
bzf3tuqsn5 -> DataMapInfo . mmi , rt_CSTATEIdx ) ; } } void
mr_robotController_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T *
modelName , int_T * retVal ) { * retVal = 0 ; { boolean_T regSubmodelsMdlinfo
= false ; ssGetRegSubmodelsMdlinfo ( mdlRefSfcnS , & regSubmodelsMdlinfo ) ;
if ( regSubmodelsMdlinfo ) { } } * retVal = 0 ; ssRegModelRefMdlInfo (
mdlRefSfcnS , modelName , rtMdlInfo_robotController , 62 ) ; * retVal = 1 ; }
static void mr_robotController_cacheDataAsMxArray ( mxArray * destArray ,
mwIndex i , int j , const void * srcData , size_t numBytes ) ; static void
mr_robotController_cacheDataAsMxArray ( mxArray * destArray , mwIndex i , int
j , const void * srcData , size_t numBytes ) { mxArray * newArray =
mxCreateUninitNumericMatrix ( ( size_t ) 1 , numBytes , mxUINT8_CLASS ,
mxREAL ) ; memcpy ( ( uint8_T * ) mxGetData ( newArray ) , ( const uint8_T *
) srcData , numBytes ) ; mxSetFieldByNumber ( destArray , i , j , newArray )
; } static void mr_robotController_restoreDataFromMxArray ( void * destData ,
const mxArray * srcArray , mwIndex i , int j , size_t numBytes ) ; static
void mr_robotController_restoreDataFromMxArray ( void * destData , const
mxArray * srcArray , mwIndex i , int j , size_t numBytes ) { memcpy ( (
uint8_T * ) destData , ( const uint8_T * ) mxGetData ( mxGetFieldByNumber (
srcArray , i , j ) ) , numBytes ) ; } static void
mr_robotController_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex i ,
int j , uint_T bitVal ) ; static void
mr_robotController_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex i ,
int j , uint_T bitVal ) { mxSetFieldByNumber ( destArray , i , j ,
mxCreateDoubleScalar ( ( real_T ) bitVal ) ) ; } static uint_T
mr_robotController_extractBitFieldFromMxArray ( const mxArray * srcArray ,
mwIndex i , int j , uint_T numBits ) ; static uint_T
mr_robotController_extractBitFieldFromMxArray ( const mxArray * srcArray ,
mwIndex i , int j , uint_T numBits ) { const uint_T varVal = ( uint_T )
mxGetScalar ( mxGetFieldByNumber ( srcArray , i , j ) ) ; return varVal & ( (
1u << numBits ) - 1u ) ; } static void
mr_robotController_cacheDataToMxArrayWithOffset ( mxArray * destArray ,
mwIndex i , int j , mwIndex offset , const void * srcData , size_t numBytes )
; static void mr_robotController_cacheDataToMxArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , const void * srcData ,
size_t numBytes ) { uint8_T * varData = ( uint8_T * ) mxGetData (
mxGetFieldByNumber ( destArray , i , j ) ) ; memcpy ( ( uint8_T * ) & varData
[ offset * numBytes ] , ( const uint8_T * ) srcData , numBytes ) ; } static
void mr_robotController_restoreDataFromMxArrayWithOffset ( void * destData ,
const mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t
numBytes ) ; static void mr_robotController_restoreDataFromMxArrayWithOffset
( void * destData , const mxArray * srcArray , mwIndex i , int j , mwIndex
offset , size_t numBytes ) { const uint8_T * varData = ( const uint8_T * )
mxGetData ( mxGetFieldByNumber ( srcArray , i , j ) ) ; memcpy ( ( uint8_T *
) destData , ( const uint8_T * ) & varData [ offset * numBytes ] , numBytes )
; } static void mr_robotController_cacheBitFieldToCellArrayWithOffset (
mxArray * destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal )
; static void mr_robotController_cacheBitFieldToCellArrayWithOffset ( mxArray
* destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal ) {
mxSetCell ( mxGetFieldByNumber ( destArray , i , j ) , offset ,
mxCreateDoubleScalar ( ( real_T ) fieldVal ) ) ; } static uint_T
mr_robotController_extractBitFieldFromCellArrayWithOffset ( const mxArray *
srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) ; static
uint_T mr_robotController_extractBitFieldFromCellArrayWithOffset ( const
mxArray * srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) {
const uint_T fieldVal = ( uint_T ) mxGetScalar ( mxGetCell (
mxGetFieldByNumber ( srcArray , i , j ) , offset ) ) ; return fieldVal & ( (
1u << numBits ) - 1u ) ; } mxArray * mr_robotController_GetDWork ( const
dx4drjuueew * mdlrefDW ) { static const char_T * ssDWFieldNames [ 3 ] = {
"rtb" , "rtdw" , "NULL->rtzce" , } ; mxArray * ssDW = mxCreateStructMatrix (
1 , 1 , 3 , ssDWFieldNames ) ; mr_robotController_cacheDataAsMxArray ( ssDW ,
0 , 0 , ( const void * ) & ( mdlrefDW -> rtb ) , sizeof ( mdlrefDW -> rtb ) )
; { static const char_T * rtdwDataFieldNames [ 14 ] = {
"mdlrefDW->rtdw.ljuqtcenlo" , "mdlrefDW->rtdw.i2zecotakp" ,
"mdlrefDW->rtdw.o0yda5zduz" , "mdlrefDW->rtdw.aapcuge3jl" ,
"mdlrefDW->rtdw.johlz4ktqs" , "mdlrefDW->rtdw.fyhxlfl0zl" ,
"mdlrefDW->rtdw.pggahx0edn" , "mdlrefDW->rtdw.avbny3pgeq" ,
"mdlrefDW->rtdw.c0setyxykh" , "mdlrefDW->rtdw.h032gcefu3" ,
"mdlrefDW->rtdw.ey3alnk1or" , "mdlrefDW->rtdw.ftjxsixfdm" ,
"mdlrefDW->rtdw.ls4ddqxq54" , "mdlrefDW->rtdw.oafkuis4gg" , } ; mxArray *
rtdwData = mxCreateStructMatrix ( 1 , 1 , 14 , rtdwDataFieldNames ) ;
mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 0 , ( const void * ) &
( mdlrefDW -> rtdw . ljuqtcenlo ) , sizeof ( mdlrefDW -> rtdw . ljuqtcenlo )
) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 1 , ( const void *
) & ( mdlrefDW -> rtdw . i2zecotakp ) , sizeof ( mdlrefDW -> rtdw .
i2zecotakp ) ) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 2 , (
const void * ) & ( mdlrefDW -> rtdw . o0yda5zduz ) , sizeof ( mdlrefDW ->
rtdw . o0yda5zduz ) ) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0
, 3 , ( const void * ) & ( mdlrefDW -> rtdw . aapcuge3jl ) , sizeof (
mdlrefDW -> rtdw . aapcuge3jl ) ) ; mr_robotController_cacheDataAsMxArray (
rtdwData , 0 , 4 , ( const void * ) & ( mdlrefDW -> rtdw . johlz4ktqs ) ,
sizeof ( mdlrefDW -> rtdw . johlz4ktqs ) ) ;
mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 5 , ( const void * ) &
( mdlrefDW -> rtdw . fyhxlfl0zl ) , sizeof ( mdlrefDW -> rtdw . fyhxlfl0zl )
) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 6 , ( const void *
) & ( mdlrefDW -> rtdw . pggahx0edn ) , sizeof ( mdlrefDW -> rtdw .
pggahx0edn ) ) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 7 , (
const void * ) & ( mdlrefDW -> rtdw . avbny3pgeq ) , sizeof ( mdlrefDW ->
rtdw . avbny3pgeq ) ) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0
, 8 , ( const void * ) & ( mdlrefDW -> rtdw . c0setyxykh ) , sizeof (
mdlrefDW -> rtdw . c0setyxykh ) ) ; mr_robotController_cacheDataAsMxArray (
rtdwData , 0 , 9 , ( const void * ) & ( mdlrefDW -> rtdw . h032gcefu3 ) ,
sizeof ( mdlrefDW -> rtdw . h032gcefu3 ) ) ;
mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 10 , ( const void * )
& ( mdlrefDW -> rtdw . ey3alnk1or ) , sizeof ( mdlrefDW -> rtdw . ey3alnk1or
) ) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 11 , ( const
void * ) & ( mdlrefDW -> rtdw . ftjxsixfdm ) , sizeof ( mdlrefDW -> rtdw .
ftjxsixfdm ) ) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0 , 12 ,
( const void * ) & ( mdlrefDW -> rtdw . ls4ddqxq54 ) , sizeof ( mdlrefDW ->
rtdw . ls4ddqxq54 ) ) ; mr_robotController_cacheDataAsMxArray ( rtdwData , 0
, 13 , ( const void * ) & ( mdlrefDW -> rtdw . oafkuis4gg ) , sizeof (
mdlrefDW -> rtdw . oafkuis4gg ) ) ; mxSetFieldByNumber ( ssDW , 0 , 1 ,
rtdwData ) ; } ( void ) mdlrefDW ; return ssDW ; } void
mr_robotController_SetDWork ( dx4drjuueew * mdlrefDW , const mxArray * ssDW )
{ ( void ) ssDW ; ( void ) mdlrefDW ;
mr_robotController_restoreDataFromMxArray ( ( void * ) & ( mdlrefDW -> rtb )
, ssDW , 0 , 0 , sizeof ( mdlrefDW -> rtb ) ) ; { const mxArray * rtdwData =
mxGetFieldByNumber ( ssDW , 0 , 1 ) ;
mr_robotController_restoreDataFromMxArray ( ( void * ) & ( mdlrefDW -> rtdw .
ljuqtcenlo ) , rtdwData , 0 , 0 , sizeof ( mdlrefDW -> rtdw . ljuqtcenlo ) )
; mr_robotController_restoreDataFromMxArray ( ( void * ) & ( mdlrefDW -> rtdw
. i2zecotakp ) , rtdwData , 0 , 1 , sizeof ( mdlrefDW -> rtdw . i2zecotakp )
) ; mr_robotController_restoreDataFromMxArray ( ( void * ) & ( mdlrefDW ->
rtdw . o0yda5zduz ) , rtdwData , 0 , 2 , sizeof ( mdlrefDW -> rtdw .
o0yda5zduz ) ) ; mr_robotController_restoreDataFromMxArray ( ( void * ) & (
mdlrefDW -> rtdw . aapcuge3jl ) , rtdwData , 0 , 3 , sizeof ( mdlrefDW ->
rtdw . aapcuge3jl ) ) ; mr_robotController_restoreDataFromMxArray ( ( void *
) & ( mdlrefDW -> rtdw . johlz4ktqs ) , rtdwData , 0 , 4 , sizeof ( mdlrefDW
-> rtdw . johlz4ktqs ) ) ; mr_robotController_restoreDataFromMxArray ( ( void
* ) & ( mdlrefDW -> rtdw . fyhxlfl0zl ) , rtdwData , 0 , 5 , sizeof (
mdlrefDW -> rtdw . fyhxlfl0zl ) ) ; mr_robotController_restoreDataFromMxArray
( ( void * ) & ( mdlrefDW -> rtdw . pggahx0edn ) , rtdwData , 0 , 6 , sizeof
( mdlrefDW -> rtdw . pggahx0edn ) ) ;
mr_robotController_restoreDataFromMxArray ( ( void * ) & ( mdlrefDW -> rtdw .
avbny3pgeq ) , rtdwData , 0 , 7 , sizeof ( mdlrefDW -> rtdw . avbny3pgeq ) )
; mr_robotController_restoreDataFromMxArray ( ( void * ) & ( mdlrefDW -> rtdw
. c0setyxykh ) , rtdwData , 0 , 8 , sizeof ( mdlrefDW -> rtdw . c0setyxykh )
) ; mr_robotController_restoreDataFromMxArray ( ( void * ) & ( mdlrefDW ->
rtdw . h032gcefu3 ) , rtdwData , 0 , 9 , sizeof ( mdlrefDW -> rtdw .
h032gcefu3 ) ) ; mr_robotController_restoreDataFromMxArray ( ( void * ) & (
mdlrefDW -> rtdw . ey3alnk1or ) , rtdwData , 0 , 10 , sizeof ( mdlrefDW ->
rtdw . ey3alnk1or ) ) ; mr_robotController_restoreDataFromMxArray ( ( void *
) & ( mdlrefDW -> rtdw . ftjxsixfdm ) , rtdwData , 0 , 11 , sizeof ( mdlrefDW
-> rtdw . ftjxsixfdm ) ) ; mr_robotController_restoreDataFromMxArray ( ( void
* ) & ( mdlrefDW -> rtdw . ls4ddqxq54 ) , rtdwData , 0 , 12 , sizeof (
mdlrefDW -> rtdw . ls4ddqxq54 ) ) ; mr_robotController_restoreDataFromMxArray
( ( void * ) & ( mdlrefDW -> rtdw . oafkuis4gg ) , rtdwData , 0 , 13 , sizeof
( mdlrefDW -> rtdw . oafkuis4gg ) ) ; } } void
mr_robotController_RegisterSimStateChecksum ( SimStruct * S ) { const
uint32_T chksum [ 4 ] = { 3266549605U , 4164274921U , 3276871821U ,
2476074854U , } ; slmrModelRefRegisterSimStateChecksum ( S ,
"robotController" , & chksum [ 0 ] ) ; } mxArray *
mr_robotController_GetSimStateDisallowedBlocks ( ) { mxArray * data =
mxCreateCellMatrix ( 1 , 3 ) ; mwIndex subs [ 2 ] , offset ; { static const
char_T * blockType [ 1 ] = { "MATLABSystem" , } ; static const char_T *
blockPath [ 1 ] = { "robotController/Pure Pursuit" , } ; static const int
reason [ 1 ] = { 6 , } ; for ( subs [ 0 ] = 0 ; subs [ 0 ] < 1 ; ++ ( subs [
0 ] ) ) { subs [ 1 ] = 0 ; offset = mxCalcSingleSubscript ( data , 2 , subs )
; mxSetCell ( data , offset , mxCreateString ( blockType [ subs [ 0 ] ] ) ) ;
subs [ 1 ] = 1 ; offset = mxCalcSingleSubscript ( data , 2 , subs ) ;
mxSetCell ( data , offset , mxCreateString ( blockPath [ subs [ 0 ] ] ) ) ;
subs [ 1 ] = 2 ; offset = mxCalcSingleSubscript ( data , 2 , subs ) ;
mxSetCell ( data , offset , mxCreateDoubleScalar ( ( real_T ) reason [ subs [
0 ] ] ) ) ; } } return data ; }
#if defined(_MSC_VER)
#pragma warning(disable: 4505) //unreferenced local function has been removed
#endif
