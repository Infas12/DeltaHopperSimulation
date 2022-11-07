//
// MATLAB Compiler: 8.0 (R2020a)
// Date: Wed May 19 00:29:10 2021
// Arguments: "-B""macro_default""-W""cpplib:DeltaFK""-T""link:lib""DeltaFK"
//

#ifndef DeltaFK_h
#define DeltaFK_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_DeltaFK_C_API 
#define LIB_DeltaFK_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_DeltaFK_C_API 
bool MW_CALL_CONV DeltaFKInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_DeltaFK_C_API 
bool MW_CALL_CONV DeltaFKInitialize(void);

extern LIB_DeltaFK_C_API 
void MW_CALL_CONV DeltaFKTerminate(void);

extern LIB_DeltaFK_C_API 
void MW_CALL_CONV DeltaFKPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_DeltaFK_C_API 
bool MW_CALL_CONV mlxDeltaFK(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_DeltaFK
#define PUBLIC_DeltaFK_CPP_API __declspec(dllexport)
#else
#define PUBLIC_DeltaFK_CPP_API __declspec(dllimport)
#endif

#define LIB_DeltaFK_CPP_API PUBLIC_DeltaFK_CPP_API

#else

#if !defined(LIB_DeltaFK_CPP_API)
#if defined(LIB_DeltaFK_C_API)
#define LIB_DeltaFK_CPP_API LIB_DeltaFK_C_API
#else
#define LIB_DeltaFK_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_DeltaFK_CPP_API void MW_CALL_CONV DeltaFK(int nargout, mwArray& footPos, const mwArray& HipMotorConfig);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
