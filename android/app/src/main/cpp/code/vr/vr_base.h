#ifndef __VR_BASE
#define __VR_BASE

#if __ANDROID__

#define _DEBUG

#include "vr_types.h"

engine_t* VR_Init( ovrJava java );
void VR_InitCvars( void );
void VR_Destroy( engine_t* engine );
void VR_EnterVR( engine_t* engine, ovrJava java );
void VR_LeaveVR( engine_t* engine );

engine_t* VR_GetEngine( void );
int VR_useScreenLayer( void );
int VR_isPauseable( void );
void VR_setRefresh( int value );

float radians(float deg);

void VR_HapticEvent(const char* event, int position, int flags, int intensity, float angle, float yHeight );

#endif

#if defined(_DEBUG)
void GLCheckErrors(const char* file, int line);
void OXRCheckErrors(XrResult result, const char* function);
#define GL(func) func; GLCheckErrors(__FILE__ , __LINE__);
#define OXR(func) OXRCheckErrors(func, #func);
#else
#define GL(func) func;
#define OXR(func) func;
#endif

#endif
