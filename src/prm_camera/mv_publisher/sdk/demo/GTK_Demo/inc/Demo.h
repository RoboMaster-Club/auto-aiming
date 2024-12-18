#ifndef DEME_H_INCLUDED
#define DEME_H_INCLUDED


#include "CameraDefine.h"
#include "CameraStatus.h"
#include "CameraApi.h"


#define  DISPLAY_WIDTH      800
#define  DISPLAY_HEIGHT     600


#ifdef __cplusplus
extern "C" {
#endif



typedef struct _WIDTH_HEIGHT{
    int     display_width;
    int     display_height;
    int     xOffsetFOV;
    int     yOffsetFOV;
    int     sensor_width;
    int     sensor_height;
    int     buffer_size;
}Width_Height;


void * preview_thread(void* arg);
void * getcounts_thread(void* arg);


int  Gtk_GUI_Init(int hCamera,tSdkCameraCapbility * pCameraInfo);

int  Gtk_SetResolution(int hCamera,tSdkCameraCapbility * pCameraInfo);
int  Gtk_SetWB(int hCamera,tSdkCameraCapbility * pCameraInfo);
int  Gtk_SetExposure  (int hCamera,tSdkCameraCapbility * pCameraInfo);
int  Gtk_SetGamma_Contrast  (int hCamera,tSdkCameraCapbility * pCameraInfo);
int  Gtk_SetSpeed(int hCamera,tSdkCameraCapbility * pCameraInfo);
int  Gtk_SetTrigger(int hCamera,tSdkCameraCapbility * pCameraInfo);
int  Gtk_SetISPProce(int hCamera,tSdkCameraCapbility * pCameraInfo);
int  Gtk_SetSnap(int hCamera,tSdkCameraCapbility * pCameraInfo);


#ifdef __cplusplus
}
#endif


#endif // DEME_H_INCLUDED
