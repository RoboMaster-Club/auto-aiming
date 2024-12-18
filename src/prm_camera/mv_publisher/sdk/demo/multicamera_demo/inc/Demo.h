#ifndef DEME_H_INCLUDED
#define DEME_H_INCLUDED


#include "CameraDefine.h"
#include "CameraStatus.h"
#include "CameraApi.h"

#define  DISPLAY_MAX     2

#define  DISPLAY_WIDTH      640
#define  DISPLAY_HEIGHT     480


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


void * preview_thread1(void* arg);
void * preview_thread2(void* arg);

int  Gtk_SetResolution(GtkWidget * display_drawingarea,Width_Height *W_H_INFO,int hCamera,tSdkCameraCapbility * pCameraInfo);

#ifdef __cplusplus
}
#endif


#endif // DEME_H_INCLUDED
