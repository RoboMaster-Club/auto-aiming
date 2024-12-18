#ifndef INTERFACE_H_INCLUDED
#define INTERFACE_H_INCLUDED

#include "CameraApi.h"
#ifdef __cplusplus
extern "C" {
#endif


GtkWidget* create_gtkdemo_window (int hCamera,tSdkCameraCapbility * pCameraInfo);
GtkWidget* create_snap_path_window (void);
GtkWidget* create_file_load_window (void);
gint fps_statistics(gpointer data);


#ifdef __cplusplus
}
#endif


#endif // INTERFACE_H_INCLUDED
