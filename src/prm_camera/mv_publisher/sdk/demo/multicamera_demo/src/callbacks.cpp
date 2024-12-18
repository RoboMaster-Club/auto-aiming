#include <gtk/gtk.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <gdk/gdkkeysyms.h>
#include <pthread.h>

#include "callbacks.h"
#include "support.h"
#include "interface.h"
#include "Demo.h"



#include "CameraApi.h"


//SDK使用
extern int                     g_hCamera[DISPLAY_MAX];              //设备句柄
extern unsigned char           * g_pRgbBuffer[DISPLAY_MAX];         //处理后数据缓存区
extern tSdkFrameHead           g_tFrameHead[DISPLAY_MAX];           //图像帧头信息
extern tSdkCameraCapbility     g_tCapability[DISPLAY_MAX];          //设备描述信息



//gtk demo使用
extern GtkBuilder           *Demo_builder ;             //glade进行界面设计
extern GtkWidget            *main_window ;              //主窗体
extern GtkWidget            *display_drawingarea[DISPLAY_MAX];      //画板1

extern pthread_t            g_thread_id[DISPLAY_MAX];               //线程
extern BYTE                 *g_readBuf[DISPLAY_MAX];                //画板显示数据区
extern Width_Height         g_W_H_INFO[DISPLAY_MAX];                //显示画板到大小和图像大小

extern int                  g_display_state[DISPLAY_MAX];            //显示状态
extern int                  g_display_max;                          //最大显示
extern int                  g_read_fps[DISPLAY_MAX];           //统计读取帧率
extern int                  g_disply_fps[DISPLAY_MAX];         //统计显示帧率

//图像显示
gboolean preview_display1( GtkWidget *widget, GdkEventExpose *event,gpointer data )
{
    int i=0;

    if(g_tFrameHead[i].uiMediaType==CAMERA_MEDIA_TYPE_MONO8){
        gdk_draw_gray_image(widget->window,widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
                            g_W_H_INFO[i].xOffsetFOV,g_W_H_INFO[i].yOffsetFOV,
                            g_W_H_INFO[i].sensor_width,g_W_H_INFO[i].sensor_height,
                            GDK_RGB_DITHER_NORMAL,(guchar *)g_readBuf[i],g_W_H_INFO[i].sensor_width);
    }
    else{
        //显示图像
        gdk_draw_rgb_image(widget->window,widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
                            g_W_H_INFO[i].xOffsetFOV,g_W_H_INFO[i].yOffsetFOV,
                            g_W_H_INFO[i].sensor_width,g_W_H_INFO[i].sensor_height,
                            GDK_RGB_DITHER_NORMAL,(guchar *)g_readBuf[i],g_W_H_INFO[i].sensor_width*3);
    }


	g_disply_fps[i]++;

	return TRUE;
}

//图像显示
gboolean preview_display2( GtkWidget *widget, GdkEventExpose *event,gpointer data )
{

    int i=1;

    if(g_tFrameHead[i].uiMediaType==CAMERA_MEDIA_TYPE_MONO8){
        gdk_draw_gray_image(widget->window,widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
                            g_W_H_INFO[i].xOffsetFOV,g_W_H_INFO[i].yOffsetFOV,
                            g_W_H_INFO[i].sensor_width,g_W_H_INFO[i].sensor_height,
                            GDK_RGB_DITHER_NORMAL,(guchar *)g_readBuf[i],g_W_H_INFO[i].sensor_width);
    }
    else{
        //显示图像
        gdk_draw_rgb_image(widget->window,widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
                            g_W_H_INFO[i].xOffsetFOV,g_W_H_INFO[i].yOffsetFOV,
                            g_W_H_INFO[i].sensor_width,g_W_H_INFO[i].sensor_height,
                            GDK_RGB_DITHER_NORMAL,(guchar *)g_readBuf[i],g_W_H_INFO[i].sensor_width*3);
    }

	g_disply_fps[i]++;

	return TRUE;
}

gboolean on_gtkdemo_window_delete_event      (GtkWidget       *widget,
                                            GdkEvent        *event,
                                            gpointer         user_data)
{
    int i=0;




    for(i=0;i<g_display_max;i++)
    {
        if(g_display_state[i]){
            g_display_state[i]=0;
            gdk_threads_leave ();
            CameraSetTriggerMode(g_hCamera[i], 0);
            pthread_join(g_thread_id[i], NULL);
        }
        if(g_readBuf[i]!=NULL){
            free(g_readBuf[i]);
            g_readBuf[i]=NULL;
        }
        if(g_pRgbBuffer[i]!=NULL){
            free(g_pRgbBuffer[i]);
            g_pRgbBuffer[i]=NULL;
        }
        if(g_hCamera[i]>0){
            //相机反初始化。释放资源。
            CameraUnInit(g_hCamera[i]);
            g_hCamera[i]=-1;
        }

    }


    gtk_main_quit();


 	return FALSE;
}
