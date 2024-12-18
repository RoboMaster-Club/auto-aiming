#include <gtk/gtk.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "Demo.h"
#include "support.h"
#include <pthread.h>
#include "CameraApi.h"

//SDK使用
extern int                  g_hCamera[DISPLAY_MAX];          //设备句柄
extern unsigned char        * g_pRawBuffer[DISPLAY_MAX];     //raw数据
extern unsigned char        * g_pRgbBuffer[DISPLAY_MAX];     //处理后数据缓存区
extern tSdkFrameHead        g_tFrameHead[DISPLAY_MAX];       //图像帧头信息
extern tSdkCameraCapbility  g_tCapability[DISPLAY_MAX];      //设备描述信息

//gtk demo使用
extern GtkBuilder           *Demo_builder ;             //glade进行界面设计
extern GtkWidget            *main_window ;              //主窗体
extern GtkWidget            *display_drawingarea[DISPLAY_MAX];       //画板1


extern int                  g_display_state[DISPLAY_MAX];    //显示状态  1读取数据 0退出


extern Width_Height         g_W_H_INFO[DISPLAY_MAX];         //显示画板到大小和图像大小
extern BYTE                 *g_readBuf[DISPLAY_MAX];         //显示数据buffer

extern int                  g_read_fps[DISPLAY_MAX];           //统计读取帧率
extern int                  g_disply_fps[DISPLAY_MAX];         //统计显示帧率
//读取相机数据
void read_data1(GtkWidget *widget,int i)
{

    if (CameraGetImageBuffer(g_hCamera[i],&g_tFrameHead[i],&g_pRawBuffer[i],1000) == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(g_hCamera[i],g_pRawBuffer[i],g_pRgbBuffer[i],&g_tFrameHead[i]);

        if(g_tFrameHead[i].uiMediaType==CAMERA_MEDIA_TYPE_MONO8){
            memcpy(g_readBuf[i],g_pRgbBuffer[i],g_W_H_INFO[i].buffer_size);
        }else{
            memcpy(g_readBuf[i],g_pRgbBuffer[i],g_W_H_INFO[i].buffer_size*3);
        }


        CameraReleaseImageBuffer(g_hCamera[i],g_pRawBuffer[i]);

        gdk_threads_enter();
        gtk_widget_queue_draw(GTK_WIDGET (widget));
        gdk_threads_leave();

        g_read_fps[i]++;//统计抓取帧率

    }


    return ;
}

//读取相机数据
void read_data2(GtkWidget *widget,int i)
{

    if (CameraGetImageBuffer(g_hCamera[i],&g_tFrameHead[i],&g_pRawBuffer[i],1000) == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(g_hCamera[i],g_pRawBuffer[i],g_pRgbBuffer[i],&g_tFrameHead[i]);

        if(g_tFrameHead[i].uiMediaType==CAMERA_MEDIA_TYPE_MONO8){
            memcpy(g_readBuf[i],g_pRgbBuffer[i],g_W_H_INFO[i].buffer_size);
        }else{
            memcpy(g_readBuf[i],g_pRgbBuffer[i],g_W_H_INFO[i].buffer_size*3);
        }
        CameraReleaseImageBuffer(g_hCamera[i],g_pRawBuffer[i]);

        gdk_threads_enter();
        gtk_widget_queue_draw(GTK_WIDGET (widget));
        gdk_threads_leave();

        g_read_fps[i]++;//统计抓取帧率

    }


    return ;
}

//preview_thread 读取数据
void * preview_thread1(void* arg)
{
	while(g_display_state[0] == 1) //1 进行读取数据
	{
		read_data1(display_drawingarea[0],0);
	}
	pthread_exit(NULL);
	return ((void *)0);
}

//preview_thread 读取数据
void * preview_thread2(void* arg)
{
    //printf("arg  =%d \n",(int)*arg);
	while(g_display_state[1] == 1) //1 进行读取数据
	{
		read_data2(display_drawingarea[1],1);
	}
	pthread_exit(NULL);
	return ((void *)0);
}


int  Gtk_SetResolution(GtkWidget * display_drawingarea,Width_Height *W_H_INFO,int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    tSdkImageResolution     sResolution;  //获取当前设置到分辨率
	int level=0;


    tSdkImageResolution     *pImageSizeDesc=pCameraInfo->pImageSizeDesc;// 预设分辨率选择
    int                     iImageSizeDesc=pCameraInfo->iImageSizeDesc; // 预设分辨率的个数，即pImageSizeDesc数组的大小


    //获得当前预览的分辨率。
    CameraGetImageResolution(hCamera,&sResolution);

	level=sResolution.iIndex;
    W_H_INFO->sensor_width=pImageSizeDesc[level].iWidth;
    W_H_INFO->sensor_height=pImageSizeDesc[level].iHeight;
    W_H_INFO->buffer_size= W_H_INFO->sensor_width*W_H_INFO->sensor_height;


    if(W_H_INFO->sensor_width <W_H_INFO->display_width){
        W_H_INFO->xOffsetFOV=(W_H_INFO->display_width-W_H_INFO->sensor_width)/2;
    }else{
        W_H_INFO->xOffsetFOV=0;
    }
    if(W_H_INFO->sensor_height < W_H_INFO->display_height){
        W_H_INFO->yOffsetFOV=(W_H_INFO->display_height-W_H_INFO->sensor_height)/2;
    }else{
         W_H_INFO->yOffsetFOV=0;
    }

    gtk_widget_set_size_request ((GtkWidget *)display_drawingarea,W_H_INFO->sensor_width,W_H_INFO->sensor_height);
    return 1;
}

