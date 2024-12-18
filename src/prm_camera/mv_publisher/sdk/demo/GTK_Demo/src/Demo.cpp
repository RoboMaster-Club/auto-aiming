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
extern int                  g_hCamera;          //设备句柄
extern unsigned char        * g_pRawBuffer;     //raw数据
extern unsigned char        * g_pRgbBuffer;     //处理后数据缓存区
extern tSdkFrameHead        g_tFrameHead;       //图像帧头信息
extern tSdkFrameHead        g_tRawFrameHead;
extern tSdkCameraCapbility  g_tCapability;      //设备描述信息

//gtk demo使用
extern GtkBuilder           *Demo_builder ;             //glade进行界面设计
extern GtkWidget            *main_window ;              //主窗体
extern GtkWidget            *display_drawingarea;       //画板
extern GtkWidget            *Demo_display_window;

extern int                  g_display_state;    //显示状态  1读取数据 0退出
extern Width_Height         g_W_H_INFO;         //显示画板到大小和图像大小
extern BYTE                 *g_readBuf;         //显示数据buffer
extern int                  g_read_fps;         //统计帧率
extern int                  g_disply_fps;         //统计帧率
extern int                  g_SaveImage_type;   //保存图像格式

extern UINT g_DispFormat;

int     tmp_state=0;
extern int                    res_test;
extern  pthread_mutex_t mutex;

//读取相机数据
void read_data(GtkWidget *widget)
{
    BYTE* pRaw;

    if (CameraGetImageBuffer(g_hCamera,&g_tRawFrameHead,&pRaw,1000) == CAMERA_STATUS_SUCCESS)
    {
        memcpy(g_pRawBuffer,pRaw,g_tRawFrameHead.uBytes);
        g_tFrameHead = g_tRawFrameHead;
        
        CameraImageProcess(g_hCamera,pRaw,g_pRgbBuffer,&g_tFrameHead);

        if(g_tFrameHead.uiMediaType==CAMERA_MEDIA_TYPE_MONO8){
            memcpy(g_readBuf,g_pRgbBuffer,g_tFrameHead.uBytes);
        }else{
            memcpy(g_readBuf,g_pRgbBuffer,g_tFrameHead.uBytes);
        }

        CameraReleaseImageBuffer(g_hCamera,pRaw);
        g_disply_fps++;

        gdk_threads_enter();
        gtk_widget_queue_draw(GTK_WIDGET (widget));
        gdk_threads_leave();
    }else{
        //printf("timeout \n");
    }
}

//preview_thread 读取数据
void * preview_thread(void* arg)
{
	while(g_display_state == 1) //1 进行读取数据
	{
        pthread_mutex_lock(&mutex);  //加互斥锁
        if(tmp_state==0){
            read_data(Demo_display_window);
        }
        pthread_mutex_unlock(&mutex); //解互斥锁
        //usleep(1);
	}
	pthread_exit(NULL);
	return ((void *)0);
}
void camera_uninit()
{
    if(g_hCamera>0){
        //相机反初始化。释放资源。
        CameraUnInit(g_hCamera);
        g_hCamera=-1;
        tmp_state=-1;
    }
    return ;
}


void camera_init()
{
    tSdkCameraDevInfo       tCameraEnumList[4];
    int                     iCameraCounts = 4;
    int                     iStatus=-1;


    //未使用的相机信息
    CameraIdleStateDevice(tCameraEnumList,&iCameraCounts);


    if(iCameraCounts>=1 && tmp_state ==-1){

        //sleep(1);

        iStatus = CameraInit(&tCameraEnumList[0],-1,-1,&g_hCamera);

        //初始化失败
        if(iStatus ==CAMERA_STATUS_SUCCESS){
            //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
            CameraGetCapability(g_hCamera,&g_tCapability);

            /*让SDK进入工作模式，开始接收来自相机发送的图像
            数据。如果当前相机是触发模式，则需要接收到
            触发帧以后才会更新图像。    */
            CameraPlay(g_hCamera);

        /*
            设置图像处理的输出格式，彩色黑白都支持RGB24位
        */
            if(g_tCapability.sIspCapacity.bMonoSensor){
                CameraSetIspOutFormat(g_hCamera,CAMERA_MEDIA_TYPE_MONO8);
            }else{
                CameraSetIspOutFormat(g_hCamera,CAMERA_MEDIA_TYPE_RGB8);
            }

            //根据设备设置参数初始化界面
            Gtk_GUI_Init(g_hCamera,&g_tCapability);

            tmp_state=0;
        }

    }
    return ;
}


//getcounts_thread 读取数据
void * getcounts_thread(void* arg)
{

    int state=0,state1=0;
    unsigned  int   data=0,data1=0;

	while(g_display_state == 1) //1 进行读取数据
	{
#if   0
        pthread_mutex_lock(&mutex);  //加互斥锁
        state=CameraConnectTest(g_hCamera);

        if(state<0 ){
            camera_uninit();
            camera_init();
        }
        pthread_mutex_unlock(&mutex); //解互斥锁
#endif
        usleep(1);
    }

	pthread_exit(NULL);
	return ((void *)0);
}

//初始化GTK 界面
int  Gtk_GUI_Init(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    Gtk_SetResolution(hCamera,pCameraInfo);
    if(!g_tCapability.sIspCapacity.bMonoSensor){
        Gtk_SetWB(hCamera,pCameraInfo);
    }
    Gtk_SetExposure(hCamera,pCameraInfo);
    Gtk_SetGamma_Contrast(hCamera,pCameraInfo);
    Gtk_SetTrigger(hCamera,pCameraInfo);
    Gtk_SetISPProce(hCamera,pCameraInfo);
    Gtk_SetSpeed(hCamera,pCameraInfo);
    Gtk_SetSnap(hCamera,pCameraInfo);
    return 1;
}

int  Gtk_SetResolution(int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    tSdkImageResolution     sResolution;  //获取当前设置到分辨率
    GtkWidget               *res_combobox=lookup_widget(main_window, "res_combobox");

//设置选中项
    //获得当前预览的分辨率。
    CameraGetImageResolution(hCamera,&sResolution);

    gtk_combo_box_set_active(GTK_COMBO_BOX(res_combobox),sResolution.iIndex);

    return 1;
}

int  Gtk_SetWB(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int RPos,GPos,BPos,Saturation;

    GtkWidget *hscale_gain_r=lookup_widget(main_window, "hscale_gain_r");
    GtkWidget *hscale_gain_g=lookup_widget(main_window, "hscale_gain_g");
    GtkWidget *hscale_gain_b=lookup_widget(main_window, "hscale_gain_b");
    GtkWidget *hscale_saturation=lookup_widget(main_window, "hscale_saturation");
//设置选中项
    //获得当前预览的分辨率。

    CameraGetGain(hCamera,&RPos,&GPos,&BPos);
    CameraGetSaturation(hCamera,&Saturation);

    gtk_range_set_value((GtkRange *)hscale_saturation,Saturation);
    gtk_range_set_value((GtkRange *)hscale_gain_r,RPos);
    gtk_range_set_value((GtkRange *)hscale_gain_g,GPos);
    gtk_range_set_value((GtkRange *)hscale_gain_b,BPos);


    return 1;
}


int  Gtk_SetExposure(int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    GtkWidget *exposure_auto=lookup_widget(main_window, "exposure_auto");
    GtkWidget *exposure_manual=lookup_widget(main_window, "exposure_manual");

    GtkWidget *scale_AE_target=lookup_widget(main_window, "AE_target");
    GtkWidget *scale_exposure_time=lookup_widget(main_window, "exposure_time");
    GtkWidget *scale_exposure_gain=lookup_widget(main_window, "exposure_gain");

    GtkWidget *flick_checkbutton=lookup_widget(main_window, "flick_checkbutton");

    GtkWidget *radio_exposure_mode_auto=lookup_widget(main_window, "exposure_mode_auto");
    GtkWidget *radio_exposure_mode_manual=lookup_widget(main_window, "exposure_mode_manual");

    GtkWidget *radio_50hz=lookup_widget(main_window, "50HZ");
    GtkWidget *radio_60hz=lookup_widget(main_window, "60HZ");


    BOOL            AEstate=FALSE;
    int             pbyAeTarget;
    double          pfExposureTime;
    int             pusAnalogGain;
    BOOL            FlickEnable=FALSE;
    int             piFrequencySel;
    double	        m_fExpLineTime=0;//当前的行曝光时间，单位为us
    tSdkExpose      *  SdkExpose =   &pCameraInfo->sExposeDesc;


    //获得相机当前的曝光模式。
    CameraGetAeState(hCamera,&AEstate);

    //获得自动曝光的亮度目标值。
    CameraGetAeTarget(hCamera,&pbyAeTarget);

    //获得自动曝光时抗频闪功能的使能状态。
    CameraGetAntiFlick(hCamera,&FlickEnable);

    //获得相机的曝光时间。
    CameraGetExposureTime(hCamera,&pfExposureTime);

    //获得图像信号的模拟增益值。
    CameraGetAnalogGain(hCamera,&pusAnalogGain);

    //获得自动曝光时，消频闪的频率选择。
    CameraGetLightFrequency(hCamera,&piFrequencySel);

/*
    获得一行的曝光时间。对于CMOS传感器，其曝光
    的单位是按照行来计算的，因此，曝光时间并不能在微秒
    级别连续可调。而是会按照整行来取舍。这个函数的
    作用就是返回CMOS相机曝光一行对应的时间。
*/
    CameraGetExposureLineTime(hCamera, &m_fExpLineTime);


//设置最大最小值
    gtk_range_set_range((GtkRange *)scale_AE_target,SdkExpose->uiTargetMin,SdkExpose->uiTargetMax);
    gtk_range_set_range((GtkRange *)scale_exposure_time,SdkExpose->uiExposeTimeMin,SdkExpose->uiExposeTimeMax);
    gtk_range_set_range((GtkRange *)scale_exposure_gain,SdkExpose->uiAnalogGainMin,SdkExpose->uiAnalogGainMax);

//曝光目标值滑动设置对应位置
    gtk_range_set_value((GtkRange *)scale_AE_target,pbyAeTarget);
    gtk_range_set_value((GtkRange *)scale_exposure_time,pfExposureTime/m_fExpLineTime);
    gtk_range_set_value((GtkRange *)scale_exposure_gain,pusAnalogGain);

//设置选中内容


    if(!AEstate){
        //设置相机曝光的模式。自动或者手动。关
        CameraSetAeState(hCamera,FALSE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_manual"), TRUE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_auto"), FALSE);
        gtk_toggle_button_set_active ((GtkToggleButton *)radio_exposure_mode_manual,TRUE);
    }else{
        //设置相机曝光的模式。自动或者手动。开
        CameraSetAeState(hCamera,TRUE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_manual"), FALSE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_auto"), TRUE);
        gtk_toggle_button_set_active ((GtkToggleButton *)radio_exposure_mode_auto,TRUE);

        if(FlickEnable==TRUE){
            //设置自动曝光时抗频闪功能的使能状态。对于手动曝光模式下无效。
            CameraSetAntiFlick(g_hCamera,TRUE);
            gtk_toggle_button_set_active ((GtkToggleButton *)flick_checkbutton,TRUE);
            if(piFrequencySel){
                //设置自动曝光时消频闪的频率。0:50HZ , 1:60HZ
                CameraSetLightFrequency(g_hCamera,piFrequencySel);
                gtk_toggle_button_set_active ((GtkToggleButton *)radio_60hz,TRUE);
            }else{
                CameraSetLightFrequency(g_hCamera,piFrequencySel);
                gtk_toggle_button_set_active ((GtkToggleButton *)radio_50hz,TRUE);
            }
        }else{
            CameraSetAntiFlick(g_hCamera,FALSE);
            gtk_widget_set_sensitive ((GtkWidget *)radio_60hz, FALSE);
            gtk_widget_set_sensitive ((GtkWidget *)radio_50hz, FALSE);
        }
    }


    return 1;
}

int  Gtk_SetGamma_Contrast(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    GtkWidget *hscale_mapp_contrast=lookup_widget(main_window, "hscale_mapp_contrast");
    GtkWidget *hscale_mapp_gamma=lookup_widget(main_window, "hscale_mapp_gamma");

    int                 gamma=0;
    int                 contrast=0;
    tGammaRange         *  sGammaRange      =   &pCameraInfo->sGammaRange;
    tContrastRange      *  sContrastRange   =   &pCameraInfo->sContrastRange;

    //获得LUT动态生成模式下的Gamma值。
    CameraGetGamma(hCamera,&gamma);

    //获得LUT动态生成模式下的对比度值
    CameraGetContrast(hCamera,&contrast);

//设置最大最小值
    gtk_range_set_range((GtkRange *)hscale_mapp_gamma,sGammaRange->iMin,sGammaRange->iMax);
    gtk_range_set_range((GtkRange *)hscale_mapp_contrast,sContrastRange->iMin,sContrastRange->iMax);

//滑动设置对应位置
    gtk_range_set_value((GtkRange *)hscale_mapp_contrast,contrast);
    gtk_range_set_value((GtkRange *)hscale_mapp_gamma,gamma);

    return 1;
}



int  Gtk_SetISPProce(int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    GtkWidget *check_isp_h=lookup_widget(main_window, "check_isp_h");
    GtkWidget *check_isp_v=lookup_widget(main_window, "check_isp_v");
    GtkWidget *hscale_isp_acutance=lookup_widget(main_window, "hscale_isp_acutance");


    BOOL        m_bHflip=FALSE;
    BOOL        m_bVflip=FALSE;
    int         m_Sharpness=0;

    tSharpnessRange  *  SharpnessRange =   &pCameraInfo->sSharpnessRange;
    //获得图像的镜像状态。
    CameraGetMirror(hCamera, MIRROR_DIRECTION_HORIZONTAL, &m_bHflip);
    CameraGetMirror(hCamera, MIRROR_DIRECTION_VERTICAL,   &m_bVflip);
    //获取当前锐化设定值。
    CameraGetSharpness(hCamera, &m_Sharpness);


//设置最大最小值 +10
    gtk_range_set_range((GtkRange *)hscale_isp_acutance,SharpnessRange->iMin,SharpnessRange->iMax);

//曝光目标值滑动设置对应位置
    gtk_range_set_value((GtkRange *)hscale_isp_acutance,m_Sharpness);


////设置选中内容
    if(m_bHflip){
        gtk_toggle_button_set_active ((GtkToggleButton *)check_isp_h,TRUE);
    }else{
        gtk_toggle_button_set_active ((GtkToggleButton *)check_isp_h,FALSE);
    }
    if(m_bVflip){
        gtk_toggle_button_set_active ((GtkToggleButton *)check_isp_v,TRUE);
    }else{
        gtk_toggle_button_set_active ((GtkToggleButton *)check_isp_v,FALSE);
    }



    return 1;
}



int  Gtk_SetSpeed(int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    int         pbyFrameSpeed=0;
    int         i,size=pCameraInfo->iFrameSpeedDesc;
    GtkWidget   *radio_speed[size];

    for(i=0;i<size;i++){
        radio_speed[i]=lookup_widget(main_window, pCameraInfo->pFrameSpeedDesc[i].acDescription);
    }

    //获得相机输出图像的帧率选择索引号。
    CameraGetFrameSpeed(hCamera,&pbyFrameSpeed);

//设置选中内容
    gtk_toggle_button_set_active ((GtkToggleButton *)radio_speed[pbyFrameSpeed],TRUE);


    return 1;
}


int  Gtk_SetTrigger(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int  pbySnapMode;
    int StrobeMode=0;
    int  uPolarity=0;

    GtkWidget *radio_collect=lookup_widget(main_window, "radio_collect");
    GtkWidget *radio_software_trigger=lookup_widget(main_window, "radio_software_trigger");

    GtkWidget *radio_trigger_hardware  = lookup_widget(main_window, "radio_trigger_hardware");
    GtkWidget *software_trigger_once_button=lookup_widget(main_window, "software_trigger_once_button");

    GtkWidget *flashlight=lookup_widget(main_window, "flashlight");
    GtkWidget *flashlight_polarity=lookup_widget(main_window, "flashlight_polarity");

    GtkWidget *radio_flashlight_auto   =lookup_widget(main_window, "radio_flashlight_auto");
    GtkWidget *radio_flashlight_manual =lookup_widget(main_window, "radio_flashlight_manual");

    GtkWidget *radio_flashlight_h   =lookup_widget(main_window, "radio_flashlight_h");
    GtkWidget *radio_flashlight_l   =lookup_widget(main_window, "radio_flashlight_l");


    //获得相机的触发模式。
    CameraGetTriggerMode(hCamera,&pbySnapMode);

//设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
    switch(pbySnapMode){
        case 0:
            gtk_toggle_button_set_active ((GtkToggleButton *)radio_collect,TRUE);
            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, FALSE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, FALSE);
        break;
        case 1:
            gtk_toggle_button_set_active ((GtkToggleButton *)radio_software_trigger,TRUE);
            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, TRUE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, FALSE);
        break;
        case 2:
            gtk_toggle_button_set_active ((GtkToggleButton *)radio_trigger_hardware,TRUE);
            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, FALSE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, TRUE);

            CameraGetStrobeMode(hCamera,&StrobeMode);
            CameraGetStrobePolarity(hCamera,&uPolarity);

            if(StrobeMode){
                CameraSetStrobePolarity(hCamera,1);
                gtk_widget_set_sensitive ((GtkWidget *)flashlight_polarity, TRUE);
                gtk_toggle_button_set_active ((GtkToggleButton *)radio_flashlight_manual,TRUE);
                if(uPolarity){
                    gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_h, TRUE);
                    CameraSetStrobePolarity(hCamera,1);
                }else{
                    gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_l, TRUE);
                    CameraSetStrobePolarity(hCamera,0);
                }
            }else{
                CameraSetStrobePolarity(hCamera,0);
                gtk_widget_set_sensitive ((GtkWidget *)flashlight_polarity, FALSE);
                gtk_toggle_button_set_active ((GtkToggleButton *)radio_flashlight_auto,TRUE);
            }

        break;
        default:
            gtk_toggle_button_set_active ((GtkToggleButton *)radio_collect,TRUE);
            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, FALSE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, FALSE);
        break;
    }



    return 1;
}


int  Gtk_SetSnap(int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    GtkWidget *radio_BMP  = lookup_widget(main_window, "BMP");
    GtkWidget *radio_RAW  = lookup_widget(main_window, "RAW");

    g_SaveImage_type=3;


    gtk_toggle_button_set_active ((GtkToggleButton *)radio_BMP,TRUE);


    return 1;
}


