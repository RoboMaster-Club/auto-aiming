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


#include <sys/time.h>
#include "CameraApi.h"


//SDK使用
extern int                     g_hCamera;           //设备句柄
extern unsigned char           *g_pRgbBuffer;      //处理后数据缓存区
extern unsigned char           *g_pRawBuffer;      //处理后数据缓存区
extern tSdkFrameHead           g_tFrameHead;        //图像帧头信息
extern tSdkFrameHead           g_tRawFrameHead;     //图像帧头信息
extern tSdkCameraCapbility     g_tCapability;       //设备描述信息
extern int                     g_EyeCount;

//gtk demo使用
extern GtkBuilder           *Demo_builder ;         //glade进行界面设计
extern GtkWidget            *main_window ;          //主窗体
extern GtkWidget            *display_drawingarea;   //画板

extern pthread_t            g_thread_id;            //线程
extern pthread_t            g_thread_getcounts;     //线程
extern BYTE                 *g_readBuf;             //画板显示数据区
extern int                  g_display_state;        //显示状态
extern int                  g_disply_fps;           //统计帧率
extern int                  g_SaveParameter_num;    //保存参数组
extern int                  g_SaveImage_type;       //保存图像格式
extern Width_Height         g_W_H_INFO;             //显示画板到大小和图像大小

GtkWidget                   *snap_path_window;
GtkWidget                   *file_load_window;
GtkWidget                   *Demo_display_window;

extern  int                    res_test;

void on_entry_num_insert_text (GtkWidget* entry, gchar* new_text, int new_text_length, int* position)
{
    if (new_text[0] > '9' || new_text [0] < '0')
    {
        new_text[0] = '\0';
    }

    return ;
}

//图像显示
gboolean preview_display( GtkWidget *widget, GdkEventExpose *event,gpointer data )
{

    if(g_tFrameHead.uiMediaType==CAMERA_MEDIA_TYPE_MONO8){
        gdk_draw_gray_image(widget->window,widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
                            g_W_H_INFO.xOffsetFOV,g_W_H_INFO.yOffsetFOV,
                            g_W_H_INFO.sensor_width,g_W_H_INFO.sensor_height,
                            GDK_RGB_DITHER_NORMAL,(guchar *)g_readBuf,g_W_H_INFO.sensor_width);
    }
    else if(g_tFrameHead.uiMediaType == CAMERA_MEDIA_TYPE_RGB8) {
        //显示图像
        gdk_draw_rgb_image (widget->window,widget->style->fg_gc[GTK_WIDGET_STATE(widget)],
                            g_W_H_INFO.xOffsetFOV,g_W_H_INFO.yOffsetFOV,
                            g_W_H_INFO.sensor_width,g_W_H_INFO.sensor_height,
                            GDK_RGB_DITHER_NORMAL,(guchar *)g_readBuf,g_W_H_INFO.sensor_width*3);
    }
  //g_disply_fps++;

	return TRUE;
}

//分辨率模块
gboolean   on_res_combobox_changed      (GtkWidget *widget,gpointer     user_data)
{
	gint level=0;
    tSdkImageResolution   *pImageSizeDesc=g_tCapability.pImageSizeDesc;// 预设分辨率选择
    int                       iImageSizeDesc=g_tCapability.iImageSizeDesc; // 预设分辨率的个数，即pImageSizeDesc数组的大小ŝ

	level = gtk_combo_box_get_active(GTK_COMBO_BOX(widget));

    g_W_H_INFO.sensor_width=pImageSizeDesc[level].iWidth * g_EyeCount;
    g_W_H_INFO.sensor_height=pImageSizeDesc[level].iHeight;
    g_W_H_INFO.buffer_size= g_W_H_INFO.sensor_width*g_W_H_INFO.sensor_height;


    if(g_W_H_INFO.sensor_width < g_W_H_INFO.display_width){
        g_W_H_INFO.xOffsetFOV=(g_W_H_INFO.display_width-g_W_H_INFO.sensor_width)/2;
    }else{
        g_W_H_INFO.xOffsetFOV=0;
    }
    if(g_W_H_INFO.sensor_height < g_W_H_INFO.display_height){
        g_W_H_INFO.yOffsetFOV=(g_W_H_INFO.display_height-g_W_H_INFO.sensor_height)/2;
    }else{
         g_W_H_INFO.yOffsetFOV=0;
    }
    res_test=1;
    //设置预览的分辨率。
    CameraSetImageResolution(g_hCamera,&(pImageSizeDesc[level]));


    gtk_widget_set_size_request ((GtkWidget *)display_drawingarea,g_W_H_INFO.sensor_width,g_W_H_INFO.sensor_height);

    return TRUE;
}

//触发模块
void on_trigger_once_button_pressed         (GtkButton       *button, gpointer         user_data)
{
    //执行一次软触发。执行后，会触发由CameraSetTriggerCount指定的帧数。
    CameraSoftTrigger(g_hCamera);
    return ;
}

void on_radio_flashlight                      (GtkWidget       *radio,int         user_data)
{
    int  uPolarity=0;
    GtkWidget *radio_flashlight_h   =lookup_widget(main_window, "radio_flashlight_h");
    GtkWidget *radio_flashlight_l   =lookup_widget(main_window, "radio_flashlight_l");
    GtkWidget *flashlight_polarity=lookup_widget(main_window, "flashlight_polarity");
    switch(user_data){
        case 0:
            //设置闪光灯信号为自动模式，这种模式下，相机只有在曝光的时间点上，输出有效闪光灯信号，曝光完成后，回到无效电平状态，信号波形是脉冲形式的。
            CameraSetStrobeMode(g_hCamera,0);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight_polarity, FALSE);

        break;

        case 1:
            //设置闪光灯信号为半自动模式，这种模式下，闪光灯信号端子可输出恒定的高、低电平。
            CameraSetStrobeMode(g_hCamera,1);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight_polarity, TRUE);

            CameraGetStrobePolarity(g_hCamera,&uPolarity);
            if(uPolarity){
                gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_h, TRUE);
                CameraSetStrobePolarity(g_hCamera,1);
            }else{
                gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_l, TRUE);
                CameraSetStrobePolarity(g_hCamera,0);
            }
        break;

        default:
            //设置闪光灯信号为半自动模式，这种模式下，闪光灯信号端子可输出恒定的高、低电平。
            CameraSetStrobeMode(g_hCamera,1);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight_polarity, TRUE);

            CameraGetStrobePolarity(g_hCamera,&uPolarity);
            if(uPolarity){
                gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_h, TRUE);
                CameraSetStrobePolarity(g_hCamera,1);
            }else{
                gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_l, TRUE);
                CameraSetStrobePolarity(g_hCamera,0);
            }
        break;
    }
    return ;
}
void on_radio_flashlight_polarity             (GtkWidget       *radio,int         user_data)
{

    switch(user_data){
        case 0:
        //设置有效极性为低电平
            CameraSetStrobePolarity(g_hCamera,0);
        break;

        case 1:
        //设置有效极性为高电平
            CameraSetStrobePolarity(g_hCamera,1);
        break;

        default:
            CameraSetStrobePolarity(g_hCamera,1);
        break;
    }
    return ;
}

void on_radio_trigger                         (GtkWidget       *radio,int         user_data)
{
    int StrobeMode=0;
    int  uPolarity=0;
		struct   timeval   start,stop,diff;
		unsigned long timer;

    GtkWidget *software_trigger_once_button=lookup_widget(main_window, "software_trigger_once_button");
    GtkWidget *flashlight=lookup_widget(main_window, "flashlight");
    GtkWidget *flashlight_polarity=lookup_widget(main_window, "flashlight_polarity");

    GtkWidget *radio_flashlight_auto   =lookup_widget(main_window, "radio_flashlight_auto");
    GtkWidget *radio_flashlight_manual =lookup_widget(main_window, "radio_flashlight_manual");

    GtkWidget *radio_flashlight_h   =lookup_widget(main_window, "radio_flashlight_h");
    GtkWidget *radio_flashlight_l   =lookup_widget(main_window, "radio_flashlight_l");

    switch(user_data){
        case 0:

            //设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
            CameraSetTriggerMode(g_hCamera, 0);
            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, FALSE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, FALSE);
        break;
        case 1:
		printf("start  \n");
		gettimeofday(&start,0);
            //设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
            CameraSetTriggerMode(g_hCamera, 1);
		gettimeofday(&stop,0);
		timer = 1000000 * (stop.tv_sec-start.tv_sec)+ stop.tv_usec-start.tv_usec;

		printf("total time = %d  us \n",timer);

            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, TRUE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, FALSE);
        break;
        case 2:
            //设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
            CameraSetTriggerMode(g_hCamera, 2);
            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, FALSE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, TRUE);

            CameraGetStrobeMode(g_hCamera,&StrobeMode);
            CameraGetStrobePolarity(g_hCamera,&uPolarity);

            if(StrobeMode){
                gtk_widget_set_sensitive ((GtkWidget *)flashlight_polarity, TRUE);
                gtk_toggle_button_set_active ((GtkToggleButton *)radio_flashlight_manual,TRUE);
                CameraSetStrobePolarity(g_hCamera,1);
                if(uPolarity){
                    gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_h, TRUE);
                    CameraSetStrobePolarity(g_hCamera,1);
                }else{
                    gtk_widget_set_sensitive ((GtkWidget *)radio_flashlight_l, TRUE);
                    CameraSetStrobePolarity(g_hCamera,0);
                }
            }else{
                gtk_widget_set_sensitive ((GtkWidget *)flashlight_polarity, FALSE);
                gtk_toggle_button_set_active ((GtkToggleButton *)radio_flashlight_auto,TRUE);
                CameraSetStrobePolarity(g_hCamera,0);
            }

        break;
        default:
            //设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
            CameraSetTriggerMode(g_hCamera, 0);
            gtk_widget_set_sensitive ((GtkWidget *)software_trigger_once_button, TRUE);
            gtk_widget_set_sensitive ((GtkWidget *)flashlight, FALSE);
        break;
    }

    return ;
}


//白平衡模块的回调函数


void on_AWB_once_button_pressed     (GtkButton       *button,
                                          gpointer         user_data)
{

    int RPos,GPos,BPos;
    GtkWidget *hscale_gain_r=lookup_widget(main_window, "hscale_gain_r");
    GtkWidget *hscale_gain_g=lookup_widget(main_window, "hscale_gain_g");
    GtkWidget *hscale_gain_b=lookup_widget(main_window, "hscale_gain_b");

    CameraSetOnceWB(g_hCamera);

    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);

    gtk_range_set_value((GtkRange *)hscale_gain_r,RPos);
    gtk_range_set_value((GtkRange *)hscale_gain_g,GPos);
    gtk_range_set_value((GtkRange *)hscale_gain_b,BPos);
	return ;
}

void on_gain_r_value_changed (GtkScale *scale, gpointer data)
{
    int i=0;
    int RPos,GPos,BPos;

//设置选中项
    //获得当前预览的分辨率。
    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);

    i = gtk_range_get_value(GTK_RANGE(scale));
    CameraSetGain(g_hCamera,i,GPos,BPos);
    return ;
}

void on_gain_g_value_changed (GtkScale *scale, gpointer data)
{
    int i=0;
    int RPos,GPos,BPos;

//设置选中项
    //获得当前预览的分辨率。
    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);

    i = gtk_range_get_value(GTK_RANGE(scale));
    CameraSetGain(g_hCamera,RPos,i,BPos);
    return ;

}

void on_gain_b_value_changed (GtkScale *scale, gpointer data)
{
    int i=0;
    int RPos,GPos,BPos;

//设置选中项
    //获得当前预览的分辨率。
    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);

    i = gtk_range_get_value(GTK_RANGE(scale));
    CameraSetGain(g_hCamera,RPos,GPos,i);
    return ;

}

void on_saturation_value_changed (GtkScale *scale, gpointer data)
{
    int i=0;
    i = gtk_range_get_value(GTK_RANGE(scale));
    CameraSetSaturation(g_hCamera,i);
    return ;
}



//曝光模块的回调函数
void on_exposure_gain_value_changed (GtkScale *scale, gpointer data)
{
    double iPos=0;
    iPos = gtk_range_get_value(GTK_RANGE(scale));
/*
    设置相机的图像模拟增益值。该值乘以CameraGetCapability获得
    的相机属性结构体中sExposeDesc.fAnalogGainStep，就
    得到实际的图像信号放大倍数。
*/
    CameraSetAnalogGain(g_hCamera,iPos);
    return ;
}

void on_exposure_time_value_changed (GtkScale *scale, gpointer data)
{
    double  iPos=0;
    double	m_fExpLineTime=0;//当前的行曝光时间，单位为us
    GtkWidget *label_exp_time=lookup_widget(main_window, "label_exp_time");
    char    buffer[16];

    iPos = gtk_range_get_value(GTK_RANGE(scale));

/*
    获得一行的曝光时间。对于CMOS传感器，其曝光
    的单位是按照行来计算的，因此，曝光时间并不能在微秒
    级别连续可调。而是会按照整行来取舍。这个函数的
    作用就是返回CMOS相机曝光一行对应的时间。
*/
    CameraGetExposureLineTime(g_hCamera, &m_fExpLineTime);

/*
    设置曝光时间。单位为微秒。对于CMOS传感器，其曝光
    的单位是按照行来计算的，因此，曝光时间并不能在微秒
    级别连续可调。而是会按照整行来取舍。在调用
    本函数设定曝光时间后，建议再调用CameraGetExposureTime
    来获得实际设定的值。
*/
    CameraSetExposureTime(g_hCamera,iPos*m_fExpLineTime);

    //除以1000是换算成毫秒
    sprintf( buffer,"%0.3f",((iPos*m_fExpLineTime)/1000));

    gtk_label_set_text((GtkLabel *)label_exp_time,(gchar*)buffer);

    return ;

}


void on_AE_target_value_changed (GtkScale *scale, gpointer data)
{
    int pbyAeTarget=0;
    pbyAeTarget = (int)gtk_range_get_value(GTK_RANGE(scale));
/*
    设定自动曝光的亮度目标值。设定范围由CameraGetCapability函数获得。
*/
    CameraSetAeTarget(g_hCamera,pbyAeTarget);
}

void  on_flick_check   (GtkButton *button,gpointer data)
{
    GtkWidget *radio_50hz;
    GtkWidget *radio_60hz;

    radio_50hz  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "50HZ"));
    radio_60hz  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "60HZ"));


    if(GTK_TOGGLE_BUTTON(button)->active)
    {
        //设置自动曝光时抗频闪功能的使能状态。对于手动曝光模式下无效。
        CameraSetAntiFlick(g_hCamera,TRUE);
        gtk_widget_set_sensitive ((GtkWidget *)radio_50hz, TRUE);
        gtk_widget_set_sensitive ((GtkWidget *)radio_60hz, TRUE);
    }
    else
    {
        //设置自动曝光时抗频闪功能的使能状态。对于手动曝光模式下无效。
        CameraSetAntiFlick(g_hCamera,FALSE);
        gtk_widget_set_sensitive ((GtkWidget *)radio_50hz, FALSE);
        gtk_widget_set_sensitive ((GtkWidget *)radio_60hz, FALSE);
    }
}

void on_radio_5060hz(GtkWidget       *radio,int         user_data)
{
    if(user_data==1){
        //设置自动曝光时消频闪的频率。0: 50hz  1:60hz
        CameraSetLightFrequency(g_hCamera,0);
    }else{
        //设置自动曝光时消频闪的频率。0: 50hz  1:60hz
        CameraSetLightFrequency(g_hCamera,1);
    }

	return ;
}

void on_radio_exposure_mode(GtkWidget       *radio,int         user_data)
{
    GtkWidget *radio_50hz=lookup_widget(main_window, "50HZ");
    GtkWidget *radio_60hz=lookup_widget(main_window, "60HZ");
    GtkWidget *flick_checkbutton=lookup_widget(main_window, "flick_checkbutton");
    BOOL        FlickEnable=FALSE;
    int         piFrequencySel;

    //获得自动曝光时抗频闪功能的使能状态。
    CameraGetAntiFlick(g_hCamera,&FlickEnable);
    //获得自动曝光时，消频闪的频率选择。
    CameraGetLightFrequency(g_hCamera,&piFrequencySel);

    if(user_data==1){
        //设置相机曝光的模式。自动或者手动。
        CameraSetAeState(g_hCamera,TRUE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_manual"), FALSE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_auto"), TRUE);

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
    }else{
        CameraSetAeState(g_hCamera,FALSE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_manual"), TRUE);
        gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "exposure_auto"), FALSE);
    }

	return ;
}



//速率切换
void on_radio_speed_clicked       (GtkWidget       *radio,int         user_data)
{
    int         i,size=g_tCapability.iFrameSpeedDesc;
    GtkWidget   *radio_speed[size];

    for(i=0;i<size;i++){
        radio_speed[i]=lookup_widget(main_window, g_tCapability.pFrameSpeedDesc[i].acDescription);
    }
/*
    设定相机输出图像的帧率。相机可供选择的帧率模式由
    CameraGetCapability获得的信息结构体中iFrameSpeedDesc
    表示最大帧率选择模式个数。
*/
    CameraSetFrameSpeed(g_hCamera,user_data);

//设置选中内容
    gtk_toggle_button_set_active ((GtkToggleButton *)radio_speed[user_data],TRUE);
	return ;
}


//抓拍
void on_radio_picture_clicked       (GtkWidget       *radio,int         user_data)
{
    g_SaveImage_type=user_data;
	return ;
}
void on_radio_snap_catch_clicked          (GtkWidget       *radio,int          user_data)
{
	return ;
}
void  on_snap_content_check   (GtkButton *button,gpointer data)
{
	return ;
}
void  on_snap_time_check   (GtkButton *button,gpointer data)
{

//    if(GTK_TOGGLE_BUTTON(button)->active)
//    {
//        printf("snap_time_check on\n");
//    }
//    else
//    {
//        printf("snap_time_check off\n");
//    }
	return ;
}
void on_snap_catch_button_pressed         (GtkButton       *button,gpointer         user_data)
{
	tSdkFrameHead	tFrameHead = g_tRawFrameHead;
	BYTE			*pbyBuffer;
	BYTE			*pbImgBuffer;
    char            filename[512]={0};
    char            *tmp=NULL;
    emSdkFileType 	fmt;
    GtkWidget * path_entry=lookup_widget(main_window, "snap_path_entry");

    tmp=(char *)gtk_entry_get_text((GtkEntry  *)path_entry);

    static int save_no = 0;
    sprintf(filename,"%s/snap%d",tmp,++save_no);

    switch(g_SaveImage_type){
        case 1:
        case 2:
        case 3:
            pbImgBuffer = (unsigned char*)malloc(g_tCapability.sResolutionRange.iHeightMax*g_tCapability.sResolutionRange.iWidthMax*g_EyeCount*3);
            /*
            将获得的相机原始输出图像数据进行处理，叠加饱和度、
            颜色增益和校正、降噪等处理效果，最后得到RGB888
            格式的图像数据。
            */
            CameraImageProcessEx(g_hCamera, g_pRawBuffer,pbImgBuffer,&tFrameHead, CAMERA_MEDIA_TYPE_RGB8, 0);

            fmt = FILE_BMP;
            if (g_SaveImage_type == 1)
            	fmt = FILE_JPG;
            else if (g_SaveImage_type == 2)
            	fmt = FILE_PNG;

            //将图像缓冲区的数据保存成图片文件。
            CameraSaveImage(g_hCamera, filename,pbImgBuffer, &tFrameHead, fmt, 100);
            
            free(pbImgBuffer);
        break;

        case 4:
            CameraSaveImage(g_hCamera, filename,g_pRawBuffer, &tFrameHead, FILE_RAW, 100);
            
        break;

        default :
        break;
    }

    return ;
}
void on_snap_res_button_pressed         (GtkButton       *button,gpointer         user_data)
{
    return ;
}


void on_snap_path_button_pressed         (GtkButton       *button,gpointer         user_data)
{

    snap_path_window = create_snap_path_window();

    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), FALSE);

    gtk_widget_show_all (snap_path_window);
    return ;
}

gboolean on_snap_period_entry_call (GtkWidget *widget,GdkEventKey *event,gpointer user_data)
{

//    if((event->keyval>=GDK_0 && event->keyval<=GDK_9)|| (event->keyval>=GDK_KP_0 && event->keyval<=GDK_KP_9))
//    {
//        printf("on_snap_period_entry_call = %s  \n",gtk_entry_get_text((GtkEntry      *)widget));
//    }

    return FALSE;
}

void on_snap_path_window_delete_event(GtkWidget *widget, gpointer *data)
{
    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), TRUE);
    gtk_widget_hide (snap_path_window);
    return ;
}

void on_snap_path_confirm_button_pressed         (GtkButton       *button,gpointer         user_data)
{
    gchar *filename;
    GtkWidget * path_entry;

    path_entry=lookup_widget(main_window, "snap_path_entry");

    filename=gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(snap_path_window));


    gtk_entry_set_text((GtkEntry      *)path_entry, filename);

    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), TRUE);



    gtk_widget_hide (snap_path_window);

    return ;
}

void on_snap_path_cancel_button_pressed         (GtkButton       *button,gpointer         user_data)
{

    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), TRUE);
    gtk_widget_hide (snap_path_window);
    return ;
}

//ISP图形处理模块
void on_isp_acutance_value_changed (GtkScale *scale, gpointer data)
{
    int iPos=0;
    iPos = gtk_range_get_value(GTK_RANGE(scale));
    gtk_range_set_value((GtkRange *)scale,iPos);
    CameraSetSharpness(g_hCamera,iPos);
    return ;
}
void  on_isp_h_check   (GtkButton *button,gpointer data)
{
    if(GTK_TOGGLE_BUTTON(button)->active)
    {
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。水平
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_HORIZONTAL, TRUE);
    }
    else
    {
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。水平
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_HORIZONTAL, FALSE);
    }
    return ;
}
void  on_isp_v_check   (GtkButton *button,gpointer data)
{
    if(GTK_TOGGLE_BUTTON(button)->active)
    {
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。垂直
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_VERTICAL, TRUE);
    }
    else
    {
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。垂直
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_VERTICAL, FALSE);
    }
    return ;
}

//查寻映射表
void on_mapp_contrast_value_changed (GtkScale *scale, gpointer data)
{
    int iPos=0;
    iPos =(int)gtk_range_get_value(GTK_RANGE(scale));
/*
    设定LUT动态生成模式下的对比度值。设定的值会
    马上保存在SDK内部，但是只有当相机处于动态
    参数生成的LUT模式时，才会生效。
*/
    CameraSetContrast(g_hCamera,iPos);
    return ;
}
void on_mapp_gamma_value_changed (GtkScale *scale, gpointer data)
{
    int iPos=0;
    iPos =(int)gtk_range_get_value(GTK_RANGE(scale));

/*
    设定LUT动态生成模式下的Gamma值。设定的值会
    马上保存在SDK内部，但是只有当相机处于动态
    参数生成的LUT模式时，才会生效。
*/

    CameraSetGamma(g_hCamera,iPos);
    return ;
}

// 参数组
void on_para_save_button_pressed         (GtkButton       *button,gpointer         user_data)
{
    /*
    存当前相机参数到指定的参数组中。相机提供了A,B,C,D
    A,B,C,D四组空间来进行参数的保存。
    */
    CameraSaveParameter(g_hCamera,g_SaveParameter_num);
    return ;
}

void on_para_acquiesce_button_pressed         (GtkButton       *button,gpointer         user_data)
{
    //加载指定组的参数到相机中。
    CameraLoadParameter(g_hCamera,PARAMETER_TEAM_DEFAULT);

    Gtk_GUI_Init(g_hCamera,&g_tCapability);
    return ;
}
void on_para_load_button_pressed         (GtkButton       *button,gpointer         user_data)
{
    file_load_window = create_file_load_window();

    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), FALSE);

    gtk_widget_show_all (file_load_window);
    return ;
}

void on_radio_para_ABCD                    (GtkWidget       *radio,int         user_data)
{
    /*
    存当前相机参数到指定的参数组中。相机提供了A,B,C,D
    A,B,C,D四组空间来进行参数的保存。
    */
    CameraSaveParameter(g_hCamera,g_SaveParameter_num);
    g_SaveParameter_num=user_data;

    //加载指定组的参数到相机中。
    CameraLoadParameter(g_hCamera,g_SaveParameter_num);

    Gtk_GUI_Init(g_hCamera,&g_tCapability);
    return ;
}




//文件加载
void on_file_load_window_delete_event(GtkWidget *widget, gpointer *data)
{
    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), TRUE);
    gtk_widget_hide (file_load_window);
}

void on_file_load_confirm_button_pressed         (GtkButton       *button,gpointer         user_data)
{
    gchar *filename;


    filename=gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(file_load_window));


    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), TRUE);

/*
    从PC上指定的参数文件中加载参数。我公司相机参数
    保存在PC上为.config后缀的文件，位于安装下的
    Camera\Configs文件夹中。
*/

    CameraReadParameterFromFile(g_hCamera,filename);
    Gtk_GUI_Init(g_hCamera,&g_tCapability);

    gtk_widget_hide (file_load_window);


    return ;
}
void on_file_load_cancel_button_pressed         (GtkButton       *button,gpointer         user_data)
{

    gtk_widget_set_sensitive ((GtkWidget *)lookup_widget(main_window, "main_window"), TRUE);
    gtk_widget_hide (file_load_window);
    return ;
}


gboolean on_gtkdemo_window_delete_event      (GtkWidget       *widget,
                                            GdkEvent        *event,
                                            gpointer         user_data)
{
    if(g_display_state){
        g_display_state=0;

        gdk_threads_leave ();
        CameraSetTriggerMode(g_hCamera, 0);

		pthread_join(g_thread_id, NULL);
		pthread_join(g_thread_getcounts, NULL);
    }

    if(g_readBuf!=NULL){
        CameraAlignFree(g_readBuf);
        g_readBuf=NULL;
    }

    if(g_pRgbBuffer!=NULL){
        CameraAlignFree(g_pRgbBuffer);
        g_pRgbBuffer=NULL;
    }

    if (g_pRawBuffer!=NULL){
        CameraAlignFree(g_pRawBuffer);
        g_pRawBuffer=NULL;
    }

    if(g_hCamera>0){
        //相机反初始化。释放资源。
        CameraUnInit(g_hCamera);
        g_hCamera=-1;
    }


    gtk_main_quit();


 	return FALSE;
}
