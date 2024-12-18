#include <stdlib.h>
#include <string.h>
#include <gtk/gtk.h>

#include "Demo.h"
#include "interface.h"
#include "callbacks.h"
#include "support.h"

extern tSdkCameraCapbility  g_tCapability;      //设备描述信息

//gtk demo使用
extern GtkBuilder       *Demo_builder ;         //glade进行界面设计
extern GtkWidget        *main_window ;          //主窗体
extern GtkWidget        *display_drawingarea;   //画板
extern GtkWidget        *Demo_display_window;

extern int              g_hCamera;
extern int              g_read_fps;             //统计读取帧率
extern int              g_disply_fps;           //统计显示帧率
extern Width_Height     g_W_H_INFO;             //显示画板到大小和图像大小

int                     g_snap_path_window_state=0; //保存图片路径窗体  0未创建 1已创建
int                     g_file_load_window_state=0; //文件加载窗体  0未创建 1已创建
int                     g_SaveParameter_num=0;      //保存参数组
int                     g_SaveImage_type=0;         //保存图像格式


#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
    gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)



//分辨率窗口初始化
void create_gtkdemo_resolution (GtkWidget *window,int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    GtkWidget               *combobox = NULL;
    GtkListStore            *store = NULL;
    GtkTreeIter             iter;
    GtkCellRenderer         *renderer = NULL;
    int                     i=0;
    tSdkImageResolution     *pImageSizeDesc=pCameraInfo->pImageSizeDesc;// 预设分辨率选择
    int                     iImageSizeDesc=pCameraInfo->iImageSizeDesc; // 预设分辨率的个数，即pImageSizeDesc数组的大小

    combobox = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "res_combobox"));


    GLADE_HOOKUP_OBJECT (window, combobox, "res_combobox");

    store = gtk_list_store_new(1, G_TYPE_STRING);

//与SDK相关到操作需要获取参数
    for(i=0;i<iImageSizeDesc;i++){
        gtk_list_store_append((GtkListStore *)store, &iter);
        gtk_list_store_set((GtkListStore *)store, &iter, 0, pImageSizeDesc[i].acDescription, -1);
    }

    // 设置GtkTreeModel中的每一项数据如何在列表框中显示
    gtk_combo_box_set_model((GtkComboBox *)combobox,(GtkTreeModel *)store );

    renderer = gtk_cell_renderer_text_new();
    gtk_cell_layout_pack_start(GTK_CELL_LAYOUT(combobox), renderer, TRUE);
    gtk_cell_layout_set_attributes(GTK_CELL_LAYOUT(combobox), renderer, "text", 0, NULL);
    gtk_signal_connect(GTK_OBJECT(combobox), "changed",G_CALLBACK (on_res_combobox_changed), NULL);


    return ;
}


void create_gtkdemo_trigger (GtkWidget *window)
{

    GtkWidget *radio_collect;
    GtkWidget *radio_software_trigger;
    GtkWidget *radio_trigger_hardware;

    GtkWidget *radio_flashlight_auto;
    GtkWidget *radio_flashlight_manual;
    GtkWidget *radio_flashlight_h;
    GtkWidget *radio_flashlight_l;

    GtkWidget *flashlight;
    GtkWidget *flashlight_polarity;

    GtkWidget *software_trigger_once_button;

    flashlight = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "flashlight"));
    flashlight_polarity = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "flashlight_polarity"));

    software_trigger_once_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "software_trigger_once_button"));
    radio_collect  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_collect"));
    radio_software_trigger  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_software_trigger"));
    radio_trigger_hardware  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_trigger_hardware"));

    radio_flashlight_auto  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_flashlight_auto"));
    radio_flashlight_manual  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_flashlight_manual"));
    radio_flashlight_h  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_flashlight_h"));
    radio_flashlight_l  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_flashlight_l"));


    g_signal_connect ((gpointer) software_trigger_once_button, "pressed",G_CALLBACK (on_trigger_once_button_pressed),NULL);
    g_signal_connect(G_OBJECT(radio_collect),"released",G_CALLBACK(on_radio_trigger),(gpointer)0);
    g_signal_connect(G_OBJECT(radio_software_trigger),"released",G_CALLBACK(on_radio_trigger),(gpointer)1);
    g_signal_connect(G_OBJECT(radio_trigger_hardware),"released",G_CALLBACK(on_radio_trigger),(gpointer)2);

    g_signal_connect(G_OBJECT(radio_flashlight_auto),"released",G_CALLBACK(on_radio_flashlight),(gpointer)0);
    g_signal_connect(G_OBJECT(radio_flashlight_manual),"released",G_CALLBACK(on_radio_flashlight),(gpointer)1);

    g_signal_connect(G_OBJECT(radio_flashlight_h),"released",G_CALLBACK(on_radio_flashlight_polarity),(gpointer)0);
    g_signal_connect(G_OBJECT(radio_flashlight_l),"released",G_CALLBACK(on_radio_flashlight_polarity),(gpointer)1);


    GLADE_HOOKUP_OBJECT (window, flashlight, "flashlight");
    GLADE_HOOKUP_OBJECT (window, flashlight_polarity, "flashlight_polarity");
    GLADE_HOOKUP_OBJECT (window, software_trigger_once_button, "software_trigger_once_button");
    GLADE_HOOKUP_OBJECT (window, radio_collect, "radio_collect");
    GLADE_HOOKUP_OBJECT (window, radio_software_trigger, "radio_software_trigger");
    GLADE_HOOKUP_OBJECT (window, radio_trigger_hardware, "radio_trigger_hardware");


    GLADE_HOOKUP_OBJECT (window, radio_flashlight_auto, "radio_flashlight_auto");
    GLADE_HOOKUP_OBJECT (window, radio_flashlight_manual, "radio_flashlight_manual");

    GLADE_HOOKUP_OBJECT (window, radio_flashlight_h, "radio_flashlight_h");
    GLADE_HOOKUP_OBJECT (window, radio_flashlight_l, "radio_flashlight_l");



    return ;
}



void create_gtkdemo_snapshot (GtkWidget *window)
{

    GtkWidget *snap_catch_button;
    GtkWidget *snap_res_button;

    GtkWidget *radio_res_costom;
    GtkWidget *radio_res_preview;

    GtkWidget *radio_JPG;
    GtkWidget *radio_PNG;
    GtkWidget *radio_BMP;
    GtkWidget *radio_RAW;

    GtkWidget *snap_content_check;
    GtkWidget *snap_time_check;

    GtkWidget *snap_period_entry;
    GtkWidget *snap_path_entry;

    GtkWidget *snap_path_button;

    radio_JPG  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "JPG"));
    radio_PNG  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "PNG"));
    radio_BMP  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "BMP"));
    radio_RAW  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "RAW"));

    radio_res_costom  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_res_costom"));
    radio_res_preview  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_res_preview"));

    snap_catch_button  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_catch_button"));
    snap_res_button  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_res_button"));
    snap_path_button  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_path_button"));

    snap_content_check  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_content_check"));
    snap_time_check  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_time_check"));

    snap_period_entry = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_period_entry"));
    snap_path_entry = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_path_entry"));

    g_signal_connect(G_OBJECT(radio_JPG),"released",G_CALLBACK(on_radio_picture_clicked),(gpointer)1);
    g_signal_connect(G_OBJECT(radio_PNG),"released",G_CALLBACK(on_radio_picture_clicked),(gpointer)2);
    g_signal_connect(G_OBJECT(radio_BMP),"released",G_CALLBACK(on_radio_picture_clicked),(gpointer)3);
    g_signal_connect(G_OBJECT(radio_RAW),"released",G_CALLBACK(on_radio_picture_clicked),(gpointer)4);

    g_signal_connect(G_OBJECT(radio_res_costom),"released",G_CALLBACK(on_radio_snap_catch_clicked),(gpointer)1);
    g_signal_connect(G_OBJECT(radio_res_preview),"released",G_CALLBACK(on_radio_snap_catch_clicked),(gpointer)2);

    g_signal_connect ((gpointer) snap_catch_button, "pressed",G_CALLBACK (on_snap_catch_button_pressed),NULL);
    g_signal_connect ((gpointer) snap_res_button, "pressed",G_CALLBACK (on_snap_res_button_pressed),NULL);
    g_signal_connect ((gpointer) snap_path_button, "pressed",G_CALLBACK (on_snap_path_button_pressed),NULL);

    g_signal_connect(G_OBJECT(snap_content_check),"clicked",G_CALLBACK(on_snap_content_check),NULL);
    g_signal_connect(G_OBJECT(snap_time_check),"clicked",G_CALLBACK(on_snap_time_check),NULL);

    g_signal_connect(G_OBJECT(snap_period_entry), "insert-text",G_CALLBACK(on_entry_num_insert_text),snap_period_entry);
    g_signal_connect(G_OBJECT(snap_period_entry), "key_release_event",G_CALLBACK(on_snap_period_entry_call),NULL);

    GLADE_HOOKUP_OBJECT (window, radio_JPG, "JPG");
    GLADE_HOOKUP_OBJECT (window, radio_PNG, "PNG");
    GLADE_HOOKUP_OBJECT (window, radio_BMP, "BMP");
    GLADE_HOOKUP_OBJECT (window, radio_RAW, "RAW");

    GLADE_HOOKUP_OBJECT (window, radio_res_costom, "radio_res_costom");
    GLADE_HOOKUP_OBJECT (window, radio_res_preview, "radio_res_preview");

    GLADE_HOOKUP_OBJECT (window, snap_catch_button, "snap_catch_button");
    GLADE_HOOKUP_OBJECT (window, snap_res_button, "snap_res_button");
    GLADE_HOOKUP_OBJECT (window, snap_path_button, "snap_path_button");

    GLADE_HOOKUP_OBJECT (window, snap_content_check, "snap_content_check");
    GLADE_HOOKUP_OBJECT (window, snap_time_check, "snap_time_check");

    GLADE_HOOKUP_OBJECT (window, snap_period_entry, "snap_period_entry");
    GLADE_HOOKUP_OBJECT (window, snap_path_entry, "snap_path_entry");
//设置选中内容

    //gtk_widget_set_sensitive ((GtkWidget *)radio_JPG,FALSE);
    //gtk_widget_set_sensitive ((GtkWidget *)radio_PNG,FALSE);
    gtk_widget_set_sensitive ((GtkWidget *)snap_content_check,FALSE);
    gtk_widget_set_sensitive ((GtkWidget *)snap_time_check,FALSE);
    gtk_widget_set_sensitive ((GtkWidget *)snap_res_button,FALSE);
    gtk_widget_set_sensitive ((GtkWidget *)radio_res_costom,FALSE);
    gtk_widget_set_sensitive ((GtkWidget *)radio_res_preview,FALSE);
    gtk_widget_set_sensitive ((GtkWidget *)snap_period_entry,FALSE);


    gtk_entry_set_text((GtkEntry      *)snap_path_entry, "./");
    return ;
}


void create_gtkdemo_speed (GtkWidget *window,int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int         i,size=pCameraInfo->iFrameSpeedDesc;
    GtkWidget   *layout_speed;
    GtkWidget   *radio_speed[size];
    GtkWidget   * box;
    GSList      *group=NULL;

    layout_speed = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "layout_speed"));

    box = gtk_hbox_new(FALSE,10);
    gtk_container_set_border_width(GTK_CONTAINER(box),10);
    gtk_container_add(GTK_CONTAINER(layout_speed),box);

    //创建多选按钮组
    for(i=0;i<size;i++){

        radio_speed[i] = gtk_radio_button_new_with_label(group,pCameraInfo->pFrameSpeedDesc[i].acDescription);
        group=gtk_radio_button_group(GTK_RADIO_BUTTON(radio_speed[i]));/*将按钮加入组装框*/
        g_signal_connect(G_OBJECT(radio_speed[i]),"released",G_CALLBACK(on_radio_speed_clicked),(gpointer)i);
        gtk_box_pack_start(GTK_BOX(box),radio_speed[i],FALSE,FALSE,5);

        GLADE_HOOKUP_OBJECT (window, radio_speed[i], pCameraInfo->pFrameSpeedDesc[i].acDescription);
    }


    gtk_widget_show_all(box);

    return ;
}


void create_gtkdemo_exposure (GtkWidget *window,int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    GtkWidget *exposure_auto;
    GtkWidget *exposure_manual;

    GtkWidget *scale_AE_target;
    GtkWidget *scale_exposure_time;
    GtkWidget *scale_exposure_gain;

    GtkWidget *flick_checkbutton;

    GtkWidget *radio_exposure_mode_auto;
    GtkWidget *radio_exposure_mode_manual;

    GtkWidget *radio_50hz;
    GtkWidget *radio_60hz;
    GtkWidget *label_exp_time;



    exposure_auto = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "exposure_auto"));
    exposure_manual = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "exposure_manual"));

    label_exp_time= GTK_WIDGET(gtk_builder_get_object(Demo_builder, "label_exp_time"));

    scale_AE_target  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "AE_target"));
    scale_exposure_time  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "exposure_time"));
    scale_exposure_gain  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "exposure_gain"));


    flick_checkbutton = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "flick_checkbutton"));


    radio_exposure_mode_auto  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "exposure_mode_auto"));
    radio_exposure_mode_manual  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "exposure_mode_manual"));

    radio_50hz  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "50HZ"));
    radio_60hz  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "60HZ"));


    g_signal_connect(G_OBJECT(flick_checkbutton),"clicked",G_CALLBACK(on_flick_check),NULL);

    g_signal_connect(G_OBJECT(scale_AE_target),"value_changed",G_CALLBACK(on_AE_target_value_changed),NULL);
    g_signal_connect(G_OBJECT(scale_exposure_time),"value_changed",G_CALLBACK(on_exposure_time_value_changed),NULL);
    g_signal_connect(G_OBJECT(scale_exposure_gain),"value_changed",G_CALLBACK(on_exposure_gain_value_changed),NULL);


    g_signal_connect(G_OBJECT(radio_exposure_mode_auto),"released",G_CALLBACK(on_radio_exposure_mode),(gpointer)1);
    g_signal_connect(G_OBJECT(radio_exposure_mode_manual),"released",G_CALLBACK(on_radio_exposure_mode),(gpointer)0);

    g_signal_connect(G_OBJECT(radio_50hz),"released",G_CALLBACK(on_radio_5060hz),(gpointer)1);
    g_signal_connect(G_OBJECT(radio_60hz),"released",G_CALLBACK(on_radio_5060hz),(gpointer)2);



    GLADE_HOOKUP_OBJECT (window, scale_AE_target, "AE_target");
    GLADE_HOOKUP_OBJECT (window, scale_exposure_time, "exposure_time");
    GLADE_HOOKUP_OBJECT (window, scale_exposure_gain, "exposure_gain");

    GLADE_HOOKUP_OBJECT (window, flick_checkbutton, "flick_checkbutton");

    GLADE_HOOKUP_OBJECT (window, radio_exposure_mode_auto, "exposure_mode_auto");
    GLADE_HOOKUP_OBJECT (window, radio_exposure_mode_manual, "exposure_mode_manual");

    GLADE_HOOKUP_OBJECT (window, radio_50hz, "50HZ");
    GLADE_HOOKUP_OBJECT (window, radio_60hz, "60HZ");

    GLADE_HOOKUP_OBJECT (window, exposure_auto, "exposure_auto");
    GLADE_HOOKUP_OBJECT (window, exposure_manual, "exposure_manual");

    GLADE_HOOKUP_OBJECT (window, label_exp_time, "label_exp_time");



    return ;
}


void create_gtkdemo_awb (GtkWidget *window)
{
    GtkWidget *frame_AWB=GTK_WIDGET(gtk_builder_get_object(Demo_builder, "frame_AWB"));

    if(g_tCapability.sIspCapacity.bMonoSensor){
        gtk_widget_set_sensitive ((GtkWidget *)frame_AWB, FALSE);
        return ;
    }

    GtkWidget *AWB_once_button;

    GtkWidget *radio_awb_mode_manual;
    GtkWidget *radio_awb_mode_auto;


    GtkWidget *hscale_saturation;
    GtkWidget *hscale_gain_r;
    GtkWidget *hscale_gain_g;
    GtkWidget *hscale_gain_b;



    AWB_once_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "AWB_once_button"));

    radio_awb_mode_manual = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_awb_mode_manual"));
    radio_awb_mode_auto = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_awb_mode_auto"));


    hscale_gain_r = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "hscale_gain_r"));
    hscale_gain_g = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "hscale_gain_g"));
    hscale_gain_b = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "hscale_gain_b"));
    hscale_saturation = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "hscale_saturation"));

    g_signal_connect ((gpointer) AWB_once_button, "pressed",G_CALLBACK (on_AWB_once_button_pressed),NULL);


    g_signal_connect(G_OBJECT(hscale_gain_r),"value_changed",G_CALLBACK(on_gain_r_value_changed),NULL);
    g_signal_connect(G_OBJECT(hscale_gain_g),"value_changed",G_CALLBACK(on_gain_g_value_changed),NULL);
    g_signal_connect(G_OBJECT(hscale_gain_b),"value_changed",G_CALLBACK(on_gain_b_value_changed),NULL);
    g_signal_connect(G_OBJECT(hscale_saturation),"value_changed",G_CALLBACK(on_saturation_value_changed),NULL);



    GLADE_HOOKUP_OBJECT (window, AWB_once_button, "AWB_once_button");

    GLADE_HOOKUP_OBJECT (window, radio_awb_mode_manual, "radio_awb_mode_manual");
    GLADE_HOOKUP_OBJECT (window, radio_awb_mode_auto, "radio_awb_mode_auto");


    GLADE_HOOKUP_OBJECT (window, hscale_gain_r, "hscale_gain_r");
    GLADE_HOOKUP_OBJECT (window, hscale_gain_g, "hscale_gain_g");
    GLADE_HOOKUP_OBJECT (window, hscale_gain_b, "hscale_gain_b");
    GLADE_HOOKUP_OBJECT (window, hscale_saturation, "hscale_saturation");

//设置选中内容

    gtk_toggle_button_set_active ((GtkToggleButton *)radio_awb_mode_manual,TRUE);
    gtk_widget_set_sensitive ((GtkWidget *)radio_awb_mode_auto, FALSE);

    return ;

}

void create_gtkdemo_image_processing (GtkWidget *window)
{
    GtkWidget *check_isp_h;
    GtkWidget *check_isp_v;
    GtkWidget *hscale_isp_acutance;

    check_isp_h = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "check_isp_h"));
    check_isp_v = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "check_isp_v"));
    hscale_isp_acutance = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "hscale_isp_acutance"));

    g_signal_connect(G_OBJECT(check_isp_h),"clicked",G_CALLBACK(on_isp_h_check),NULL);
    g_signal_connect(G_OBJECT(check_isp_v),"clicked",G_CALLBACK(on_isp_v_check),NULL);
    g_signal_connect(G_OBJECT(hscale_isp_acutance),"value_changed",G_CALLBACK(on_isp_acutance_value_changed),NULL);


    GLADE_HOOKUP_OBJECT (window, check_isp_h, "check_isp_h");
    GLADE_HOOKUP_OBJECT (window, check_isp_v, "check_isp_v");
    GLADE_HOOKUP_OBJECT (window, hscale_isp_acutance, "hscale_isp_acutance");

    return ;

}

void create_gtkdemo_parameter (GtkWidget *window)
{
    GtkWidget *para_save_button;
    GtkWidget *para_acquiesce_button;
    GtkWidget *para_load_button;

    GtkWidget *radio_para_A;
    GtkWidget *radio_para_B;
    GtkWidget *radio_para_C;
    GtkWidget *radio_para_D;

    para_save_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "para_save_button"));
    para_acquiesce_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "para_acquiesce_button"));
    para_load_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "para_load_button"));


    radio_para_A = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_para_A"));
    radio_para_B = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_para_B"));
    radio_para_C = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_para_C"));
    radio_para_D = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "radio_para_D"));

    g_signal_connect ((gpointer) para_save_button, "pressed",G_CALLBACK (on_para_save_button_pressed),NULL);
    g_signal_connect ((gpointer) para_acquiesce_button, "pressed",G_CALLBACK (on_para_acquiesce_button_pressed),NULL);
    g_signal_connect ((gpointer) para_load_button, "pressed",G_CALLBACK (on_para_load_button_pressed),NULL);

    g_signal_connect(G_OBJECT(radio_para_A),"released",G_CALLBACK(on_radio_para_ABCD),(gpointer)0);
    g_signal_connect(G_OBJECT(radio_para_B),"released",G_CALLBACK(on_radio_para_ABCD),(gpointer)1);
    g_signal_connect(G_OBJECT(radio_para_C),"released",G_CALLBACK(on_radio_para_ABCD),(gpointer)2);
    g_signal_connect(G_OBJECT(radio_para_D),"released",G_CALLBACK(on_radio_para_ABCD),(gpointer)3);

    GLADE_HOOKUP_OBJECT (window, para_save_button, "para_save_button");
    GLADE_HOOKUP_OBJECT (window, para_acquiesce_button, "para_acquiesce_button");
    GLADE_HOOKUP_OBJECT (window, para_load_button, "para_load_button");

    GLADE_HOOKUP_OBJECT (window, radio_para_A, "radio_para_A");
    GLADE_HOOKUP_OBJECT (window, radio_para_B, "radio_para_B");
    GLADE_HOOKUP_OBJECT (window, radio_para_C, "radio_para_C");
    GLADE_HOOKUP_OBJECT (window, radio_para_D, "radio_para_D");

//设置选中内容  设置A组参数
    gtk_toggle_button_set_active ((GtkToggleButton *)radio_para_A,TRUE);


    return ;

}

void create_gtkdemo_mapp (GtkWidget *window)
{

    GtkWidget *hscale_mapp_contrast;
    GtkWidget *hscale_mapp_gamma;

    hscale_mapp_contrast = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "hscale_mapp_contrast"));
    hscale_mapp_gamma = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "hscale_mapp_gamma"));

    g_signal_connect(G_OBJECT(hscale_mapp_contrast),"value_changed",G_CALLBACK(on_mapp_contrast_value_changed),NULL);
    g_signal_connect(G_OBJECT(hscale_mapp_gamma),"value_changed",G_CALLBACK(on_mapp_gamma_value_changed),NULL);

    GLADE_HOOKUP_OBJECT (window, hscale_mapp_contrast, "hscale_mapp_contrast");
    GLADE_HOOKUP_OBJECT (window, hscale_mapp_gamma, "hscale_mapp_gamma");

    return ;
}


GtkWidget* create_gtkdemo_statusbar (GtkWidget *window)
{

    GtkWidget *statusbar;

    statusbar = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "statusbar"));

    GLADE_HOOKUP_OBJECT (window, statusbar, "statusbar");

    return statusbar;
}



GtkWidget* create_gtkdemo_display_drawingarea (GtkWidget *window)
{
    GtkWidget * drawingarea;

    GtkWidget *Display_box;
    GtkWidget *scrolledwindow;
    GtkWidget *viewport;

    Display_box = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "Display_box"));


    scrolledwindow = gtk_scrolled_window_new (NULL, NULL);
    gtk_widget_show (scrolledwindow);
    gtk_box_pack_start (GTK_BOX (Display_box), scrolledwindow, TRUE, TRUE, 0);

    viewport = gtk_viewport_new (NULL, NULL);
    gtk_widget_show (viewport);
    gtk_container_add (GTK_CONTAINER (scrolledwindow), viewport);
    gtk_container_set_border_width (GTK_CONTAINER (viewport), 2);


    drawingarea = gtk_drawing_area_new ();
    gtk_widget_show (drawingarea);
    gtk_container_add (GTK_CONTAINER (viewport), drawingarea);
    gtk_widget_set_size_request (drawingarea,DISPLAY_WIDTH,DISPLAY_HEIGHT);


	gtk_widget_show(drawingarea);

	g_signal_connect( G_OBJECT( drawingarea ), "expose_event",G_CALLBACK( preview_display ), NULL );


    GLADE_HOOKUP_OBJECT (window, drawingarea, "display_drawingarea");


//显示画板大小
    g_W_H_INFO.display_width=DISPLAY_WIDTH;
    g_W_H_INFO.display_height=DISPLAY_HEIGHT;

    return drawingarea;
}


GtkWidget* create_gtkdemo_window (int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    GtkWidget *window;


    window = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "main_window"));

	g_signal_connect(G_OBJECT(window),"delete_event",G_CALLBACK(on_gtkdemo_window_delete_event),NULL);

    /* Store pointers to all widgets, for use by lookup_widget(). */
    GLADE_HOOKUP_OBJECT_NO_REF (window, window, "main_window");


    display_drawingarea=create_gtkdemo_display_drawingarea(window);

    create_gtkdemo_awb(window);



    create_gtkdemo_exposure(window,hCamera,pCameraInfo);

    create_gtkdemo_snapshot(window);

    create_gtkdemo_resolution(window,hCamera,pCameraInfo);

    create_gtkdemo_image_processing(window);

    create_gtkdemo_parameter(window);

    create_gtkdemo_trigger (window);

    create_gtkdemo_mapp (window);

    create_gtkdemo_speed (window,hCamera,pCameraInfo);

    create_gtkdemo_statusbar (window);

    g_timeout_add(1000,fps_statistics,NULL);/*创建定时器*/


    Demo_display_window=(GtkWidget *)display_drawingarea;

    return window;
}

gint fps_statistics(gpointer data)
{
    char buffer[256];

    tSdkFrameStatistic stat = { 0 };
    CameraGetFrameStatistic(g_hCamera, &stat);

    sprintf(buffer,"Capture = %d  Display = %d Lost = %d", stat.iCapture - g_read_fps, g_disply_fps, stat.iLost);

    UINT nResend;
    if (CameraGetStatisticResend(g_hCamera, &nResend) == 0)
    {
        char buf[64];
        sprintf(buf, " Resend = %u", nResend);
        strcat(buffer, buf);
    }

    gtk_statusbar_push (GTK_STATUSBAR (lookup_widget(main_window,"statusbar")), 1, buffer);
    g_read_fps=stat.iCapture;
    g_disply_fps=0;
    return TRUE;
}


GtkWidget* create_snap_path_window (void)
{
    GtkWidget *window;
    GtkWidget *snap_path_confirm_button;
    GtkWidget *snap_path_cancel_button;

    window = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_path_window"));

    if(g_snap_path_window_state==0)
    {
        snap_path_confirm_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_path_confirm_button"));
        snap_path_cancel_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "snap_path_cancel_button"));

        g_signal_connect(G_OBJECT(window),"delete_event",G_CALLBACK(on_snap_path_window_delete_event),window);

        g_signal_connect ((gpointer) snap_path_confirm_button, "pressed",G_CALLBACK (on_snap_path_confirm_button_pressed),window);
        g_signal_connect ((gpointer) snap_path_cancel_button, "pressed",G_CALLBACK (on_snap_path_cancel_button_pressed),NULL);
        g_snap_path_window_state=1;
    }
    return window;
}

GtkWidget* create_file_load_window (void)
{
    GtkWidget *window;
    GtkWidget *file_load_confirm_button;
    GtkWidget *file_load_cancel_button;

    window = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "file_loading_window"));

    if(g_file_load_window_state==0)
    {
        file_load_confirm_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "file_load_confirm_button"));
        file_load_cancel_button = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "file_load_cancel_button"));

        g_signal_connect(G_OBJECT(window),"delete_event",G_CALLBACK(on_file_load_window_delete_event),window);

        g_signal_connect ((gpointer) file_load_confirm_button, "pressed",G_CALLBACK (on_file_load_confirm_button_pressed),window);
        g_signal_connect ((gpointer) file_load_cancel_button, "pressed",G_CALLBACK (on_file_load_cancel_button_pressed),NULL);
        g_file_load_window_state=1;
    }
    return window;
}

