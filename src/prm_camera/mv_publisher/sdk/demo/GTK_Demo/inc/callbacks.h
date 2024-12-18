#ifndef CALLBACKS_H_INCLUDED
#define CALLBACKS_H_INCLUDED

gboolean preview_display( GtkWidget *widget, GdkEventExpose *event,gpointer data );

void on_entry_num_insert_text (GtkWidget* entry, gchar* new_text, int new_text_length, int* position);

//分辨率处理
gboolean on_res_combobox_changed      (GtkWidget *widget,gpointer         user_data);

//触发模块
void on_trigger_once_button_pressed         (GtkButton       *button, gpointer         user_data);
void on_radio_trigger                         (GtkWidget       *radio,int         user_data);
void on_radio_flashlight                      (GtkWidget       *radio,int         user_data);
void on_radio_flashlight_polarity             (GtkWidget       *radio,int         user_data);

//白平衡
void on_AWB_once_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_gain_r_value_changed (GtkScale *scale, gpointer data);
void on_gain_g_value_changed (GtkScale *scale, gpointer data);
void on_gain_b_value_changed (GtkScale *scale, gpointer data);
void on_saturation_value_changed (GtkScale *scale, gpointer data);

// 曝光
void on_exposure_window_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_exposure_gain_value_changed (GtkScale *scale, gpointer data);
void on_exposure_time_value_changed (GtkScale *scale, gpointer data);
void on_AE_target_value_changed (GtkScale *scale, gpointer data);
void on_radio_5060hz                    (GtkWidget       *radio,int         user_data);
void on_radio_exposure_mode            (GtkWidget       *radio,int         user_data);
void on_flick_check   (GtkButton *button,gpointer data);

//速率切换
void on_radio_speed_clicked       (GtkWidget       *radio,int         user_data);

//抓拍
void on_radio_picture_clicked          (GtkWidget       *radio,int          user_data);
void on_radio_snap_catch_clicked          (GtkWidget       *radio,int          user_data);
void on_snap_content_check   (GtkButton *button,gpointer data);
void on_snap_time_check   (GtkButton *button,gpointer data);
void on_snap_catch_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_snap_res_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_snap_path_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_snap_path_confirm_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_snap_path_cancel_button_pressed         (GtkButton       *button,gpointer         user_data);
gboolean on_snap_period_entry_call (GtkWidget *widget,GdkEventKey *event,gpointer user_data);
void on_snap_path_window_delete_event(GtkWidget *widget, gpointer *data);
//gboolean on_AWB_set_window_delete_event       (GtkWidget       *widget,
//                                                   GdkEvent        *event,
//                                                   gpointer         user_data);

//isp图形处理
void on_isp_acutance_value_changed (GtkScale *scale, gpointer data);
void  on_isp_h_check   (GtkButton *button,gpointer data);
void  on_isp_v_check   (GtkButton *button,gpointer data);

//查寻映射表
void on_mapp_contrast_value_changed (GtkScale *scale, gpointer data);
void on_mapp_gamma_value_changed (GtkScale *scale, gpointer data);
// 参数组
void on_para_save_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_para_acquiesce_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_para_load_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_radio_para_ABCD                    (GtkWidget       *radio,int         user_data);


//文件加载
void on_file_load_confirm_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_file_load_cancel_button_pressed         (GtkButton       *button,gpointer         user_data);
void on_file_load_window_delete_event           (GtkWidget *widget, gpointer *data);



gboolean on_gtkdemo_window_delete_event       (GtkWidget       *widget,
                                                        GdkEvent        *event,
                                                        gpointer         user_data);


#endif // CALLBACKS_H_INCLUDED
