#include <stdlib.h>
#include <gtk/gtk.h>

#include "Demo.h"
#include "interface.h"
#include "callbacks.h"
#include "support.h"


//gtk demo使用
extern GtkBuilder       *Demo_builder ;             //glade进行界面设计
extern GtkWidget        *main_window ;              //主窗体
extern GtkWidget        *display_drawingarea[DISPLAY_MAX];      //画板


extern Width_Height     g_W_H_INFO[DISPLAY_MAX];                //显示画板到大小和图像大小


extern int              g_read_fps[DISPLAY_MAX];           //统计读取帧率
extern int              g_disply_fps[DISPLAY_MAX];         //统计显示帧率


extern int              g_display_max;                          //最大显示
#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
    gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)

typedef gboolean (*preview_display_Func)(GtkWidget *widget, GdkEventExpose *event,gpointer data);

GtkWidget* create_gtkdemo_display_drawingarea(GtkWidget *window,Width_Height *W_H_INFO,char *box,char *display_draw,preview_display_Func func)
{
    GtkWidget * drawingarea;

    GtkWidget *Display_box;
    GtkWidget *scrolledwindow;
    GtkWidget *viewport;

    Display_box = GTK_WIDGET(gtk_builder_get_object(Demo_builder,box));


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

    g_signal_connect( G_OBJECT( drawingarea ), "expose_event",G_CALLBACK( func ), NULL );

    GLADE_HOOKUP_OBJECT (window, drawingarea,display_draw);


//显示画板大小
    W_H_INFO->display_width=DISPLAY_WIDTH;
    W_H_INFO->display_height=DISPLAY_HEIGHT;

    return drawingarea;
}

GtkWidget* create_gtkdemo_statusbar (GtkWidget *window)
{

    GtkWidget *statusbar[DISPLAY_MAX];

    statusbar[0] = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "statusbar1"));

    GLADE_HOOKUP_OBJECT (window, statusbar[0], "statusbar1");

    statusbar[1] = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "statusbar2"));

    GLADE_HOOKUP_OBJECT (window, statusbar[1], "statusbar2");

    return statusbar[0];
}

GtkWidget* create_gtkdemo_window (void)
{
    GtkWidget                   *window;
    preview_display_Func        func[2];
    int                         i=0;
    char                        box_name[DISPLAY_MAX][32]={"Display_box1","Display_box2"};
    char                        drawingarea_name[DISPLAY_MAX][64]={"display_drawingarea1","display_drawingarea2"};


    func[0]=preview_display1;
    func[1]=preview_display2;

    window = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "main_window"));


	g_signal_connect(G_OBJECT(window),"delete_event",G_CALLBACK(on_gtkdemo_window_delete_event),NULL);

    /* Store pointers to all widgets, for use by lookup_widget(). */
    GLADE_HOOKUP_OBJECT_NO_REF (window, window, "main_window");

    for(i=0;i<g_display_max;i++){
        display_drawingarea[i]=create_gtkdemo_display_drawingarea(window,&g_W_H_INFO[i],box_name[i],drawingarea_name[i],func[i]);
   }

    create_gtkdemo_statusbar (window);

    g_timeout_add(1000,fps_statistics,NULL);/*创建定时器*/

    return window;
}

gint fps_statistics(gpointer data)
{
    char buffer[64];

    sprintf(buffer,"Capture = %d  Display = %d ",g_read_fps[0],g_disply_fps[0]);
    gtk_statusbar_push (GTK_STATUSBAR (lookup_widget(main_window,"statusbar1")), 1, buffer);

    sprintf(buffer,"Capture = %d  Display = %d ",g_read_fps[1],g_disply_fps[1]);
    gtk_statusbar_push (GTK_STATUSBAR (lookup_widget(main_window,"statusbar2")), 1, buffer);

    g_read_fps[0]=0;
    g_disply_fps[0]=0;

    g_read_fps[1]=0;
    g_disply_fps[1]=0;

    return TRUE;
}

