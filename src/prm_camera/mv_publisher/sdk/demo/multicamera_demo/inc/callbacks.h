#ifndef CALLBACKS_H_INCLUDED
#define CALLBACKS_H_INCLUDED

gboolean preview_display1( GtkWidget *widget, GdkEventExpose *event,gpointer data );
gboolean preview_display2( GtkWidget *widget, GdkEventExpose *event,gpointer data );
gboolean on_gtkdemo_window_delete_event       (GtkWidget       *widget,
                                                        GdkEvent        *event,
                                                        gpointer         user_data);


#endif // CALLBACKS_H_INCLUDED
