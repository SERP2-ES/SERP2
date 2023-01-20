#include "graphical_interface.h"

void gtk_dialog_destroy(GtkDialog *dialog, gint response_id, gpointer user_data);
void cb_gtk_resize_last_image(GtkWindow *window, GdkEvent *event, gpointer data);
void cb_gtk_resize_dialog_image(GtkWindow *window, GdkEvent *event, gpointer data);



// Set the style provider for the widgets
static void apply_css_provider(GtkWidget *widget, GtkCssProvider *cssstyleProvider)
{
    gtk_style_context_add_provider(gtk_widget_get_style_context(widget), GTK_STYLE_PROVIDER(cssstyleProvider), GTK_STYLE_PROVIDER_PRIORITY_USER);
    // For container widgets, apply to every child widget on the container
    if (GTK_IS_CONTAINER(widget))
    {
        gtk_container_forall(GTK_CONTAINER(widget), (GtkCallback)apply_css_provider, cssstyleProvider);
    }
}

std::string stampToString(const ros::Time& stamp, const std::string format="%H:%M:%S")
{
    const int output_size = 100;
    char output[output_size];
    std::time_t raw_time = static_cast<time_t>(stamp.sec);
    struct tm* timeinfo = localtime(&raw_time);
    std::strftime(output, output_size, format.c_str(), timeinfo);
    std::stringstream ss;
    ss << std::setw(9) << std::setfill('0') << stamp.nsec;
    const size_t fractional_second_digits = 3;
    return std::string(output) + "." + ss.str().substr(0, fractional_second_digits);
}

void insert_text_to_log(GtkTextView* text_view, GtkTextBuffer* text_buffer, GtkTextIter* text_iter, const char* text, bool erro=false) {
    //get bounds of buffer
    //gtk_text_buffer_get_bounds(text_buffer, log_text_iter_start, log_text_iter_end);
    //delete buffer text
    //gtk_text_buffer_delete(text_buffer,log_text_iter_start, log_text_iter_end);
    
    gtk_text_buffer_get_end_iter(text_buffer, text_iter);
    std::string str_text(text);
    std::string str_actual_time;
    std::string str_text_to_insert;
    std::string color;
    str_actual_time = stampToString(ros::Time::now());
    if(!erro) color = "black";
    else color = "red";
    str_text_to_insert = "<span color=\"" + color + "\">> [" + str_actual_time + "] - " + str_text + "</span>\n";
    //str_text_to_insert = str_text;
    //gtk_text_buffer_set_text (text_buffer, str_text_to_insert.c_str(), str_text_to_insert.length());
    gtk_text_buffer_insert_markup(text_buffer, text_iter, str_text_to_insert.c_str(), -1);
    //gtk_text_view_scroll_to_mark(text_view, gtk_text_buffer_get_insert(text_buffer), 0,true,1,1);
    gtk_text_view_scroll_to_mark(text_view, gtk_text_buffer_create_mark(text_buffer, "FUNCIONA", text_iter, false), 0,true,1,1);
}

void enable_interface_button(GtkWidget *button, bool is_green=true) {
    context = gtk_widget_get_style_context(button);
    if(is_green) {
        gtk_style_context_add_class(context, "button_green_active");
    }
    else {
        gtk_style_context_add_class(context, "button_red_active");
    }
}

void disable_interface_button(GtkWidget *button, bool is_green=true) {
    context = gtk_widget_get_style_context(button);
    if(is_green) {
        gtk_style_context_remove_class(context, "button_green_active");
    }
    else {
        gtk_style_context_remove_class(context, "button_red_active");
    }
}

void* cb_timer10s(gpointer data) {
    int8_t last_battery_level = 0;
    while(ros::ok()) {
        usleep(12000000); // 12s
        // Call service to get battery level
        std_srvs::Trigger srv;
        if(client_battery_level.call(srv) && srv.response.success) {
            robot.battery_level = (int8_t)std::stoi(srv.response.message);
            /*
            if(robot.battery_level < 0 || robot.battery_level > 100) {
                ROS_ERROR("Erro: Nível de bateria inválido!");
                ros::shutdown();
                std::exit(-1);
            }
            */
            if(robot.battery_level != last_battery_level) {
                // Update battery level in the interface
                std::string color;
                if(robot.battery_level < 20) {
                    color = "red";
                }
                else color = "black";
                std::string str_label = "<span color=\"" + color + "\">" + srv.response.message + "%</span>";
                gtk_label_set_markup(label_battery, str_label.c_str());
            }
        }
        else {
            gtk_label_set_markup(label_battery, "---");
        }
    }
    g_thread_exit(NULL);
    return NULL;
}

void* cb_ros_spin(gpointer data) {
    ros::Duration(0.03).sleep();
    ros::spin();
    return NULL;
}

gboolean setImage(gpointer data) {
    GdkPixbuf* buf = (GdkPixbuf*)data;
    gtk_image_set_from_pixbuf(camera_image_frame, buf);
    g_object_unref(buf);
    return false;
}



gboolean showDialogLastDetectedSheet(gpointer data) {
    GtkWidget *dialog, *content_area;
    GtkDialogFlags flags;
    // Create widgets
    flags = GTK_DIALOG_MODAL;
    dialog = gtk_dialog_new_with_buttons("Folha Detetada", (GtkWindow*)window, flags, "Recusar deteção", GTK_RESPONSE_REJECT, "Aceitar deteção", GTK_RESPONSE_ACCEPT, NULL);
    gtk_widget_add_events(GTK_WIDGET(dialog), GDK_CONFIGURE);
    gtk_window_set_resizable(GTK_WINDOW(dialog), true);

    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

    g_signal_connect_swapped(dialog, "response", G_CALLBACK (gtk_dialog_destroy), dialog);
    g_signal_connect(G_OBJECT(dialog), "configure-event", G_CALLBACK(cb_gtk_resize_dialog_image), NULL);

    GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_min_content_width(GTK_SCROLLED_WINDOW(scrolled_window), 557);
    gtk_scrolled_window_set_min_content_height(GTK_SCROLLED_WINDOW(scrolled_window), 421);
    gtk_container_add(GTK_CONTAINER(content_area), scrolled_window);
    gtk_container_add (GTK_CONTAINER(scrolled_window), dialog_image);
    gtk_widget_show_all(dialog);
    return false;
}

gboolean showLastDetectedSheet(gpointer data) {
    GtkWidget *new_window;
    // Create window
    new_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(new_window), "Última Folha Detetada");
    gtk_widget_add_events(GTK_WIDGET(new_window), GDK_CONFIGURE);
    gtk_window_set_resizable(GTK_WINDOW(new_window), true);
    g_signal_connect(new_window, "destroy", G_CALLBACK (gtk_widget_destroy), NULL);
    g_signal_connect(G_OBJECT(new_window), "configure-event", G_CALLBACK(cb_gtk_resize_last_image), NULL);
    GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_min_content_width(GTK_SCROLLED_WINDOW(scrolled_window), 800);
    gtk_scrolled_window_set_min_content_height(GTK_SCROLLED_WINDOW(scrolled_window), 700);
    gtk_container_add(GTK_CONTAINER(new_window), scrolled_window);
    gtk_container_add(GTK_CONTAINER(scrolled_window), dialog_image);
    gtk_window_set_position(GTK_WINDOW(new_window), GTK_WIN_POS_CENTER);
    gtk_widget_show_all(new_window);
    return false;
}

static void destroy_pixbuf(guchar *pixels, gpointer data)
{
    g_free(pixels);
}

/*void cb_camera_img(const sensor_msgs::ImageConstPtr &msg) {
    if(g_mutex_trylock(&mutex_camera)) {
        // Convert ROS message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
        current_frame = cv_ptr->image.clone();

        GdkPixbuf *pixbuf_rgb;
        pixbuf_rgb = gdk_pixbuf_new_from_data((guint8*) current_frame.data, GDK_COLORSPACE_RGB,FALSE, 8,
                                                     current_frame.cols, current_frame.rows, current_frame.step, 0, NULL);
        g_idle_add ((GSourceFunc) setImage, pixbuf_rgb);
    }
}*/

/*void cb_camera_detections(const sensor_msgs::ImageConstPtr &msg) {
    if(g_mutex_trylock(&mutex_camera_detections)) {
        // Convert ROS message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
        current_detected_sheet = cv_ptr->image;
        GdkPixbuf *last_pixbuf_rgb = gdk_pixbuf_new_from_data((guint8*) current_detected_sheet.data, GDK_COLORSPACE_RGB, FALSE, 8,
                                              current_detected_sheet.cols, current_detected_sheet.rows, current_detected_sheet.step, NULL, NULL);
        dialog_image = gtk_image_new();
        gtk_image_set_from_pixbuf(GTK_IMAGE(dialog_image), last_pixbuf_rgb);
        g_idle_add((GSourceFunc) showDialogLastDetectedSheet, NULL);
        g_object_unref(last_pixbuf_rgb);
        g_mutex_unlock(&mutex_camera_detections);
    }
}*/

void gtk_update_robot_state() {
    std::string state, log_message;
    log_message = "Estado do robô atualizado para: ";
    if(robot.state == Stopped) {
        state = "Parado";
        gtk_widget_hide(button_global_stop);
    }
    else {
        gtk_widget_show(button_global_stop);
        if(robot.state == ManualControl) {
            state = "Controlo Manual";
            gtk_widget_hide(button_global_stop);
        }
        else if(robot.state == ReadingProgrammingSheet) {
            state = "A ler folha de programação...";
        }
        else if(robot.state == Executing) {
            state = "A executar programação efetuada...";
        }
    }
    robot_state_num_chars = state.length();
    gtk_label_set_label(label_robot_state, state.c_str());
    if(is_startup) {
        insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Robô iniciado com sucesso!");
        is_startup = false;
    }
    else {
        insert_text_to_log(log_mensagens, log_buffer, log_text_iter, (log_message+state).c_str());
    }
    // Update top label left margin
    //gtk_widget_set_margin_start(widget_robot_state, (actual_window_width-IMAGE_WIDTH)/2 + (IMAGE_WIDTH-(130+robot_state_num_chars*7.3))/2);
    // Publish message to update state in the other nodes -> vamos precisar disto!
    //std_msgs::String msg;
    //if(robot.state == Stopped) msg.data = "Stopped";
    //else if(robot.state == ManualControl) msg.data = "ManualControl";
    //else if(robot.state == ReadingProgrammingSheet) msg.data = "ReadingProgrammingSheet";
    //else if(robot.state == Executing) msg.data = "Executing";
    //pub_robot_state.publish(msg);
}

void initializeGtkInterface() {
    robot.state = Stopped;
    robot.battery_level = -1;
    robot.error_logic = 1;
    robot.error_vision = 0;
    gtk_label_set_label(label_battery, "---");
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(window), "SERP2");
    gtk_widget_add_events(GTK_WIDGET(window), GDK_CONFIGURE);
}

void gtk_dialog_destroy(GtkDialog *dialog, gint response_id, gpointer user_data) {
    gtk_window_close(GTK_WINDOW(dialog));
    if(response_id == GTK_RESPONSE_ACCEPT) {
        robot.state = Executing;
        gtk_update_robot_state();
        last_detected_sheet = current_detected_sheet.clone();
    }
    else if(response_id == GTK_RESPONSE_REJECT) {
        insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível ativar o modo de leitura da folha...", true);
    }
    else if(response_id == GTK_RESPONSE_DELETE_EVENT && dialog_destroy_is_first_time) {
        robot.state = Stopped;
        gtk_update_robot_state();
    }
    if(dialog_destroy_is_first_time && response_id != GTK_RESPONSE_DELETE_EVENT)
        dialog_destroy_is_first_time = false;
    else
        dialog_destroy_is_first_time = true;
}

void gtk_main_quit() {
    ros::shutdown();
    system("rosnode kill -a");
    std::exit(0);
}

void cb_gtk_window_size(GtkWindow *window, GdkEvent *event, gpointer data) {
    actual_window_width = event->configure.width;
    //gtk_widget_set_margin_start(GTK_WIDGET(camera_image_frame), (actual_window_width-IMAGE_WIDTH)/4);
    //gtk_widget_set_margin_start(widget_robot_state, (actual_window_width-IMAGE_WIDTH)/2 + (IMAGE_WIDTH-(130+robot_state_num_chars*7.3))/2);
}

void cb_gtk_resize_last_image(GtkWindow *window, GdkEvent *event, gpointer data) {
    int width, height;
    width = event->configure.width;
    height = event->configure.height;

    if(width != last_detected_image_last_width || height != last_detected_image_last_height) {
        GdkPixbuf *last_pixbuf_rgb = gdk_pixbuf_new_from_data((guint8*) last_detected_sheet.data, GDK_COLORSPACE_RGB, FALSE, 8,
                                                              last_detected_sheet.cols, last_detected_sheet.rows, last_detected_sheet.step, NULL, NULL);
        GdkPixbuf *scaled_pixbuf = gdk_pixbuf_scale_simple(last_pixbuf_rgb, width, height, GDK_INTERP_BILINEAR);
        gtk_image_set_from_pixbuf(GTK_IMAGE(dialog_image), scaled_pixbuf);
        g_object_unref(scaled_pixbuf);
    }
    last_detected_image_last_width = width;
    last_detected_image_last_height = height;
}

void cb_gtk_resize_dialog_image(GtkWindow *window, GdkEvent *event, gpointer data) {
    int width, height;
    width = event->configure.width;
    height = event->configure.height;

    if(width != dialog_last_width || height != dialog_last_height) {
        GdkPixbuf *last_pixbuf_rgb = gdk_pixbuf_new_from_data((guint8*) current_detected_sheet.data, GDK_COLORSPACE_RGB, FALSE, 8,
                                                              current_detected_sheet.cols, current_detected_sheet.rows, current_detected_sheet.step, NULL, NULL);
        GdkPixbuf *scaled_pixbuf = gdk_pixbuf_scale_simple(last_pixbuf_rgb, width, height, GDK_INTERP_BILINEAR);
        gtk_image_set_from_pixbuf(GTK_IMAGE(dialog_image), scaled_pixbuf);
        g_object_unref(scaled_pixbuf);
    }
    last_detected_image_last_width = width;
    last_detected_image_last_height = height;
}

gboolean error_function(gpointer data) {
    
        if(robot.error_vision != 0){
            
            gtk_update_robot_state();
            if(robot.error_vision == -1){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO VISÃO: Not all corners detected", true);
            }       
            else if(robot.error_vision == -2){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO VISÃO: Não foi possível detetar ArUcos na folha de programação!", true);
            }
            else if(robot.error_vision == -3){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO VISÃO: Existe um ou mais ArUcos com a orientação na folha errada!", true);
            }

            robot.error_vision = 0;
        }
        if(robot.error_logic != 1){

            if(robot.error_logic == 0){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "SUCESSO: Folha de programação processada e compilada!", false);
            }       
            else if(robot.error_logic == -1){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Combinações lógicas incorretas (p.ex Constante - condição IF)", true);
            }
            else if(robot.error_logic == -2){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Falta/excesso de ligações!", true);
            }
            else if(robot.error_logic == -3){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Ligações erradas (p.ex input - input)", true);
            }
            else if(robot.error_logic == -4){
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Blocos inicias erradados (só podem ser constantes e sensores)", true);
            }
            gtk_update_robot_state();
            robot.error_logic = 1;
        }

    return false;
}

void* usb_livefeed(gpointer data)
{
    cv::VideoCapture cap("/dev/video1");
    if(!cap.isOpened()) {
        ROS_ERROR("Can't open usb camera!");
    }
    //ros::Rate FPS(20);
    while(ros::ok())
    {
        usleep(100000); 
        cap.read(frame);
        if (frame.empty())
            continue;
        // Update LiveFeed
        aux_frame1 = frame.clone();
        cv::cvtColor(aux_frame1, aux_frame1, cv::COLOR_BGR2RGB);
        GdkPixbuf *pixbuf_rgb;
        pixbuf_rgb = gdk_pixbuf_new_from_data(aux_frame1.data, GDK_COLORSPACE_RGB,FALSE, 8,
                                                    aux_frame1.cols, aux_frame1.rows, aux_frame1.step, 0, NULL);
        g_idle_add ((GSourceFunc) setImage, pixbuf_rgb);
        
        //aux_frame1.release(); 
        //FPS.sleep();

        
            //         insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Não foi possível detetar todos os ArUcos de Fronteira, enquadre melhor a folha!", true);
    //         robot.state = Stopped;
    //         gtk_update_robot_state();
    //         break;
    //     case -2:
    //     // code block
    //         insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Não foi possível detetar ArUcos na folha de programação!", true);
    //         robot.state = Stopped;
    //         gtk_update_robot_state();
    //         break;
    //     case -3:
    //     // code block
    //         insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Existe um ou mais ArUcos com a orientação na folha errada!", true);
    //         robot.state = Stopped;
    //         gtk_update_robot_state();
    //         break;
    //     case -4:
    //     // code block
    //         insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: Existe um ou mais ArUcos com a orientação na folha errada!", true);
    //         robot.state = Stopped;
    //         gtk_update_robot_state();
    //         break;
    //     default:
    //     // code block
    //         insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "ERRO LÓGICA: erro no COMPILAMENTO da folha de programação!", true);
    //         robot.state = Stopped;
    //         gtk_update_robot_state();
    // }
    }
    cap.release();
    return NULL;
}





//podemos receber 3 tipos de erro
// -1 -> falta de corners dos cantos
// -2 -> detetou cantos mas a folha tá em branco -> não há código para analisar
// -3 -> aRuco virado ao contrario
void errorVisionCallback(const std_msgs::Int8 &msg)
{
    // analisar qual o tipo de erro que foi detetado
        //mudo o estado
        //escrevo o tipo de erro na box de terminal

   
    robot.state = Stopped;
    robot.error_vision = msg.data;
    g_idle_add ((GSourceFunc) error_function, NULL);
}





void errorLogicCallback(const std_msgs::Int8 &msg) // TO DO POR FAZER
{
    // analisar qual o tipo de erro que foi detetado
        //mudo o estado
        //escrevo o tipo de erro na box de terminal
        robot.error_logic = msg.data;
        if(msg.data == 0){
            robot.state = Executing;
            g_idle_add ((GSourceFunc) error_function, NULL);
        }
        else{
            robot.state = Stopped;
            g_idle_add ((GSourceFunc) error_function, NULL);
        }
        
}

gboolean sensor_update(gpointer data) {

    if(left_sensor==0){
        gtk_widget_set_visible (sensor_left_3, 1);
        gtk_widget_set_visible (sensor_left_2,0);
        gtk_widget_set_visible (sensor_left_1, 0);
        gtk_widget_set_visible (sensor_left_0,0);

    }
    else if( (left_sensor>0) && (left_sensor<=30) ){
        gtk_widget_set_visible (sensor_left_3, 0);
        gtk_widget_set_visible (sensor_left_2,1);
        gtk_widget_set_visible (sensor_left_1, 0);
        gtk_widget_set_visible (sensor_left_0,0);

    }
    else if((left_sensor>30) && (left_sensor<95)){

        gtk_widget_set_visible (sensor_left_3, 0);
        gtk_widget_set_visible (sensor_left_2,0);
        gtk_widget_set_visible (sensor_left_1, 1);
        gtk_widget_set_visible (sensor_left_0,0);
    }
    else{
        gtk_widget_set_visible (sensor_left_3, 0);
        gtk_widget_set_visible (sensor_left_2,0);
        gtk_widget_set_visible (sensor_left_1, 0);
        gtk_widget_set_visible (sensor_left_0,1);
    }


    if(right_sensor==0){
        gtk_widget_set_visible (sensor_right_3, 1);
        gtk_widget_set_visible (sensor_right_2,0);
        gtk_widget_set_visible (sensor_right_1, 0);
        gtk_widget_set_visible (sensor_right_0,0);

    }
    else if( (right_sensor>0) && (right_sensor<=30) ){
        gtk_widget_set_visible (sensor_right_3, 0);
        gtk_widget_set_visible (sensor_right_2,1);
        gtk_widget_set_visible (sensor_right_1, 0);
        gtk_widget_set_visible (sensor_right_0,0);

    }
    else if((right_sensor>30) && (right_sensor<95)){

        gtk_widget_set_visible (sensor_right_3, 0);
        gtk_widget_set_visible (sensor_right_2,0);
        gtk_widget_set_visible (sensor_right_1, 1);
        gtk_widget_set_visible (sensor_right_0,0);
    }
    else{
        gtk_widget_set_visible (sensor_right_3, 0);
        gtk_widget_set_visible (sensor_right_2,0);
        gtk_widget_set_visible (sensor_right_1, 0);
        gtk_widget_set_visible (sensor_right_0,1);
    }


    if(back_sensor==0){
        gtk_widget_set_visible (sensor_back_3, 1);
        gtk_widget_set_visible (sensor_back_2,0);
        gtk_widget_set_visible (sensor_back_1, 0);
        gtk_widget_set_visible (sensor_back_0,0);

    }
    else if( (back_sensor>0) && (back_sensor<=30) ){
        gtk_widget_set_visible (sensor_back_3, 0);
        gtk_widget_set_visible (sensor_back_2,1);
        gtk_widget_set_visible (sensor_back_1, 0);
        gtk_widget_set_visible (sensor_back_0,0);

    }
    else if((back_sensor>30) && (back_sensor<95)){

        gtk_widget_set_visible (sensor_back_3, 0);
        gtk_widget_set_visible (sensor_back_2,0);
        gtk_widget_set_visible (sensor_back_1, 1);
        gtk_widget_set_visible (sensor_back_0,0);
    }
    else{
        gtk_widget_set_visible (sensor_back_3, 0);
        gtk_widget_set_visible (sensor_back_2,0);
        gtk_widget_set_visible (sensor_back_1, 0);
        gtk_widget_set_visible (sensor_back_0,1);
    }


    if(front_sensor==0){
        gtk_widget_set_visible (sensor_front_3, 1);
        gtk_widget_set_visible (sensor_front_2,0);
        gtk_widget_set_visible (sensor_front_1, 0);
        gtk_widget_set_visible (sensor_front_0,0);

    }
    else if( (front_sensor>0) && (front_sensor<=30) ){
        gtk_widget_set_visible (sensor_front_3, 0);
        gtk_widget_set_visible (sensor_front_2,1);
        gtk_widget_set_visible (sensor_front_1, 0);
        gtk_widget_set_visible (sensor_front_0,0);

    }
    else if((front_sensor>30) && (front_sensor<95)){

        gtk_widget_set_visible (sensor_front_3, 0);
        gtk_widget_set_visible (sensor_front_2,0);
        gtk_widget_set_visible (sensor_front_1, 1);
        gtk_widget_set_visible (sensor_front_0,0);
    }
    else{
        gtk_widget_set_visible (sensor_front_3, 0);
        gtk_widget_set_visible (sensor_front_2,0);
        gtk_widget_set_visible (sensor_front_1, 0);
        gtk_widget_set_visible (sensor_front_0,1);
    }
       

    return false;
}


void sensorCallback(const serp::ObjectDetection::ConstPtr &msg)
{
    left_sensor=msg->left;
    right_sensor=msg->right;
    back_sensor=msg->back;
    front_sensor=msg->front;
    g_idle_add ((GSourceFunc) sensor_update, NULL);

}

int main(int argc, char *argv[])
{
    GtkBuilder *builder;
    GtkLabel *label_motor_esquerda;
    GtkLabel *label_motor_direita;
    GtkCssProvider *css = gtk_css_provider_new();

    //button = gtk_button_new ();

    //g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "cool button");

    // Get path of the base project using a environment variable
    char css_file_path[200];
    char glade_file_path[200];
    char *project_path = getenv("SERP_PROJECT_PATH");
    char css_file_name[38] = "include/graphical_interface_style.css";
    char glade_file_name[34] = "include/graphical_interface.glade";
    sprintf(css_file_path, "%s%s", project_path, css_file_name);
    sprintf(glade_file_path, "%s%s", project_path, glade_file_name);

    // Load CSS file
    gtk_css_provider_load_from_path(css, css_file_path, NULL);

    gtk_init(&argc, &argv);

    builder = gtk_builder_new();

    // Load Glade file
    gtk_builder_add_from_file(builder, glade_file_path, NULL);

    // Load widgets from glade file
    window = GTK_WIDGET(gtk_builder_get_object(builder, "window"));
    widget_robot_state = GTK_WIDGET(gtk_builder_get_object(builder, "widget_robot_state"));
    log_mensagens = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "log_text_view"));
    log_buffer = GTK_TEXT_BUFFER(gtk_builder_get_object(builder, "log_buffer"));
    button_manual_go = GTK_WIDGET(gtk_builder_get_object(builder, "button_manual_go"));
    button_manual_stop = GTK_WIDGET(gtk_builder_get_object(builder, "button_manual_stop"));
    range_motor_left = GTK_RANGE(gtk_builder_get_object(builder, "range_motor_left"));
    range_motor_right = GTK_RANGE(gtk_builder_get_object(builder, "range_motor_right"));
    label_battery = GTK_LABEL(gtk_builder_get_object(builder, "battery_level"));
    button_global_stop = GTK_WIDGET(gtk_builder_get_object(builder, "button_global_stop"));
    label_robot_state = GTK_LABEL(gtk_builder_get_object(builder, "robot_state"));
    camera_image_frame = GTK_IMAGE(gtk_builder_get_object(builder, "camera_frame"));

    sensor_left_0 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_left_0"));
    sensor_left_1 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_left_1"));
    sensor_left_2 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_left_2"));
    sensor_left_3 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_left_3"));

    sensor_back_0 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_back_0"));
    sensor_back_1 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_back_1"));
    sensor_back_2 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_back_2"));
    sensor_back_3 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_back_3"));

    sensor_front_0 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_front_0"));
    sensor_front_1 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_front_1"));
    sensor_front_2 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_front_2"));
    sensor_front_3 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_front_3"));

    sensor_right_0 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_right_0"));
    sensor_right_1 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_right_1"));
    sensor_right_2 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_right_2"));
    sensor_right_3 = GTK_WIDGET(gtk_builder_get_object(builder, "sensor_right_3"));

    // Create text buffer iterator
    log_text_iter = new GtkTextIter();
    //log_text_iter_start = new GtkTextIter();
    //log_text_iter_end = new GtkTextIter();
    gtk_text_view_set_buffer(log_mensagens, log_buffer);

    // Apply the provided css in the window widget
    apply_css_provider(window, css);

    // Initialize variables
    initializeGtkInterface();

    // GTK connect signals
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(G_OBJECT(window), "configure-event", G_CALLBACK(cb_gtk_window_size), NULL);
    gtk_builder_connect_signals(builder, NULL);

    // Init ROS node
    ros::init(argc, argv, "serp_interface_node");
    ros::NodeHandle n_public;
    // Give some time for the other nodes start
    ros::Duration(2.0).sleep();
    // Create ROS Clients - averiguar se isto vale a pena
    //client_velocity_setpoint = n_public.serviceClient<serp::VelocitySetPoint>("velocity_setpoint");
    client_battery_level = n_public.serviceClient<std_srvs::Trigger>("srv_battery_level");
    
    //esquecer este servico
    //client_read_programming_sheet = n_public.serviceClient<std_srvs::Trigger>("srv_read_programming_sheet");

    // Create ROS Subscribers
    //it = new image_transport::ImageTransport(n_public);
    image_transport::ImageTransport it(n_public);
    captured_frame = it.advertise("/image", 1);

    //CRIAR SUBSCRICAO PARA MENSAGENS DE ERRO DO VISION_NODE
    //TO DO: DEFINIR TOPICO E CALLBACK FUNCTION - EM CASO DE ERRO VAI PARA ESTADO PARADO
    ros::Subscriber errorVision = n_public.subscribe("/error_vision", 1, errorVisionCallback);

    //CRIAR SUBSCRICAO PARA MENSAGENS DE ERRO DO LOGIC_NODE
    //TO DO: DEFINIR TOPICO E CALLBACK FUNCTION - EM CASO DE ERRO VAI PARA ESTADO PARADO; EM CASO DE SUCESSO VAI PARA ESTADO EXECUTING 
    ros::Subscriber errorLogic = n_public.subscribe("/error_logic", 1, errorLogicCallback);
    
    //CRIAR SUBSCRICAO PARA MENSAGENS DE SENSORES
    //TO DO: DEFINIR TOPICO E CALLBACK FUNCTION - MUDAR IMAGEM DO SENSOR
    ros::Subscriber sensors = n_public.subscribe("/sensors", 1, sensorCallback);
    
    // Subscribe camera image
    //image_transport::Publisher captured_frame = it->advertise("raw_frame", 1);
    // Subscribe detected programming sheets
    //image_transport::Subscriber sub_detected_sheets = it->subscribe("camera/sheet_detections", 1, cb_camera_detections);
    
    // Create ROS Publisher
    //pub_robot_state = n_public.advertise<std_msgs::String>("robot_state", 1);
    
    //CRIAR PUBLICACAO PARA MATRIZ DE VELOCIDADE
    //TO DO: CRIAR VARIAVEL NO .H E DEFINIR TOPICO
    vel_matrix = n_public.advertise<serp::Matrix>("/matrix", 1);

    gtk_update_robot_state();

    GThread *thread_timer;
    GThread *thread_ros_sub;
    thread_timer = g_thread_new("Timer", cb_timer10s, NULL);
    thread_ros_sub = g_thread_new("Ros Spin", cb_ros_spin, NULL);

    vision_flag=false;
    vision_error=0;

    // --------------------NOVO CODIGO-------------------------------------
    // criar thread para live feed da usb camera
    GThread *thread_usb_camera;
    thread_usb_camera = g_thread_new("USB Camera", usb_livefeed, NULL);
    // -------------------------------------------------------------------

    g_object_unref(css);
    g_object_unref(G_OBJECT(builder));
    gtk_widget_show(window);
    gtk_main();

    return 0;
}

extern "C"
{
    void on_button_go_manual_control_clicked(GtkButton *clicked_button) {
        if(robot.state == Stopped) {
            // Send service request
            // serp::VelocitySetPoint srv;
            // srv.request.state = true;
            // srv.request.vel_motor_left = robot.motor_left_requested_velocity;
            // srv.request.vel_motor_right = robot.motor_right_requested_velocity;
            serp::Matrix man_control;
            man_control.manual_mode = true;
            man_control.vel_motor_left = robot.motor_left_requested_velocity;
            man_control.vel_motor_right = robot.motor_right_requested_velocity;

            vel_matrix.publish(man_control);
            
            robot.state = ManualControl;
            gtk_update_robot_state();
            /*if(client_velocity_setpoint.call(srv) && srv.response.success) {
                // Change buttons color
                enable_interface_button(button_manual_go, true);
                disable_interface_button(button_manual_stop, false);
                robot.state = ManualControl;
                gtk_update_robot_state();
            }
            else {
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível ativar o modo de controlo manual...", true);
            }*/
        }
        else {
            insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Para ativar o modo de controlo manual o robô tem que estar parado!", true);
        }
    }

    void on_button_stop_manual_control_clicked(GtkButton *clicked_button) {
        if(robot.state == ManualControl) {
            serp::Matrix man_control;
            man_control.manual_mode = true;
            man_control.vel_motor_left = 0;
            man_control.vel_motor_right = 0;

            vel_matrix.publish(man_control);
            
            robot.state = Stopped;
            gtk_update_robot_state();
            // Send service request
            // serp::VelocitySetPoint srv;
            // srv.request.state = false;
            // srv.request.vel_motor_left = robot.motor_left_velocity;
            // srv.request.vel_motor_right = robot.motor_right_velocity;
            // if(client_velocity_setpoint.call(srv) && srv.response.success) {
            //     // Change button color to appear disabled
            //     disable_interface_button(button_manual_go, true);
            //     enable_interface_button(button_manual_stop, false);
            //     robot.state = Stopped;
            //     gtk_update_robot_state();
            //     // Reset Ranges
            //     gtk_range_set_value(range_motor_left, 0);
            //     gtk_range_set_value(range_motor_right, 0);
            // }
            // else {
            //     insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível desativar o modo de controlo manual...", true);
            // }
        }
        else {
            insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: O robô não se encontra no modo de controlo manual...", true);
        }
    }

    void on_range_left_motor_changed(GtkRange *range) {
        gdouble value;
        value = gtk_range_get_value(range);
        if (robot.state == ManualControl) {
            serp::Matrix man_control;
            man_control.manual_mode = true;
            man_control.vel_motor_left = (int8_t)value;
            man_control.vel_motor_right = robot.motor_right_requested_velocity;

            vel_matrix.publish(man_control);
            // Send service request
            // serp::VelocitySetPoint srv;
            // srv.request.state = true; // activate setpoint mode
            // srv.request.vel_motor_left = (int8_t) value;
            // srv.request.vel_motor_right = robot.motor_right_requested_velocity; // Only update motor left velocity
            // if (!(client_velocity_setpoint.call(srv) && srv.response.success)) {
            //     insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível atualizar manualmente a velocidade do robô.", true);
            //     insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Controlo manual desativado, por segurança!", true);
            //     robot.state = Stopped;
            //     enable_interface_button(button_manual_stop, false);
            //     disable_interface_button(button_manual_go, true);
            // }
            // else {
            //     robot.motor_left_requested_velocity = srv.request.vel_motor_left;
            // }
        }
        else {
            robot.motor_left_requested_velocity = (int8_t) value;
        }
    }



    void on_range_right_motor_changed(GtkRange *range) {
        gdouble value;
        value = gtk_range_get_value(range);
        if (robot.state == ManualControl) {
            serp::Matrix man_control;
            man_control.manual_mode = true;
            man_control.vel_motor_left = robot.motor_left_requested_velocity;
            man_control.vel_motor_right = (int8_t)value;

            vel_matrix.publish(man_control);
            // Send service request
            // serp::VelocitySetPoint srv;
            // srv.request.state = true; // activate setpoint mode
            // srv.request.vel_motor_left = robot.motor_left_requested_velocity; // Only update motor right velocity
            // srv.request.vel_motor_right = (int8_t) value;
            // if (!(client_velocity_setpoint.call(srv) && srv.response.success)) {
            //     insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível atualizar manualmente a velocidade do robô.", true);
            //     insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Controlo manual desativado, por segurança!", true);
            //     robot.state = Stopped;
            //     enable_interface_button(button_manual_stop, false);
            //     disable_interface_button(button_manual_go, true);
            // }
            // else {
            //     robot.motor_right_requested_velocity = srv.request.vel_motor_right;
            // }
        }
        else {
            robot.motor_right_requested_velocity = (int8_t) value;
        }
    }

    void on_button_read_sheet_clicked(GtkButton *button) {
        if(robot.state == Stopped) {
            //std_srvs::Trigger srv;
            //if(client_read_programming_sheet.call(srv) && srv.response.success) {
                
                //cv::VideoCapture cap("/dev/video1");
                //cap.open(0);
                //if(!cap.isOpened()) {
                //    ROS_ERROR("Can't open usb camera!");
//}
                
                //cap.read(frame);
                //if (!frame.empty())
                //{

                    
                    aux_frame2 = frame.clone();
                    //cv::cvtColor(aux_frame2, aux_frame2, cv::COLOR_BGR2RGB);
                    //GdkPixbuf *last_pixbuf_rgb = gdk_pixbuf_new_from_data(aux_frame.data, GDK_COLORSPACE_RGB, FALSE, 8,
                    //                                                    aux_frame.cols, aux_frame.rows, aux_frame.step, NULL, NULL);
                    //dialog_image = gtk_image_new();
                    //gtk_image_set_from_pixbuf(GTK_IMAGE(dialog_image), last_pixbuf_rgb);
                    //g_object_unref(last_pixbuf_rgb);
                    //g_idle_add ((GSourceFunc) showLastDetectedSheet, NULL);


                    // Convert OpenCV image to ROS data
                    cv_bridge::CvImage img_bridge;
                    sensor_msgs::Image img_msg;
                    std_msgs::Header header;
                    header.stamp = ros::Time::now();
                    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, aux_frame2);
                    img_bridge.toImageMsg(img_msg);

                    captured_frame.publish(img_msg);

                    robot.state = ReadingProgrammingSheet;
                    gtk_update_robot_state();
                //}
                //aux_frame2.release();
            //}
            //else {
            //    insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível ativar o modo de leitura da folha...", true);
            //}
        }
        else {
            insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Para ativar o modo de leitura da folha o robô tem que estar no modo \"Parado\"!...", true);
        }
    }
    //esta função poderá desaparecer - por averiguar
    void on_button_see_last_detected_sheet_clicked(GtkButton *button) {
        /*if(g_mutex_trylock(&mutex_camera_detections)) {
            if(last_detected_sheet.empty()) {
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Ainda não foi detetada qualquer folha de programação...", true);
            }
            else {
                GdkPixbuf *last_pixbuf_rgb = gdk_pixbuf_new_from_data((guint8*) last_detected_sheet.data, GDK_COLORSPACE_RGB, FALSE, 8,
                                                                      last_detected_sheet.cols, last_detected_sheet.rows, last_detected_sheet.step, NULL, NULL);
                dialog_image = gtk_image_new();
                gtk_image_set_from_pixbuf(GTK_IMAGE(dialog_image), last_pixbuf_rgb);
                g_object_unref(last_pixbuf_rgb);
                g_idle_add ((GSourceFunc) showLastDetectedSheet, NULL);
            }
            g_mutex_unlock(&mutex_camera_detections);
        }*/
        gtk_widget_set_visible (sensor_back_3, 1);
        gtk_widget_set_visible (sensor_back_2,0);
        gtk_widget_set_visible (sensor_back_1, 0);
        gtk_widget_set_visible (sensor_back_0,0);
    }

    void on_button_global_stop_clicked(GtkButton *button) {
        robot.state = Stopped;
        gtk_update_robot_state();
        // enviar matriz com velocidades a 0 !!!!!!
        serp::Matrix msg;
        msg.manual_mode = true;
        msg.vel_motor_left = 0;
        msg.vel_motor_right = 0;
        vel_matrix.publish(msg);
    }

    void callback (GtkWidget *widget, gpointer *data)
    {
        insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "TEste", true);
    }

}
