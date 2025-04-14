#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>

// Structure to hold pipeline data
typedef struct {
    GstElement *pipeline;
    GstElement *appsink;
    bool running;
    std::mutex frame_mutex;
    cv::Mat current_frame;
} PipelineData;

// Callback for new sample from appsink
static GstFlowReturn new_sample_callback(GstElement *sink, gpointer data) {
    PipelineData *pipeline_data = (PipelineData *)data;
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    
    if (sample) {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            // Create OpenCV Mat from buffer data
            cv::Mat frame(480, 640, CV_8UC3, map.data);
            
            // Lock mutex and update current frame
            {
                std::lock_guard<std::mutex> lock(pipeline_data->frame_mutex);
                frame.copyTo(pipeline_data->current_frame);
            }
            
            gst_buffer_unmap(buffer, &map);
        }
        
        gst_sample_unref(sample);
    }
    
    return GST_FLOW_OK;
}

// Function to calculate focus measure (using Laplacian variance)
double calculate_focus_measure(const cv::Mat& frame) {
    cv::Mat gray, laplacian;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplacian, CV_64F);
    
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    
    return stddev.val[0] * stddev.val[0];  // Variance of Laplacian
}

// Auto-focus processing thread
void autofocus_thread(PipelineData *pipeline_data) {
    while (pipeline_data->running) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(pipeline_data->frame_mutex);
            if (!pipeline_data->current_frame.empty()) {
                frame = pipeline_data->current_frame.clone();
            }
        }
        
        if (!frame.empty()) {
            double focus_measure = calculate_focus_measure(frame);
            std::cout << "Focus measure: " << focus_measure << std::endl;
            
            // TODO: Implement focus adjustment logic based on focus measure
            // This would involve sending commands to adjust the camera focus
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char *argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);
    
    // Create pipeline data structure
    PipelineData data = {0};
    data.running = true;
    
    // Create the pipeline
    const char *pipeline_str = 
        "v4l2src device=/dev/video0 ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! tee name=t "
        "t. ! queue ! appsink name=autofocus_sink emit-signals=true max-buffers=1 drop=true "
        "t. ! queue ! videoconvert ! x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=127.0.0.1 port=5000";
    
    GError *error = NULL;
    data.pipeline = gst_parse_launch(pipeline_str, &error);
    if (error) {
        g_printerr("Failed to create pipeline: %s\n", error->message);
        g_error_free(error);
        return -1;
    }
    
    // Get appsink element
    data.appsink = gst_bin_get_by_name(GST_BIN(data.pipeline), "autofocus_sink");
    if (!data.appsink) {
        g_printerr("Failed to get appsink element\n");
        return -1;
    }
    
    // Connect to new-sample signal
    g_signal_connect(data.appsink, "new-sample", G_CALLBACK(new_sample_callback), &data);
    
    // Start auto-focus thread
    std::thread focus_thread(autofocus_thread, &data);
    
    // Start playing
    gst_element_set_state(data.pipeline, GST_STATE_PLAYING);
    
    // Create main loop
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    
    // Run the main loop
    g_main_loop_run(loop);
    
    // Clean up
    data.running = false;
    focus_thread.join();
    g_main_loop_unref(loop);
    gst_element_set_state(data.pipeline, GST_STATE_NULL);
    gst_object_unref(data.pipeline);
    
    return 0;
}