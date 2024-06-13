#include <iostream>
#include <thread>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

void gst_init_pipeline(GstElement **pipeline, GstElement **appsink, const std::string &rtsp_url) {
    *pipeline = gst_parse_launch(("rtspsrc location=" + rtsp_url + " latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink").c_str(), NULL);
    *appsink = gst_bin_get_by_name(GST_BIN(*pipeline), "sink");

    g_object_set(G_OBJECT(*appsink), "emit-signals", TRUE, "sync", FALSE, NULL);
}

int main() {
    /********************************************************
     * First create a RTSP server that would read RTP/H264
     * from localhost UDP port 5004 and stream as RTSP to
     * rtsp://127.0.0.1:8554/test
     ********************************************************/
    gst_init(NULL, NULL);
    GMainLoop *serverloop = g_main_loop_new(NULL, FALSE);
    GstRTSPServer *server = gst_rtsp_server_new();
    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, "( udpsrc port=5004 ! application/x-rtp,encoding-name=H264 ! rtph264depay ! h264parse ! rtph264pay name=pay0 )");
    gst_rtsp_mount_points_add_factory(mounts, "/test", factory);
    gst_rtsp_server_attach(server, NULL);
    std::thread serverloopthread(g_main_loop_run, serverloop);
    std::cout << "stream ready at rtsp://127.0.0.1:8554/test" << std::endl;

    /********************************************************
     * Now RTSP server is running in its own thread, let's 
     * create an application reading from an RTSP IP camera,
     * encoding into H264 and sending as RTP/H264 
     * to localhost UDP/5004
     ********************************************************/
    std::string rtsp_url = "rtsp://admin:sd123456@192.168.1.89:554/h264/ch1/main/av_stream"; // Replace with your IP camera RTSP URL

    GstElement *pipeline, *appsink;
    gst_init_pipeline(&pipeline, &appsink, rtsp_url);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo map;
    cv::Mat frame;
    gboolean res;
    
    int width = 1920;
    int height = 1080;
    int fps = 30;
    cv::VideoWriter rtph264_writer("appsrc ! queue ! videoconvert ! video/x-raw,format=I420 ! x264enc key-int-max=30 insert-vui=1 tune=zerolatency ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=5004", cv::CAP_GSTREAMER, 0, fps, cv::Size(width, height));
    
    if (!rtph264_writer.isOpened()) {
        std::cerr << "Failed to open writer. Exiting" << std::endl;
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        g_main_loop_quit(serverloop);
        serverloopthread.join();
        return -1;
    }

    while (true) {
        g_signal_emit_by_name(appsink, "pull-sample", &sample);
        if (sample) {
            buffer = gst_sample_get_buffer(sample);
            gst_buffer_map(buffer, &map, GST_MAP_READ);
            
            frame = cv::Mat(height, width, CV_8UC3, (char*)map.data);
            // cv::imwrite("frame.jpg", frame);
            rtph264_writer.write(frame);

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        } else {
            break;
        }
    }

    rtph264_writer.release();
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_quit(serverloop);
    serverloopthread.join();

    return 0;
}
