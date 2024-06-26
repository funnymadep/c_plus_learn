gst-launch-1.0 rtspsrc location=rtsp://admin:sd123456@192.168.1.89:554/h264/ch1/main/av_stream latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
