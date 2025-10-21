#include <qtextstream.h>
#include "x11recorder.h"
#include <QDebug>
#include <QDateTime>

X11Recorder::~X11Recorder()
{
    stop();
#ifdef Q_OS_UNIX
    if (dpy) XCloseDisplay(dpy);
#endif
}

#ifdef Q_OS_UNIX
X11Recorder::X11Recorder(Window winId, int fps, QObject *parent)
    : QObject(parent), x11Win(winId), frameRate(fps)
{
    dpy = XOpenDisplay(nullptr);
    if (!dpy) qFatal("Cannot open X display");

    gst_init(nullptr, nullptr);
    connect(&timer, &QTimer::timeout, this, &X11Recorder::captureFrame);
}
#endif

void X11Recorder::start()
{
    QString outFile = QString("record_%1.mp4")
                      .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    QString pipelineStr =
        "appsrc name=mysrc is-live=true format=time ! "
        "video/x-raw,format=BGRA,framerate=30/1 ! videoconvert ! "
        "x264enc bitrate=4000 speed-preset=veryfast tune=zerolatency ! "
        "mp4mux ! filesink location=" + outFile;

    GError *error = nullptr;
    pipeline = gst_parse_launch(pipelineStr.toUtf8().constData(), &error);
    if (!pipeline) qFatal("Failed to create pipeline: %s", error->message);

    appsrc = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline), "mysrc"));
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    timer.start(1000 / 30);
    qDebug() << "Recording started to" << outFile;
}

void X11Recorder::stop()
{
    timer.stop();
    if (pipeline) {
        gst_element_send_event(pipeline, gst_event_new_eos());
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
        appsrc = nullptr;
        qDebug() << "Recording stopped";
    }
}

#ifdef Q_OS_UNIX
void X11Recorder::captureFrame()
{
    if (!dpy || !appsrc) return;

    XWindowAttributes attrs;
    XGetWindowAttributes(dpy, x11Win, &attrs);
    if (attrs.width == 0 || attrs.height == 0) return;

    XImage *img = XGetImage(dpy, x11Win, 0, 0, attrs.width, attrs.height, AllPlanes, ZPixmap);
    if (!img) return;

    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, img->bytes_per_line * attrs.height, nullptr);
    gst_buffer_fill(buffer, 0, img->data, img->bytes_per_line * attrs.height);

    GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(timer.interval() * GST_MSECOND, 1, 1);
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1000 / 30, GST_MSECOND, 1);

    gst_app_src_push_buffer(appsrc, buffer);
    XDestroyImage(img);
}
#endif
