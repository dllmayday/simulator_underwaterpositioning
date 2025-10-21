#pragma once
#include <QObject>
#include <QTimer>
#include <gst/gst.h>
#include <gst/app/app.h>

#ifdef Q_OS_UNIX
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#endif

class X11Recorder : public QObject {
    Q_OBJECT
public:
#ifdef Q_OS_UNIX
    X11Recorder(Window winId, int fps = 30, QObject *parent = nullptr);
#endif
    ~X11Recorder();

    void start();
    void stop();

private slots:
    void captureFrame();

private:
#ifdef Q_OS_UNIX
    Window x11Win;
    Display *dpy = nullptr;
#endif
    int frameRate;
    QTimer timer;
    GstElement *pipeline = nullptr;
    GstAppSrc *appsrc = nullptr;
};
