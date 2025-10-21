#include "screencast.h"
#include <QDBusInterface>
#include <QDBusPendingCallWatcher>
#include <QDBusPendingReply>
#include <QDBusUnixFileDescriptor>
#include <QDBusObjectPath>
#include <QCoreApplication>
#include <QDebug>
#include <QDateTime>
#include <QUuid>

#include <gst/gst.h>

ScreenCast::ScreenCast(QObject *parent)
    : QObject(parent)
{
    // 初始化 GStreamer（在 main 中也会调用，但这里保险起见）
    gst_init(nullptr, nullptr);
}

ScreenCast::~ScreenCast()
{
    stopRecording();
}

QString ScreenCast::makeToken() const {
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

void ScreenCast::startFlow()
{
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] startFlow called";

    QDBusInterface portal("org.freedesktop.portal.Desktop",
                         "/org/freedesktop/portal/desktop",
                         "org.freedesktop.portal.ScreenCast",
                         QDBusConnection::sessionBus());
    if (!portal.isValid()) {
        qWarning() << "Portal interface invalid; is xdg-desktop-portal running?";
        return;
    }

    // 生成并保存 token，必须传给 CreateSession
    sessionToken = makeToken();
    QVariantMap opts;
    opts["session_handle_token"] = sessionToken;

    QDBusPendingReply<QDBusObjectPath> reply = portal.asyncCall("CreateSession", opts);
    QDBusPendingCallWatcher *watcher = new QDBusPendingCallWatcher(reply, this);
    connect(watcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onCreateSessionFinished);
}

void ScreenCast::onCreateSessionFinished(QDBusPendingCallWatcher *watcher)
{
    QDBusPendingReply<QDBusObjectPath> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "CreateSession failed:" << reply.error().message();
        return;
    }

    sessionPath = reply.value().path();
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] Session created:" << sessionPath;

    // SelectSources must be called with (o session_handle, a{sv} options)
    QDBusInterface portal("org.freedesktop.portal.Desktop",
                         "/org/freedesktop/portal/desktop",
                         "org.freedesktop.portal.ScreenCast",
                         QDBusConnection::sessionBus());

    QVariantMap opts;
    opts["types"] = (uint)1;       // 1 = monitor, 2 = window, 4 = virtual
    opts["multiple"] = false;
    opts["cursor_mode"] = (uint)2; // 0 hidden,1 embedded,2 metadata

    QDBusPendingReply<void> selReply = portal.asyncCall("SelectSources", QDBusObjectPath(sessionPath), opts);
    QDBusPendingCallWatcher *selWatcher = new QDBusPendingCallWatcher(selReply, this);
    connect(selWatcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onSelectSourcesFinished);

    qDebug() << "SelectSources called (portal will show chooser UI)";
}

void ScreenCast::onSelectSourcesFinished(QDBusPendingCallWatcher *watcher)
{
    QDBusPendingReply<void> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "SelectSources failed:" << reply.error().message();
        return;
    }

    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] SelectSources OK";

    QDBusInterface portal("org.freedesktop.portal.Desktop",
                         "/org/freedesktop/portal/desktop",
                         "org.freedesktop.portal.ScreenCast",
                         QDBusConnection::sessionBus());

    QVariantMap opts;
    opts["handle_token"] = makeToken(); // 可选 handle token
    QDBusPendingReply<QVariantMap> startReply = portal.asyncCall("Start", QDBusObjectPath(sessionPath), opts);
    QDBusPendingCallWatcher *startWatcher = new QDBusPendingCallWatcher(startReply, this);
    connect(startWatcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onStartFinished);

    qDebug() << "Start called (portal will ask for permission)";
}

void ScreenCast::onStartFinished(QDBusPendingCallWatcher *watcher)
{
    QDBusPendingReply<QVariantMap> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "Start failed:" << reply.error().message();
        return;
    }

    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] Start approved";

    // 请求 OpenPipeWireRemote 来获取 unix fd
    QDBusInterface sessionIface("org.freedesktop.portal.Desktop",
                               sessionPath,
                               "org.freedesktop.portal.ScreenCast.Session",
                               QDBusConnection::sessionBus());
    if (!sessionIface.isValid()) {
        qWarning() << "Session interface invalid:" << sessionPath;
        return;
    }

    QDBusPendingReply<QDBusUnixFileDescriptor> pwReply = sessionIface.asyncCall("OpenPipeWireRemote");
    QDBusPendingCallWatcher *pwWatcher = new QDBusPendingCallWatcher(pwReply, this);
    connect(pwWatcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onOpenPipeWireRemoteFinished);
}

void ScreenCast::onOpenPipeWireRemoteFinished(QDBusPendingCallWatcher *watcher)
{
    QDBusPendingReply<QDBusUnixFileDescriptor> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "OpenPipeWireRemote failed:" << reply.error().message();
        return;
    }

    QDBusUnixFileDescriptor ufd = reply.value();
    int fd = ufd.fileDescriptor();
    if (fd < 0) {
        qWarning() << "Invalid fd from portal";
        return;
    }

    // Duplicate fd to own it safely
    int dupfd = -1;
#ifdef _WIN32
    Q_UNUSED(dupfd);
#else
    dupfd = dup(fd);
#endif

    if (dupfd < 0) {
        qWarning() << "Failed to dup fd";
        return;
    }

    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] Got PipeWire FD:" << dupfd;

    // 启动 GStreamer 管线（驻留在当前进程中）
    startGstWithFd(dupfd);
}

void ScreenCast::startGstWithFd(int fd)
{
    if (gstPipeline) {
        qWarning() << "GStreamer pipeline already running";
        return;
    }

    QString outFile = QString("screencast_%1.mp4")
                      .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    // 一个通常能兼容 DMABUF 的 pipeline：gl upload -> color convert -> download -> encode
    QString pipelineStr = QStringLiteral(
        "pipewiresrc fd=%1 ! video/x-raw,framerate=30/1 ! "
        "glupload ! glcolorconvert ! gldownload ! videoconvert ! "
        "x264enc bitrate=4000 speed-preset=veryfast tune=zerolatency ! "
        "mp4mux ! filesink location=%2")
        .arg(fd)
        .arg(outFile);

    qDebug() << "Creating GStreamer pipeline:" << pipelineStr;

    GError *error = nullptr;
    gstPipeline = gst_parse_launch(pipelineStr.toUtf8().constData(), &error);
    if (!gstPipeline) {
        qWarning() << "gst_parse_launch failed:" << (error ? error->message : "unknown");
        if (error) g_error_free(error);
        return;
    }

    GstStateChangeReturn ret = gst_element_set_state(gstPipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        qWarning() << "Failed to set pipeline to PLAYING";
        gst_object_unref(gstPipeline);
        gstPipeline = nullptr;
        return;
    }

    qDebug() << "Recording started to" << outFile << "(fd =" << fd << ")";
}

void ScreenCast::stopRecording()
{
    if (gstPipeline) {
        gst_element_send_event(gstPipeline, gst_event_new_eos());
        gst_element_set_state(gstPipeline, GST_STATE_NULL);
        gst_object_unref(gstPipeline);
        gstPipeline = nullptr;
        qDebug() << "GStreamer pipeline stopped";
    }
}
