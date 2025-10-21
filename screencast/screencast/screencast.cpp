#include "screencast.h"
#include <QDBusInterface>
#include <QDBusPendingCallWatcher>
#include <QDBusPendingReply>
#include <QDBusUnixFileDescriptor>
#include <QProcess>
#include <QDebug>
#include <QDateTime>
#include <QUuid>

ScreenCast::ScreenCast(QObject *parent)
    : QObject(parent)
{
}

void ScreenCast::start()
{
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] start called";
    QDBusInterface portal("org.freedesktop.portal.Desktop",
                          "/org/freedesktop/portal/desktop",
                          "org.freedesktop.portal.ScreenCast",
                          QDBusConnection::sessionBus());

    // 生成唯一 token
    token = QUuid::createUuid().toString(QUuid::WithoutBraces);

    QVariantMap opts;
    opts["session_handle_token"] = token;

    QDBusPendingCall asyncCall = portal.asyncCall("CreateSession", opts);
    QDBusPendingCallWatcher *watcher = new QDBusPendingCallWatcher(asyncCall, this);
    connect(watcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onCreateSessionFinished);
}

void ScreenCast::onCreateSessionFinished(QDBusPendingCallWatcher *watcher)
{
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] onCreateSessionFinished called";
    QDBusPendingReply<QDBusObjectPath> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "CreateSession failed:" << reply.error().message();
        return;
    }

    sessionPath = reply.value().path();
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] Session created:" << sessionPath;

    QDBusInterface portal("org.freedesktop.portal.Desktop",
                          "/org/freedesktop/portal/desktop",
                          "org.freedesktop.portal.ScreenCast",
                          QDBusConnection::sessionBus());

    QVariantMap opts;
    opts["types"] = (uint)1;       // Monitor
    opts["multiple"] = false;
    opts["cursor_mode"] = (uint)2; // metadata

    QDBusPendingCall asyncCall = portal.asyncCall("SelectSources", QDBusObjectPath(sessionPath), opts);
    QDBusPendingCallWatcher *selectWatcher = new QDBusPendingCallWatcher(asyncCall, this);
    connect(selectWatcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onSelectSourcesFinished);
}

void ScreenCast::onSelectSourcesFinished(QDBusPendingCallWatcher *watcher)
{
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] onSelectSourcesFinished called";
    QDBusPendingReply<void> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "SelectSources failed:" << reply.error().message();
        return;
    }
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] SelectSources success";

    QDBusInterface portal("org.freedesktop.portal.Desktop",
                          "/org/freedesktop/portal/desktop",
                          "org.freedesktop.portal.ScreenCast",
                          QDBusConnection::sessionBus());

    QVariantMap opts;
    QDBusPendingCall asyncCall = portal.asyncCall("Start", QDBusObjectPath(sessionPath), QString("ConsoleApp"), opts);
    QDBusPendingCallWatcher *startWatcher = new QDBusPendingCallWatcher(asyncCall, this);
    connect(startWatcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onStartFinished);
}

void ScreenCast::onStartFinished(QDBusPendingCallWatcher *watcher)
{
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] onStartFinished called";
    QDBusPendingReply<QVariantMap> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "Start failed:" << reply.error().message();
        return;
    }

    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] Start success";

    QDBusInterface portal("org.freedesktop.portal.Desktop",
                          "/org/freedesktop/portal/desktop",
                          "org.freedesktop.portal.ScreenCast",
                          QDBusConnection::sessionBus());

    QDBusPendingCall asyncCall = portal.asyncCall("OpenPipeWireRemote", QDBusObjectPath(sessionPath), QVariantMap());
    QDBusPendingCallWatcher *pipewireWatcher = new QDBusPendingCallWatcher(asyncCall, this);
    connect(pipewireWatcher, &QDBusPendingCallWatcher::finished,
            this, &ScreenCast::onOpenPipeWireRemoteFinished);
}

void ScreenCast::onOpenPipeWireRemoteFinished(QDBusPendingCallWatcher *watcher)
{
    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] onOpenPipeWireRemoteFinished called";
    QDBusPendingReply<QDBusUnixFileDescriptor> reply = *watcher;
    watcher->deleteLater();

    if (reply.isError()) {
        qWarning() << "OpenPipeWireRemote failed:" << reply.error().message();
        return;
    }

    int fd = reply.value().fileDescriptor();
    if (fd < 0) {
        qWarning() << "Invalid PipeWire FD";
        return;
    }

    qDebug() << "[" << QDateTime::currentDateTime().toString() << "] Got PipeWire FD:" << fd;

    // 启动 GStreamer 保存 mp4
    QString outFile = QString("record_%1.mp4")
                      .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    QString pipeline = QString("pipewiresrc fd=%1 ! "
                               "video/x-raw,framerate=30/1 ! "
                               "videoconvert ! "
                               "x264enc bitrate=4000 speed-preset=veryfast tune=zerolatency ! "
                               "mp4mux ! filesink location=%2")
                               .arg(fd).arg(outFile);

    qDebug() << "Starting GStreamer:" << pipeline;
    QProcess::startDetached("gst-launch-1.0", QStringList() << "-e" << pipeline);

    qDebug() << "Recording to" << outFile;
}
