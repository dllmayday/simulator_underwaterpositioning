#pragma once
#include <QObject>
#include <QString>
#include <gst/gst.h>

class QDBusPendingCallWatcher;
class QDBusUnixFileDescriptor;

class ScreenCast : public QObject {
    Q_OBJECT
public:
    explicit ScreenCast(QObject *parent = nullptr);
    ~ScreenCast();

    void startFlow();   // 启动整个 portal -> gst 流程
    void stopRecording();

private slots:
    void onCreateSessionFinished(QDBusPendingCallWatcher *watcher);
    void onSelectSourcesFinished(QDBusPendingCallWatcher *watcher);
    void onStartFinished(QDBusPendingCallWatcher *watcher);
    void onOpenPipeWireRemoteFinished(QDBusPendingCallWatcher *watcher);

private:
    QString makeToken() const;
    void startGstWithFd(int fd);

    QString sessionPath;
    QString sessionToken;
    GstElement *gstPipeline = nullptr;
};
