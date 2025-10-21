#pragma once
#include <QObject>
#include <QString>
#include <QDBusObjectPath>
#include <QDBusPendingCallWatcher>

class ScreenCast : public QObject {
    Q_OBJECT
public:
    explicit ScreenCast(QObject *parent = nullptr);
    void start();

private slots:
    void onCreateSessionFinished(QDBusPendingCallWatcher *watcher);
    void onSelectSourcesFinished(QDBusPendingCallWatcher *watcher);
    void onStartFinished(QDBusPendingCallWatcher *watcher);
    void onOpenPipeWireRemoteFinished(QDBusPendingCallWatcher *watcher);

private:
    QString sessionPath;
    QString token; // 保存 session token
};
