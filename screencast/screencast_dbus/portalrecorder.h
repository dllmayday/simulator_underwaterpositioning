#pragma once
#include <QObject>
#include <QString>
#include <QDBusObjectPath>
#include <QVariantMap>
#include <QProcess>
#include <QStandardPaths>


class QTextEdit;
class QPushButton;

class PortalRecorder : public QObject {
    Q_OBJECT
public:
    explicit PortalRecorder(QObject *parent = nullptr);
    ~PortalRecorder();

    QWidget* createUi(); // returns a QWidget pointer (owner: caller)

public slots:
    void startFlow();   // start the portal flow (CreateSession -> SelectSources -> Start -> OpenPipeWireRemote)
    void stopRecording();

private slots:
    // D-Bus response handlers (connected to portal Request.Response per request token)
    void onCreateSessionResponse(uint response, const QVariantMap &results);
    void onSelectSourcesResponse(uint response, const QVariantMap &results);
    void onStartResponse(uint response, const QVariantMap &results);

private:
    QString makeToken() const;
    QString requestPathForToken(const QString &token) const;
    void log(const QString &line);
    void startGstWithFd(int fd);

    // state
    QString sessionPath;
    QString appTokenCreate;
    QString appTokenSelect;
    QString appTokenStart;

    // UI
    QWidget *widget = nullptr;
    QTextEdit *logView = nullptr;
    QPushButton *startBtn = nullptr;
    QPushButton *stopBtn = nullptr;

    QProcess *gstProc = nullptr;
};
