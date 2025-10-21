#include "portalrecorder.h"
#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTextEdit>
#include <QDateTime>
#include <QDBusInterface>
#include <QDBusConnection>
#include <QDBusPendingCall>
#include <QDBusPendingReply>
#include <QDBusMessage>
#include <QDBusUnixFileDescriptor>
#include <QDBusObjectPath>
#include <QProcess>
#include <QCoreApplication>
#include <QDebug>
#include <qdbusreply.h>

PortalRecorder::PortalRecorder(QObject *parent)
    : QObject(parent), gstProc(nullptr)
{
}

PortalRecorder::~PortalRecorder()
{
    stopRecording();
}

QWidget* PortalRecorder::createUi()
{
    widget = new QWidget;
    auto layout = new QVBoxLayout(widget);
    startBtn = new QPushButton("Start Portal Screencast (Select & Start)");
    stopBtn = new QPushButton("Stop Recording");
    stopBtn->setEnabled(false);
    logView = new QTextEdit;
    logView->setReadOnly(true);
    layout->addWidget(startBtn);
    layout->addWidget(stopBtn);
    layout->addWidget(logView);
    widget->setLayout(layout);

    connect(startBtn, &QPushButton::clicked, this, &PortalRecorder::startFlow);
    connect(stopBtn, &QPushButton::clicked, this, &PortalRecorder::stopRecording);

    return widget;
}

void PortalRecorder::log(const QString &line)
{
    if (logView) {
        QString t = QDateTime::currentDateTime().toString("HH:mm:ss");
        logView->append(QString("[%1] %2").arg(t, line));
    }
    qDebug() << line;
}

QString PortalRecorder::makeToken() const
{
    // create a semi-unique token per request; portal uses this token to form request object path
    return QString("qt_%1_%2").arg(QCoreApplication::applicationPid()).arg(QDateTime::currentMSecsSinceEpoch());
}

QString PortalRecorder::requestPathForToken(const QString &token) const
{
    // portal creates request objects under /org/freedesktop/portal/desktop/request/<APPID>/<TOKEN>
    // use applicationName() as appid (not strictly required to be a real .desktop id in many implementations)
    QString appId = QCoreApplication::applicationName();
    if (appId.isEmpty()) appId = QString::number(QCoreApplication::applicationPid());
    return QString("/org/freedesktop/portal/desktop/request/%1/%2").arg(appId, token);
}

void PortalRecorder::startFlow()
{
    // reset state
    sessionPath.clear();

    // 1) CreateSession (async) — need to listen for Request.Response on the matching request object path
    appTokenCreate = makeToken();
    QString reqPath = requestPathForToken(appTokenCreate);

    // Connect to the Response signal of the Request object we expect
    QDBusConnection::sessionBus().connect(
        "org.freedesktop.portal.Desktop",
        reqPath,
        "org.freedesktop.portal.Request",
        "Response",
        this,
        SLOT(onCreateSessionResponse(uint,QVariantMap))
    );

    QVariantMap opts;
    opts["handle_token"] = appTokenCreate;
    // other options can be set: e.g., opts["session_options"] = ...
    log(QString("Calling CreateSession (handle_token=%1)").arg(appTokenCreate));

    QDBusInterface iface("org.freedesktop.portal.Desktop",
                        "/org/freedesktop/portal/desktop",
                        "org.freedesktop.portal.ScreenCast",
                        QDBusConnection::sessionBus());

    // Use asyncCall so portal creates a Request object and emits Response to our connected path
    iface.asyncCall("CreateSession", QVariant::fromValue(opts));
}

void PortalRecorder::onCreateSessionResponse(uint response, const QVariantMap &results)
{
    // This is called when portal responds for CreateSession.
    // Response code 0 means success.
    Q_UNUSED(results);
    // We need to find session_handle in results.
    // The signature is something like { "session_handle": objectpath("/org/freedesktop/...") }
    // Results arrives as QVariantMap with values; session_handle may be a QDBusObjectPath or string.
    QVariantMap res = results;
    if (response != 0) {
        log(QString("CreateSession response error: %1").arg(response));
        return;
    }

    // find session_handle
    QString sPath;
    if (res.contains("session_handle")) {
        QVariant v = res.value("session_handle");
        // v may be QDBusObjectPath or string
        if (v.canConvert<QDBusObjectPath>()) {
            sPath = v.value<QDBusObjectPath>().path();
        } else {
            sPath = v.toString();
        }
    }

    if (sPath.isEmpty()) {
        log("CreateSession: no session_handle returned");
        return;
    }

    sessionPath = sPath;
    log(QString("Session created: %1").arg(sessionPath));

    // disconnect the response signal for create token (no longer needed)
    QString reqPath = requestPathForToken(appTokenCreate);
    QDBusConnection::sessionBus().disconnect(
        "org.freedesktop.portal.Desktop",
        reqPath,
        "org.freedesktop.portal.Request",
        "Response",
        this,
        SLOT(onCreateSessionResponse(uint,QVariantMap))
    );

    // Proceed to SelectSources
    appTokenSelect = makeToken();
    QString reqPathSelect = requestPathForToken(appTokenSelect);
    QDBusConnection::sessionBus().connect(
        "org.freedesktop.portal.Desktop",
        reqPathSelect,
        "org.freedesktop.portal.Request",
        "Response",
        this,
        SLOT(onSelectSourcesResponse(uint,QVariantMap))
    );

    // Call SelectSources(session, opts)
    QDBusInterface iface("org.freedesktop.portal.Desktop",
                        "/org/freedesktop/portal/desktop",
                        "org.freedesktop.portal.ScreenCast",
                        QDBusConnection::sessionBus());

    QVariantMap opts;
    opts["types"] = (uint)1; // 1 = monitor; 2 = window; 4 = virtual; combine via bitmask
    opts["multiple"] = false;
    opts["cursor_mode"] = (uint)2; // 0=hidden,1=embedded,2=metadata (show)
    opts["handle_token"] = appTokenSelect;

    // session passed as object path
    QDBusObjectPath sessionObj(sessionPath);

    log("Calling SelectSources (portal dialog will allow you choose screen/window)");
    iface.asyncCall("SelectSources", QVariant::fromValue(sessionObj), QVariant::fromValue(opts));
}

void PortalRecorder::onSelectSourcesResponse(uint response, const QVariantMap &results)
{
    Q_UNUSED(results);
    if (response != 0) {
        log(QString("SelectSources failed response: %1").arg(response));
        return;
    }
    log("SelectSources: user selection success");

    // disconnect select response watcher
    QString reqPathSelect = requestPathForToken(appTokenSelect);
    QDBusConnection::sessionBus().disconnect(
        "org.freedesktop.portal.Desktop",
        reqPathSelect,
        "org.freedesktop.portal.Request",
        "Response",
        this,
        SLOT(onSelectSourcesResponse(uint,QVariantMap))
    );

    // Now call Start(session, opts) — must also pass a handle_token and watch response
    appTokenStart = makeToken();
    QString reqPathStart = requestPathForToken(appTokenStart);
    QDBusConnection::sessionBus().connect(
        "org.freedesktop.portal.Desktop",
        reqPathStart,
        "org.freedesktop.portal.Request",
        "Response",
        this,
        SLOT(onStartResponse(uint,QVariantMap))
    );

    QDBusInterface iface("org.freedesktop.portal.Desktop",
                        "/org/freedesktop/portal/desktop",
                        "org.freedesktop.portal.ScreenCast",
                        QDBusConnection::sessionBus());

    QVariantMap opts;
    opts["handle_token"] = appTokenStart;
    // optionally other options: e.g., pipeline options, session options
    QDBusObjectPath sessionObj(sessionPath);

    log("Calling Start (portal will ask permission to allow screen casting)");
    iface.asyncCall("Start", QVariant::fromValue(sessionObj), QVariant::fromValue(opts));
}

void PortalRecorder::onStartResponse(uint response, const QVariantMap &results)
{
    if (response != 0) {
        log(QString("Start failed response: %1").arg(response));
        return;
    }
    log("Start: approved. Now calling OpenPipeWireRemote to get PipeWire FD.");

    // disconnect start response watcher
    QString reqPathStart = requestPathForToken(appTokenStart);
    QDBusConnection::sessionBus().disconnect(
        "org.freedesktop.portal.Desktop",
        reqPathStart,
        "org.freedesktop.portal.Request",
        "Response",
        this,
        SLOT(onStartResponse(uint,QVariantMap))
    );

    // Call OpenPipeWireRemote on the session object path to get the unix fd
    // Interface name for session object is "org.freedesktop.portal.Session" or "org.freedesktop.portal.ScreenCast.Session"
    QDBusInterface sessionIface("org.freedesktop.portal.Desktop",
                               sessionPath,
                               "org.freedesktop.portal.ScreenCast.Session",
                               QDBusConnection::sessionBus());

    // Some portal implementations expose method "OpenPipeWireRemote" that returns a unix fd (h)
    // Use call() and try to read QDBusUnixFileDescriptor
    QDBusMessage msg = sessionIface.call("OpenPipeWireRemote");
    if (msg.type() == QDBusMessage::ErrorMessage) {
        log(QString("OpenPipeWireRemote call error: %1").arg(msg.errorMessage()));
        // try alternative: some portals return a dictionary with "fd"
        QDBusReply<QVariantMap> alt = sessionIface.call("OpenPipeWireRemote");
        if (!alt.isValid()) {
            log("OpenPipeWireRemote: no valid reply.");
            return;
        }
    } else {
        // parse unix fd from reply
        if (!msg.arguments().isEmpty()) {
            // expecting a QDBusUnixFileDescriptor as first arg or a map
            QVariant v = msg.arguments().at(0);
            if (v.canConvert<QDBusUnixFileDescriptor>()) {
                QDBusUnixFileDescriptor ufd = v.value<QDBusUnixFileDescriptor>();
                int fd = ufd.fileDescriptor();
                if (fd >= 0) {
                    log(QString("Got PipeWire FD: %1").arg(fd));
                    startGstWithFd(fd);
                    stopBtn->setEnabled(true);
                    startBtn->setEnabled(false);
                    return;
                }
            } else if (v.type() == QVariant::Map) {
                QVariantMap m = v.toMap();
                // sometimes the fd is inside a map under key "fd" or "pipewire_remote"
                if (m.contains("fd")) {
                    int fd = m.value("fd").toInt();
                    log(QString("Got PipeWire fd (map): %1").arg(fd));
                    startGstWithFd(fd);
                    stopBtn->setEnabled(true);
                    startBtn->setEnabled(false);
                    return;
                }
            }
        }
    }

    // fallback: try QDBusReply<QDBusUnixFileDescriptor>
    QDBusReply<QDBusUnixFileDescriptor> r = sessionIface.call("OpenPipeWireRemote");
    if (r.isValid()) {
        int fd = r.value().fileDescriptor();
        if (fd >= 0) {
            log(QString("Got PipeWire FD (reply): %1").arg(fd));
            startGstWithFd(fd);
            stopBtn->setEnabled(true);
            startBtn->setEnabled(false);
            return;
        }
    }

    log("Failed to obtain PipeWire FD from OpenPipeWireRemote");
}

// start gst-launch-1.0 as a child process to record from pipewiresrc fd=<fd>
void PortalRecorder::startGstWithFd(int fd)
{
    if (gstProc) {
        log("GStreamer already running");
        return;
    }

    QString out = QString("%1/record_%2.mp4")
                  .arg(QStandardPaths::writableLocation(QStandardPaths::MoviesLocation))
                  .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    // construct a robust pipeline: handle dma-buf via GL, then encode h264
    QString pipeline = QString("pipewiresrc fd=%1 ! video/x-raw,framerate=30/1 ! glupload ! glcolorconvert ! gldownload ! videoconvert ! x264enc bitrate=4000 speed-preset=veryfast tune=zerolatency ! mp4mux ! filesink location=%2")
                       .arg(fd)
                       .arg(out);

    log(QString("Starting gst-launch: %1").arg(pipeline));
    gstProc = new QProcess(this);
    connect(gstProc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int code, QProcess::ExitStatus st){
                Q_UNUSED(code); Q_UNUSED(st);
                log("GStreamer process finished.");
                stopBtn->setEnabled(false);
                startBtn->setEnabled(true);
                if (gstProc) { gstProc->deleteLater(); gstProc = nullptr; }
            });

    // start gst-launch-1.0 -e <pipeline>
    gstProc->start("gst-launch-1.0", QStringList() << "-e" << pipeline);
    if (!gstProc->waitForStarted(3000)) {
        log("Failed to start gst-launch-1.0. Is GStreamer installed?");
        gstProc->deleteLater();
        gstProc = nullptr;
        return;
    }
    log(QString("Recording to: %1").arg(out));
}

void PortalRecorder::stopRecording()
{
    if (gstProc && gstProc->state() == QProcess::Running) {
        log("Stopping GStreamer...");
        gstProc->terminate();
        if (!gstProc->waitForFinished(3000)) gstProc->kill();
        // cleanup will be handled in finished slot
    } else {
        log("No GStreamer process running.");
    }

    // Additionally, you could call Cancel on the session via D-Bus to free portal resources.
    if (!sessionPath.isEmpty()) {
        QDBusInterface sessionIface("org.freedesktop.portal.Desktop",
                                   "/org/freedesktop/portal/desktop",
                                   "org.freedesktop.portal.ScreenCast",
                                   QDBusConnection::sessionBus());
        // Some portals provide Cancel or Close methods; skipping in demo.
    }
}
