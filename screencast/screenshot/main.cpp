#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QDateTime>
#include <QStandardPaths>
#include <QFileInfo>
#include <QDir>
#include <QMessageBox>
#include <QDBusInterface>
#include <QDBusPendingCall>
#include <QDBusPendingCallWatcher>
#include <QDBusReply>

static QString makeOutputPath() {
    QString pics = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    if (pics.isEmpty()) pics = QDir::homePath() + "/Pictures";
    QDir dir(pics);
    if (!dir.exists()) dir.mkpath(".");
    QString filename = QStringLiteral("screenshot_%1.png")
                           .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    return dir.filePath(filename);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QWidget w;
    w.setWindowTitle("Simple GNOME Screenshot (via D-Bus)");

    auto layout = new QVBoxLayout(&w);
    auto btn = new QPushButton("Take Screenshot (GNOME D-Bus)");
    layout->addWidget(btn);
    w.setLayout(layout);
    w.resize(360,120);

    QObject::connect(btn, &QPushButton::clicked, [&]() {
        QString outPath = makeOutputPath();

        // Create D-Bus interface to org.gnome.Shell.Screenshot
        QDBusInterface ifc("org.gnome.Shell.Screenshot",
                           "/org/gnome/Shell/Screenshot",
                           "org.gnome.Shell.Screenshot",
                           QDBusConnection::sessionBus());

        if (!ifc.isValid()) {
            QMessageBox::critical(nullptr, "D-Bus Error",
                                  "Cannot access org.gnome.Shell.Screenshot on the session bus.\n"
                                  "Are you running GNOME Shell?");
            return;
        }

        // Parameters: include pointer (bool), flash (bool), filename (string)
        // Many examples call: Screenshot(bool include_pointer, bool flash, string filename)
        QDBusPendingCall pcall = ifc.asyncCall("Screenshot", true, true, outPath);
        auto watcher = new QDBusPendingCallWatcher(pcall);
        QObject::connect(watcher, &QDBusPendingCallWatcher::finished, [watcher, outPath]() {
            QDBusPendingReply<QVariant> reply(*watcher);
            if (reply.isError()) {
                QString err = reply.error().message();
                QMessageBox::critical(nullptr, "Screenshot failed", QString("D-Bus error: %1").arg(err));
            } else {
                // The GNOME method may return (s) or (s, ...) depending on version. We only need to check the file.
                QFileInfo fi(outPath);
                if (fi.exists() && fi.size() > 0) {
                    QMessageBox::information(nullptr, "Screenshot saved", QString("Saved: %1").arg(outPath));
                } else {
                    // Sometimes GNOME returns a temporary path; attempt to read returned variant
                    QVariant returned = reply.argumentAt(0);
                    QString retstr = returned.toString();
                    if (!retstr.isEmpty() && QFileInfo(retstr).exists()) {
                        QMessageBox::information(nullptr, "Screenshot saved", QString("Saved: %1").arg(retstr));
                    } else {
                        QMessageBox::warning(nullptr, "Screenshot",
                                             "No file found after request. GNOME may have denied permission or returned an unexpected result.");
                    }
                }
            }
            watcher->deleteLater();
        });
    });

    w.show();
    return a.exec();
}
