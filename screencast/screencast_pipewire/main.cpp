#include <QCoreApplication>
#include "screencast.h"
#include <QTimer>
#include <QDebug>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    a.setApplicationName("console_pipewire_screencast");

    ScreenCast sc;
    sc.startFlow();

    // 自动在 60 秒后停止（可按需改）
    QTimer::singleShot(60000, [&sc](){
        qDebug() << "Auto stopping recording after timeout";
        sc.stopRecording();
        QCoreApplication::quit();
    });

    return a.exec();
}
