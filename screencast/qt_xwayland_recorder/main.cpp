#include <qtextstream.h>
#include <QApplication>
#include <QWidget>
#include <QTimer>
#include "x11recorder.h"
#include <QDebug>
#include <QPixmap>
#include <QImage>
#include <QPainter>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QWidget win;
    win.resize(640, 480);
    win.setWindowTitle("Qt Xwayland Recorder Demo");
    win.show();

    QPixmap pix = win.grab(); // 返回窗口图像
    QImage img = pix.toImage();
    img.save("window.png");


#ifdef Q_OS_UNIX
    WId winId = win.winId(); // 获取 X11 窗口 ID
    qDebug() << "X11 Window ID:" << winId;

    X11Recorder recorder(winId, 30);
    recorder.start();

    QTimer::singleShot(60000, [&recorder, &app](){
        recorder.stop();
        app.quit();
    });
#endif

    return app.exec();
}
