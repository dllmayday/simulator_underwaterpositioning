#include <QApplication>
#include <QWidget>
#include "portalrecorder.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QCoreApplication::setApplicationName("qt_portal_screencast_demo");

    PortalRecorder rec;
    QWidget *w = rec.createUi();
    w->setWindowTitle("Qt Portal Screencast Demo");
    w->resize(700, 400);
    w->show();

    return a.exec();
}
