#include <QCoreApplication>
#include "screencast.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    ScreenCast sc;
    sc.start();

    return a.exec();
}
