#include <QApplication>
#include <QFileDialog>
#include <QString>
#include <QImageReader>

#include "imagefeedwidget.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    qDebug() << "Supported formats:" << QImageReader::supportedImageFormats();

    QString folder;
    if (argc > 1) folder = QString::fromLocal8Bit(argv[1]);
    else folder = QFileDialog::getExistingDirectory(nullptr, "选择 TIFF 文件夹", QDir::homePath());

    if (folder.isEmpty()) return 0;

    ImageFeedWidget w(folder);
    w.setWindowTitle("声图实时展示");
    w.resize(1000, 800);
    w.show();

    return a.exec();
}
