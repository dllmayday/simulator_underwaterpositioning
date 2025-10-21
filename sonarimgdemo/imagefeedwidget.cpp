#include "imagefeedwidget.h"
#include <QScrollArea>
#include <QVBoxLayout>
#include <QLabel>
#include <QImageReader>
#include <QFileInfo>
#include <QDateTime>
#include <QScrollBar>
#include <QDebug>
#include <QPainter>
#include <algorithm>
#include <tiffio.h>

ImageFeedWidget::ImageFeedWidget(const QString &folder, QWidget *parent)
    : QWidget(parent),
      m_folder(folder),
      m_dir(folder)
{
    // UI: scroll area + vertical layout
    auto *scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    m_container = new QWidget;
    m_layout = new QVBoxLayout(m_container);
    m_layout->setSpacing(0);
    m_layout->setContentsMargins(0,0,0,0);
    m_container->setLayout(m_layout);
    scroll->setWidget(m_container);

    auto *mainLay = new QVBoxLayout(this);
    mainLay->addWidget(scroll);
    setLayout(mainLay);

    // watcher + timer
    m_watcher = new QFileSystemWatcher(this);
    if (m_dir.exists()) {
        m_watcher->addPath(m_folder);
    }
    connect(m_watcher, &QFileSystemWatcher::directoryChanged,
            this, &ImageFeedWidget::onDirectoryChanged);

    m_scanTimer = new QTimer(this);
    m_scanTimer->setInterval(2000); // 每2秒扫描一次防止遗漏
    connect(m_scanTimer, &QTimer::timeout, this, &ImageFeedWidget::scanFolder);
    m_scanTimer->start();

    scanFolder(); // 初次加载
}

void ImageFeedWidget::onDirectoryChanged(const QString &path)
{
    Q_UNUSED(path);
    scanFolder();
}

void ImageFeedWidget::scanFolder()
{
    if (!m_dir.exists()) return;

    // 匹配 tiff 文件（不区分大小写）
    QStringList nameFilters = {"*.tif", "*.tiff", "*.TIF", "*.TIFF"};

    // ✅ 按文件名排序（字母序）
    QFileInfoList fileInfos = m_dir.entryInfoList(nameFilters, QDir::Files, QDir::Name);

    QStringList files;
    for (const QFileInfo &fi : fileInfos)
        files << fi.absoluteFilePath();

    // 找出新文件
    QStringList newFiles;
    for (const QString &f : files) {
        if (!m_knownFiles.contains(f))
            newFiles << f;
    }

    if (!newFiles.isEmpty()) {
        // ✅ 文件名顺序从上到下显示（不需要倒序）
        loadNewImages(newFiles);

        // 更新已知文件集
        for (const QString &f: newFiles)
            m_knownFiles.insert(f);

        // 清理已删除文件
        QSet<QString> currentSet = QSet<QString>::fromList(files);
        QList<QString> removed = m_knownFiles.toList();
        for (const QString &k : removed) {
            if (!currentSet.contains(k))
                m_knownFiles.remove(k);
        }
    }
}

void ImageFeedWidget::loadNewImages(const QStringList &files)
{
    // 用 QTimer 实现逐个动态加载
    if (files.isEmpty()) return;
    if (files.isEmpty()) return;
    // 只保留一个 QLabel
    QLabel *lab = nullptr;
    QSpacerItem *spring = nullptr;
    // 检查布局是否已有弹簧和 QLabel
    if (m_layout->count() == 2) {
        QLayoutItem *item0 = m_layout->itemAt(0);
        spring = dynamic_cast<QSpacerItem *>(item0->spacerItem());
        QLayoutItem *item1 = m_layout->itemAt(1);
        lab = qobject_cast<QLabel *>(item1->widget());
    }
    if (!lab) {
        // 清空布局
        while (m_layout->count() > 0) {
            QLayoutItem *it = m_layout->takeAt(0);
            if (it) {
                QWidget *w = it->widget();
                delete it;
                if (w) delete w;
            }
        }
        
        // 添加 QLabel 用于图像刷新
        lab = new QLabel;
        lab->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
        lab->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        m_layout->addWidget(lab);
        // 添加弹簧
        spring = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);
        m_layout->addItem(spring);
    }

    // 动态合并 QPixmap
    QList<QPixmap> pixmaps;
    QPixmap merged;
    int totalWidth = 0;
    int totalHeight = 0;
    int idx = 0;
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [=]() mutable {
        if (idx >= files.size()) {
            timer->stop();
            timer->deleteLater();
            // 加载完毕后滚动到底部
            QScrollArea *scroll = findChild<QScrollArea *>();
            if (scroll) {
                QScrollBar *vbar = scroll->verticalScrollBar();
                vbar->setValue(vbar->maximum());
            }
            return;
        }
        QPixmap pix = makeThumbnail(files.at(idx));
        if (!pix.isNull()) {
            pixmaps << pix;
            totalWidth = std::max(totalWidth, pix.width());
            totalHeight += pix.height();
            // 合并已有 pixmaps
            QPixmap tmp(totalWidth, totalHeight);
            tmp.fill(Qt::black); // 可选：背景色
            QPainter painter(&tmp);
            int y = 0;
            // 新图片在最上方，倒序绘制
            int y2 = 0;
            for (int i = pixmaps.size() - 1; i >= 0; --i) {
                const QPixmap &p = pixmaps[i];
                painter.drawPixmap(0, y2, p);
                y2 += p.height();
            }
            painter.end();
            lab->setPixmap(tmp);
            // 每次加载后滚动到底部，确保最新图片在视野底部
            QScrollArea *scroll = findChild<QScrollArea *>();
            if (scroll) {
                // 强制刷新布局，确保滚动条高度及时更新
                scroll->widget()->updateGeometry();
                scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
            }
        }
        idx++;
    });
    timer->start(120); // 每120ms加载一个
}

QPixmap ImageFeedWidget::makeThumbnail(const QString &path)
{
    // 1️⃣ 尝试用 QImageReader 读取
    {
        QImageReader reader(path);
        reader.setAutoTransform(true);
        QImage img = reader.read();
        if (!img.isNull()) {
            if (img.width() > m_thumbWidth)
                img = img.scaledToWidth(m_thumbWidth, Qt::SmoothTransformation);
            return QPixmap::fromImage(img);
        }
        qWarning() << "Qt reader failed for" << path << ":" << reader.errorString();
    }

    // 2️⃣ 如果失败，使用 libtiff 读取
    TIFF* tif = TIFFOpen(path.toLocal8Bit().data(), "r");
    if (!tif) {
        qWarning() << "Cannot open TIFF:" << path;
        return {};
    }

    uint32 w, h;
    uint16 samples = 1, bits = 8;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &samples);
    TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &bits);

    QImage img;

    if (samples == 1 && bits == 16) {
        // ✅ 16-bit 灰度声图
        QVector<uint16> buf(w * h);
        for (uint32 y = 0; y < h; y++)
            TIFFReadScanline(tif, buf.data() + y * w, y);

        img = QImage(w, h, QImage::Format_Grayscale8);
        for (uint32 y = 0; y < h; y++) {
            uchar* dst = img.scanLine(y);
            const uint16* src = buf.constData() + y * w;
            for (uint32 x = 0; x < w; x++)
                dst[x] = static_cast<uchar>(src[x] >> 8);  // normalize
        }
    } else {
        // ✅ 彩色或8位灰度（回退方案）
        QVector<uint32> raster(w * h);
        if (TIFFReadRGBAImageOriented(tif, w, h, raster.data(), ORIENTATION_TOPLEFT, 0)) {
            QImage rgb(w, h, QImage::Format_RGB888);
            for (uint32 y = 0; y < h; ++y) {
                uchar* dst = rgb.scanLine(y);
                const uint32* src = raster.constData() + y * w;
                for (uint32 x = 0; x < w; ++x) {
                    uint32 p = src[x];
                    dst[x * 3 + 0] = TIFFGetR(p);
                    dst[x * 3 + 1] = TIFFGetG(p);
                    dst[x * 3 + 2] = TIFFGetB(p);
                }
            }
            img = rgb;
        }
    }

    TIFFClose(tif);

    if (img.isNull()) {
        qWarning() << "libtiff read failed:" << path;
        return {};
    }

    // 3️⃣ 生成缩略图
    if (img.width() > m_thumbWidth)
        img = img.scaledToWidth(m_thumbWidth, Qt::SmoothTransformation);

    return QPixmap::fromImage(img);
}
