#pragma once
#include <QWidget>
#include <QDir>
#include <QSet>
#include <QFileSystemWatcher>
#include <QTimer>

class QVBoxLayout;
class QLabel;

class ImageFeedWidget : public QWidget {
    Q_OBJECT
public:
    explicit ImageFeedWidget(const QString &folder, QWidget *parent = nullptr);
    ~ImageFeedWidget() override = default;

    void setMaxImagesDisplayed(int n) { m_maxImages = n; }
    void setThumbWidth(int w) { m_thumbWidth = w; }

private slots:
    void onDirectoryChanged(const QString &path);
    void scanFolder();

private:
    void loadNewImages(const QStringList &files);
    QPixmap makeThumbnail(const QString &path);

    QString m_folder;
    QDir m_dir;
    QFileSystemWatcher *m_watcher;
    QTimer *m_scanTimer;
    QSet<QString> m_knownFiles; // 已经加载/显示的文件
    QVBoxLayout *m_layout;
    QWidget *m_container;
    int m_maxImages = 200;
    int m_thumbWidth = 1200;
};
