#pragma once

#include <QsLog/QsLog.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>

#include <QFile>
#include <QThread>
#include <QTime>

namespace gcs {

class AssetParserThread : public QThread
{
    Q_OBJECT

  public:
    void init(const QString& file, const int& assetSize);

  Q_SIGNALS:
    void loadedAssetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

  protected:
    void run() override;

  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr readAndSampleSTL();

    QString _filePath;
    int     _assetSize;
};
} // namespace gcs
