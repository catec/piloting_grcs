
#include "AssetParserThread.h"

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "MeshSampling.h"

namespace gcs {
void AssetParserThread::init(const QString& filePath, const int& assetSize)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _filePath  = filePath;
    _assetSize = assetSize;
}

void AssetParserThread::run()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QTime conversionCloudTimer;
    conversionCloudTimer.start();

    const auto binPath = _filePath.left(_filePath.lastIndexOf(".")) + ".bin";
    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Loading point cloud from:" << _filePath;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr readedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (!QFile::exists(binPath)) {
        if (_filePath.endsWith(".ply")) {
            if (pcl::io::loadPLYFile(_filePath.toStdString(), *readedCloud) == -1) {
                throw std::runtime_error("Couldn't read file: " + _filePath.toStdString());
            }

            std::for_each(readedCloud->begin(), readedCloud->end(), [](auto& point) {
                point.r = 255.0;
                point.g = 255.0;
                point.b = 255.0;
            });

        } else if (_filePath.endsWith(".stl")) {
            readedCloud = readAndSampleSTL();
        } else if (_filePath.endsWith(".pcd")) {
            if (pcl::io::loadPCDFile(_filePath.toStdString(), *readedCloud) == -1) {
                throw std::runtime_error("Couldn't read file: " + _filePath.toStdString());
            }
        }
        if (pcl::io::savePCDFile(binPath.toStdString(), *readedCloud, true) == -1) {
            throw std::runtime_error("Couldn't save file: " + binPath.toStdString());
        }
    } else {
        if (pcl::io::loadPCDFile(binPath.toStdString(), *readedCloud) == -1) {
            throw std::runtime_error("Couldn't load file: " + binPath.toStdString());
        }
    }

    const auto elapsedMs = conversionCloudTimer.elapsed();
    QLOG_DEBUG() << __PRETTY_FUNCTION__
                 << "Successfully readed point cloud in " + QString::number(elapsedMs / 1000.0f, 'f', 3) + " sec";

    Q_EMIT loadedAssetPointCloud(readedCloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AssetParserThread::readAndSampleSTL()
{
    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Sampling STL mesh from: " << _filePath;

    pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh);
    pcl::io::loadPolygonFileSTL(_filePath.toStdString(), *mesh);

    /// \note. If the asset size is too small, less than 1000 bytes, the number of sampled points is fixed to 1000.
    ///		   I know it isnt the best solution but I still can't figure out something better.
    int numberPointsToSample{1000};
    if (_assetSize > 1000) {
        numberPointsToSample = _assetSize;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampledPCD(new pcl::PointCloud<pcl::PointXYZRGB>);
    sampledPCD = sampleMeshIntoPcd(numberPointsToSample, *mesh);

    return sampledPCD;
}

} // namespace gcs
