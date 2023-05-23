
#include "QPointCloud2Display.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/VisualizationConfigDTO.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz/default_plugin/point_cloud_common.h>
namespace gcs {

QPointCloud2Display::QPointCloud2Display() : rviz::PointCloud2Display() {}

QPointCloud2Display::~QPointCloud2Display() {}

void QPointCloud2Display::configureVisualization(const VisualizationConfigDTO& visualizationConfig)
{
    setName("QCloudCustom");

    subProp("Queue Size")->setValue("1");
    subProp("Alpha")->setValue("1.0");
    subProp("Color Transformer")->setValue(visualizationConfig.getPointCloudColorStyle());
    subProp("Color")->setValue(QVariant(visualizationConfig.getPointCloudColor()));
    subProp("Style")->setValue("Spheres");

    /// \note. Be careful! The higher this value the longer the rendering time.
    subProp("Size (m)")->setValue(visualizationConfig.getPointCloudPtsSize());

    setEnabled(true);
}

void QPointCloud2Display::setVisible(bool visible)
{
    setEnabled(visible);
}

void QPointCloud2Display::attachData(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    processMessage(cloud);
}

void QPointCloud2Display::attachData(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    sensor_msgs::PointCloud2Ptr outCloud(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *outCloud);
    /// \note. Hardcoded frame id
    outCloud->header.frame_id = "map";
    attachData(outCloud);
}

} // namespace gcs