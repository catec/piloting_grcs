#pragma once

#include <pcl/common/common.h>

#include "rviz/default_plugin/point_cloud2_display.h"

namespace gcs {
class VisualizationConfigDTO;
class QPointCloud2Display : public rviz::PointCloud2Display
{
    Q_OBJECT

  public:
    QPointCloud2Display();
    virtual ~QPointCloud2Display();

    void configureVisualization(const VisualizationConfigDTO&);

    void attachData(const sensor_msgs::PointCloud2ConstPtr&);
    void attachData(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
    void setVisible(bool);
};
} // namespace gcs
