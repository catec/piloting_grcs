#pragma once

#include <common/Types.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

namespace gcs {
class CloudEditionWorker
{
  public:
    CloudEditionWorker(){};
    ~CloudEditionWorker(){};

    void cutPointCloudByPlane(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
            const Axis&                                   planeAxis,
            const bool&                                   isPositiveDir,
            const float&                                  value,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr        output_cloud)
    {
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*input_cloud, minPt, maxPt);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(input_cloud);
        pass.setFilterFieldName(ToString(planeAxis));

        if (planeAxis == Axis::X) {
            if (isPositiveDir) {
                pass.setFilterLimits(value, maxPt.x);
            } else {
                pass.setFilterLimits(minPt.x, value);
            }
        } else if (planeAxis == Axis::Y) {
            if (isPositiveDir) {
                pass.setFilterLimits(value, maxPt.y);
            } else {
                pass.setFilterLimits(minPt.y, value);
            }
        } else if (planeAxis == Axis::Z) {
            if (isPositiveDir) {
                pass.setFilterLimits(value, maxPt.z);
            } else {
                pass.setFilterLimits(minPt.z, value);
            }
        }
        pass.filter(*output_cloud);
    }
};
} // namespace gcs