
#pragma once

#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>

namespace gcs {
class GcsOrbitViewController : public rviz::OrbitViewController
{
    Q_OBJECT

  public:
    GcsOrbitViewController();
    ~GcsOrbitViewController() override;

    void handleMouseEvent(rviz::ViewportMouseEvent& evt) override;
};

} // end namespace gcs
