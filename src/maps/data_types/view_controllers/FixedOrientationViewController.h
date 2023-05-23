
#pragma once

#include <rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h>

namespace gcs {
class FixedOrientationViewController : public rviz::FixedOrientationOrthoViewController
{
    Q_OBJECT

  public:
    FixedOrientationViewController();
    ~FixedOrientationViewController() override;

    void reset() override;

    void handleMouseEvent(rviz::ViewportMouseEvent& evt) override;

    bool& movingWaypoint();

  private:
    bool _movingWaypoint = false;
};

} // end namespace gcs
