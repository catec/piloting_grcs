#pragma once

#include <common/Types.h>

#include <QTreeWidget>
#include <memory>
namespace Ui {
class WaypointsTreeWidget;
}

namespace gcs {

class MissionDTO;
class WaypointDTO;

class WaypointsTreeWidget : public QTreeWidget
{
    Q_OBJECT

  public:
    explicit WaypointsTreeWidget(QWidget* parent = nullptr);
    ~WaypointsTreeWidget();

    void redrawItems(const MissionDTO& missionDTO);

  Q_SIGNALS:
    void changeCurrentWaypoint(QSharedPointer<WaypointDTO> waypoint);
    void editSelectedWaypoint(QSharedPointer<WaypointDTO> waypoint);

  private Q_SLOTS:
    void waypointSelectionChanged();

  private:
    std::unique_ptr<Ui::WaypointsTreeWidget> _ui;
};

} // namespace gcs
