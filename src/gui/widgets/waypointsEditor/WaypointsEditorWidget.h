#pragma once

#include <QWidget>
#include <memory>

#include "WaypointsFeaturesWidget.h"
#include "WaypointsTreeWidget.h"

namespace Ui {
class WaypointsEditorWidget;
}

namespace gcs {

class MissionDTO;
class InspectionTaskDTO;

class WaypointsEditorWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit WaypointsEditorWidget(QWidget* parent = nullptr);
    ~WaypointsEditorWidget();

    void setVerticalLayout();
    void setHorizontalLayout();

    void updateMission(const MissionDTO&);
    void updateStatus(const bool);
    void updateInspectionTaskList(const QList<InspectionTaskDTO>);

  Q_SIGNALS:
    void changeCurrentWaypoint(QSharedPointer<WaypointDTO>);
    void editWaypointFeatures(QSharedPointer<WaypointDTO>);

  private:
    void loadGeometry();
    void writeGeometry();

    void resetLayout(QLayout*);

    std::unique_ptr<Ui::WaypointsEditorWidget> _ui;
    WaypointsFeaturesWidget                    _waypointsFeaturesWidget;
    WaypointsTreeWidget                        _waypointsTreeWidget;

    bool _isVertical{false};
    bool _inEditionMode{false};
    bool _hasRoutes{false};
    bool _hasMission{false};
};
} // namespace gcs
