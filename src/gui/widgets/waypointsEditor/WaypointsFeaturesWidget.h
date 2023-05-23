#pragma once

#include <common/Types.h>

#include <QWidget>
#include <memory>
namespace Ui {
class WaypointsFeaturesWidget;
}

namespace gcs {
class MissionDTO;
class WaypointDTO;
class InspectionTaskDTO;

class WaypointsFeaturesWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit WaypointsFeaturesWidget(QWidget* parent = nullptr);
    ~WaypointsFeaturesWidget();

    void updateFeatures(const MissionDTO& mission);
    void setEnableFeatures(const bool value);
    void updateInspectionTaskList(const QList<InspectionTaskDTO>);

  Q_SIGNALS:
    void editWaypointFeatures(QSharedPointer<WaypointDTO>);

  private Q_SLOTS:
    void indicateChanges();
    void editWaypoint();

  private:
    void waypointTypeChanged(const WaypointType&);
    void createComboBoxes();
    void setConnections();
    void blockChanges(const bool value);
    void clearFeatures();

    std::unique_ptr<Ui::WaypointsFeaturesWidget> _ui;

    bool _hasChanges{false};
};

} // namespace gcs
