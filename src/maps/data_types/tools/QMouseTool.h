#pragma once

#include <OgreViewport.h>
#include <maps/items/Waypoint3dItem.h>
#include <rviz/tool.h>

namespace gcs {
class Map3dDTO;
class InspectionTaskDTO;

class QMouseTool : public rviz::Tool
{
    Q_OBJECT

  public:
    QMouseTool(Map3dDTO& mapDTO);
    virtual ~QMouseTool();

    void onInitialize() override;

    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz::ViewportMouseEvent& event) override;
    int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

    void updateWpScaleRangeSelectable(const float&);

  private Q_SLOTS:
    void showWaypointContextMenu(QPoint pos, Waypoint3dItem wp);
    void showInspTaskContextMenu(QPoint pos, InspectionTaskDTO inspTask);
    void showHomeContextMenu(QPoint pos);

  Q_SIGNALS:
    void cursorPosition(QVector3D);
    void addWaypoint(QVector3D);
    void editWaypoint(QSharedPointer<WaypointDTO>);
    void addHome(QPointF);
    void changeSelectedWaypoint(quint16);
    void removeWaypoint(quint16);
    void addRoute(QVector3D);
    void removeRoute(quint16);
    void sendCurrentRoute();
    void copyLoadedRouteToLocal();
    void addMeasuringPoint(QVector3D);
    void editHomeParameters();

    void editWaypointHeading();

    void updateViewInspectionTask(QString);

    void addActionWpOverInspTask(QString);
    void addPoseWpOverInspTask(QString);

    void manageDirectionArrow(Ogre::Vector3, float, bool);

  private:
    QSharedPointer<Waypoint3dItem>    getSelectableWaypoint(const QVector3D&);
    QSharedPointer<InspectionTaskDTO> getSelectableInspectionTask(const QVector3D&);
    bool                              getSelectableHome(const QPointF&);

    void updateWaypointItemById(const quint16&, const Ogre::Vector3&);

  private:
    Map3dDTO& _mapDTO;

    std::pair<bool, quint16> _movingWaypoint;

    /// \note. These value is experimental and in base of element size
    float _wptsRangeSafety{0.1f};
    float _homeRangeSafety{0.2f};
    float _inspTaskRangeSafety{0.3f};
};
} // namespace gcs
