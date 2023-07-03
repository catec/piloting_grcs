
#include "QMouseTool.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/InspectionTaskDTO.h>
#include <maps/data_types/view_controllers/FixedOrientationViewController.h>
#include <maps/dtos/Map3dDTO.h>
#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/render_panel.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/viewport_mouse_event.h>

#include <QGraphicsView>
#include <QMenu>
#include <QMetaEnum>
#include <QVector3D>

/// @brief Returns true if X is in range [low-high], else false
template <typename dataType>
bool inRange(dataType low, dataType high, dataType x)
{
    return ((x - high) * (x - low) <= 0);
}

namespace gcs {

QMouseTool::QMouseTool(Map3dDTO& mapDTO) : rviz::Tool(), _mapDTO(mapDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // setClassId("gcs/MouseTool");
}

QMouseTool::~QMouseTool()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void QMouseTool::onInitialize()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // setName("QMouseToolCustom");

    _movingWaypoint = std::make_pair<bool, quint16>(false, 0);
}

void QMouseTool::activate()
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void QMouseTool::deactivate()
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void QMouseTool::updateWpScaleRangeSelectable(const float& scale)
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;

    _wptsRangeSafety = 0.1f * scale;
}

int QMouseTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    // QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (event.panel->getViewController()) {
        event.panel->getViewController()->handleMouseEvent(event);

        if (!_movingWaypoint.first) {
            setCursor(Qt::ArrowCursor);
        }
        // setCursor(event.panel->getViewController()->getCursor());
    }

    /// \note. Depending on the edition mode, the position of the mouse in the visualizer is captured differently.
    const bool    mapInEditionMode2D = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::TwoD);
    const bool    mapInEditionMode3D = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::ThreeD);
    Ogre::Vector3 cursor_intersection;
    QVector3D     cursor_position;
    if (mapInEditionMode2D) {
        Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
        if (!rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cursor_intersection)) {
            return Finished;
        }
    } else if (mapInEditionMode3D) {
        if (!event.panel->getManager()->getSelectionManager()->get3DPoint(
                    event.viewport, event.x, event.y, cursor_intersection)) {
            return Finished;
        }
    } else {
        return Finished;
    }

    cursor_position = QVector3D{cursor_intersection.x, cursor_intersection.y, cursor_intersection.z};

    /// \note Actions by States:
    ///  - Panning:               Select waypoint or Move map
    ///  - Measuring:             Select waypoint or Move map
    ///  - Creating Route:        Select waypoint or Move map
    ///  - Editing Route:         Select waypoint, Move waypoint or Move map
    ///  - Editing Loaded Home:   Select waypoint, Move map or Move loaded home
    const bool mapInEditingRouteState      = _mapDTO.getAppStatus() == AppStatusType::EditingRoute;
    const bool mapInCreatingRouteState     = _mapDTO.getAppStatus() == AppStatusType::CreatingRoute;
    const bool mapInEditingLoadedHomeState = _mapDTO.getAppStatus() == AppStatusType::EditingLoadedHome;
    const bool mapInMeasuringState         = _mapDTO.getAppStatus() == AppStatusType::Measuring;

    QSharedPointer<Waypoint3dItem> selectedWp;
    if (event.type == QEvent::MouseMove) {
        Q_EMIT cursorPosition(cursor_position);
    } else if (event.left()) {
        selectedWp = getSelectableWaypoint(cursor_position);

        /// \note. It's only possible to change the position of a waypoint by dragging in Edition2D mode.
        if (selectedWp && !selectedWp->inLoadedRoute()) {
            QLOG_DEBUG() << "QMouseTool::event.left() - "
                            "Changing selected waypoint";

            Q_EMIT changeSelectedWaypoint(selectedWp->id());

            /// If clicking and dragging on the waypoint --> Changes its position
            if (mapInEditingRouteState && mapInEditionMode2D) {
                setCursor(Qt::ClosedHandCursor);
                QLOG_DEBUG() << "QMouseTool::event.leftDown() - "
                                "Editing selected waypoint in 2D View";

                QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Editing waypoint with id: " << selectedWp->id()
                             << " in position: (" << selectedWp->waypoint().getPosition().getX() << ","
                             << selectedWp->waypoint().getPosition().getY() << ","
                             << selectedWp->waypoint().getPosition().getZ() << ")";

                _movingWaypoint.first  = true;
                _movingWaypoint.second = selectedWp->id();

                dynamic_cast<FixedOrientationViewController*>(event.panel->getViewController())->movingWaypoint()
                        = true;
            }
        }
        if (event.type == QEvent::MouseButtonDblClick) {
            if (mapInCreatingRouteState) {
                QLOG_DEBUG() << "QMouseTool::mouseDoubleClickEvent() - "
                                "In creating route status";

                Q_EMIT addRoute(cursor_position);
            } else if (mapInEditingRouteState) {
                /// If there is an existing Waypoint --> Remove it
                if (selectedWp && !selectedWp->inLoadedRoute()) {
                    QLOG_DEBUG() << "QMouseTool::mouseDoubleClickEvent() - "
                                    "Removing selected waypoint";

                    Q_EMIT removeWaypoint(selectedWp->id());
                } else /// If there isn't an existing Waypoint --> Add it
                {
                    QLOG_DEBUG() << "QMouseTool::mouseDoubleClickEvent() - "
                                    "Adding new waypoint";

                    /// \note. It must not be possible to place waypoints over inspection location shapes
                    const auto selectedInspTask = getSelectableInspectionTask(cursor_position);
                    if (!selectedInspTask) {
                        Q_EMIT addWaypoint(cursor_position);
                    }
                }
            } else if (mapInEditingLoadedHomeState) {
                QLOG_DEBUG() << "QMouseTool::mouseDoubleClickEvent() - "
                                "In edition home status";

                Q_EMIT addHome(QPointF{cursor_position.x(), cursor_position.y()});
            } else if (mapInMeasuringState) {
                QLOG_DEBUG() << "QMouseTool::mouseDoubleClickEvent() - "
                                "Adding New Measuring Point";
                emit addMeasuringPoint(cursor_position);
            }
        }
    }
    /// When click the right button, it's shown a menu to apply differents actions
    else if (event.right()) {
        selectedWp = getSelectableWaypoint(cursor_position);
        if (selectedWp) {
            if (!selectedWp->inLoadedRoute()) {
                Q_EMIT changeSelectedWaypoint(selectedWp->id());
            }

            showWaypointContextMenu(event.panel->mapToGlobal(QPoint{event.x, event.y}), *selectedWp);
        }

        if (getSelectableHome(QPointF{cursor_position.x(), cursor_position.y()}) && mapInEditingLoadedHomeState) {
            showHomeContextMenu(event.panel->mapToGlobal(QPoint{event.x, event.y}));
        }

        const auto selectedInspTask = getSelectableInspectionTask(cursor_position);
        if (selectedInspTask) {
            updateViewInspectionTask(selectedInspTask->getUUID());
            showInspTaskContextMenu(event.panel->mapToGlobal(QPoint{event.x, event.y}), *selectedInspTask);
        }
    }
    /// \note. It's only possible to change the position of a waypoint by dragging in Edition2D mode.
    ///        If clicking over waypoint and release the button --> Changes its position
    else if (event.leftUp() && mapInEditionMode2D) {
        if (_movingWaypoint.first) {
            updateWaypointItemById(_movingWaypoint.second, cursor_intersection);
            _movingWaypoint = std::make_pair<bool, quint16>(false, 0);
            dynamic_cast<FixedOrientationViewController*>(event.panel->getViewController())->movingWaypoint() = false;
            setCursor(Qt::ArrowCursor);
        }
    }

    return Finished;
}

int QMouseTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (panel->getManager()->getViewManager()->getCurrent()) {
        panel->getManager()->getViewManager()->getCurrent()->handleKeyEvent(event, panel);
    }

    /// \note. Waypoint can only be moved using the arrow keys in Edition3D mode.
    const bool mapInEditionMode3D = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::ThreeD);

    if (!mapInEditionMode3D) {
        return Render;
    }

    /// \note. Get selected waypoint
    const auto it = std::find_if(
            _mapDTO.getVisibleWaypointsList().cbegin(),
            _mapDTO.getVisibleWaypointsList().cend(),
            [&](const QSharedPointer<Waypoint3dItem> wp) { return wp->isSelectedWaypoint(); });
    if (it == _mapDTO.getVisibleWaypointsList().cend()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - "
                    << "Unable to move waypoint. No waypoint selected";
        return Render;
    }

    const auto    waypoint_id = (*it)->id();
    const auto    waypoint_pt = (*it)->waypoint().getPosition();
    Ogre::Vector3 updated_waypoint(waypoint_pt.getX(), waypoint_pt.getY(), waypoint_pt.getZ());

    /// \note. To be able to move waypoint using arrow keys, must be pressed Control
    float targetAngle;
    int   visible = false;
    if (event->type() == QKeyEvent::KeyPress && event->modifiers() & Qt::ControlModifier) {
        visible                       = true;
        const float movementIncrement = 0.1f;
        if (event->key() == Qt::Key_Left) {
            updated_waypoint[1] = waypoint_pt.getY() + movementIncrement;
            targetAngle         = 0.0;
        } else if (event->key() == Qt::Key_Up) {
            updated_waypoint[0] = waypoint_pt.getX() + movementIncrement;
            targetAngle         = -1.0f * (M_PI / 2.0f);
        } else if (event->key() == Qt::Key_Right) {
            updated_waypoint[1] = waypoint_pt.getY() - movementIncrement;
            targetAngle         = M_PI;
        } else if (event->key() == Qt::Key_Down) {
            updated_waypoint[0] = waypoint_pt.getX() - movementIncrement;
            targetAngle         = M_PI / 2.0f;
        } else {
            return Render;
        }
    }

    Q_EMIT manageDirectionArrow(updated_waypoint, targetAngle, visible);

    updateWaypointItemById(waypoint_id, updated_waypoint);
    return Render;
}

void QMouseTool::showWaypointContextMenu(QPoint pos, Waypoint3dItem wp)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QMenu currentWpMenu;

    QAction deleteWp(trUtf8("Delete waypoint"), this);
    QAction deleteRoute(trUtf8("Delete route"), this);
    QAction copyToLocal(trUtf8("Copy loaded route to local"), this);
    QAction uploadRoute(trUtf8("Upload route"), this);
    QAction modifyCurrHeading(trUtf8("Modify waypoint heading"), this);

    currentWpMenu.addAction(&deleteWp);
    currentWpMenu.addAction(&deleteRoute);
    currentWpMenu.addAction(&copyToLocal);
    QAction* separator = currentWpMenu.addSeparator();
    currentWpMenu.addAction(&uploadRoute);
    currentWpMenu.addAction(&modifyCurrHeading);

    const bool inLoadedRoute   = wp.inLoadedRoute();
    const bool inSelectedRoute = wp.inSelectedRoute();

    deleteWp.setVisible(inSelectedRoute);
    deleteRoute.setVisible(inSelectedRoute);
    copyToLocal.setVisible(inLoadedRoute);
    separator->setVisible(inSelectedRoute || inLoadedRoute);
    uploadRoute.setVisible(inSelectedRoute);
    modifyCurrHeading.setVisible(inSelectedRoute);

    const bool isConnected             = _mapDTO.getCommsIsConnected();
    const bool inEditionMode           = _mapDTO.getInEditionMode();
    const bool mapInEditingRouteState  = _mapDTO.getAppStatus() == AppStatusType::EditingRoute;
    const bool mapInCreatingRouteState = _mapDTO.getAppStatus() == AppStatusType::CreatingRoute;

    deleteWp.setEnabled(inEditionMode && mapInEditingRouteState && !inLoadedRoute);
    deleteRoute.setEnabled(inEditionMode && mapInEditingRouteState && !inLoadedRoute);
    copyToLocal.setEnabled(inEditionMode && inLoadedRoute && (mapInEditingRouteState || mapInCreatingRouteState));
    uploadRoute.setEnabled(inEditionMode && isConnected && !inLoadedRoute);

    modifyCurrHeading.setEnabled(inEditionMode && mapInEditingRouteState && !inLoadedRoute);

    QAction* selectedAct = currentWpMenu.exec(pos);

    if (selectedAct == &deleteWp) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - Delete waypoint action has been selected";

        Q_EMIT removeWaypoint(wp.id());
    } else if (selectedAct == &deleteRoute) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - Delete waypoint action has been selected";

        Q_EMIT removeRoute(wp.routeId());
    } else if (selectedAct == &copyToLocal) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - Copy to local action has been selected";
        Q_EMIT copyLoadedRouteToLocal();
    } else if (selectedAct == &uploadRoute) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - Upload route action has been selected";

        Q_EMIT sendCurrentRoute();
    } else if (selectedAct == &modifyCurrHeading) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - Modify Waypoint heading action has been selected";

        Q_EMIT editWaypointHeading();
    }
}

void QMouseTool::showHomeContextMenu(QPoint pos)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QMenu loadedHomeMenu;

    QAction editHomeParam(trUtf8("Edit Home Parameters"), this);

    loadedHomeMenu.addAction(&editHomeParam);

    const bool mapInEditingLoadedHomeStatus = _mapDTO.getAppStatus() == AppStatusType::EditingLoadedHome;

    editHomeParam.setEnabled(mapInEditingLoadedHomeStatus);

    QAction* selectedAct = loadedHomeMenu.exec(pos);
    if (selectedAct == &editHomeParam) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Edit home parameters action has been selected";

        Q_EMIT editHomeParameters();
    }
}

void QMouseTool::showInspTaskContextMenu(QPoint pos, InspectionTaskDTO selectedInspTask)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QMenu inspTaskMenu;

    QAction addActionWpAct(trUtf8("Add Action Waypoint"), this); // Add Inspection Task Action
    QAction addPoseWpAct(trUtf8("Add Pose Waypoint"), this);
    QAction deleteActionWpAct(trUtf8("Delete Action Waypoint"), this);
    QAction deletePoseWpAct(trUtf8("Delete Pose Waypoint"), this);
    inspTaskMenu.addAction(&addActionWpAct);
    inspTaskMenu.addAction(&addPoseWpAct);
    inspTaskMenu.addAction(&deleteActionWpAct);
    inspTaskMenu.addAction(&deletePoseWpAct);

    bool inSelectedActionWp{false};
    auto itActionWp = std::find_if(
            _mapDTO.getVisibleWaypointsList().cbegin(),
            _mapDTO.getVisibleWaypointsList().cend(),
            [&selectedInspTask](const QSharedPointer<Waypoint3dItem>& wp) {
                return (selectedInspTask.getPosition() == wp->waypoint().getPosition()
                        && wp->waypoint().getWpType() == WaypointType::ACTION);
            });
    if (itActionWp != _mapDTO.getVisibleWaypointsList().cend()) {
        inSelectedActionWp = true;
    }

    bool inSelectedPoseWp{false};
    auto itPoseWp = std::find_if(
            _mapDTO.getVisibleWaypointsList().cbegin(),
            _mapDTO.getVisibleWaypointsList().cend(),
            [&selectedInspTask](const QSharedPointer<Waypoint3dItem>& wp) {
                return (selectedInspTask.getPosition() == wp->waypoint().getPosition()
                        && wp->waypoint().getWpType() == WaypointType::POSE);
            });
    if (itPoseWp != _mapDTO.getVisibleWaypointsList().cend()) {
        inSelectedPoseWp = true;
    }

    addActionWpAct.setVisible(!inSelectedActionWp);
    addPoseWpAct.setVisible(!inSelectedPoseWp);
    deleteActionWpAct.setVisible(inSelectedActionWp);
    deletePoseWpAct.setVisible(inSelectedPoseWp);

    QAction* selectedAct = inspTaskMenu.exec(pos);
    if (selectedAct == &addActionWpAct) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Add ACTION Waypoint Action has been selected";

        Q_EMIT addActionWpOverInspTask(selectedInspTask.getUUID());
    } else if (selectedAct == &addPoseWpAct) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Add POSE Waypoint Action has been selected";

        Q_EMIT addPoseWpOverInspTask(selectedInspTask.getUUID());
    } else if (selectedAct == &deleteActionWpAct) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Delete ACTION Waypoint Action has been selected";

        Q_EMIT removeWaypoint((*itActionWp)->id());
    } else if (selectedAct == &deletePoseWpAct) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Delete POSE Waypoint Action has been selected";

        Q_EMIT removeWaypoint((*itPoseWp)->id());
    } else {
        updateViewInspectionTask("");
    }
}

QSharedPointer<Waypoint3dItem> QMouseTool::getSelectableWaypoint(const QVector3D& pos)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const bool mapInEditionMode2D = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::TwoD);
    const bool mapInEditionMode3D = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::ThreeD);

    QSharedPointer<Waypoint3dItem> selectedWp = nullptr;
    auto                           it         = std::find_if(
            _mapDTO.getVisibleWaypointsList().cbegin(),
            _mapDTO.getVisibleWaypointsList().cend(),
            [&](const QSharedPointer<Waypoint3dItem> wp) {
                /// \note A range of ±px is added to the selection by clicking
                const bool selectedWpPosition = inRange(wp->waypoint().getPosition().getX() - _wptsRangeSafety,
                                                        wp->waypoint().getPosition().getX() + _wptsRangeSafety,
                                                        pos.x())
                                             && inRange(wp->waypoint().getPosition().getY() - _wptsRangeSafety,
                                                        wp->waypoint().getPosition().getY() + _wptsRangeSafety,
                                                        pos.y());
                const bool selectedWpAltitude = inRange(
                        wp->waypoint().getPosition().getZ() - _wptsRangeSafety,
                        wp->waypoint().getPosition().getZ() + _wptsRangeSafety,
                        pos.z());
                if (mapInEditionMode2D) {
                    return selectedWpPosition;
                } else if (mapInEditionMode3D) {
                    return selectedWpPosition && selectedWpAltitude;
                }

                return false;
            });

    if (it != _mapDTO.getVisibleWaypointsList().cend()) {
        selectedWp = QSharedPointer<Waypoint3dItem>::create();
        selectedWp = *it;

        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Selected waypoint with id: " << selectedWp->id()
                     << "seqNumber:" << selectedWp->seqNumber() << "routeId:" << selectedWp->routeId();
    } else {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Not selected any waypoint";
    }

    return selectedWp;
}

bool QMouseTool::getSelectableHome(const QPointF& pos)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_mapDTO.getLoadedHome().getIsEnabled()) {
        return false;
    }

    QPointF loadedHomePosition
            = QPointF{_mapDTO.getLoadedHome().getPosition().getX(), _mapDTO.getLoadedHome().getPosition().getY()};

    return inRange(loadedHomePosition.x() - _homeRangeSafety, loadedHomePosition.x() + _homeRangeSafety, pos.x())
        && inRange(loadedHomePosition.y() - _homeRangeSafety, loadedHomePosition.y() + _homeRangeSafety, pos.y());
}

QSharedPointer<InspectionTaskDTO> QMouseTool::getSelectableInspectionTask(const QVector3D& pos)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QSharedPointer<InspectionTaskDTO> selectedInspTask = nullptr;

    if (_mapDTO.getInspectionTaskList().empty()) {
        return selectedInspTask;
    }

    const bool mapInEditionMode3D     = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::ThreeD);
    const bool mapInEditingRouteState = _mapDTO.getAppStatus() == AppStatusType::EditingRoute;
    if (!mapInEditionMode3D || !mapInEditingRouteState) {
        return selectedInspTask;
    }

    auto it = std::find_if(
            _mapDTO.getInspectionTaskList().cbegin(),
            _mapDTO.getInspectionTaskList().cend(),
            [&](const InspectionTaskDTO& insp_task) {
                if (insp_task.getLocationType() == InspectionTaskLocationType::POINT) {
                    /// \note A range of ±px is added to the selection by clicking
                    const bool selectedInspTask = inRange(insp_task.getPosition().getX() - _inspTaskRangeSafety,
                                                          insp_task.getPosition().getX() + _inspTaskRangeSafety,
                                                          pos.x())
                                               && inRange(insp_task.getPosition().getY() - _inspTaskRangeSafety,
                                                          insp_task.getPosition().getY() + _inspTaskRangeSafety,
                                                          pos.y())
                                               && inRange(insp_task.getPosition().getZ() - _inspTaskRangeSafety,
                                                          insp_task.getPosition().getZ() + _inspTaskRangeSafety,
                                                          pos.z());
                    if (selectedInspTask) {
                        return true;
                    }
                } else if (insp_task.getLocationType() == InspectionTaskLocationType::AREA) {
                    /// \note. This needs to be improved as it only works well near the centre of the area.
                    const bool selectedInspTask
                            = inRange(insp_task.getPosition().getX() - (insp_task.getWidth() / 2.0f)
                                              - _inspTaskRangeSafety,
                                      insp_task.getPosition().getX() + (insp_task.getWidth() / 2.0f)
                                              + _inspTaskRangeSafety,
                                      pos.x())
                           && inRange(insp_task.getPosition().getY() - (insp_task.getDepth() / 2.0f)
                                              - _inspTaskRangeSafety,
                                      insp_task.getPosition().getY() + (insp_task.getDepth() / 2.0f)
                                              + _inspTaskRangeSafety,
                                      pos.y())
                           && inRange(insp_task.getPosition().getZ() - (insp_task.getHeight() / 2.0f)
                                              - _inspTaskRangeSafety,
                                      insp_task.getPosition().getZ() + (insp_task.getHeight() / 2.0f)
                                              + _inspTaskRangeSafety,
                                      pos.z());

                    if (selectedInspTask) {
                        return true;
                    }
                } else if (insp_task.getLocationType() == InspectionTaskLocationType::PART) {
                    QLOG_WARN() << __PRETTY_FUNCTION__ << "Not implemented logic to this type of inspection task";
                } else {
                    QLOG_WARN() << __PRETTY_FUNCTION__ << "Unknown Inspection Task type";
                }
                return false;
            });
    if (it != _mapDTO.getInspectionTaskList().cend()) {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Selected inspection task with name: " << it->getName();

        selectedInspTask  = QSharedPointer<InspectionTaskDTO>::create();
        *selectedInspTask = *it;
    }

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Not selected any inspection task";
    return selectedInspTask;
}

void QMouseTool::updateWaypointItemById(const quint16& movedId, const Ogre::Vector3& clickedPosition)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto it = std::find_if(
            _mapDTO.getVisibleWaypointsList().cbegin(),
            _mapDTO.getVisibleWaypointsList().cend(),
            [&](const QSharedPointer<Waypoint3dItem> wp) { return wp->id() == movedId; });

    if (it != _mapDTO.getVisibleWaypointsList().cend()) {
        /// \note. When updating an already created waypoint it is assumed that only its X and Y will be modified.
        (*it)->waypoint().getPosition().getX() = clickedPosition[0];
        (*it)->waypoint().getPosition().getY() = clickedPosition[1];

        auto wpToUpdate = createSharedDTO<WaypointDTO>();
        *wpToUpdate     = (*it)->waypoint();

        Q_EMIT editWaypoint(wpToUpdate);
    }
}

} // namespace gcs

/// \note. This plugin cant be exported because it needs empty constructor
///        to be able to include it in the factory.

// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(gcs::QMouseTool, rviz::Tool)
