#include "RouteBusiness.h"

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <QsLog/QsLog.h>

#include "../dtos/MissionDTO.h"
#include "common/CommonDefines.h"
#include "dataModel/PoseDTO.h"

namespace gcs {

RouteBusiness::RouteBusiness(MissionDTO& missionDTO) : _missionDTO(missionDTO)
{
    QLOG_TRACE() << "RouteBusiness::RouteBusiness()";
}

RouteBusiness::~RouteBusiness()
{
    QLOG_TRACE() << "RouteBusiness::~RouteBusiness()";
}

void RouteBusiness::addRoute(const WaypointDTO& wpToAdd) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "RouteBusiness::addRoute()";

    QLOG_DEBUG() << "RouteBusiness::addRoute() - "
                    "Id:"
                 << wpToAdd.getId() << "Type:" << static_cast<uint8_t>(wpToAdd.getWpType())
                 << "Task UUID:" << wpToAdd.getTaskUUID() << "X:" << wpToAdd.getPosition().getX()
                 << "Y:" << wpToAdd.getPosition().getY() << "Z:" << wpToAdd.getPosition().getZ()
                 << "Qw:" << wpToAdd.getOrientation().getW() << "Qx:" << wpToAdd.getOrientation().getX()
                 << "Qy:" << wpToAdd.getOrientation().getY() << "Qz:" << wpToAdd.getOrientation().getZ();

    WaypointDTO waypoint = wpToAdd;
    waypoint.getId()     = _missionDTO.getSeqNWaypoint()++;

    RouteDTO routeToAdd;
    routeToAdd.getId() = _missionDTO.getSeqNRoute()++;
    routeToAdd.getWpsList().append(waypoint);

    _missionDTO.getLocalRoutesMap().insert(routeToAdd.getId(), routeToAdd);
    _missionDTO.getCurrentWaypointId() = waypoint.getId();
    _missionDTO.getCurrentRouteId()    = routeToAdd.getId();
}

void RouteBusiness::addRoute(const RouteDTO& routeToAdd)
{
    QLOG_TRACE() << "RouteBusiness::addRoute(RouteDTO)";

    const QList<WaypointDTO>& wpsList = routeToAdd.getWpsList();

    if (wpsList.isEmpty()) {
        const QString msg("Route to add is empty");
        QLOG_ERROR() << "RouteBusiness::addRoute(RouteDTO) -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    addRoute(wpsList.first());

    for (int i = 1; i < wpsList.size(); i++) {
        addWaypoint(wpsList.at(i));
    }
}

void RouteBusiness::removeRoute(const quint16 routeId)
{
    QLOG_TRACE() << "RouteBusiness::removeRoute()";

    QLOG_DEBUG() << "RouteBusiness::removeRoute() - "
                    "Route Id:"
                 << routeId;

    if (_missionDTO.getLocalRoutesMap().remove(routeId) == 0) {
        const QString msg = QString("Route to delete does not exist: %1").arg(routeId);
        QLOG_ERROR() << "RouteBusiness::removeRoute() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    selectLastWaypoint();
}

void RouteBusiness::addWaypoint(const PoseDTO& poseToAdd, const bool& autoHeading)
{
    QLOG_TRACE() << "RouteBusiness::addWaypoint()";

    QLOG_DEBUG() << "RouteBusiness::addWaypoint() - "
                    "Autoheading:"
                 << autoHeading << "posX:" << poseToAdd.getPosition().getX()
                 << "posY:" << poseToAdd.getPosition().getY() << "posZ:" << poseToAdd.getPosition().getZ()
                 << "quatW:" << poseToAdd.getOrientation().getW() << "quatX:" << poseToAdd.getOrientation().getX()
                 << "quatY:" << poseToAdd.getOrientation().getY() << "quatZ:" << poseToAdd.getOrientation().getZ();

    auto& localRoutesMap = _missionDTO.getLocalRoutesMap();
    if (!localRoutesMap.contains(_missionDTO.getCurrentRouteId())) {
        const auto msg = QString("Current route does not exist: %1").arg(_missionDTO.getCurrentRouteId());
        QLOG_ERROR() << "RouteBusiness::addWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    auto& wpsList = localRoutesMap[_missionDTO.getCurrentRouteId()].getWpsList();
    auto  it      = std::find_if(wpsList.begin(), wpsList.end(), [&](const WaypointDTO& waypoint) {
        return (waypoint.getId() == _missionDTO.getCurrentWaypointId());
    });

    if (it == wpsList.end()) {
        const auto msg
                = QString("Waypoint does not exist in current route: %1").arg(_missionDTO.getCurrentWaypointId());
        QLOG_ERROR() << "RouteBusiness::editWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    WaypointDTO waypoint;
    waypoint.getTaskUUID()    = it->getTaskUUID();
    waypoint.getPosition()    = poseToAdd.getPosition();
    waypoint.getOrientation() = poseToAdd.getOrientation();

    addWaypoint(waypoint, autoHeading);
}

void RouteBusiness::addWaypoint(const WaypointDTO& wpToAdd, const bool& autoHeading)
{
    QLOG_TRACE() << "RouteBusiness::addWaypoint()";

    QLOG_DEBUG() << "RouteBusiness::addWaypoint() - "
                    "Autoheading:"
                 << autoHeading << "Id:" << wpToAdd.getId() << "Type:" << static_cast<uint8_t>(wpToAdd.getWpType())
                 << "Task Id:" << wpToAdd.getTaskUUID() << "X:" << wpToAdd.getPosition().getX()
                 << "Y:" << wpToAdd.getPosition().getY() << "Z:" << wpToAdd.getPosition().getZ()
                 << "Qw:" << wpToAdd.getOrientation().getW() << "Qx:" << wpToAdd.getOrientation().getX()
                 << "Qy:" << wpToAdd.getOrientation().getY() << "Qz:" << wpToAdd.getOrientation().getZ();

    if (!_missionDTO.getLocalRoutesMap().contains(_missionDTO.getCurrentRouteId())) {
        const QString msg = QString("Current route does not exist: %1").arg(_missionDTO.getCurrentRouteId());
        QLOG_ERROR() << "RouteBusiness::addWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    WaypointDTO waypoint = wpToAdd;
    waypoint.getId()     = _missionDTO.getSeqNWaypoint()++;

    RouteDTO& currentRoute  = _missionDTO.getLocalRoutesMap()[_missionDTO.getCurrentRouteId()];
    bool      existWaypoint = false;

    QList<WaypointDTO>::iterator iter = currentRoute.getWpsList().begin();

    while (iter != currentRoute.getWpsList().end()) {
        if (iter->getId() == _missionDTO.getCurrentWaypointId()) {
            currentRoute.getWpsList().insert(++iter, waypoint);
            existWaypoint = true;
            break;
        }

        ++iter;
    }

    if (!existWaypoint) {
        const QString msg = QString("Current waypoint does not exist in current route: %1")
                                    .arg(_missionDTO.getCurrentWaypointId());
        QLOG_ERROR() << "RouteBusiness::addWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    _missionDTO.getCurrentWaypointId() = waypoint.getId();

    if (autoHeading) {
        manageRouteAutoHeading();
    }
}

void RouteBusiness::removeWaypoint(const quint16 wpId)
{
    QLOG_TRACE() << "RouteBusiness::removeWaypoint()";

    QLOG_DEBUG() << "RouteBusiness::removeWaypoint() - "
                    "Waypoint Id:"
                 << wpId;

    if (!_missionDTO.getLocalRoutesMap().contains(_missionDTO.getCurrentRouteId())) {
        const QString msg = QString("Current route does not exist: %1").arg(_missionDTO.getCurrentRouteId());
        QLOG_ERROR() << "RouteBusiness::removeWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    RouteDTO& currentRoute = _missionDTO.getLocalRoutesMap()[_missionDTO.getCurrentRouteId()];

    QList<WaypointDTO>& wpsList = currentRoute.getWpsList();

    const auto it = std::find_if(wpsList.begin(), wpsList.end(), [&wpId](const WaypointDTO& waypoint) {
        return (waypoint.getId() == wpId);
    });
    if (it == wpsList.end()) {
        const QString msg = QString("Waypoint does not exist in current route: %1").arg(wpId);
        QLOG_ERROR() << "RouteBusiness::removeWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    int prevWpId = 0;
    if (it != wpsList.begin()) {
        prevWpId = std::prev(it)->getId();
    }

    bool    isActionWp{false};
    QString removedActionWpUuid{""};
    QString prevActionWpUuid{""};
    if (it->getWpType() == WaypointType::ACTION) {
        isActionWp          = true;
        removedActionWpUuid = it->getTaskUUID();

        /// \note. Get inmediate prev UUID to reassociate the wpts to it before remove the ACTION wp
        for (auto itPtr = it; itPtr != wpsList.begin(); --itPtr) {
            if (itPtr->getTaskUUID() != removedActionWpUuid) {
                prevActionWpUuid = itPtr->getTaskUUID();
                break;
            }
        }
    }

    /// \note. Remove the waypoint
    wpsList.erase(it);

    if (!wpsList.isEmpty()) {
        if (prevWpId != 0) {
            _missionDTO.getCurrentWaypointId() = prevWpId;
        } else {
            _missionDTO.getCurrentWaypointId() = wpsList.last().getId();
        }
    } else {
        QLOG_DEBUG() << "RouteBusiness::removeWaypoint() - "
                        "Current route is empty, it can be deleted";

        removeRoute(currentRoute.getId());
    }

    if (isActionWp) {
        QLOG_DEBUG() << "RouteBusiness::removeWaypoint() - "
                     << "Removed action waypoint UUID:" << removedActionWpUuid;

        /// \note. Reassociate the wpts to the prev action waypoint UUID
        for (auto itPtr = it; itPtr != wpsList.end(); ++itPtr) {
            if (itPtr->getTaskUUID() == removedActionWpUuid && prevActionWpUuid != "") {
                QLOG_DEBUG() << "RouteBusiness::removeWaypoint() - "
                             << "Reassociate waypoint UUID:" << itPtr->getId()
                             << "to action waypoint UUID:" << prevActionWpUuid;
                itPtr->getTaskUUID() = prevActionWpUuid;
            }
        }
    }
}

void RouteBusiness::editWaypoint(const WaypointDTO& wpToChange)
{
    QLOG_TRACE() << "RouteBusiness::editWaypoint()";

    QLOG_DEBUG() << "RouteBusiness::editWaypoint() - "
                    "Id:"
                 << wpToChange.getId() << "Type:" << static_cast<uint8_t>(wpToChange.getWpType())
                 << "Task UUID:" << wpToChange.getTaskUUID() << "AutoContinue:" << wpToChange.getAutoContinue()
                 << "X:" << wpToChange.getPosition().getX() << "Y:" << wpToChange.getPosition().getY()
                 << "Z:" << wpToChange.getPosition().getZ() << "Qw:" << wpToChange.getOrientation().getW()
                 << "Qx:" << wpToChange.getOrientation().getX() << "Qy:" << wpToChange.getOrientation().getY()
                 << "Qz:" << wpToChange.getOrientation().getZ();

    auto& localRoutesMap = _missionDTO.getLocalRoutesMap();
    if (!localRoutesMap.contains(_missionDTO.getCurrentRouteId())) {
        const auto msg = QString("Current route does not exist: %1").arg(_missionDTO.getCurrentRouteId());
        QLOG_ERROR() << "RouteBusiness::editWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    auto& wpsList = localRoutesMap[_missionDTO.getCurrentRouteId()].getWpsList();

    auto it = std::find_if(wpsList.begin(), wpsList.end(), [&](const WaypointDTO& waypoint) {
        return (waypoint.getId() == wpToChange.getId());
    });

    if (it == wpsList.end()) {
        const auto msg = QString("Waypoint does not exist in current route: %1").arg(wpToChange.getId());
        QLOG_ERROR() << "RouteBusiness::editWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    /// \note Propagate TaskId if changed
    if (it->getTaskUUID() != wpToChange.getTaskUUID()) {
        /// \note All indexes are updated up to the end of the route
        for (auto remainWptsIt = it; remainWptsIt != wpsList.end(); ++remainWptsIt) {
            remainWptsIt->getTaskUUID() = wpToChange.getTaskUUID();
        }
    }

    *it = wpToChange;

    _missionDTO.getCurrentWaypointId() = wpToChange.getId();
}

void RouteBusiness::changeSelectedWaypoint(const quint16 wpId)
{
    QLOG_TRACE() << "RouteBusiness::changeSelectedWaypoint()";

    QLOG_DEBUG() << "RouteBusiness::changeSelectedWaypoint() - "
                    "Waypoint Id:"
                 << wpId;

    bool existWaypoint = false;

    for (const auto& route : _missionDTO.getLocalRoutesMap().values()) {
        for (const auto& waypoint : route.getWpsList()) {
            if (waypoint.getId() == wpId) {
                _missionDTO.getCurrentWaypointId() = wpId;
                _missionDTO.getCurrentRouteId()    = route.getId();
                existWaypoint                      = true;
                break;
            }
        }
    }

    if (!existWaypoint) {
        const QString msg = QString("Waypoint does not exist: %1").arg(wpId);
        QLOG_ERROR() << "RouteBusiness::changeSelectedWaypoint() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }
}

void RouteBusiness::editLoadedRouteWithLocal()
{
    QLOG_TRACE() << "RouteBusiness::editLoadedRouteWithLocal()";

    const QMap<int, RouteDTO>& routesMap = _missionDTO.getLocalRoutesMap();
    if (!routesMap.contains(_missionDTO.getCurrentRouteId())) {
        const QString msg = QString("Current route does not exist: %1").arg(_missionDTO.getCurrentRouteId());
        QLOG_ERROR() << "RouteBusiness::editLoadedRouteWithLocal() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    editLoadedRoute(routesMap[_missionDTO.getCurrentRouteId()]);
}

void RouteBusiness::editLoadedRoute(const RouteDTO& routeToEdit)
{
    QLOG_TRACE() << "RouteBusiness::editLoadedRoute()";

    /// \note. Commented to avoid warning when route is empty
    // if(routeToEdit.getWpsList().isEmpty())
    // {
    //    const QString msg("Route to update is empty");
    //    QLOG_ERROR() << "RouteBusiness::editLoadedRoute() -" << msg;
    //    throw std::runtime_error(msg.toStdString());
    // }

    _missionDTO.getLoadedRoute().getWpsList() = routeToEdit.getWpsList();

    QLOG_DEBUG() << "RouteBusiness::editLoadedRoute() - "
                    "Size:"
                 << _missionDTO.getLoadedRoute().getWpsList().size();
}

void RouteBusiness::removeLoadedRoute() Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "RouteBusiness::removeLoadedRoute()";

    _missionDTO.getLoadedRoute() = RouteDTO();
}

void RouteBusiness::copyLoadedRouteToLocal()
{
    QLOG_TRACE() << "RouteBusiness::copyLoadedRouteToLocal()";

    QList<WaypointDTO>& wpsList = _missionDTO.getLoadedRoute().getWpsList();

    if (wpsList.isEmpty()) {
        const QString msg("Loaded route is empty");
        QLOG_ERROR() << "RouteBusiness::copyLoadedRouteToLocal() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    addRoute(wpsList.first());

    for (int i = 1; i < wpsList.size(); i++) {
        addWaypoint(wpsList.at(i));
    }
}

void RouteBusiness::updateSeqNumbers() Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << "RouteBusiness::updateSeqNumbers()";

    quint16 maxRouteId    = 0;
    quint16 maxWaypointId = 0;

    for (const auto& route : _missionDTO.getLocalRoutesMap().values()) {
        if (route.getId() > maxRouteId) {
            maxRouteId = route.getId();
        }

        for (const auto& waypoint : route.getWpsList()) {
            if (waypoint.getId() > maxWaypointId) {
                maxWaypointId = waypoint.getId();
            }
        }
    }

    _missionDTO.getSeqNRoute()    = maxRouteId + 1;
    _missionDTO.getSeqNWaypoint() = maxWaypointId + 1;

    QLOG_DEBUG() << "RouteBusiness::updateSeqNumbers() - "
                    "SeqNumber route:"
                 << _missionDTO.getSeqNRoute() << "SeqNumber waypoint:" << _missionDTO.getSeqNWaypoint();
}

void RouteBusiness::selectLastWaypoint()
{
    QLOG_TRACE() << "RouteBusiness::selectLastWaypoint()";

    if (!_missionDTO.getLocalRoutesMap().isEmpty()) {
        if (_missionDTO.getLocalRoutesMap().last().getWpsList().isEmpty()) {
            const QString msg("The last route is empty");
            QLOG_ERROR() << "RouteBusiness::selectLastWaypoint() -" << msg;
            throw std::runtime_error(msg.toStdString());
        }

        changeSelectedWaypoint(_missionDTO.getLocalRoutesMap().last().getWpsList().last().getId());
    } else {
        QLOG_DEBUG() << "RouteBusiness::selectLastWaypoint() - "
                        "No existing routes";

        _missionDTO.getSeqNRoute()         = 1;
        _missionDTO.getSeqNWaypoint()      = 1;
        _missionDTO.getCurrentWaypointId() = 0;
        _missionDTO.getCurrentRouteId()    = 0;
    }
}

void RouteBusiness::manageRouteAutoHeading()
{
    QLOG_TRACE() << "RouteBusiness::manageRouteAutoHeading()";

    auto& wpsList = _missionDTO.getLocalRoutesMap()[_missionDTO.getCurrentRouteId()].getWpsList();
    auto  lastWp  = wpsList.last();
    if (lastWp != wpsList.first()) {
        auto         prevIt    = wpsList.end() - 2;
        WaypointDTO& secLastWp = *prevIt;

        applyAutoHeading(lastWp, secLastWp);
    }
}

void RouteBusiness::applyAutoHeading(const WaypointDTO& lastWp, WaypointDTO& secondLastWp)
{
    QLOG_TRACE() << "RouteBusiness::applyAutoHeading()";

    auto vecFromSecondLastToLast = Ogre::Vector3(
            lastWp.getPosition().getX() - secondLastWp.getPosition().getX(),
            lastWp.getPosition().getY() - secondLastWp.getPosition().getY(),
            lastWp.getPosition().getZ() - secondLastWp.getPosition().getZ());
    vecFromSecondLastToLast.normalise();

    auto secondLastQ = Ogre::Quaternion(
            secondLastWp.getOrientation().getW(),
            secondLastWp.getOrientation().getX(),
            secondLastWp.getOrientation().getY(),
            secondLastWp.getOrientation().getZ());

    auto vecNormal = secondLastQ.zAxis();
    vecNormal.normalise();

    auto vecOrtho = vecNormal.crossProduct(vecFromSecondLastToLast);
    vecOrtho.normalise();

    auto quat                            = Ogre::Quaternion(vecFromSecondLastToLast, vecOrtho, vecNormal);
    secondLastWp.getOrientation().getW() = quat.w;
    secondLastWp.getOrientation().getX() = quat.x;
    secondLastWp.getOrientation().getY() = quat.y;
    secondLastWp.getOrientation().getZ() = quat.z;
}
} // namespace gcs
