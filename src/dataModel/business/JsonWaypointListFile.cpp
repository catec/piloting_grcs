#include "JsonWaypointListFile.h"

#include <QsLog/QsLog.h>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "../dtos/MissionDTO.h"

namespace gcs {

JsonWaypointListFile::JsonWaypointListFile(MissionDTO& missionDTO) : _missionDTO(missionDTO)
{
    QLOG_TRACE() << "JsonWaypointListFile::JsonWaypointListFile()";
}

JsonWaypointListFile::~JsonWaypointListFile()
{
    QLOG_TRACE() << "JsonWaypointListFile::~JsonWaypointListFile()";
}

void JsonWaypointListFile::load(const QString& missionPath, const QString& inspPlanUUID)
{
    QLOG_TRACE() << "JsonWaypointListFile::load()";

    QLOG_DEBUG() << "JsonWaypointListFile::load() - "
                    "Mission path to load:"
                 << missionPath;

    QFile missionFile(missionPath);
    if (!missionFile.open(QIODevice::ReadOnly)) {
        const QString msg = QString("Could not open waypoint list file: %1").arg(missionPath);
        QLOG_ERROR() << "JsonWaypointListFile::load() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    if (missionFile.size() == 0) {
        const QString msg = QString("Waypoint list file: %1 is empty or corrupted").arg(missionPath);
        QLOG_ERROR() << "JsonWaypointListFile::load() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    /// \note. This is similar to use MissionBusiness::closeMission() but ensuring that would be
    ///        possible create new mission.
    _missionDTO = MissionDTO();

    QByteArray        missionData = missionFile.readAll();
    QJsonDocument     missionJDoc(QJsonDocument::fromJson(missionData));
    const QJsonObject missionJObject = missionJDoc.object();
    _jsonCheckKeys.checkExistWaypointListKeys(missionJObject);

    if (missionJObject["InspectionPlanUUID"].toString() != inspPlanUUID) {
        const QString msg("The inspection plan UUID does not match the one in the waypoint list file");
        QLOG_ERROR() << "JsonWaypointListFile::load() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    QJsonArray wptsListJO = missionJObject["WaypointList"].toArray();
    if (wptsListJO.size() == 0) {
        const QString msg("Waypoint List is empty");
        QLOG_ERROR() << "JsonWaypointListFile::load() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    MissionDTO           missionDTO;
    QMap<int, RouteDTO>& routesMap       = missionDTO.getLocalRoutesMap();
    QJsonArray           waypointsJArray = wptsListJO;

    RouteDTO routeDTO;
    routeDTO.getId()            = 1;
    QList<WaypointDTO>& wpsList = routeDTO.getWpsList();
    for (int i = 0; i < waypointsJArray.size(); ++i) {
        QJsonObject waypointJO = waypointsJArray[i].toObject();
        _jsonCheckKeys.checkExistWaypointKeys(waypointJO);

        WaypointDTO waypointDTO;
        waypointDTO.getId()           = waypointJO["Index"].toInt();
        waypointDTO.getWpType()       = static_cast<WaypointType>(waypointJO["Type"].toInt());
        waypointDTO.getTaskUUID()     = waypointJO["TaskUuid"].toString();
        waypointDTO.getAutoContinue() = waypointJO["AutoContinue"].toInt() == 1 ? true : false;

        QJsonObject positionJO = waypointJO["Position"].toObject();
        _jsonCheckKeys.checkExistPositionKeys(positionJO);
        Position3dDTO& positionDTO = waypointDTO.getPosition();
        positionDTO.getX()         = positionJO["X"].toString().toDouble();
        positionDTO.getY()         = positionJO["Y"].toString().toDouble();
        positionDTO.getZ()         = positionJO["Z"].toString().toDouble();

        QJsonObject orientationJO = waypointJO["Orientation"].toObject();
        _jsonCheckKeys.checkExistOrientationKeys(orientationJO);
        QuaternionDTO& quatDTO = waypointDTO.getOrientation();
        quatDTO.getW()         = orientationJO["Qw"].toString().toDouble();
        quatDTO.getX()         = orientationJO["Qx"].toString().toDouble();
        quatDTO.getY()         = orientationJO["Qy"].toString().toDouble();
        quatDTO.getZ()         = orientationJO["Qz"].toString().toDouble();

        QJsonObject actionParametersJO = waypointJO["ActionParameters"].toObject();
        _jsonCheckKeys.checkExistActionParametersKeys(actionParametersJO);
        ActionParametersDTO& actionParametersDTO = waypointDTO.getActionParameters();
        actionParametersDTO.getParam1()          = actionParametersJO["Param1"].toString().toDouble();
        actionParametersDTO.getParam2()          = actionParametersJO["Param2"].toString().toDouble();
        actionParametersDTO.getParam3()          = actionParametersJO["Param3"].toString().toDouble();
        actionParametersDTO.getParam4()          = actionParametersJO["Param4"].toString().toDouble();

        wpsList.append(waypointDTO);
    }

    routesMap.insert(routeDTO.getId(), routeDTO);

    _missionDTO = missionDTO;
    missionFile.close();
}

void JsonWaypointListFile::save(const QString& missionPath, const QString& inspPlanUUID)
{
    QLOG_TRACE() << "JsonWaypointListFile::save()";

    QLOG_DEBUG() << "JsonWaypointListFile::save() - "
                    "Mission path to save:"
                 << missionPath;

    QFile missionFile(missionPath);
    if (!missionFile.open(QIODevice::WriteOnly)) {
        const QString msg = QString("Could not create file to save mission: %1").arg(missionPath);
        QLOG_ERROR() << "JsonWaypointListFile::save() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    QJsonObject missionJObject;
    missionJObject["MissionName"]        = _missionDTO.getBasicMission().getName();
    missionJObject["InspectionPlanUUID"] = inspPlanUUID;

    if (_missionDTO.getLocalRoutesMap().size() > 1) {
        const QString msg = QString("There is more than one route in the mission: %1")
                                    .arg(_missionDTO.getBasicMission().getName());
        QLOG_ERROR() << "JsonWaypointListFile::save() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    const auto& currRouteDTO = _missionDTO.getLocalRoutesMap().first();

    QJsonArray waypointsJArray;
    for (const auto& waypointDTO : currRouteDTO.getWpsList()) {
        QJsonObject waypointJO;
        waypointJO["Index"]        = waypointDTO.getId();
        waypointJO["Type"]         = static_cast<int>(waypointDTO.getWpType());
        waypointJO["TaskUuid"]     = waypointDTO.getTaskUUID();
        waypointJO["AutoContinue"] = waypointDTO.getAutoContinue() ? 1 : 0;

        QJsonObject positionJO;
        positionJO["X"]        = QString::number(waypointDTO.getPosition().getX(), 'f', 6);
        positionJO["Y"]        = QString::number(waypointDTO.getPosition().getY(), 'f', 6);
        positionJO["Z"]        = QString::number(waypointDTO.getPosition().getZ(), 'f', 6);
        waypointJO["Position"] = positionJO;

        QJsonObject orientationJO;
        orientationJO["Qw"]       = QString::number(waypointDTO.getOrientation().getW(), 'f', 6);
        orientationJO["Qx"]       = QString::number(waypointDTO.getOrientation().getX(), 'f', 6);
        orientationJO["Qy"]       = QString::number(waypointDTO.getOrientation().getY(), 'f', 6);
        orientationJO["Qz"]       = QString::number(waypointDTO.getOrientation().getZ(), 'f', 6);
        waypointJO["Orientation"] = orientationJO;

        QJsonObject actionParametersJO;
        actionParametersJO["Param1"]   = QString::number(waypointDTO.getActionParameters().getParam1(), 'f', 6);
        actionParametersJO["Param2"]   = QString::number(waypointDTO.getActionParameters().getParam2(), 'f', 6);
        actionParametersJO["Param3"]   = QString::number(waypointDTO.getActionParameters().getParam3(), 'f', 6);
        actionParametersJO["Param4"]   = QString::number(waypointDTO.getActionParameters().getParam4(), 'f', 6);
        waypointJO["ActionParameters"] = actionParametersJO;

        waypointsJArray.append(waypointJO);
    }
    missionJObject["WaypointList"] = waypointsJArray;

    QJsonDocument missionJDoc(missionJObject);
    const auto    bytesWritten = missionFile.write(missionJDoc.toJson());
    if (bytesWritten == -1) {
        const QString msg = QString("An error ocurred writing json file: %1").arg(missionPath);
        QLOG_ERROR() << "JsonWaypointListFile::save() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }
    missionFile.close();
}

} // namespace gcs
