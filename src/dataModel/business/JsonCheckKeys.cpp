#include "JsonCheckKeys.h"

#include <QsLog/QsLog.h>

#include <QJsonObject>

namespace gcs {

JsonCheckKeys::JsonCheckKeys()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

JsonCheckKeys::~JsonCheckKeys()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void JsonCheckKeys::checkExistWaypointListKeys(const QJsonObject& wptsListJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(wptsListJO, "MissionName");
    checkKeyExist(wptsListJO, "InspectionPlanUUID");
    checkKeyExist(wptsListJO, "WaypointList");
}

void JsonCheckKeys::checkExistWaypointKeys(const QJsonObject& waypointJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(waypointJO, "Index");
    checkKeyExist(waypointJO, "Type");
    checkKeyExist(waypointJO, "TaskUuid");
    checkKeyExist(waypointJO, "AutoContinue");

    checkKeyExist(waypointJO, "Position");
    checkKeyExist(waypointJO, "Orientation");
    checkKeyExist(waypointJO, "ActionParameters");
}

void JsonCheckKeys::checkExistPositionKeys(const QJsonObject& positionJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(positionJO, "X");
    checkKeyExist(positionJO, "Y");
    checkKeyExist(positionJO, "Z");
}

void JsonCheckKeys::checkExistOrientationKeys(const QJsonObject& orientationJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(orientationJO, "Qw");
    checkKeyExist(orientationJO, "Qx");
    checkKeyExist(orientationJO, "Qy");
    checkKeyExist(orientationJO, "Qz");
}

void JsonCheckKeys::checkExistInspPlanBasicInfoKeys(const QJsonObject& basicInfoJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(basicInfoJO, "Name");
    checkKeyExist(basicInfoJO, "SyncId");
    checkKeyExist(basicInfoJO, "Description");
    checkKeyExist(basicInfoJO, "CreationTimeStamp");
    checkKeyExist(basicInfoJO, "MissionTasks");
}

void JsonCheckKeys::checkExistInspPlanKeys(const QJsonObject& inspPlanJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(inspPlanJO, "Name");
    checkKeyExist(inspPlanJO, "Description");
    checkKeyExist(inspPlanJO, "UniqueID");
    checkKeyExist(inspPlanJO, "CreationDatetime");
    checkKeyExist(inspPlanJO, "UpdateDatetime");
    checkKeyExist(inspPlanJO, "InspectionTasks");
    checkKeyExist(inspPlanJO, "Asset");
}

void JsonCheckKeys::checkExistInspTaskKeys(const QJsonObject& inspTaskJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(inspTaskJO, "Name");
    checkKeyExist(inspTaskJO, "Description");
    checkKeyExist(inspTaskJO, "UniqueID");
    checkKeyExist(inspTaskJO, "CreationDatetime");
    checkKeyExist(inspTaskJO, "UpdateDatetime");
    checkKeyExist(inspTaskJO, "InspectionLocation");
    checkKeyExist(inspTaskJO, "InspectionType");
}

void JsonCheckKeys::checkExistAssetKeys(const QJsonObject& assetJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(assetJO, "Name");
    checkKeyExist(assetJO, "UniqueID");
    checkKeyExist(assetJO, "CreationDatetime");
    checkKeyExist(assetJO, "UpdateDatetime");
    checkKeyExist(assetJO, "Path");
}

void JsonCheckKeys::checkExistActionParametersKeys(const QJsonObject& actionParametersJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(actionParametersJO, "Param1");
    checkKeyExist(actionParametersJO, "Param2");
    checkKeyExist(actionParametersJO, "Param3");
    checkKeyExist(actionParametersJO, "Param4");
}

void JsonCheckKeys::checkExistMissionTaskKeys(const QJsonObject& missionTaskJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(missionTaskJO, "InspTaskUuid");
    checkKeyExist(missionTaskJO, "Name");
    checkKeyExist(missionTaskJO, "Description");
    checkKeyExist(missionTaskJO, "CreationTimeStamp");
    checkKeyExist(missionTaskJO, "LastUpdateTimeStamp");
}

void JsonCheckKeys::checkKeyExist(const QJsonObject& jsonObject, const QString& key)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (jsonObject.value(key) == QJsonValue::Undefined) {
        const QString msg = QString("Key does not exist: %1").arg(key);
        QLOG_ERROR() << "JsonCheckKeys::checkKeyExist() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }
}

} // namespace gcs
