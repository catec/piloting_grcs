#include "InspectionPlanBusiness.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>
#include <config.h>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "../dtos/InspectionPlanDTO.h"

namespace gcs {

InspectionPlanBusiness::InspectionPlanBusiness(InspectionPlanDTO& inspectionPlanDTO) :
        _inspectionPlanDTO(inspectionPlanDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

InspectionPlanBusiness::~InspectionPlanBusiness()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void InspectionPlanBusiness::saveInspectionPlan()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QDir inspectionPlansDir(QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/inspectionPlans/").arg(PROJECT_VERSION_MAJOR).arg(PROJECT_VERSION_MINOR)));
    if (!QDir().mkpath(inspectionPlansDir.path())) {
        const auto msg
                = QString("Main inspection plans directory has not been created: %1").arg(inspectionPlansDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    QDir currentInspectionPlanDir(
            inspectionPlansDir.absoluteFilePath(QString("inspectionPlan_%1").arg(_inspectionPlanDTO.getUUID())));
    if (!QDir().mkpath(currentInspectionPlanDir.path())) {
        const auto msg
                = QString("Inspection plan directory has not been created: %1").arg(currentInspectionPlanDir.path());
        throw std::runtime_error(msg.toStdString());
    }

    saveInspectionPlanAsJson(currentInspectionPlanDir.absoluteFilePath("inspection_plan_info.json"));
}

void InspectionPlanBusiness::loadInspectionPlanFromLocalPath(const QString& filePath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QFile inspectionPlanInfoFile(filePath);
    if (!inspectionPlanInfoFile.open(QIODevice::ReadOnly)) {
        const auto msg = QString("Could not read inspection plan information file from: %1")
                                 .arg(inspectionPlanInfoFile.fileName());
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }

    auto              inspectionPlanJDoc(QJsonDocument::fromJson(inspectionPlanInfoFile.readAll()));
    const QJsonObject rootJO = inspectionPlanJDoc.object();
    _jsonCheckKeys.checkExistInspPlanKeys(rootJO);

    _inspectionPlanDTO.getUUID()        = rootJO["UniqueID"].toString();
    _inspectionPlanDTO.getName()        = rootJO["Name"].toString();
    _inspectionPlanDTO.getDescription() = rootJO["Description"].toString();
    _inspectionPlanDTO.getCreationDateTime()
            = QDateTime::fromString(rootJO["CreationDatetime"].toString(), Qt::ISODateWithMs);
    _inspectionPlanDTO.getLastUpdateDateTime()
            = QDateTime::fromString(rootJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);

    const QJsonObject assetJO = rootJO["Asset"].toObject();
    _jsonCheckKeys.checkExistAssetKeys(assetJO);
    _inspectionPlanDTO.getAsset().getUUID()                 = assetJO["UniqueID"].toString();
    _inspectionPlanDTO.getAsset().getLoadedFile().getPath() = assetJO["Path"].toString();
    _inspectionPlanDTO.getAsset().getName()                 = assetJO["Name"].toString();
    _inspectionPlanDTO.getAsset().getCreationDateTime()
            = QDateTime::fromString(assetJO["CreationDatetime"].toString(), Qt::ISODateWithMs);
    ;
    _inspectionPlanDTO.getAsset().getLastUpdateDateTime()
            = QDateTime::fromString(assetJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);

    _inspectionPlanDTO.getInspectionTaskList().clear();
    QJsonArray inspectionTasks = rootJO["InspectionTasks"].toArray();

    for (const auto inspectionTask : inspectionTasks) {
        InspectionTaskDTO inspectionTaskDTO;

        const QJsonObject inspectionTaskJO = inspectionTask.toObject();
        _jsonCheckKeys.checkExistInspTaskKeys(inspectionTaskJO);

        inspectionTaskDTO.getUUID()        = inspectionTaskJO["UniqueID"].toString();
        inspectionTaskDTO.getName()        = inspectionTaskJO["Name"].toString();
        inspectionTaskDTO.getDescription() = inspectionTaskJO["Description"].toString();
        inspectionTaskDTO.getCreationDateTime()
                = QDateTime::fromString(inspectionTaskJO["CreationDatetime"].toString(), Qt::ISODateWithMs);
        inspectionTaskDTO.getLastUpdateDateTime()
                = QDateTime::fromString(inspectionTaskJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);

        const QJsonObject locationJO        = inspectionTaskJO["InspectionLocation"].toObject();
        inspectionTaskDTO.getLocationType() = toInspectionTaskLocationType(locationJO["SubType"].toString());

        const QJsonObject propertiesJO         = locationJO["Properties"].toObject();
        inspectionTaskDTO.getPosition().getX() = propertiesJO["posX"].toString().toDouble();
        inspectionTaskDTO.getPosition().getY() = propertiesJO["posY"].toString().toDouble();
        inspectionTaskDTO.getPosition().getZ() = propertiesJO["posZ"].toString().toDouble();

        if (inspectionTaskDTO.getLocationType() == InspectionTaskLocationType::AREA) {
            inspectionTaskDTO.getWidth()  = propertiesJO["width"].toString().toDouble();
            inspectionTaskDTO.getDepth()  = propertiesJO["depth"].toString().toDouble();
            inspectionTaskDTO.getHeight() = propertiesJO["height"].toString().toDouble();

            float roll, pitch, yaw;
            roll  = static_cast<float>(propertiesJO["roll"].toString().toDouble());
            pitch = static_cast<float>(propertiesJO["pitch"].toString().toDouble());
            yaw   = static_cast<float>(propertiesJO["yaw"].toString().toDouble());

            auto areaOrientation                      = degToQuaternion(roll, pitch, yaw);
            inspectionTaskDTO.getOrientation().getW() = areaOrientation[0];
            inspectionTaskDTO.getOrientation().getX() = areaOrientation[1];
            inspectionTaskDTO.getOrientation().getY() = areaOrientation[2];
            inspectionTaskDTO.getOrientation().getZ() = areaOrientation[3];
        }

        const auto inspTaskType                      = inspectionTaskJO["InspectionType"].toObject();
        inspectionTaskDTO.getType().getUUID()        = inspTaskType["UniqueID"].toString();
        inspectionTaskDTO.getType().getName()        = inspTaskType["Name"].toString();
        inspectionTaskDTO.getType().getDescription() = inspTaskType["Description"].toString();
        inspectionTaskDTO.getType().getProperties()  = inspTaskType["Properties"].toString();

        _inspectionPlanDTO.getInspectionTaskList().push_back(inspectionTaskDTO);
    }

    inspectionPlanInfoFile.close();
}

InspectionTaskLocationType InspectionPlanBusiness::toInspectionTaskLocationType(const QString& key)
{
    InspectionTaskLocationType type;

    if (key == ToString(InspectionTaskLocationType::AREA)) {
        type = InspectionTaskLocationType::AREA;
    } else if (key == ToString(InspectionTaskLocationType::PART)) {
        type = InspectionTaskLocationType::PART;
    } else if (key == ToString(InspectionTaskLocationType::POINT)) {
        type = InspectionTaskLocationType::POINT;
    } else {
        const auto msg = QString("Key: '%1' does not match with any Inspection Task Type").arg(key);
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }

    return type;
}

void InspectionPlanBusiness::saveInspectionPlanAsJson(const QString& inspectionPlanPath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QFile inspectionPlanInfoFile(inspectionPlanPath);
    if (!inspectionPlanInfoFile.open(QIODevice::WriteOnly)) {
        const auto msg = QString("Could not create file to save inspection plan information: %1")
                                 .arg(inspectionPlanInfoFile.fileName());
        QLOG_ERROR() << __PRETTY_FUNCTION__ << " - " << msg;
        throw std::runtime_error(msg.toStdString());
    }

    QJsonObject rootJO;
    rootJO["UniqueID"]         = _inspectionPlanDTO.getUUID();
    rootJO["Name"]             = _inspectionPlanDTO.getName();
    rootJO["Description"]      = _inspectionPlanDTO.getDescription();
    rootJO["CreationDatetime"] = _inspectionPlanDTO.getCreationDateTime().toString(Qt::ISODateWithMs);
    rootJO["UpdateDatetime"]   = _inspectionPlanDTO.getLastUpdateDateTime().toString(Qt::ISODateWithMs);

    QJsonObject assetJO;
    assetJO["UniqueID"]         = _inspectionPlanDTO.getAsset().getUUID();
    assetJO["Name"]             = _inspectionPlanDTO.getAsset().getName();
    assetJO["CreationDatetime"] = _inspectionPlanDTO.getAsset().getCreationDateTime().toString(Qt::ISODateWithMs);
    assetJO["UpdateDatetime"]   = _inspectionPlanDTO.getAsset().getLastUpdateDateTime().toString(Qt::ISODateWithMs);
    assetJO["Path"]             = _inspectionPlanDTO.getAsset().getLoadedFile().getPath();

    rootJO["Asset"] = assetJO;

    QJsonArray inspectionTasks;
    for (const auto inspectionTask : _inspectionPlanDTO.getInspectionTaskList()) {
        QJsonObject inspectionTaskJO;
        inspectionTaskJO["UniqueID"]         = inspectionTask.getUUID();
        inspectionTaskJO["Name"]             = inspectionTask.getName();
        inspectionTaskJO["Description"]      = inspectionTask.getDescription();
        inspectionTaskJO["CreationDatetime"] = inspectionTask.getCreationDateTime().toString(Qt::ISODateWithMs);
        inspectionTaskJO["UpdateDatetime"]
                = _inspectionPlanDTO.getAsset().getLastUpdateDateTime().toString(Qt::ISODateWithMs);

        QJsonObject locationJO;
        locationJO["SubType"] = ToString(inspectionTask.getLocationType());

        QJsonObject propertiesJO;
        propertiesJO["posX"] = QString::number(inspectionTask.getPosition().getX(), 'f', 6);
        propertiesJO["posY"] = QString::number(inspectionTask.getPosition().getY(), 'f', 6);
        propertiesJO["posZ"] = QString::number(inspectionTask.getPosition().getZ(), 'f', 6);

        if (inspectionTask.getLocationType() == InspectionTaskLocationType::AREA) {
            propertiesJO["width"]  = QString::number(inspectionTask.getWidth(), 'f', 6);
            propertiesJO["depth"]  = QString::number(inspectionTask.getDepth(), 'f', 6);
            propertiesJO["height"] = QString::number(inspectionTask.getHeight(), 'f', 6);

            auto eaDeg = ToEulerAnglesDeg(
                    inspectionTask.getOrientation().getW(),
                    inspectionTask.getOrientation().getX(),
                    inspectionTask.getOrientation().getY(),
                    inspectionTask.getOrientation().getZ());
            propertiesJO["roll"]  = QString::number(eaDeg[0], 'f', 6);
            propertiesJO["pitch"] = QString::number(eaDeg[1], 'f', 6);
            propertiesJO["yaw"]   = QString::number(eaDeg[2], 'f', 6);
        }

        locationJO["Properties"]               = propertiesJO;
        inspectionTaskJO["InspectionLocation"] = locationJO;

        QJsonObject typeJO;
        typeJO["UniqueID"]                 = inspectionTask.getType().getUUID();
        typeJO["Name"]                     = inspectionTask.getType().getName();
        typeJO["Description"]              = inspectionTask.getType().getDescription();
        typeJO["Properties"]               = inspectionTask.getType().getProperties();
        inspectionTaskJO["InspectionType"] = typeJO;

        inspectionTasks.push_back(inspectionTaskJO);
    }

    rootJO["InspectionTasks"] = inspectionTasks;

    QJsonDocument inspectionPlanJDoc(rootJO);
    inspectionPlanInfoFile.write(inspectionPlanJDoc.toJson());
    inspectionPlanInfoFile.close();
}

} // namespace gcs
