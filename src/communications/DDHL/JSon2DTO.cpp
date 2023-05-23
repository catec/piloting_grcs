#include "JSon2DTO.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>

#include <Eigen/Eigen>
#include <QJsonArray>
#include <QJsonObject>

namespace gcs {
JSon2DTO::JSon2DTO(QObject* parent) : QObject(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

JSon2DTO::~JSon2DTO()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

InspectionPlanDTO JSon2DTO::convertInspectionPlanJsonToDTO(const QJsonObject& inspPlanJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _jsonKeys.checkIfValidInspPlan(inspPlanJObject);

    InspectionPlanDTO inspPlanDTO;
    inspPlanDTO.getUUID()        = inspPlanJObject["UniqueID"].toString();
    inspPlanDTO.getName()        = inspPlanJObject["Name"].toString();
    inspPlanDTO.getDescription() = inspPlanJObject["Description"].toString();
    inspPlanDTO.getCreationDateTime()
            = QDateTime::fromString(inspPlanJObject["CreationDatetime"].toString(), Qt::ISODateWithMs);
    inspPlanDTO.getLastUpdateDateTime()
            = QDateTime::fromString(inspPlanJObject["UpdateDatetime"].toString(), Qt::ISODateWithMs);

    const QJsonObject assetJObject = inspPlanJObject["Asset"].toObject();
    _jsonKeys.checkIfValidAsset(assetJObject);
    inspPlanDTO.getAsset() = convertAssetJsonToDTO(assetJObject);

    const QJsonArray inspectionTasksJArray = inspPlanJObject["InspectionTask"].toArray();
    inspPlanDTO.getInspectionTaskList()    = convertInspectionTaskJsonArrayToList(inspectionTasksJArray);

    return inspPlanDTO;
}

SiteListDTO JSon2DTO::convertSiteListJsonToDTO(const QJsonObject& siteListJO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    SiteListDTO siteListDTO;

    for (const auto& siteKey : siteListJO.keys()) {
        auto siteJA = siteListJO[siteKey].toArray();

        for (const auto& siteJValue : siteJA) {
            auto siteJO = siteJValue.toObject();
            _jsonKeys.checkIfValidSite(siteJO);
            auto siteDTO = convertSiteJsonToDTO(siteJO);
            siteListDTO.getSiteList().append(siteDTO);
        }
    }

    return siteListDTO;
}

SiteDTO JSon2DTO::convertSiteJsonToDTO(const QJsonObject& siteJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    SiteDTO siteDTO;
    siteDTO.getName()                                 = siteJObject["site"].toString();
    siteDTO.getBasicInspectionPlan().getUUID()        = siteJObject["UniqueID"].toString();
    siteDTO.getBasicInspectionPlan().getDescription() = siteJObject["Description"].toString();
    siteDTO.getBasicInspectionPlan().getName()        = siteJObject["Name"].toString();
    siteDTO.getBasicInspectionPlan().getCreationDateTime()
            = QDateTime::fromString(siteJObject["CreationDatetime"].toString(), Qt::ISODateWithMs);
    siteDTO.getBasicInspectionPlan().getLastUpdateDateTime()
            = QDateTime::fromString(siteJObject["UpdateDatetime"].toString(), Qt::ISODateWithMs);

    auto basicAssetJO = siteJObject["Asset"].toObject();
    _jsonKeys.checkIfValidBasicAsset(basicAssetJO);
    siteDTO.getBasicAsset().getUUID()        = basicAssetJO["UniqueID"].toString();
    siteDTO.getBasicAsset().getDescription() = basicAssetJO["Description"].toString();
    siteDTO.getBasicAsset().getName()        = basicAssetJO["Name"].toString();
    siteDTO.getBasicAsset().getCreationDateTime()
            = QDateTime::fromString(basicAssetJO["CreationDatetime"].toString(), Qt::ISODateWithMs);
    siteDTO.getBasicAsset().getLastUpdateDateTime()
            = QDateTime::fromString(basicAssetJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);

    auto propertiesJO = basicAssetJO["Properties"].toObject();
    _jsonKeys.checkKeyExist(propertiesJO, "subtype");
    siteDTO.getSubtype() = propertiesJO["subtype"].toString();

    return siteDTO;
}

AssetDTO JSon2DTO::convertAssetJsonToDTO(const QJsonObject& assetJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    AssetDTO assetDTO;
    assetDTO.getUUID() = assetJObject["UniqueID"].toString();
    assetDTO.getName() = assetJObject["Name"].toString();
    assetDTO.getCreationDateTime()
            = QDateTime::fromString(assetJObject["CreationDatetime"].toString(), Qt::ISODateWithMs);
    assetDTO.getLastUpdateDateTime()
            = QDateTime::fromString(assetJObject["UpdateDatetime"].toString(), Qt::ISODateWithMs);

    const QJsonArray assetFilesJArray = assetJObject["Files"].toArray();
    const auto       fileList         = convertAssetFileJsonArrayToList(assetFilesJArray);
    if (fileList.getDownloadedFileList().isEmpty()) {
        throw std::runtime_error("Downloaded file list is empty");
    }

    assetDTO.getDownloadedFileList() = fileList;
    return assetDTO;
}

DownloadedFileListDTO JSon2DTO::convertAssetFileJsonArrayToList(const QJsonArray& assetFilesJArray)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    DownloadedFileListDTO fileList;

    for (const auto& file : assetFilesJArray) {
        QJsonObject fileJO = file.toObject();
        _jsonKeys.checkIfValidAssetFile(fileJO);

        FileDTO fileDTO;
        fileDTO.getUUID()         = fileJO["UniqueID"].toString();
        fileDTO.getUrl()          = fileJO["URL"].toString();
        fileDTO.getOriginalName() = fileJO["OriginalName"].toString();
        if (!fileDTO.getOriginalName().endsWith(".ply") && !fileDTO.getOriginalName().endsWith(".pcd")
            && !fileDTO.getOriginalName().endsWith(".stl")) {
            QLOG_WARN() << "Invalid downloaded original file: " << fileDTO.getOriginalName()
                        << " It must be a .ply, .pcd or .stl file";
            continue;
        }

        fileDTO.getSize()             = fileJO["Size"].toString().toInt();
        fileDTO.getCreationDateTime() = QDateTime::fromString(fileJO["CreationDatetime"].toString(), Qt::ISODateWithMs);
        fileDTO.getLastUpdateDateTime() = QDateTime::fromString(fileJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);
        fileList.getDownloadedFileList().append(fileDTO);
    }
    return fileList;
}

QList<InspectionTaskDTO> JSon2DTO::convertInspectionTaskJsonArrayToList(const QJsonArray& inspectionTasksJArray)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QList<InspectionTaskDTO> inspTasksList;
    for (const auto& inspectonTask : inspectionTasksJArray) {
        QJsonObject inspTaskJO = inspectonTask.toObject();
        _jsonKeys.checkIfValidInspTask(inspTaskJO);

        InspectionTaskDTO inspTaskDTO;
        inspTaskDTO.getUUID()        = inspTaskJO["UniqueID"].toString();
        inspTaskDTO.getName()        = inspTaskJO["Name"].toString();
        inspTaskDTO.getDescription() = inspTaskJO["Description"].toString();
        inspTaskDTO.getCreationDateTime()
                = QDateTime::fromString(inspTaskJO["CreationDatetime"].toString(), Qt::ISODateWithMs);
        inspTaskDTO.getLastUpdateDateTime()
                = QDateTime::fromString(inspTaskJO["UpdateDatetime"].toString(), Qt::ISODateWithMs);

        const QJsonObject inspTaskLocationJO = inspTaskJO["InspectionLocation"].toObject();
        _jsonKeys.checkIfValidInspTaskLocation(inspTaskLocationJO);

        const auto inspTaskDimensionsJO = inspTaskLocationJO["Properties"].toObject();
        _jsonKeys.checkKeyExist(inspTaskDimensionsJO, "objectType");

        /// \note. When we receive an inspection part, we treat it as an inspection point.
        const QString readedType = inspTaskDimensionsJO["objectType"].toString();
        if (QString::compare(readedType, "InspectionPoint", Qt::CaseInsensitive) == 0
            || QString::compare(readedType, "InspectionPart", Qt::CaseInsensitive) == 0) {
            inspTaskDTO.getLocationType() = InspectionTaskLocationType::POINT;
        } else if (QString::compare(readedType, "InspectionArea", Qt::CaseInsensitive) == 0) {
            inspTaskDTO.getLocationType() = InspectionTaskLocationType::AREA;
            _jsonKeys.checkIfValidInspTaskAreaDimensions(inspTaskDimensionsJO);
            inspTaskDTO.getWidth()  = inspTaskDimensionsJO["width"].toDouble();
            inspTaskDTO.getDepth()  = inspTaskDimensionsJO["depth"].toDouble();
            inspTaskDTO.getHeight() = inspTaskDimensionsJO["height"].toDouble();

            const QJsonObject reprJO = inspTaskDimensionsJO["repr"].toObject();
            _jsonKeys.checkKeyExist(reprJO, "matrix");

            const QJsonArray matrixJA = reprJO["matrix"].toArray();
            if (matrixJA.size() != 16) {
                throw std::runtime_error("Invalid repr matrix size");
            }

            /// \note. Column-major order arrived matrix
            Eigen::Matrix3f m(3, 3);
            m << matrixJA[0].toDouble(), matrixJA[4].toDouble(), matrixJA[8].toDouble(), matrixJA[1].toDouble(),
                    matrixJA[5].toDouble(), matrixJA[9].toDouble(), matrixJA[2].toDouble(), matrixJA[6].toDouble(),
                    matrixJA[10].toDouble();
            Eigen::Quaternionf q(m);
            inspTaskDTO.getOrientation().getX() = q.x();
            inspTaskDTO.getOrientation().getY() = q.y();
            inspTaskDTO.getOrientation().getZ() = q.z();
            inspTaskDTO.getOrientation().getW() = q.w();
        } else {
            throw std::runtime_error("Invalid inspection task type readed from DDHL");
        }

        _jsonKeys.checkIfValidInspTaskPointDimensions(inspTaskDimensionsJO);
        inspTaskDTO.getPosition().getX() = inspTaskDimensionsJO["posX"].toDouble();
        inspTaskDTO.getPosition().getY() = inspTaskDimensionsJO["posY"].toDouble();
        inspTaskDTO.getPosition().getZ() = inspTaskDimensionsJO["posZ"].toDouble();

        const auto inspTaskType = inspTaskJO["InspectionType"].toObject();
        _jsonKeys.checkIfValidInspTaskType(inspTaskType);
        inspTaskDTO.getType().getUUID()        = inspTaskType["UniqueID"].toString();
        inspTaskDTO.getType().getName()        = inspTaskType["Name"].toString();
        inspTaskDTO.getType().getDescription() = inspTaskType["Description"].toString();
        inspTaskDTO.getType().getProperties()  = inspTaskType["Properties"].toString();

        inspTasksList.append(inspTaskDTO);
    }

    return inspTasksList;
}
} // namespace gcs
