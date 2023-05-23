#include "JSonKeys.h"

#include <QsLog/QsLog.h>

#include <QJsonDocument>

namespace gcs {

JSonKeys::JSonKeys(QObject* parent) : QObject(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

JSonKeys::~JSonKeys()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void JSonKeys::checkIfValidAssetFile(const QJsonObject& fileJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(fileJObject, "UniqueID");
    checkKeyExist(fileJObject, "URL");
    checkKeyExist(fileJObject, "Size");
    checkKeyExist(fileJObject, "OriginalName");
    checkKeyExist(fileJObject, "CreationDatetime");
    checkKeyExist(fileJObject, "UpdateDatetime");
}

void JSonKeys::checkIfValidInspPlan(const QJsonObject& inspPlanJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(inspPlanJObject, "UniqueID");
    checkKeyExist(inspPlanJObject, "Name");
    checkKeyExist(inspPlanJObject, "Description");
    checkKeyExist(inspPlanJObject, "CreationDatetime");
    checkKeyExist(inspPlanJObject, "UpdateDatetime");

    checkKeyExist(inspPlanJObject, "Asset");
    checkKeyExist(inspPlanJObject, "InspectionTask");
}

void JSonKeys::checkIfValidInspTask(const QJsonObject& inspTaskJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(inspTaskJObject, "UniqueID");
    checkKeyExist(inspTaskJObject, "Name");
    checkKeyExist(inspTaskJObject, "Description");
    checkKeyExist(inspTaskJObject, "CreationDatetime");
    checkKeyExist(inspTaskJObject, "UpdateDatetime");
    checkKeyExist(inspTaskJObject, "InspectionLocation");
    checkKeyExist(inspTaskJObject, "InspectionType");
}

void JSonKeys::checkIfValidInspTaskLocation(const QJsonObject& inspTaskLocation)
{
    checkKeyExist(inspTaskLocation, "Subtype");
    checkKeyExist(inspTaskLocation, "Properties");
}

void JSonKeys::checkIfValidInspTaskAreaDimensions(const QJsonObject& inspTaskAreaDimensions)
{
    checkKeyExist(inspTaskAreaDimensions, "width");
    checkKeyExist(inspTaskAreaDimensions, "depth");
    checkKeyExist(inspTaskAreaDimensions, "height");
    checkKeyExist(inspTaskAreaDimensions, "repr");
}

void JSonKeys::checkIfValidInspTaskPointDimensions(const QJsonObject& inspTaskPointDimensions)
{
    checkKeyExist(inspTaskPointDimensions, "posX");
    checkKeyExist(inspTaskPointDimensions, "posY");
    checkKeyExist(inspTaskPointDimensions, "posZ");
}

void JSonKeys::checkIfValidInspTaskType(const QJsonObject& inspTaskType)
{
    checkKeyExist(inspTaskType, "UniqueID");
    checkKeyExist(inspTaskType, "Name");
    checkKeyExist(inspTaskType, "Description");
    checkKeyExist(inspTaskType, "Properties");
}

void JSonKeys::checkIfValidAsset(const QJsonObject& assetJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(assetJObject, "UniqueID");
    checkKeyExist(assetJObject, "Files");
    checkKeyExist(assetJObject, "Name");
    checkKeyExist(assetJObject, "Description");
    checkKeyExist(assetJObject, "UpdateDatetime");
    checkKeyExist(assetJObject, "CreationDatetime");
    checkKeyExist(assetJObject, "Properties");
}

void JSonKeys::checkIfValidSite(const QJsonObject& siteJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(siteJObject, "Asset");
    checkKeyExist(siteJObject, "CreationDatetime");
    checkKeyExist(siteJObject, "Description");
    checkKeyExist(siteJObject, "Name");
    checkKeyExist(siteJObject, "UniqueID");
    checkKeyExist(siteJObject, "UpdateDatetime");
    checkKeyExist(siteJObject, "site");
}

void JSonKeys::checkIfValidBasicAsset(const QJsonObject& basicAssetJObject)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    checkKeyExist(basicAssetJObject, "CreationDatetime");
    checkKeyExist(basicAssetJObject, "Description");
    checkKeyExist(basicAssetJObject, "Name");
    checkKeyExist(basicAssetJObject, "UniqueID");
    checkKeyExist(basicAssetJObject, "UpdateDatetime");
    checkKeyExist(basicAssetJObject, "Properties");
}
void JSonKeys::checkKeyExist(const QJsonObject& jsonObject, const QString& key)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (jsonObject.value(key) == QJsonValue::Undefined) {
        QJsonDocument doc(jsonObject);
        QString       strJson(doc.toJson(QJsonDocument::Indented));

        QLOG_DEBUG().noquote() << __PRETTY_FUNCTION__ << strJson;

        const QString msg = QString("Key does not exist: %1").arg(key);
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << msg;
        throw std::runtime_error(msg.toStdString());
    }
}

} // namespace gcs
