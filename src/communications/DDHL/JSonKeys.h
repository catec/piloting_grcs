
#pragma once

#include <QJsonObject>
#include <QObject>

namespace gcs {

class JSonKeys : public QObject
{
    Q_OBJECT

  public:
    explicit JSonKeys(QObject* parent = nullptr);
    ~JSonKeys();

  public:
    void checkIfValidAssetFile(const QJsonObject&);
    void checkIfValidInspPlan(const QJsonObject&);
    void checkIfValidInspTask(const QJsonObject&);
    void checkIfValidInspTaskLocation(const QJsonObject&);
    void checkIfValidInspTaskAreaDimensions(const QJsonObject&);
    void checkIfValidInspTaskPointDimensions(const QJsonObject&);
    void checkIfValidInspTaskType(const QJsonObject&);
    void checkIfValidAsset(const QJsonObject&);
    void checkIfValidSite(const QJsonObject&);
    void checkIfValidBasicAsset(const QJsonObject&);
    void checkKeyExist(const QJsonObject&, const QString&);
};
} // namespace gcs
