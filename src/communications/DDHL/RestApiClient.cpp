#include "RestApiClient.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>
#include <common/Types.h>

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QNetworkReply>
#include <iostream>

namespace gcs {

RestApiClient::RestApiClient(QObject* parent) : QObject(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

RestApiClient::~RestApiClient()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void RestApiClient::getInspectionPlanJSonResult(Parameters params)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString       url        = params.url_str;
    const QString httpMethod = Literals::getMethod;

    if (!params.id_str.isEmpty()) {
        url = params.url_str + "/" + params.id_str;
    }

    QLOG_INFO() << __PRETTY_FUNCTION__ << "Target url of " << httpMethod << " to RestAPI:" << url;

    HttpRequestInput input(url, httpMethod);
    input.add_auth(params.username, params.password);

    if (_worker) {
        _worker.reset();
    }

    _worker = std::make_unique<HttpRequestWorker>(this);
    // clang-format off
    connect(_worker.get(), &HttpRequestWorker::sendConsoleMsg,
            this,          &RestApiClient::sendConsoleMsg);
    connect(this,          &RestApiClient::abort,
            _worker.get(), &HttpRequestWorker::abort);

    connect(_worker.get(), &HttpRequestWorker::executionFinished,
            this,          &RestApiClient::handleGetInspectionPlanJSonResult);
    // clang-format on

    _worker->execute(&input);
}

void RestApiClient::getPCDResult(Parameters params)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString       url        = params.url_str;
    const QString httpMethod = Literals::getMethod;

    if (!params.id_str.isEmpty()) {
        url = params.url_str + "/" + params.id_str;
    }

    QLOG_INFO() << __PRETTY_FUNCTION__ << "Target url of " << httpMethod << " to RestAPI:" << url;

    HttpRequestInput input(url, httpMethod);
    input.add_auth(params.username, params.password);

    if (_worker) {
        _worker.reset();
    }

    _worker = std::make_unique<HttpRequestWorker>(this);
    // clang-format off
    connect(_worker.get(), &HttpRequestWorker::sendConsoleMsg,
            this,          &RestApiClient::sendConsoleMsg);
    connect(this,          &RestApiClient::abort,
            _worker.get(), &HttpRequestWorker::abort);
    connect(_worker.get(), &HttpRequestWorker::executionFinished,
            this,          &RestApiClient::handleGetPCDResult);
    connect(_worker.get(), &HttpRequestWorker::updateDownloadProgress,
            this,          &RestApiClient::updateDownloadProgress);
    // clang-format on

    _worker->execute(&input);
}

void RestApiClient::getSiteListJSonResult(Parameters params)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString       url        = params.url_str;
    const QString httpMethod = Literals::getMethod;

    if (!params.id_str.isEmpty()) {
        url = params.url_str + "/" + params.id_str;
    }

    QLOG_INFO() << __PRETTY_FUNCTION__ << "Target url of " << httpMethod << " to RestAPI:" << url;

    HttpRequestInput input(url, httpMethod);
    input.add_auth(params.username, params.password);

    if (_worker) {
        _worker.reset();
    }

    _worker = std::make_unique<HttpRequestWorker>(this);
    // clang-format off
    connect(_worker.get(), &HttpRequestWorker::sendConsoleMsg,
            this,          &RestApiClient::sendConsoleMsg);
    connect(this,          &RestApiClient::abort,
            _worker.get(), &HttpRequestWorker::abort);

    connect(_worker.get(), &HttpRequestWorker::executionFinished,
             this,         &RestApiClient::handleGetSiteListJSonResult);
    // clang-format on

    _worker->execute(&input);
}

void RestApiClient::postResult(Parameters params, const QString& filePathToPost)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString       url        = params.url_str;
    const QString httpMethod = Literals::postMethod;

    if (!params.id_str.isEmpty()) {
        url = params.url_str + "/" + params.id_str;
    }

    QLOG_INFO() << __PRETTY_FUNCTION__ << "Target url of " << httpMethod << " to RestAPI:" << url;

    HttpRequestInput input(url, httpMethod);
    input.add_auth(params.username, params.password);

    if (_worker) {
        _worker.reset();
    }

    _worker = std::make_unique<HttpRequestWorker>(this);
    // clang-format off
    connect(_worker.get(), &HttpRequestWorker::sendConsoleMsg,
            this,          &RestApiClient::sendConsoleMsg);
    connect(this,          &RestApiClient::abort,
            _worker.get(), &HttpRequestWorker::abort);
    connect(_worker.get(), SIGNAL(executionFinished(HttpRequestWorker*)),
            this,          SLOT(handlePostResult(HttpRequestWorker*)));
    // clang-format on

    QString missionName = params.missionName;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Sending mission: " << filePathToPost << " with name: " << missionName;

    input.add_file("files", filePathToPost, missionName, "application/json");

    /// \note. This is not the best solution but idk how to do it better.
    _lastMissionUploadedName = missionName;

    _worker->execute(&input);
}

void RestApiClient::handlePostResult(HttpRequestWorker* worker)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        QLOG_INFO() << __PRETTY_FUNCTION__ << "Post request response: " << worker->response;

        if (worker->error_type != QNetworkReply::NoError) {
            throw std::runtime_error(
                    worker->error_str.toStdString() + " | Server Response: " + worker->response.toStdString());
        }

        QJsonParseError* e            = nullptr;
        QJsonDocument    jsonResponse = QJsonDocument::fromJson(worker->response, e);
        if (e != nullptr) {
            throw std::runtime_error(e->errorString().toStdString());
        }

        if (jsonResponse.isEmpty()) {
            throw std::runtime_error(QString(worker->response).toStdString());
        }

        QLOG_DEBUG().noquote() << __PRETTY_FUNCTION__ << jsonResponse.toJson(QJsonDocument::Indented);

        /// \todo. Important know how to get the name or similar of the uploaded mission
        if (!_lastMissionUploadedName.isEmpty()) {
            Q_EMIT successfullyUploadedMissionACK(_lastMissionUploadedName);
        }
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        Q_EMIT finishProgressBar();

        _lastMissionUploadedName.clear();

        Q_EMIT sendPopupMsg(
                tr("Handle RestAPI POST Result"),
                MsgType::WARNING,
                tr("The uploaded data is invalid due to: ") + tr(e.what()));
    }
}

void RestApiClient::handleGetSiteListJSonResult(HttpRequestWorker* worker)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (worker->error_type != QNetworkReply::NoError) {
            throw std::runtime_error(worker->error_str.toStdString());
        }

        QJsonParseError* e            = nullptr;
        QJsonDocument    jsonResponse = QJsonDocument::fromJson(worker->response, e);
        QLOG_DEBUG().noquote() << __PRETTY_FUNCTION__ << jsonResponse.toJson(QJsonDocument::Indented);
        if (e != nullptr) {
            throw std::runtime_error(e->errorString().toStdString());
        }

        const auto sitesJO = jsonResponse.object();

        if (sitesJO.empty()) {
            throw std::runtime_error("Invalid inspection plans sites data");
        }

        auto dto         = _json2DTO.convertSiteListJsonToDTO(sitesJO);
        auto siteListDTO = createSharedDTO<SiteListDTO>();
        *siteListDTO     = dto;

        Q_EMIT downloadedSiteList(siteListDTO);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        Q_EMIT finishProgressBar();

        Q_EMIT sendPopupMsg(
                tr("Handle RestAPI GET Result"),
                MsgType::WARNING,
                tr("The downloaded data is invalid due to: ") + tr(e.what()));
    }
}

void RestApiClient::handleGetInspectionPlanJSonResult(HttpRequestWorker* worker)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (worker->error_type != QNetworkReply::NoError) {
            throw std::runtime_error(worker->error_str.toStdString());
        }

        QJsonParseError* e            = nullptr;
        QJsonDocument    jsonResponse = QJsonDocument::fromJson(worker->response, e);
        if (e != nullptr) {
            throw std::runtime_error(e->errorString().toStdString());
        }

        const QJsonObject inspPlanJObject = jsonResponse.array().first().toObject();

        if (inspPlanJObject.empty()) {
            throw std::runtime_error("Invalid UUID or empty Inspection Plan");
        }

        QJsonDocument document(inspPlanJObject);
        QByteArray    jsonData = document.toJson(QJsonDocument::Indented);
        QLOG_DEBUG().noquote() << __PRETTY_FUNCTION__ << jsonData;

        auto dto                 = _json2DTO.convertInspectionPlanJsonToDTO(inspPlanJObject);
        auto receivedInspPlanDTO = createSharedDTO<InspectionPlanDTO>();
        *receivedInspPlanDTO     = dto;

        Q_EMIT downloadedInspectionPlan(receivedInspPlanDTO);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        Q_EMIT finishProgressBar();

        Q_EMIT sendPopupMsg(
                tr("Handle RestAPI GET Result"),
                MsgType::WARNING,
                tr("The downloaded data is invalid due to: ") + tr(e.what()));
    }
}

void RestApiClient::handleGetPCDResult(HttpRequestWorker* worker)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (worker->error_type != QNetworkReply::NoError) {
            throw std::runtime_error(worker->error_str.toStdString());
        }

        auto pcdFileName = QString(QTime::currentTime().toString("hh:mm:ss"));

        QFile file(QDir::tempPath() + QString("/%1.bin").arg(pcdFileName));
        if (!file.open(QIODevice::WriteOnly)) {
            auto msg = QString("Cannot create temporary file %1").arg(file.fileName());
            throw std::runtime_error("cannot open file");
        }
        file.write(worker->response);

        Q_EMIT downloadedAssetFile(file);
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        Q_EMIT finishProgressBar();

        Q_EMIT sendConsoleMsg(MsgType::WARNING, tr("The downloaded data is invalid due to: ") + tr(e.what()));
    }
}

} // namespace gcs
