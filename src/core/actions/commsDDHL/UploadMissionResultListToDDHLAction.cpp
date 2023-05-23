#include "UploadMissionResultListToDDHLAction.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <core/dtos/BasicStringListDTO.h>
#include <dataModel/DataModel.h>

#include <QMetaObject>

namespace gcs {

UploadMissionResultListToDDHLAction::UploadMissionResultListToDDHLAction(
        DDHLComms& comms, DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _comms(comms),
        _commsObj(dynamic_cast<QObject*>(&_comms)),
        _dataModel(dataModel),
        _missionResultsBusiness(dataModel.getMissionResultsDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UploadMissionResultListToDDHLAction::~UploadMissionResultListToDDHLAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UploadMissionResultListToDDHLAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPLOAD_MISSION_RESULT_LIST_TO_DDHL;
}

void UploadMissionResultListToDDHLAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringListDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringListDTO");
        }

        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QList<MissionResultDTO> listToUpload;
        _missionResultsBusiness.obtainMissionResultList(parsedDTO->getStringList(), listToUpload);

        QMetaObject::invokeMethod(
                _commsObj,
                "uploadMissionResultListToDDHL",
                Qt::BlockingQueuedConnection,
                Q_ARG(const CommsLinkDDHLDTO&, _dataModel.getCommsLinkDDHLDTO()),
                Q_ARG(QList<MissionResultDTO>, listToUpload),
                Q_ARG(const QString&, _dataModel.getMissionResultsDTO().getFolderPath()));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "- Error:" << e.what();

        sendWarningMsg(tr("Upload Mission Result List to DDHL error - ") + tr(e.what()));
    }
}

} // namespace gcs
