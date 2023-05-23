
#include "DownloadInspPlanFromDDHLAction.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/DataModel.h>

#include <QMetaObject>

namespace gcs {

DownloadInspPlanFromDDHLAction::DownloadInspPlanFromDDHLAction(
        DDHLComms& comms, DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

DownloadInspPlanFromDDHLAction::~DownloadInspPlanFromDDHLAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType DownloadInspPlanFromDDHLAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::DOWNLOAD_INSP_PLAN_FROM_DDHL;
}

void DownloadInspPlanFromDDHLAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        QMetaObject::invokeMethod(
                _commsObj,
                "downloadInspectionPlanFromDDHL",
                Qt::BlockingQueuedConnection,
                Q_ARG(const CommsLinkDDHLDTO&, _dataModel.getCommsLinkDDHLDTO()),
                Q_ARG(const QString&, parsedDTO->getString()));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Download data from DDHL error - ") + tr(e.what()));
    }
}

} // namespace gcs
