
#include "UploadWaypointsListAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>
#include <dataModel/DataModel.h>

#include <QMetaObject>

namespace gcs {

UploadWaypointsListAction::UploadWaypointsListAction(DataModel& dataModel, ICommunications& comms, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UploadWaypointsListAction::~UploadWaypointsListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UploadWaypointsListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPLOAD_WAYPOINTS_LIST;
}

void UploadWaypointsListAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "UploadWaypointsListAction::execute()";

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(
                _commsObj,
                "sendWaypointsList",
                Qt::BlockingQueuedConnection,
                Q_ARG(QString, _dataModel.getInspectionPlanDTO().getUUID()),
                Q_ARG(quint32, _dataModel.getCurrentMissionResultDTO().getSyncId()),
                Q_ARG(MissionDTO, _dataModel.getMissionDTO()),
                Q_ARG(QList<InspectionTaskDTO>, _dataModel.getInspectionPlanDTO().getInspectionTaskList()));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UploadWaypointsListAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Upload waypoints list error - ") + tr(e.what()));
    }
}

} // namespace gcs
