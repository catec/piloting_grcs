
#include "DownloadAlarmsListAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>

#include <QMetaObject>

namespace gcs {

DownloadAlarmsListAction::DownloadAlarmsListAction(ICommunications& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

DownloadAlarmsListAction::~DownloadAlarmsListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType DownloadAlarmsListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::DOWNLOAD_ALARMS_LIST;
}

void DownloadAlarmsListAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "DownloadAlarmsListAction::execute()";

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(_commsObj, "downloadAlarmsList", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "DownloadAlarmsListAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Download alarms list error - ") + tr(e.what()));
    }
}

} // namespace gcs
