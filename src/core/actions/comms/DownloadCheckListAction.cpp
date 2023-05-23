
#include "DownloadCheckListAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>

#include <QMetaObject>

namespace gcs {

DownloadCheckListAction::DownloadCheckListAction(ICommunications& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

DownloadCheckListAction::~DownloadCheckListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType DownloadCheckListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::DOWNLOAD_CHECKLIST;
}

void DownloadCheckListAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "DownloadCheckListAction::execute()";

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(_commsObj, "downloadCheckList", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "DownloadCheckListAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Download checklist error - ") + tr(e.what()));
    }
}

} // namespace gcs
