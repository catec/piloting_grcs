
#include "DownloadHLActionsAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>

#include <QMetaObject>

namespace gcs {

DownloadHLActionsAction::DownloadHLActionsAction(ICommunications& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

DownloadHLActionsAction::~DownloadHLActionsAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType DownloadHLActionsAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::DOWNLOAD_HLACTION_LIST;
}

void DownloadHLActionsAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "DownloadHLActionsAction::execute()";

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(_commsObj, "downloadHighLevelActions", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "DownloadHLActionsAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Download high level actions error - ") + tr(e.what()));
    }
}

} // namespace gcs
