
#include "CancelAssetDownloadAction.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <config.h>

namespace gcs {

CancelAssetDownloadAction::CancelAssetDownloadAction(DDHLComms& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

CancelAssetDownloadAction::~CancelAssetDownloadAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType CancelAssetDownloadAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::CANCEL_ASSET_DOWNLOAD;
}

void CancelAssetDownloadAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        sendInfoMsg("Cancel asset download");
        QMetaObject::invokeMethod(_commsObj, "abortServerOperation", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Cancel Asset download error - ") + tr(e.what()));
    }
}

} // namespace gcs
