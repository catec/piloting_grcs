
#include "RemoveLoadedHomeAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

RemoveLoadedHomeAction::RemoveLoadedHomeAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _homeBusiness(dataModel.getMissionDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

RemoveLoadedHomeAction::~RemoveLoadedHomeAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType RemoveLoadedHomeAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::REMOVE_LOADED_HOME;
}

void RemoveLoadedHomeAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        _homeBusiness.removeLoadedHome();

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Remove loaded home error - ") + tr(e.what()));
    }
}
} // namespace gcs
