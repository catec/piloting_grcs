
#include "UpdateLoadedHomeAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
UpdateLoadedHomeAction::UpdateLoadedHomeAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _homeBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateLoadedHomeAction::~UpdateLoadedHomeAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateLoadedHomeAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_LOADED_HOME;
}

void UpdateLoadedHomeAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto homeDTO = dto.dynamicCast<HomeDTO>();
        if (!homeDTO) {
            throw std::runtime_error("DTO type should be HomeDTO");
        }

        _missionStatusBusiness.changeStatus(MissionStatusType::WithChanges);
        auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
        *missionStatusDTO     = _dataModel.getMissionStatusDTO();
        Q_EMIT sendDTO(missionStatusDTO);

        _homeBusiness.updateLoadedHome(*homeDTO);

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update loaded home error - ") + tr(e.what()));
    }
}

} // namespace gcs
