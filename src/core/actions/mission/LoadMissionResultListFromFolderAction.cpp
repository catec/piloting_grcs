#include "LoadMissionResultListFromFolderAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/DataModel.h>

namespace gcs {

LoadMissionResultListFromFolderAction::LoadMissionResultListFromFolderAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _missionResultsBusiness(dataModel.getMissionResultsDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

LoadMissionResultListFromFolderAction::~LoadMissionResultListFromFolderAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType LoadMissionResultListFromFolderAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::LOAD_MISSION_RESULT_LIST_FROM_FOLDER;
}

void LoadMissionResultListFromFolderAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        _missionResultsBusiness.loadMissionResults(parsedDTO->getString());

        auto missionResultsDTO = createSharedDTO<MissionResultsDTO>();
        *missionResultsDTO     = _dataModel.getMissionResultsDTO();
        Q_EMIT sendDTO(missionResultsDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "- Error:" << e.what();

        sendWarningMsg(tr("Load mission result list from folder error - ") + tr(e.what()));
    }
}

} // namespace gcs
