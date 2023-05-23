
#include "UpdateCommsLinkDDHLAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>
#include <dataModel/dtos/CommsLinkDDHLDTO.h>

namespace gcs {
UpdateCommsLinkDDHLAction::UpdateCommsLinkDDHLAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateCommsLinkDDHLAction::~UpdateCommsLinkDDHLAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateCommsLinkDDHLAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_COMMS_LINK_DDHL;
}

void UpdateCommsLinkDDHLAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto commsLinkDDHLDTO = dto.dynamicCast<CommsLinkDDHLDTO>();
    try {
        if (!commsLinkDDHLDTO) {
            throw std::runtime_error("DTO type should be CommsLinkDDHLDTO");
        }

        _dataModel.getCommsLinkDDHLDTO() = *commsLinkDDHLDTO;
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update Comms Link DDHL error - ") + tr(e.what()));
    }
}
} // namespace gcs
