
#include "ManageAutoHeadingWptsAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicBoolDTO.h>
#include <dataModel/DataModel.h>

namespace gcs {

ManageAutoHeadingWptsAction::ManageAutoHeadingWptsAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << "ManageAutoHeadingWptsAction::ManageAutoHeadingWptsAction()";
}

ManageAutoHeadingWptsAction::~ManageAutoHeadingWptsAction()
{
    QLOG_TRACE() << "ManageAutoHeadingWptsAction::~ManageAutoHeadingWptsAction()";
}

ActionType ManageAutoHeadingWptsAction::getActionType()
{
    QLOG_TRACE() << "ManageAutoHeadingWptsAction::getActionType()";

    return ActionType::MANAGE_AUTOHEADING_WPTS;
}

void ManageAutoHeadingWptsAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ManageAutoHeadingWptsAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<BasicBoolDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicBoolDTO");
        }

        _dataModel.getAutoHeadingMode() = parsedDTO->getBool();
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ManageAutoHeadingWptsAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Manage Auto Heading error - ") + tr(e.what()));
    }
}

} // namespace gcs
