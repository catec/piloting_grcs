
#include "LoadXmlDataFromFolderAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/DataModel.h>

namespace gcs {

LoadXmlDataFromFolderAction::LoadXmlDataFromFolderAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _XmlDataBusiness(dataModel.getSupportedCommandListDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

LoadXmlDataFromFolderAction::~LoadXmlDataFromFolderAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType LoadXmlDataFromFolderAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::LOAD_XML_DATA_FROM_FOLDER;
}

void LoadXmlDataFromFolderAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        _XmlDataBusiness.loadXmlFromFolder(parsedDTO->getString());

    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Error:" << e.what();

        sendWarningMsg(tr("Load xml data from folder error - ") + tr(e.what()));
    }
}

} // namespace gcs
