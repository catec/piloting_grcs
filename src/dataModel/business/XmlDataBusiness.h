#pragma once

#include <QXmlStreamReader>

#include "gcs_dataModel_export.h"

namespace gcs {
class SupportedCommandListDTO;
class CommandLongDTO;
class ParamDTO;
class HLActionListDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT XmlDataBusiness
{
  public:
    explicit XmlDataBusiness(SupportedCommandListDTO& supportedCommandListDTO);
    virtual ~XmlDataBusiness();

    void loadXmlFromFolder(const QString&) Q_DECL_NOEXCEPT;
    void associateHLActionWithSupportedCommand(HLActionListDTO&) Q_DECL_NOEXCEPT;

  private:
    void            loadXmlFile(const QString&) Q_DECL_NOEXCEPT;
    bool            isMavCmdEnum(const QXmlStreamReader&) Q_DECL_NOEXCEPT;
    QList<ParamDTO> generateEmptyParamList() Q_DECL_NOEXCEPT;
    void            readMavlinkTitle(QXmlStreamReader&) Q_DECL_NOEXCEPT;
    CommandLongDTO  converToCommandDTO(QXmlStreamReader&) Q_DECL_NOEXCEPT;
    ParamDTO        convertToParamDTO(QXmlStreamReader&) Q_DECL_NOEXCEPT;
    bool            isValidCommand(const QXmlStreamReader&) Q_DECL_NOEXCEPT;
    bool            isValidParam(const QXmlStreamReader&) Q_DECL_NOEXCEPT;

    SupportedCommandListDTO& _supportedCommandListDTO;
};

} // namespace gcs
