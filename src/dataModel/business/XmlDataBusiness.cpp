#include "XmlDataBusiness.h"

#include <QsLog/QsLog.h>
#include <communications/MavSDK/dtos/HLActionListDTO.h>
#include <communications/MavSDK/dtos/SupportedCommandListDTO.h>

#include <QDir>
#include <QFile>

namespace gcs {

XmlDataBusiness::XmlDataBusiness(SupportedCommandListDTO& supportedCommandListDTO) :
        _supportedCommandListDTO(supportedCommandListDTO)

{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

XmlDataBusiness::~XmlDataBusiness()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void XmlDataBusiness::loadXmlFromFolder(const QString& xmlFolderPath) Q_DECL_NOEXCEPT
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - XML To Load: " << xmlFolderPath;

    if (xmlFolderPath.isEmpty()) {
        // throw std::runtime_error("XML folder path is empty");
        QLOG_DEBUG() << "XML folder path is empty";
        return;
    }
    QDir mainXmlDir(xmlFolderPath);

    if (!mainXmlDir.exists()) {
        // throw std::runtime_error("XML folder path does not exist");
        QLOG_DEBUG() << "XML folder path does not exist";
        return;
    }

    const auto xmlFiles = mainXmlDir.entryList(QDir::NoDot | QDir::NoDotDot | QDir::Files);
    if (xmlFiles.isEmpty()) {
        const auto msg = QString("There is no files inside folder: %1").arg(mainXmlDir.path());
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - " << msg;
        // throw std::runtime_error(msg.toStdString());
        return;
    }

    for (const auto files : xmlFiles) {
        loadXmlFile(mainXmlDir.absoluteFilePath(files));
    }
}

void XmlDataBusiness::loadXmlFile(const QString& filePath) Q_DECL_NOEXCEPT
{
    QFile xmlFile(filePath);
    if (!xmlFile.open(QFile::ReadOnly | QFile::Text)) {
        const QString msg = QString("Could not open xml file: %1").arg(filePath);
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - " << msg;
        // throw std::runtime_error(msg.toStdString());
        return;
    }

    QXmlStreamReader reader(&xmlFile);
    readMavlinkTitle(reader);

    while (reader.readNextStartElement()) {
        if (reader.name() == "enums") {
            while (reader.readNextStartElement()) {
                if (isMavCmdEnum(reader)) {
                    while (reader.readNextStartElement()) {
                        if (isValidCommand(reader)) {
                            CommandLongDTO commandLongDTO;
                            commandLongDTO = converToCommandDTO(reader);
                            _supportedCommandListDTO.getSupportedCommandList().append(commandLongDTO);
                        } else {
                            reader.skipCurrentElement();
                        }
                    }
                } else {
                    reader.skipCurrentElement();
                }
            }
        } else {
            reader.skipCurrentElement();
        }
    }

    if (reader.hasError()) {
        const QString msg = QString("Error parsing: %1").arg(filePath);
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - " << msg;
        // throw std::runtime_error(msg.toStdString());
        return;
    }
}

CommandLongDTO XmlDataBusiness::converToCommandDTO(QXmlStreamReader& reader) Q_DECL_NOEXCEPT
{
    CommandLongDTO commandLongDTO;

    commandLongDTO.getId()   = reader.attributes().value("value").toUInt();
    commandLongDTO.getName() = reader.attributes().value("name").toString();

    while (reader.readNextStartElement()) {
        if (reader.name() == "description") {
            commandLongDTO.getDescription() = reader.readElementText();
        } else if (reader.name() == "param") {
            if (isValidParam(reader)) {
                ParamDTO param;
                param = convertToParamDTO(reader);
                commandLongDTO.getParamList().append(param);
            }
        } else {
            reader.skipCurrentElement();
        }
    }

    return commandLongDTO;
}

void XmlDataBusiness::associateHLActionWithSupportedCommand(HLActionListDTO& hlActionListDTO) Q_DECL_NOEXCEPT
{
    for (auto& hlActionItemDTO : hlActionListDTO.getHLActionList()) {
        auto cmd_it = std::find_if(
                _supportedCommandListDTO.getSupportedCommandList().begin(),
                _supportedCommandListDTO.getSupportedCommandList().end(),
                [&](CommandLongDTO& commandLongDTO) {
                    return (hlActionItemDTO.getAssociatedCommand().getId() == commandLongDTO.getId());
                });

        if (cmd_it != _supportedCommandListDTO.getSupportedCommandList().end()) {
            hlActionItemDTO.getAssociatedCommand().getName()        = cmd_it->getName();
            hlActionItemDTO.getAssociatedCommand().getDescription() = cmd_it->getDescription();
            hlActionItemDTO.getAssociatedCommand().getParamList()   = cmd_it->getParamList();
        } else {
            const auto msg = QString("HLAction with name %1 has an unsupported command associated with id: %2")
                                     .arg(hlActionItemDTO.getName())
                                     .arg(hlActionItemDTO.getAssociatedCommand().getId());
            QLOG_DEBUG() << "XmlDataBusiness::associateHLActionWithSupportedCommand() -" << msg;
            hlActionItemDTO.getAssociatedCommand().getParamList() = generateEmptyParamList();
        }
    }
}

QList<ParamDTO> XmlDataBusiness::generateEmptyParamList() Q_DECL_NOEXCEPT
{
    QList<ParamDTO> emptyParamList;

    for (auto i = 1; i <= 7; i++) {
        ParamDTO paramDTO;
        paramDTO.getIndex() = i;
        paramDTO.getLabel() = QString("param%1").arg(i);
        emptyParamList.append(paramDTO);
    }
    return emptyParamList;
}

ParamDTO XmlDataBusiness::convertToParamDTO(QXmlStreamReader& reader) Q_DECL_NOEXCEPT
{
    ParamDTO param;

    param.getIndex() = reader.attributes().value("index").toInt();
    if (reader.attributes().hasAttribute("label")) {
        param.getLabel() = reader.attributes().value("label").toString();
        if (reader.attributes().hasAttribute("units")) {
            param.getUnits() = reader.attributes().value("units").toString();
        }
        param.getDescription() = reader.readElementText();
    } else {
        param.getLabel() = reader.readElementText();
    }
    return param;
}

bool XmlDataBusiness::isValidCommand(const QXmlStreamReader& reader) Q_DECL_NOEXCEPT
{
    if (reader.attributes().hasAttribute("value") && reader.attributes().hasAttribute("name")) {
        return true;
    } else {
        return false;
    }
}

bool XmlDataBusiness::isValidParam(const QXmlStreamReader& reader) Q_DECL_NOEXCEPT
{
    if (reader.attributes().hasAttribute("index")) {
        return true;
        if (reader.attributes().value("index").toInt() > 0 && reader.attributes().value("index").toInt() <= 7) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

bool XmlDataBusiness::isMavCmdEnum(const QXmlStreamReader& reader) Q_DECL_NOEXCEPT
{
    if (reader.attributes().hasAttribute("name")) {
        if (reader.attributes().value("name").toString() == "MAV_CMD") {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

void XmlDataBusiness::readMavlinkTitle(QXmlStreamReader& reader) Q_DECL_NOEXCEPT
{
    if (!(reader.readNextStartElement() && reader.name() == "mavlink")) {
        const QString msg = QString("XML invalid format name: %1").arg(reader.name().toString());
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - " << msg;
        // throw std::runtime_error(msg.toStdString());
        return;
    }
}
} // namespace gcs
