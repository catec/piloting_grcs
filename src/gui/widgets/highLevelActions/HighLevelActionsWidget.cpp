#include "HighLevelActionsWidget.h"

#include <QsLog/QsLog.h>
#include <communications/MavSDK/dtos/HLActionListDTO.h>
#include <dialogs/commandParameters/CommandParametersDialog.h>

#include <QMessageBox>

#include "ui_HighLevelActionsWidget.h"

namespace gcs {

HighLevelActionsWidget::HighLevelActionsWidget(QWidget* parent) :
        QWidget(parent), _ui(std::make_unique<Ui::HighLevelActionsWidget>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);
}

HighLevelActionsWidget::~HighLevelActionsWidget()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void HighLevelActionsWidget::receiveHLActionList(const HLActionListDTO& hlActionListDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    for (const auto& button : _hlActionListMap.keys()) {
        delete button;
    }
    _hlActionListMap.clear();

    for (const auto& hlAction : hlActionListDTO.getHLActionList()) {
        auto btn = new QPushButton(hlAction.getName(), this);
        btn->setFixedHeight(27);
        btn->setToolTip(hlAction.getDescription());
        // clang-format off
      connect(btn,  &QPushButton::clicked, 
              this, &HighLevelActionsWidget::HLActionButtonClicked);
        // clang-format on
        _ui->grpBoxLayout->addWidget(btn);

        _hlActionListMap.insert(btn, hlAction);
    }
}

void HighLevelActionsWidget::resetHLActionList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    for (const auto& button : _hlActionListMap.keys()) {
        delete button;
    }
    _hlActionListMap.clear();
}

void HighLevelActionsWidget::HLActionButtonClicked()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto button = dynamic_cast<QPushButton*>(QObject::sender());
    if (!button) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "Unknown action button clicked";
        return;
    }

    CommandParametersDialog dialog(this);
    dialog.setHLActionInfo(_hlActionListMap.value(button));

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    const auto response = QMessageBox::question(
            this,
            tr("High Level Action"),
            QString("Are you sure that you want send the following command?: \n%1")
                    .arg(_hlActionListMap.value(button).getName()));

    if (response == QMessageBox::No) {
        return;
    }

    auto commandLongDTO            = createSharedDTO<CommandLongDTO>();
    *commandLongDTO                = _hlActionListMap.value(button).getAssociatedCommand();
    commandLongDTO->getParamList() = dialog.getParamList();

    Q_EMIT sendCommand(commandLongDTO);
}

} // namespace gcs
