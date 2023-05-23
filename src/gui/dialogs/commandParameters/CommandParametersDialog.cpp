
#include "CommandParametersDialog.h"

#include <QsLog/QsLog.h>
#include <communications/MavSDK/dtos/HLActionItemDTO.h>
#include <communications/MavSDK/dtos/ParamDTO.h>

#include <QHostAddress>
#include <QMessageBox>

#include "ui_CommandParametersDialog.h"

#define LIST_SIZE 7

#define INDEX 0
#define NAME  1
#define DESCR 2
#define VALUE 3
#define UNITS 4

namespace gcs {

CommandParametersDialog::CommandParametersDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::CommandParametersDialog>())
{
    _ui->setupUi(this);
    _ui->treeWidget->header()->setDefaultAlignment(Qt::AlignCenter);
}

CommandParametersDialog::~CommandParametersDialog() {}

void CommandParametersDialog::setHLActionInfo(const HLActionItemDTO& hlActionItemDTO)
{
    _ui->hlActNameLE->setText(hlActionItemDTO.getName());
    _ui->commandNameLE->setText(hlActionItemDTO.getAssociatedCommand().getName());
    _ui->commandDescrTE->setPlainText(hlActionItemDTO.getAssociatedCommand().getDescription());
    _ui->commandDescrTE->setAlignment(Qt::AlignCenter);

    _paramList = hlActionItemDTO.getAssociatedCommand().getParamList();

    insertParamList(_paramList);

    if (_paramList.size() < LIST_SIZE) {
        addEmptyParameters();
    }
}

void CommandParametersDialog::addEmptyParameters()
{
    std::generate_n(std::back_inserter(_paramList), LIST_SIZE - _paramList.size(), [this]() {
        ParamDTO emptyParam;
        emptyParam.getLabel() = "Empty";
        emptyParam.getIndex() = _paramList.last().getIndex() + 1;
        return emptyParam;
    });

    insertParamList(_paramList);
}
void CommandParametersDialog::insertParamList(const QList<ParamDTO>& paramList)
{
    blockSignals(true);
    _paramsMap.clear();
    _ui->treeWidget->clear();

    for (const auto paramDTO : paramList) {
        auto treeItem = new QTreeWidgetItem(_ui->treeWidget);

        treeItem->setText(INDEX, QString::number(paramDTO.getIndex()));
        treeItem->setText(NAME, paramDTO.getLabel());

        QLabel* descriptionL = new QLabel(paramDTO.getDescription(), this);
        descriptionL->setWordWrap(true);
        descriptionL->setAlignment(Qt::AlignCenter);
        descriptionL->setStyleSheet("QLabel { background-color: transparent;}");
        descriptionL->setMinimumWidth(300);
        descriptionL->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        _ui->treeWidget->setItemWidget(treeItem, DESCR, descriptionL);

        auto doubleSpinBox = new QDoubleSpinBox(this);
        doubleSpinBox->setMinimumWidth(80);
        doubleSpinBox->setRange(-999.0, 999.0);
        doubleSpinBox->setAlignment(Qt::AlignCenter);
        doubleSpinBox->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        doubleSpinBox->setStyleSheet("QDoubleSpinBox { background-color: transparent;}");

        treeItem->setText(UNITS, paramDTO.getUnits());

        _ui->treeWidget->setItemWidget(treeItem, VALUE, doubleSpinBox);
        _paramsMap.insert(doubleSpinBox, paramDTO);

        _ui->treeWidget->sortItems(INDEX, Qt::AscendingOrder);
        for (int idx = 0; idx < _ui->treeWidget->columnCount(); ++idx) {
            _ui->treeWidget->resizeColumnToContents(idx);
            treeItem->setTextAlignment(idx, Qt::AlignCenter);
        }
        _ui->treeWidget->header()->resizeSection(UNITS, 50);
    }

    blockSignals(false);
}

QList<ParamDTO> CommandParametersDialog::getParamList()
{
    QList<ParamDTO> paramsResult;
    for (auto& paramDTO : _paramsMap.values()) {
        paramDTO.getValue() = _paramsMap.key(paramDTO)->value();
        paramsResult.append(paramDTO);
    }

    return paramsResult;
}
void CommandParametersDialog::accept()
{
    QDialog::accept();
}

} // namespace gcs
