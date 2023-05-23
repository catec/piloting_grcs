#include "CheckListDialog.h"

#include <QsLog/QsLog.h>
#include <common/Types.h>
#include <communications/MavSDK/dtos/CheckListDTO.h>

#include <QCheckBox>
#include <QTreeWidgetItem>

#include "ui_CheckListDialog.h"

#define CHECK_OK   0
#define CHECK_FAIL 1
#define ID         2
#define NAME       3
#define DESCR      4

namespace gcs {

CheckListDialog::CheckListDialog(QWidget* parent) : QDialog(parent), _ui(std::make_unique<Ui::CheckListDialog>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);

    _ui->treeWidget->setSelectionMode(QAbstractItemView::NoSelection);
}

CheckListDialog::~CheckListDialog()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void CheckListDialog::setCheckList(const CheckListDTO& checklistDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    blockSignals(true);

    _ui->treeWidget->clear();
    for (const auto& checkItem : checklistDTO.getCheckItemsList()) {
        auto taskItem = new QTreeWidgetItem;
        if (!taskItem) {
            continue;
        }

        taskItem->setFlags(Qt::NoItemFlags);
        // taskItem->setFlags(static_cast<Qt::ItemFlags>(
        // Qt::ItemIsSelectable | Qt::ItemIsEnabled));

        taskItem->setData(ID, Qt::DisplayRole, QVariant(checkItem.getIndex()));
        taskItem->setData(NAME, Qt::DisplayRole, QVariant(checkItem.getName()));
        taskItem->setData(DESCR, Qt::DisplayRole, QVariant(checkItem.getDescription()));

        _ui->treeWidget->addTopLevelItem(taskItem);

        auto checkboxOk = new QCheckBox(this);
        checkboxOk->setChecked(checkItem.getCheckStatus() == CheckStatus::OK);
        checkboxOk->setStyleSheet("QCheckBox::indicator:hover{ background-color: #286e08;}");

        auto checkboxFail = new QCheckBox(this);
        checkboxFail->setChecked(checkItem.getCheckStatus() == CheckStatus::FAIL);
        checkboxFail->setStyleSheet(
                "QCheckBox::indicator:checked{ background-color: #F33A12;} QCheckBox::indicator:hover{ "
                "background-color: #C13112;}");

        connect(checkboxOk, &QCheckBox::stateChanged, [=]() {
            if (checkboxOk->isChecked()) {
                checkboxFail->setCheckState(Qt::CheckState::Unchecked);
            }
        });

        connect(checkboxFail, &QCheckBox::stateChanged, [=]() {
            if (checkboxFail->isChecked()) {
                checkboxOk->setCheckState(Qt::CheckState::Unchecked);
            }
        });

        _ui->treeWidget->setItemWidget(taskItem, CHECK_OK, checkboxOk);
        _ui->treeWidget->setItemWidget(taskItem, CHECK_FAIL, checkboxFail);

        _items.append(taskItem);
    }

    _ui->treeWidget->sortItems(ID, Qt::AscendingOrder);
    for (int idx = 0; idx <= ID; ++idx) {
        _ui->treeWidget->resizeColumnToContents(idx);
    }

    /// \note. Bad fix
    _ui->treeWidget->header()->resizeSection(NAME, 150);

    blockSignals(false);
}

CheckListDTO CheckListDialog::getCheckList() const
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    CheckListDTO readedCheckList;

    for (const auto item : _items) {
        CheckListItemDTO checkItem;
        checkItem.getIndex()       = item->data(ID, 0).toUInt();
        checkItem.getName()        = item->data(NAME, 0).toString();
        checkItem.getDescription() = item->data(DESCR, 0).toString();

        const auto checkBoxOk   = qobject_cast<QCheckBox*>(_ui->treeWidget->itemWidget(item, CHECK_OK));
        const auto checkBoxFail = qobject_cast<QCheckBox*>(_ui->treeWidget->itemWidget(item, CHECK_FAIL));

        checkItem.getCheckStatus() = (checkBoxOk && checkBoxOk->isChecked())     ? CheckStatus::OK
                                   : (checkBoxFail && checkBoxFail->isChecked()) ? CheckStatus::FAIL
                                                                                 : CheckStatus::NONE;

        readedCheckList.getCheckItemsList().append(checkItem);
    }

    return readedCheckList;
}

} // namespace gcs
