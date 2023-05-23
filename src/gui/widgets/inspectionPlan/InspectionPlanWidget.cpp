#include "InspectionPlanWidget.h"

#include <QsLog/QsLog.h>

#include <QMessageBox>
#include <QTreeWidget>

#include "../../dialogs/inspectionPlanInfo/InspectionPlanInfoDialog.h"
#include "../../dialogs/inspectionTaskInfo/InspectionTaskInfoDialog.h"
#include "../../dtos/GuiDTO.h"
#include "ui_InspectionPlanWidget.h"

#define INFO 0
#define ID   1
#define NAME 2

namespace gcs {

InspectionPlanWidget::InspectionPlanWidget(QWidget* parent) :
        QWidget(parent), _ui(std::make_unique<Ui::InspectionPlanWidget>())
{
    _ui->setupUi(this);

    // clang-format off
   connect(_ui->inspPlanInfoB, &QPushButton::clicked,
           this,               &InspectionPlanWidget::informationButtonClicked);

   connect(_ui->treeWidget, &QTreeWidget::itemSelectionChanged,
           this,            &InspectionPlanWidget::taskSelectionChanged);

   connect(_ui->treeWidget, &QTreeWidget::itemDoubleClicked,
           this,            &InspectionPlanWidget::taskSelectionDoubleClicked);
    // clang-format on
}

InspectionPlanWidget::~InspectionPlanWidget()
{
    _ui->treeWidget->clear();
}

void InspectionPlanWidget::setInspectionPlan(const QSharedPointer<InspectionPlanDTO>& inspectionPlanDTO)
{
    blockSignals(true);

    if (_inspectionPlanDTO != inspectionPlanDTO) {
        _inspectionPlanDTO = inspectionPlanDTO;
    }

    _ui->treeWidget->clear();
    for (const auto& insp_task : _inspectionPlanDTO->getInspectionTaskList()) {
        auto taskItem = new QTreeWidgetItem;
        if (taskItem) {
            taskItem->setFlags(static_cast<Qt::ItemFlags>(Qt::ItemIsSelectable | Qt::ItemIsEnabled));

            taskItem->setData(ID, Qt::DisplayRole, insp_task.getUUID());
            taskItem->setData(NAME, Qt::DisplayRole, insp_task.getName());

            _ui->treeWidget->insertTopLevelItem(0, taskItem);

            const int btn_radius = 20;
            auto      btn        = createCircularButton(btn_radius, [=]() {
                InspectionTaskInfoDialog dialog(this);
                dialog.setInspectionTaskInfo(insp_task);
                if (dialog.exec() != QDialog::Accepted) {
                    return;
                }
            });

            /// \note. These two additional pixels are to avoid overlapping between icons
            taskItem->setSizeHint(INFO, QSize(btn_radius + 2, btn_radius + 2));

            _ui->treeWidget->setItemWidget(taskItem, INFO, btn);
        }
    }

    _ui->treeWidget->sortItems(ID, Qt::AscendingOrder);
    _ui->treeWidget->resizeColumnToContents(INFO);

    // for(int idx = 0; idx < _ui->treeWidget->columnCount(); ++idx)
    //    _ui->treeWidget->resizeColumnToContents(idx);

    blockSignals(false);
}

void InspectionPlanWidget::informationButtonClicked()
{
    if (!_inspectionPlanDTO) {
        auto msgBox = new QMessageBox(this);
        msgBox->setModal(true);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setWindowTitle(tr("Inspection Plan Information"));
        msgBox->setText(tr("There is no inspection plan downloaded"));
        msgBox->setIcon(QMessageBox::Warning);
        msgBox->show();

        return;
    }

    InspectionPlanInfoDialog dialog(this);
    dialog.setInspectionPlanInfo(*_inspectionPlanDTO);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
}

void InspectionPlanWidget::taskSelectionChanged()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto selectItems = _ui->treeWidget->selectedItems();
    if (!selectItems.isEmpty()) {
        auto selectItem = selectItems.at(0);
        if (selectItem) {
            Q_EMIT updateViewInspectionTask(selectItem->data(ID, Qt::DisplayRole).toString());
        }
    }
}

void InspectionPlanWidget::taskSelectionDoubleClicked(QTreeWidgetItem*, int)
{
    _ui->treeWidget->clearSelection();

    /// \note send -1 if value isnt valid or if want to unselect any task
    Q_EMIT updateViewInspectionTask("");
}

QPushButton* InspectionPlanWidget::createCircularButton(const int radius, std::function<void()> btn_callback)
{
    auto btn = new QPushButton(this);
    btn->setIcon(QIcon(":/images/info"));
    btn->setStyleSheet(QString("border-width: 0px"));

    QRect btn_rect(0, 0, radius, radius);
    btn->setIconSize(btn_rect.size());
    btn->setFixedSize(btn_rect.size());
    btn->setMask(QRegion(btn_rect, QRegion::Ellipse));

    connect(btn, &QPushButton::clicked, btn_callback);

    return btn;
}

} // namespace gcs
