#pragma once

#include <QPushButton>
#include <QTreeWidgetItem>
#include <QWidget>
#include <memory>

namespace Ui {
class InspectionPlanWidget;
}

namespace gcs {

class InspectionPlanDTO;

class InspectionPlanWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit InspectionPlanWidget(QWidget* parent = nullptr);
    virtual ~InspectionPlanWidget();

    void setInspectionPlan(const QSharedPointer<InspectionPlanDTO>&);

  private Q_SLOTS:
    void informationButtonClicked();
    void taskSelectionChanged();
    void taskSelectionDoubleClicked(QTreeWidgetItem* item, int column);

  private:
    QPushButton* createCircularButton(const int, std::function<void()>);

    std::unique_ptr<Ui::InspectionPlanWidget> _ui;

    QSharedPointer<InspectionPlanDTO> _inspectionPlanDTO;

  Q_SIGNALS:
    void updateViewInspectionTask(const QString&);
};

} // namespace gcs
