#pragma once

#include <QButtonGroup>
#include <QDialog>
#include <memory>

namespace Ui {
class CloudEditorDialog;
}

namespace gcs {

class CloudDimensionsDTO;
class CloudEditionParametersDTO;

class CloudEditorDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit CloudEditorDialog(QWidget* parent = nullptr);
    virtual ~CloudEditorDialog();

    void updateStatus(const bool, const bool);

    void updateDimensions(const QSharedPointer<CloudDimensionsDTO>&);

  Q_SIGNALS:
    void editPointCloud(QSharedPointer<CloudEditionParametersDTO>);
    void restorePointCloud();
    void hideCutPlane();
    void hidePointCloud();
    void closeDialog();
    void dialogClosed();

  private Q_SLOTS:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;
    void closeEvent(QCloseEvent*) override;
    void indicateChanges(int);
    void indicateReleased();
    void hidePointCloudButtonClicked();

    void restoreButtonClicked();

  private:
    void initPlaneCutParameters();

    void sendCutParameters(const bool&);

    void updateSliderRange();
    bool getAxisDirection();

  private:
    std::unique_ptr<Ui::CloudEditorDialog> _ui;

    QList<QPair<int, int>> _axisRanges;

    QButtonGroup _axisBtnGroup;
    QButtonGroup _directionBtnGroup;
};
} // namespace gcs