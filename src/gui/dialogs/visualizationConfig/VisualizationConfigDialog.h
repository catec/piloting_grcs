#pragma once

#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <memory>

namespace Ui {
class VisualizationConfigDialog;
}

namespace gcs {

class VisualizationConfigDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit VisualizationConfigDialog(QWidget* parent = nullptr);
    virtual ~VisualizationConfigDialog();

    bool getShowFloorGrid() const;
    void setShowFloorGrid(const bool);

    bool getShowItemsText() const;
    void setShowItemsText(const bool);

    bool getShowGlobalAxis() const;
    void setShowGlobalAxis(const bool);

    bool getShowHomePosition() const;
    void setShowHomePosition(const bool);

    float getWaypointSizeScale() const;
    void  setWaypointSizeScale(const float);

    float getInspPtSizeScale() const;
    void  setInspectionPtScale(const float);

    float getInspLocationTransp() const;
    void  setInspLocationTransp(const float);

    float getCloudPtsSize() const;
    void  setCloudPtsSize(const float);

    QVariant getPointCloudColorStyle() const;
    void     setPointCloudColorStyle(const QVariant&);

    QColor getPointCloudColor() const;
    void   setPointCloudColor(const QColor&);

  private:
    void setupComboBoxBoolean(QComboBox* combobox);

  private Q_SLOTS:
    void flatColorSelect();
    void colorStyleChanged(int);

  private:
    std::unique_ptr<Ui::VisualizationConfigDialog> _ui;
    QColor                                         _currentColor;
};

} // namespace gcs
