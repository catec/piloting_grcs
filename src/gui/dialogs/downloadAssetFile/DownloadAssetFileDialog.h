#pragma once

#include <dataModel/dtos/FileDTO.h>

#include <QDialog>
#include <QMap>
#include <QRadioButton>
#include <QTreeWidgetItem>
#include <memory>

namespace Ui {
class DownloadAssetFileDialog;
}

namespace gcs {

class FileDTO;

class DownloadAssetFileDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit DownloadAssetFileDialog(QWidget* parent = nullptr);
    virtual ~DownloadAssetFileDialog();

    void setDownloadedFileList(const QList<FileDTO>&);

    FileDTO getSelectedFile();

  private:
    void accept();

    QPushButton* createCircularButton(const int, std::function<void()>);

    void insertMissionRoot(const FileDTO&);
    void insertMissionItem(QTreeWidgetItem*, const QString&, const QVariant&);

  private:
    std::unique_ptr<Ui::DownloadAssetFileDialog> _ui;

    QMap<QRadioButton*, FileDTO> _fileMapToDownload;
    FileDTO                      _selectedFile;
};

} // namespace gcs
