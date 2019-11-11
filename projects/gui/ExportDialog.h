#ifndef ExportDialog_h
#define ExportDialog_h

#include <QDialog>
#include "ProjectDialog.h"

namespace Ui {
class ExportDialog;
}

class ExportDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ExportDialog(ProjectDialog*& iProjDetails, QWidget* ipParent = 0);
    ~ExportDialog();
     void PopulateTrackingFiles();

private slots:
    void on_pushButton_cancel_clicked();
    void on_pushButton_export_clicked();
    //void on_pushButton_triBrowse_clicked();
    void on_pushButton_triSave_clicked();
    void on_fileSelectionBox_currentTextChanged(const QString& iText);
    void on_spinBox_startTimestep_editingFinished();
    void on_spinBox_endTimestep_editingFinished();

    protected:
    void FreezeDialog( bool iFreeze );
    void closeEvent(QCloseEvent* iEvent);

private:
    Ui::ExportDialog* mpUi;
    ProjectDialog* mpProjDetails;
};

#endif // ExportDialog_h
