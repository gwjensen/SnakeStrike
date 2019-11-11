#ifndef BufferingDialog_h
#define BufferingDialog_h

#include <QDialog>
#include <QMutex>
#include <QCloseEvent>

namespace Ui {
class BufferingDialog;
}


class BufferingDialog : public QDialog
{
    Q_OBJECT

    public:
        explicit BufferingDialog(QWidget* ipParent = 0);
        ~BufferingDialog();

        void ResetValues( const int iStartingValue, const int iMaxValue );

    public slots:
        void on_pushButton_abort_clicked();
        void IncrementImageSavedCount();
        void IncrementImageDroppedCount();

    signals:
        void QuitSavingImages();

    protected:
        void closeEvent(QCloseEvent* ipEvent);

    private:
        Ui::BufferingDialog* mpUi;
        int mImagesSaved;
        int mImagesDropped;
        QMutex mMutex;
};

#endif // BufferingDialog_h
