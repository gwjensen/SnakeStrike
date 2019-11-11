#ifndef CountdownDialog_h
#define CountdownDialog_h

#include <QDialog>
#include <QMutex>

namespace Ui {
class CountdownDialog;
}

class CountdownDialog : public QDialog
{
    Q_OBJECT

    public:
        explicit CountdownDialog(QWidget* ipParent = 0);
        ~CountdownDialog();
        bool Countdown( int iStartingSeconds );

    signals:
        //void abortClicked();

    public slots:
        void on_abortButton_clicked();

    private:
        Ui::CountdownDialog* mpUi;
        bool    mAbort;
        QMutex  mMutex;

};

#endif // CountdownDialog_h
