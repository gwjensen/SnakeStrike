#include <QThread>

#include "CountdownDialog.h"
#include "ui_Countdown.h"

#ifdef Q_OS_WIN
#include <windows.h> // for Sleep
#endif

CountdownDialog::CountdownDialog(QWidget *parent)
    : QDialog( parent ),
      mpUi( new Ui::CountdownDialog ),
      mAbort( false )
{
    mpUi->setupUi(this);
    setModal( true );
}

CountdownDialog::~CountdownDialog()
{
    delete mpUi;
}

bool CountdownDialog::Countdown( int iStartingSeconds )
{
    int real_seconds = iStartingSeconds + 2;
    setWindowTitle( "You Sure?");
    repaint();
    show();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

    mMutex.lock();
    mAbort = false;
    mMutex.unlock();
    for (int i = real_seconds * 4; i > 0; --i)
    {
        if (mAbort)
        {
            return false;
        }

        mpUi->label->setText( QString::number(i/real_seconds) );
        repaint();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        QThread::msleep(250);
    }
    if (mAbort)
    {
        return false;
    }
    return true;
}

void CountdownDialog::on_abortButton_clicked(){
    mMutex.lock();
    mAbort = true;
    mMutex.unlock();
    //emit abortClicked();
    hide();
}
