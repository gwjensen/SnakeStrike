
#include "BufferingDialog.h"
#include "ui_Buffering.h"

BufferingDialog::BufferingDialog(QWidget* ipParent) :
    QDialog(ipParent),
    mpUi(new Ui::BufferingDialog)
{
    mpUi->setupUi(this);
    mpUi->progressBar->setMinimum(0);
}

BufferingDialog::~BufferingDialog()
{
    delete mpUi;
}

void BufferingDialog::closeEvent(QCloseEvent* ipEvent)
{
    ipEvent->ignore();
}

void BufferingDialog::ResetValues( int iStartingValue, int iMaxValue )
{
    mMutex.lock();
    mImagesSaved = iStartingValue;
    mpUi->progressBar->setValue( iStartingValue );
    mpUi->progressBar->setMaximum( iMaxValue );
    mImagesDropped = 0;
    repaint();
    mMutex.unlock();
}

void BufferingDialog::on_pushButton_abort_clicked()
{
    hide();
    emit QuitSavingImages();
}

void BufferingDialog::IncrementImageSavedCount()
{
    mMutex.lock();
    mpUi->progressBar->setValue( ++mImagesSaved );
    repaint();
    mMutex.unlock();
}

void BufferingDialog::IncrementImageDroppedCount()
{
    mMutex.lock();
    ++mImagesDropped;
    mMutex.unlock();
}
