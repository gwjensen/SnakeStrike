#ifndef CORRESPONDENCEDIALOG_H
#define CORRESPONDENCEDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPixmap>
#include <vector>
#include <set>
#include <QPoint>
#include <QMutex>
#include <QList>

#include "image/SmtImage.h"
#include "image/PixelSet.h"
#include "image/ImageSet.h"


namespace Ui {
    class CorrespondenceDialog;
}

class CorrespondenceDialog : public QDialog
{
        Q_OBJECT

    public:
        explicit CorrespondenceDialog(QWidget *parent,
                                       std::vector< SmtImage > iTimestepImages,
                                      const int iTimestepValue,
                                       std::vector< std::vector< SmtPixel > > iTimestepMarkersFound,
                                       std::pair<unsigned long, std::set<unsigned long> > iCamsToExclude,
                                      const int iNumMarkersToFind);
        ~CorrespondenceDialog();

        void Cancel(){ mIsCancelled = true;}
        const std::vector< std::vector< cv::Point2d > > MarkedPoints(){ return mMarkedPoints;}
        const std::vector< std::vector< std::vector< int32_t > > > BestFitInfoForPoints();
        bool Complete(){return mIsComplete;}
        //void SignalGrabbedPixelValue( int iX, int iY );

    public slots:
        void SavePixelValueAndUpdateImage( int iX, int iY );


    signals:
        //void GrabbedPixelValue( int iX, int iY );
        void Cancelled();
        void Completed();
        void ProcessingDone();
        void UpdateImage( QPixmap iPixmap, int iWidth, int iHeight );

    private slots:
        void on_undoClickButton_clicked();

        void on_CancelButton_clicked();

    protected:
        void closeEvent(QCloseEvent *event);


    private:
        Ui::CorrespondenceDialog* mpUi;
        bool mIsCancelled;
        SmtPixel  mClickCoords;
        uint64_t  mTimestep;
        int  mCurCamIdx;
        int  mCurPointNum;
        uint32_t mMaxPointNum;
        std::vector<int> mCamList;
        std::vector< std::vector< SmtPixel > > mFoundMarkers; //Cams(markers)
        std::vector< std::vector< cv::Point2d> > mMarkedPoints;
        std::vector< SmtImage > mTimestepImages;
        std::vector< std::list< unsigned long> > mPointIdxLeft;//used in distance calculation of closest point
        std::vector< std::vector< int32_t > > mPointIdxList; //holds the original point indexes in the order they were clicked by the user.
        QMutex mLock;
        bool mIsComplete;
        bool mIsUpdating;
        QWidget* mpParent;
};

#endif // CORRESPONDENCEDIALOG_H
