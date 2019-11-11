#ifndef ThresholdDialog_h
#define ThresholdDialog_h

#include <QDialog>
#include <QReadWriteLock>

#include "CamPreviewCanvas.h"
#include "GuiImage.h"
#include "TrackingDialog.h"
#include "CamPreviewWindow.h"
#include "image/ImageSet.h"

namespace Ui {
class ThresholdDialog;
}

class ThresholdDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ThresholdDialog( ProjectDialog*& iProjInfo, QString iImagesXml, QWidget *parent = 0 );
    ~ThresholdDialog();
    void UpdateSourceInfo( );
    ImageSet& rImagesReadIn(){ return mImageSet;}

public slots:
    void on_pushButton_visualize_clicked();
    void on_pushButton_cancel_clicked();
    void on_pushButton_ok_clicked();
    void on_pushButton_leftbound_clicked();
    void on_pushButton_rightbound_clicked();


    void on_spinBox_HLeftbound_editingFinished();
    void on_spinBox_HRightbound_editingFinished();
    void on_spinBox_SLeftbound_editingFinished();
    void on_spinBox_SRightbound_editingFinished();
    void on_spinBox_VLeftbound_editingFinished();
    void on_spinBox_VRightbound_editingFinished();

    void on_spinBox_numCams_editingFinished();
    void on_spinBox_numIterations_editingFinished();
    void on_spinBox_filterSize_editingFinished();
    void on_spinBox_timestep_editingFinished();
    void on_checkBox_showOriginalImage_toggled();

    void UpdateGrabbedPixel( uint32_t iCamIdx, uint32_t iX, uint32_t iY);
    void UpdateThresholdPreviewWindow(int iLabelIdx, QPixmap iImage);
    static bool Cancelled();

protected:
    void ReadXMLImagesFile( const QString& iLocation,
                            QVector< QString >& oImageLocations,
                            int& oNumCams,
                            uint32_t& oNumImagesFound );
    void ThresholdImage();

    private slots:
    void on_spinBox_maxNumPoints_editingFinished();

    private:
    Ui::ThresholdDialog*    mpUi;
    TrackingDialog*         mpParent;
    ProjectDialog*          mpProjInfo;

    bool            mGrabColor;
    uint32_t        mMaxTimestep;
    QString         mImagesXml;
    int             mNumCams;
    QMutex          mMutex;
    QReadWriteLock  mImageLock;

    //masks are indexed by cam number, while images indexed by "cam#-Image#"
    QMap<QString, QString>       mFileListMap;
    QVector< CamPreviewCanvas* > mThreshPreviewList;
    QVector< CamPreviewWindow* > mThreshWindowList;
    QVector< GuiImage* >         mImageList;
    ImageSet                     mImageSet;




    bool GetLabel( const int iLabelIdx, CamPreviewCanvas*& oLabelPtr );
    void SetThresholdBounds( uint32_t iLH,
                             uint32_t iLS,
                             uint32_t iLV,
                             uint32_t iRH,
                             uint32_t iRS,
                             uint32_t iRV );
    void SetThresholdBounds( cv::Scalar iLeftBound, cv::Scalar iRightBound );
    uint32_t ReadImages( );
    void UpdateImages();
    void UpdateVisiblePreviews();

};

#endif // ThresholdDialog_h
