#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <regex>
#include <math.h>
#include <string>
#include <utility>
#include <assert.h>
#include <QPixmap>
#include <QPair>
#include <QColorDialog>
#include <QPixmap>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QDesktopWidget>
#include <QMouseEvent>
#include <QMessageBox>
#include <QScreen>
#include <malloc.h>

#include "utils/utils.h"
#include "io/File.h"
#include "GuiImage.h"

#include "ui_ThresholdDialog.h"
#include "ThresholdDialog.h"

ThresholdDialog::ThresholdDialog( ProjectDialog*& iProjInfo, QString iImagesXml, QWidget* ipParent )
    : QDialog( ipParent ),
      mpUi( new Ui::ThresholdDialog ),
      mpParent( (TrackingDialog*)ipParent ),
      mpProjInfo( iProjInfo ),
      mGrabColor( false ),
      mMaxTimestep(0),
      mImagesXml( iImagesXml )
{
    mpUi->setupUi( this );
    this->setWindowTitle("Visualize Color Thresholds");
    resize(QDesktopWidget().availableGeometry(this).size() * 0.7);
    mpUi->imageframe->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mpUi->imageframe->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mpUi->spinBox_filterSize->setValue( mpParent->FilterSize() );
    mpUi->spinBox_numIterations->setValue( mpParent->NumIterations() );
    mpUi->horizontalSlider_threshold->setValue( mpParent->LowerNoiseThreshold() );
    mpUi->spinBox_maxNumPoints->setValue( mpParent->MaxNumPoints() );
    SetThresholdBounds( mpParent->Leftbound(), mpParent->Rightbound() );
    UpdateSourceInfo( );

    if (mImageList.size() > 0)
    {
        int width = mImageList[0]->Width();
        int height = mImageList[0]->Height();
        for (int i = 0; i < mThreshPreviewList.size(); ++i)
        {
            (mThreshPreviewList[i])->setMinimumSize( width, height );
        }
        UpdateVisiblePreviews();
    }
    mpUi->spinBox_timestep->setMaximum( mMaxTimestep );
    mpUi->spinBox_numCams->setMaximum( mNumCams );
}

void ThresholdDialog::UpdateVisiblePreviews()
{
    for (int i = 0; i < mThreshWindowList.size(); ++i)
    {
        (mThreshWindowList[i])->setVisible( i < mpUi->spinBox_numCams->value() );
        (mThreshWindowList[i])->setEnabled( i < mpUi->spinBox_numCams->value() );
    }
}

ThresholdDialog::~ThresholdDialog()
{
    std::cerr << "Destructing Thresholddialog." << std::endl;
    if (mImageList.size() > 0)
    {
        for (int i = 0; i < mImageList.size(); ++i )
        {
            delete mImageList[i];
        }
    }
    if (mImageSet.size() > 0)
    {
        ImageSet delete_vector;
        mImageSet.swap( delete_vector );
    }
    delete mpUi;
    //::INFO:: we have to ask politely for the C memory allocator to return memory to OS
    //otherwise it looks like we have a memory leak, when we don't.
    malloc_trim(0);
}

void ThresholdDialog::UpdateGrabbedPixel( uint32_t iCamIdx, uint32_t iX, uint32_t iY )
{
    mMutex.lock();
    CamPreviewCanvas* label;
    GetLabel( iCamIdx, label );
    if (NULL == label->pixmap())
    {
        std::cerr << "Error calling the pixmap of the label, it doesn't exist for some reason?" << std::endl;
        return;
    }
    QColor pixel_color(QImage(label->pixmap()->toImage()).pixel(iX,iY) );

    int h = 128; //128 chosen simply so that it wouldn't be confused with black.
    int s = 128;
    int v = 128;
    pixel_color.getHsv( &h, &s, &v);
    if ( !mpUi->checkBox_showOriginalImage->isChecked())
    {
        //If we click black, show black, otherwise show the value from the OriginalImage, not the single
        //threshold color
        if (0 != s || 0 != v)
        {
            if (NULL == mImageList[iCamIdx])
            {
                std::cerr << "No image in the image list( imagelist size:" << mImageList.size() << " ) for cam index " << iCamIdx << std::endl;
                return;
            }
            pixel_color = mImageList[iCamIdx]->ToQImage().pixel(iX,iY);
            pixel_color.getHsv( &h, &s, &v );
        }
    }
    if (UINT_MAX == static_cast<uint32_t>(h)) //Even though h is an int, its value can come back as UINT_MAX
    {
        h = 0;
    }
    mpUi->label_HGrabbed->setText( QString::number(h) );
    mpUi->label_SGrabbed->setText( QString::number(s) );
    mpUi->label_VGrabbed->setText( QString::number(v) );
    mMutex.unlock();
}

void ThresholdDialog::on_pushButton_cancel_clicked( )
{
    close();
}

bool ThresholdDialog::Cancelled()
{
    //:NOTE: We have a null op here because if we are loading from a video, we don't give the chance to
    //interrupt the reading from the video into images. And if we have images, we are only reading
    //each timestep on demand, so no real reason to check for cancel as it should be pretty responsive.
    return false;
}

void ThresholdDialog::UpdateSourceInfo( )
{
    //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
    {
        ImageSet delete_vector;
        delete_vector.swap(mImageSet);
    }
    //Read in image file locations
    QVector< QString > image_locations;
    QVector< QString > mask_locations;
    uint32_t num_mask_files = 0;
    int num_mask_cams = 0;
    uint32_t num_image_files = 0;
    //int numImageCams = 0;
    std::string regex_str = "([0-9]+)-([0-9]+)\\.(\\w+)";

    if (mpProjInfo->UseMask())
    {
        ReadXMLImagesFile( mpProjInfo->ProjectDir() + QDir::separator() + mpProjInfo->ProjectMaskXml(),
                           mask_locations,
                           num_mask_cams,
                           num_mask_files );

        std::string location_str;
        for (int index = 0; index < mask_locations.size(); ++index)
        {
            bool reg_found = false;
            std::smatch match_result;
            try
            {
                std::regex regexPattern( regex_str );
                location_str = (mpProjInfo->ProjectDir() +
                                QDir::separator() +
                                mask_locations[index]).toStdString();
                reg_found = regex_search( location_str, match_result, regexPattern );
            }
            catch (const std::regex_error& e)
            {
                std::cerr << "Problem with regex instantiation: " << e.what() << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << "Problem with regex_search: " << e.what() << std::endl;
            }

            if (!reg_found)
            {
                std::cerr << "Regular expression not found in mask file candidate. "
                             "Possible error with regex or with candidate"
                          << std::endl;
                std::cerr << "Regex: " << regex_str << std::endl;
                std::cerr << "Candidate: " << (mask_locations[index]).toStdString() << std::endl;
            }
            else
            {
                //For masks we only use the camera #
                std::string res1( match_result[1] );
                mFileListMap.insert(  QString( res1.c_str() ), mask_locations[index] );
            }
        }
    }

    ReadXMLImagesFile( mImagesXml, image_locations, mNumCams, num_image_files );

    if (mpProjInfo->UseMask())
    {
        assert( num_mask_cams == mNumCams );
    }
    mpUi->spinBox_numCams->setValue( mNumCams );

    //first find out the highest number of cameras listed for the images, then the highest timestep number
    //:TODO: these numbers could/should be read from the python generated xml image list files.

    TrackerConfigFile track_config;
    uint32_t capture_count = 0;
    std::string tri_dir;
    if (!mpParent->PopulateTrackerConfig(track_config, capture_count, tri_dir))
    {
        fprintf(stderr, "Error populating tracker config file.\n");
    }
    std::string location_str;
    for (int index = 0; index < image_locations.size(); ++index)
    {
        std::string suffix = GetFileExt( image_locations[index].toStdString() );
        if ( "avi" == suffix )
        {
            if (0 == mImageSet.size())
            {
                //We are making the assumption that if one of the files is a video file then all
                //the rest are as well. No mix of video and images files.
                mImageSet.resize( image_locations.size() );
            }
            unsigned int cam_num = 0;
            unsigned int image_width = 0;
            unsigned int image_height = 0;
            uint64_t    total_frames = 0;
            float cam_hz = 0;
            float video_hz = 0;
            std::string file_type = "";

            bool parseable = ParseVideoFilename( image_locations[index].toStdString(),
                                                 cam_num,
                                                 image_width,
                                                 image_height,
                                                 total_frames,
                                                 cam_hz,
                                                 video_hz,
                                                 file_type);
            if (parseable)
            {
                std::vector<SmtImage> camera_images;
                ::ReadImages( image_locations[index].toStdString(),
                            track_config,
                            total_frames,
                            image_width,
                            image_height,
                            cam_num,
                            camera_images,
                            ThresholdDialog::Cancelled );
                if (camera_images.size() -1  > mMaxTimestep)
                {
                    mMaxTimestep = camera_images.size() - 1 ;
                }
                mImageSet[cam_num] = camera_images;
            }
        }
        else
        {
            int camNum = 0;
            bool reg_found = false;
            std::smatch match_result;

            try
            {
                std::regex regex_pattern( regex_str );
                location_str = (mpProjInfo->ProjectDir() +
                                QDir::separator() +
                                image_locations[index]).toStdString();
                reg_found = regex_search( location_str, match_result, regex_pattern );
            }
            catch (const std::regex_error& e)
            {
                std::cerr << "Problem with regex instantiation: " << e.what() << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << "Problem with regex_search: " << e.what() << std::endl;
            }

            if (!reg_found)
            {
                std::cerr << "Regular expression not found in filename candidate. "
                             "Possible error with regex or with candidate"
                          << std::endl;
                std::cerr << "Regex: " << regex_str << std::endl;
                std::cerr << "Candidate: " << (image_locations[index]).toStdString() << std::endl;
            }
            else
            {
                std::string res1( match_result[1] );
                std::string res2( match_result[2] );
                std::string filename( res1 + "-" + res2 );

                mFileListMap.insert(  QString( filename.c_str() ), image_locations[index] );
                if (std::stoi(res1) > camNum)
                {
                    camNum = std::stoi(res1);
                }
                if (stoui(res2) > mMaxTimestep)
                {
                    mMaxTimestep = std::stoi(res2);
                }
            }
        }

    }
//    for (int i =0; i < mThreshPreviewList.size(); ++i)
//    {
//        delete mThreshPreviewList[i];
//    }
//    for (int i =0; i < mThreshWindowList.size(); ++i)
//    {
//        delete mThreshWindowList[i];
//    }
    mThreshPreviewList.clear();
    mThreshWindowList.clear();

    ReadImages( );

    for (int i =0; i < mNumCams; ++i)
    {
        CamPreviewWindow* prev_window = new CamPreviewWindow( mpUi->imageframe, i );
        mThreshPreviewList.push_back( prev_window->GetLabel() );
        if (mImageList.size() > 0)
        {
            prev_window->UpdateImageSize( mImageList[0]->Width(), mImageList[0]->Height() );
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        mThreshWindowList.push_back( prev_window );
        connect( prev_window,
                 SIGNAL(SignalGrabbedPixelValue(uint32_t,uint32_t,uint32_t)),
                 this,
                 SLOT(UpdateGrabbedPixel(uint32_t,uint32_t,uint32_t)) );

    }
    UpdateImages();
}

void ThresholdDialog::on_pushButton_ok_clicked()
{
    mpParent->SetThresholdBounds( mpUi->spinBox_HLeftbound->value(),
                                mpUi->spinBox_SLeftbound->value(),
                                mpUi->spinBox_VLeftbound->value(),
                                mpUi->spinBox_HRightbound->value(),
                                mpUi->spinBox_SRightbound->value(),
                                mpUi->spinBox_VRightbound->value() );
    mpParent->SetNumIterations( (uint32_t)mpUi->spinBox_numIterations->value() );
    mpParent->SetLowerNoiseThreshold( (uint32_t)mpUi->horizontalSlider_threshold->value() );
    mpParent->SetFilterSize( (uint32_t)mpUi->spinBox_filterSize->value() );
    close();
}

void ThresholdDialog::ReadXMLImagesFile( const QString& iLocation,
                                         QVector< QString >& oImageLocations,
                                         int& oNumCams,
                                         uint32_t& oNumImagesFound )
{
    std::string location = iLocation.toStdString();
    std::string base_dir = mpProjInfo->ProjectDir().toStdString();
    std::vector< std::string > image_locations;
    uint32_t num_cams = 0;
    uint64_t num_images = 0;
    ReadXMLImages( location, base_dir, image_locations, num_cams, num_images);

    foreach (std::string str, image_locations)
    {
        oImageLocations.push_back( QString(str.c_str()) );
    }
    assert( num_cams < INT_MAX );
    assert( num_images < UINT_MAX );
    oNumCams = static_cast<int>(num_cams);
    oNumImagesFound = static_cast<uint32_t>( num_images );
}

uint32_t ThresholdDialog::ReadImages( )
{
    mImageLock.lockForWrite();

    for (int32_t i =0; i < mImageList.size(); ++i)
    {
        delete mImageList[i];
    }
    mImageList.clear();

    uint32_t imageCount = 0;

    for (int32_t i = 0; i < mNumCams; ++i)
    {
        if (mImageSet.size() > 0)
        {
            assert( mImageSet[i].size() > 0);
            uint64_t timestep = mpUi->spinBox_timestep->value();
            GuiImage* newImage = new GuiImage( mImageSet[i][timestep],
                                               timestep );
            mImageList.push_back( newImage );
        }
        else
        {
            std::ostringstream stringStream;
            stringStream << i << "-" << mpUi->spinBox_timestep->value();

            QMap< QString, QString >::const_iterator fileIter = mFileListMap.find( QString( stringStream.str().c_str() ));
            if ( fileIter != mFileListMap.end())
            {
                QMap< QString, QString >::const_iterator maskIter = mFileListMap.find( QString::number(i) );
                if ( maskIter != mFileListMap.end())
                {
                    GuiImage* newImage = new GuiImage( fileIter.value().toUtf8().constData(),
                                                       i,
                                                       mpUi->spinBox_timestep->value(),
                                                       0 );
                    mImageList.push_back( newImage );
                }
                else
                {
                    GuiImage* newImage = new GuiImage( fileIter.value().toUtf8().constData(),
                                                       i,
                                                       mpUi->spinBox_timestep->value(),
                                                       0 );
                    mImageList.push_back( newImage );
                }
            }
            else
            {
                std::fprintf( stderr,
                              "Error attempting to read '%s' image from the fileListMap.\n",
                              stringStream.str().c_str() );
            }
        }
    }
    mImageLock.unlock();

    return imageCount;
}

bool ThresholdDialog::GetLabel( const int iLabelIdx, CamPreviewCanvas*& oLabelPtr )
{
    if (iLabelIdx < mThreshPreviewList.size())
    {
        oLabelPtr = mThreshPreviewList[iLabelIdx];
        if (0x0 != oLabelPtr)
        {
            return true;
        }
    }
    return false;
}

void ThresholdDialog::UpdateThresholdPreviewWindow(int iLabelIdx, QPixmap iImage)
{
    CamPreviewCanvas* label;
    if (this->GetLabel( iLabelIdx, label ))
    {
        label->setPixmap(iImage);
        label->update();
    }
    else
    {
        std::cerr << "Error updating Viewfinder because label idx was not valid. Idx="
                  << iLabelIdx
                  <<  std::endl;
    }
}

void ThresholdDialog::on_pushButton_visualize_clicked()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_HLeftbound_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_HRightbound_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_SLeftbound_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_SRightbound_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_VLeftbound_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_VRightbound_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_numCams_editingFinished()
{
   UpdateVisiblePreviews();
}

void ThresholdDialog::on_checkBox_showOriginalImage_toggled()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_numIterations_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_filterSize_editingFinished()
{
    UpdateImages();
}

void ThresholdDialog::on_spinBox_timestep_editingFinished()
{
    ReadImages( );
    UpdateImages();
}

void ThresholdDialog::SetThresholdBounds( uint32_t iLH,
                                          uint32_t iLS,
                                          uint32_t iLV,
                                          uint32_t iRH,
                                          uint32_t iRS,
                                          uint32_t iRV )
{
    mpUi->spinBox_HLeftbound->setValue( iLH );
    mpUi->spinBox_SLeftbound->setValue( iLS );
    mpUi->spinBox_VLeftbound->setValue( iLV );

    mpUi->spinBox_HRightbound->setValue( iRH );
    mpUi->spinBox_SRightbound->setValue( iRS );
    mpUi->spinBox_VRightbound->setValue( iRV );

}

void ThresholdDialog::SetThresholdBounds( cv::Scalar iLeftBound, cv::Scalar iRightBound )
{
    mpUi->spinBox_HLeftbound->setValue( iLeftBound[0] );
    mpUi->spinBox_SLeftbound->setValue( iLeftBound[1] );
    mpUi->spinBox_VLeftbound->setValue( iLeftBound[2] );

    mpUi->spinBox_HRightbound->setValue( iRightBound[0] );
    mpUi->spinBox_SRightbound->setValue( iRightBound[1] );
    mpUi->spinBox_VRightbound->setValue( iRightBound[2] );
}

void ThresholdDialog::UpdateImages()
{
    if (mpUi->checkBox_showOriginalImage->isChecked())
    {
        for (int i = 0; i < mpUi->spinBox_numCams->value(); ++i)
        {
            if (!mImageLock.tryLockForRead())
            {
                std::fprintf(stderr, "Failed to get Read lock1\n");
                return; //must be re-writing images
            }
            GuiImage* image = mImageList[i];
            mImageLock.unlock();
            UpdateThresholdPreviewWindow( i, image->ToQPixmap() );
        }
    }
    else{
        for (int i = 0; i < mpUi->spinBox_numCams->value(); ++i)
        {
            if (!mImageLock.tryLockForRead())
            {
                std::fprintf(stderr, "Failed to get Read lock2\n");
                return; //must be re-writing images
            }
            GuiImage* image = mImageList[i];
            mImageLock.unlock();
            QPixmap pixmap;
            image->ThresholdImage( mpUi->spinBox_filterSize->value(),
                                   mpUi->spinBox_numIterations->value(),
                                   mpUi->spinBox_maxNumPoints->value(),
                                   mpUi->horizontalSlider_threshold->value(),
                                   cv::Scalar( mpUi->spinBox_HLeftbound->value(),
                                               mpUi->spinBox_SLeftbound->value(),
                                               mpUi->spinBox_VLeftbound->value() ),
                                   cv::Scalar( mpUi->spinBox_HRightbound->value(),
                                               mpUi->spinBox_SRightbound->value(),
                                               mpUi->spinBox_VRightbound->value() ),
                                   pixmap);

            UpdateThresholdPreviewWindow( i, pixmap );
        }
    }
}

void ThresholdDialog::on_pushButton_rightbound_clicked()
{
    QColorDialog color_diag;
    QColor color;
    color.setHsv( mpUi->spinBox_HRightbound->value(),
                  mpUi->spinBox_SRightbound->value(),
                  mpUi->spinBox_VRightbound->value() );
    color_diag.setCurrentColor( color );
    color_diag.exec();

    color = color_diag.currentColor();

    int h, s, v;
    color.getHsv( &h, &s, &v );
    QString back_str("background-color: #"
                          + QString(color.red() < 16? "0" : "") + QString::number(color.red(),16)
                          + QString(color.green() < 16? "0" : "") + QString::number(color.green(),16)
                          + QString(color.blue() < 16? "0" : "") + QString::number(color.blue(),16) + ";");
    std::cerr << "From ColorDialog: " << back_str.toStdString() << std::endl;

    mpUi->spinBox_HRightbound->setValue( h );
    mpUi->spinBox_SRightbound->setValue( s );
    mpUi->spinBox_VRightbound->setValue( v );
}


void ThresholdDialog::on_pushButton_leftbound_clicked()
{
    QColorDialog color_diag;
    QColor color;
    color.setHsv( mpUi->spinBox_HLeftbound->value(),
                  mpUi->spinBox_SLeftbound->value(),
                  mpUi->spinBox_VLeftbound->value() );
    color_diag.setCurrentColor( color );
    color_diag.exec();

    color = color_diag.currentColor();

    int h, s, v;
    color.getHsv( &h, &s, &v );
    QString backstr("background-color: #"
                          + QString(color.red() < 16? "0" : "") + QString::number(color.red(),16)
                          + QString(color.green() < 16? "0" : "") + QString::number(color.green(),16)
                          + QString(color.blue() < 16? "0" : "") + QString::number(color.blue(),16) + ";");
    std::cerr << "From ColorDialog: " << backstr.toStdString() << std::endl;
    mpUi->spinBox_HLeftbound->setValue( h );
    mpUi->spinBox_SLeftbound->setValue( s );
    mpUi->spinBox_VLeftbound->setValue( v );

}

void ThresholdDialog::on_spinBox_maxNumPoints_editingFinished()
{
    mpParent->MaxNumPoints( mpUi->spinBox_maxNumPoints->value() );
}
