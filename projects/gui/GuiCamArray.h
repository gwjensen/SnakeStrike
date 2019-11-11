#ifndef GuiCamArray_h
#define GuiCamArray_h

#include <QObject>

#include <GuiCamera.h>

class GuiCamArray: public QObject
{
    Q_OBJECT
    public:
        explicit GuiCamArray( QObject* ipParent, const QString& iCamLibPath );
        ~GuiCamArray();
        GuiCamera& operator[] (int i) const {return *(mCamList[i]);}

        void PopulateCams( QStringList& oCameraNames );
        bool GetCamera( const uint32_t iCamIndex, GuiCamera*& oCamera );
        void StartGrabbingPreview();
        void StartGrabbingRecord( const QString& iDir, int iNumImages );
        void StopGrabbing();
        int  Size();
        void Open();
        void Close();
        void MakeConnect();
        bool Open() const { return mOpened; }
        bool Stopped() const { return mStopped; }
        bool Connected() const { return mConnected; }
        void SetMaxBuffer( int64_t iNumImages );
        void SetRetrievalTimeout( uint64_t iTimeoutMilSecs );
        void OutputVideo( );
        void OutputImages( );
    //public slots:

    signals:
        void SignalStartGrabbingPreviewAll();
        void SignalStartGrabbingRecordAll( QString iDir, int iNumImages );
        void SignalStopGrabbingAll();
        //void Error( QString iError );

    private:
        std::vector< std::unique_ptr<GuiCamera> > mCamList;
        QObject *mpParent;
        bool mStopped;
        bool mConnected;
        bool mOpened;
        QVector<QThread*> mCamThreads;
        void* mpLibHndl;
        QString mCamLibPath;

};

#endif // GuiCamArray_h

