#ifndef TriangulationTask_h
#define TriangulationTask_h

#include <QObject>
#include <QRunnable>
#include <QString>
#include <QEventLoop>
#include <QMainWindow>

#include "TriangulationPipeline.h"

class TriangulationTask : public QObject, public QRunnable
{
    Q_OBJECT

    friend class TrackingDialog;

    public:
        TriangulationTask( const QString& iTriConfigPath, QWidget* ipMainWindow );
        virtual ~TriangulationTask();
        void run( );
        QEventLoop* WaitLoop(){ return mpEventLoop;}
        //static QMutex mutex;

    signals:
        void finished( int RetCode );
        void GetUserHelpForCorrespondence(TriangulationTask* iTaskWaitLoop,
                         std::vector< SmtImage > iTimestepImages,
                         const int iTimestepValue,
                         std::vector< std::vector< SmtPixel > > iTimestepMarkersFound,
                         std::pair<unsigned long, std::set<unsigned long> > iCamsToExclude,
                         const int iNumMarkersToFind);
    public slots:
        void ContinueRunning();
        void Cancel();

    private:
        QString mConfigPath;
        QWidget* mpMainWindow;
        QEventLoop* mpEventLoop;
        bool mWaitingOnUserHelp;
};

#endif // TriangulationTask_h
