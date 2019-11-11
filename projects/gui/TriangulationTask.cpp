#include <QEventLoop>
#include <QCoreApplication>

#include "TriangulationTask.h"
#include "CorrespondenceDialog.h"


TriangulationTask::TriangulationTask( const QString& iTriConfigPath, QWidget* ipMainWindow )
    :mConfigPath(iTriConfigPath),
     mpMainWindow(ipMainWindow),
     mWaitingOnUserHelp(false)
{
    mpEventLoop = new QEventLoop();
}

TriangulationTask::~TriangulationTask()
{
    if (NULL != mpEventLoop)
    {
        delete mpEventLoop;
    }
}

void TriangulationTask::run()
{
    TriangulationPipeline* pipeline = TriangulationPipeline::Instance();

    int ret_code = 0;
    TrackerConfigFile config_file;
    ProcessConfigFile( mConfigPath.toStdString(), config_file );
    pipeline->Initialize( config_file );

    if (!pipeline->Cancelled())
    {
        pipeline->Undistort();
    }
    else
    {
        ret_code = -1;
    }

    if (!pipeline->Cancelled())
    {
        pipeline->Threshold();
    }
    else
    {
        ret_code = -1;
    }

    if (!pipeline->Cancelled())
    {
        pipeline->ClusterPixels();
    }
    else
    {
        ret_code = -1;
    }

    if (!pipeline->Cancelled())
    {
        pipeline->FirstMatchingTimestep();
    }
    else
    {
        ret_code = -1;
    }

    if ( !pipeline->Cancelled() && pipeline->RunUserMatchingHelper() )
    {
        //We need user help to give us the inital configuration for the markers. Call
        //back to the main window to bring up a dialog for user input.
        std::cerr << "Sending request for user help." << std::endl;
        emit GetUserHelpForCorrespondence(this,
                         pipeline->Images(),
                         pipeline->TimestepUsed(),
                         pipeline->MarkersFound(),
                         pipeline->CamsToExclude(),
                         pipeline->NumMarkersToFind());

        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        mWaitingOnUserHelp = true;
        //wait for user processing to finish
        while ( mWaitingOnUserHelp )
        {
            sleep(1);
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        }
        //mpEventLoop->exec();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);

        std::cerr << "User Signaled they were done helping." << std::endl;
        pipeline->GetNewMarkedPointsInfo();
    }



    if (!pipeline->Cancelled())
    {
        bool lib_load_succ = pipeline->PointCorrespondence();
        if (!lib_load_succ)
        {
            std::cerr << "There was an error with the Matching algorithm library." << std::endl;
            ret_code = -2;
        }
    }
    else
    {
        ret_code = -1;
    }
    time_t startFull = std::time(nullptr);

    if (!pipeline->Cancelled())
    {
        pipeline->Triangulate();
    }
    else
    {
        ret_code = -1;
        time_t endFull = std::time(nullptr);
        std::fprintf( stderr, "%lu seconds to calculate all optimal corrections.\n", endFull - startFull);
    }
    pipeline->Cleanup();
    TriangulationPipeline::CloseInstance();
    emit finished( ret_code );
}

void TriangulationTask::ContinueRunning()
{
    mpEventLoop->quit();
    mWaitingOnUserHelp = false;
}

void TriangulationTask::Cancel()
{
    mpEventLoop->quit();
    mWaitingOnUserHelp = false;
    TriangulationPipeline* pipeline = TriangulationPipeline::Instance();
    pipeline->Cancel();

}
