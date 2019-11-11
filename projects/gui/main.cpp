#include <dlfcn.h>
#include <QApplication>

#include "stacktrace.h"
#include "utils/ErrorHandling.h"

#include "MainWindow.h"


int main(int argc, char *argv[]) {
    InstallSignal(SIGSEGV);   // install our handler
    InstallSignal(SIGTERM);
    InstallSignal(SIGABRT);



    int res = 127;
    {
        QApplication app(argc, argv);
        MainWindow w;
        w.show();
        //Make sure the preview windows are set correctly according to their size
        w.RefreshPreviewWindows();

        res = app.exec();
    }
    return res;
}
