#include <execinfo.h> //SIGSEV handler
#include <signal.h> //SIGSEV handler
#include <cxxabi.h> //SIGSEV handler
#include <cstdlib> //SIBABRT
#include <execinfo.h>
#include <signal.h>
#include <string.h>
#include <cstdio>
#include <ucontext.h>
#include <unistd.h>
#include "opensource/stacktrace.h"

#include "ErrorHandling.h"


[[ noreturn ]] void CritErrHdlr( int sig_num, siginfo_t * info, void * ucontext )
{
    if (sig_num > 0 || info ==NULL || ucontext ==NULL)
    {
        //just put this here to make sure I didnt' change the function signature
    }
    //void *             caller_address;
    //char **            messages;
    //int                size, i;
    //sig_ucontext_t *   uc;

    //uc = reinterpret_cast<sig_ucontext_t *>(ucontext);

    /* Get the address at the time the signal was raised */
    #if defined(__i386__) // gcc specific
    //caller_address = (void *) uc->uc_mcontext.eip; // EIP: x86 specific
            #elif defined(__x86_64__) // gcc specific
    //caller_address = reinterpret_cast<void *>(uc->uc_mcontext.rip); // RIP: x86_64 specific
    #else
    #error Unsupported architecture. // TODO: Add support for other arch.
    #endif

    std::fprintf( stderr, "\n" );
    //FILE * backtraceFile;

    // In this example we write the stacktrace to a file. However, we can also just fprintf to stderr (or do both).
    opensource::print_stacktrace();
    /*string backtraceFilePath = "ERROR_stacktrace.txt";
    backtraceFile = fopen(backtraceFilePath.c_str(),"w");

    if (sig_num == SIGSEGV)
        fprintf(backtraceFile, "signal %d (%s), address is %p from %p\n",sig_num, strsignal(sig_num), info->si_addr,(void *)caller_address);
    else
        fprintf(backtraceFile, "signal %d (%s)\n",sig_num, strsignal(sig_num));

    size = backtrace(array, 50);
    // overwrite sigaction with caller's address
    array[1] = caller_address;
    messages = backtrace_symbols(array, size);
    // skip first stack frame (points here)
    for (i = 1; i < size && messages != NULL; ++i) {
        fprintf(backtraceFile, "[bt]: (%d) %s\n", i, messages[i]);
    }

    fclose(backtraceFile);
    free(messages);*/

    exit( EXIT_FAILURE );
}

void InstallSignal( int __sig )
{
    struct sigaction sigact = {0};
    sigact.sa_sigaction = CritErrHdlr;
    sigact.sa_flags = SA_RESTART | SA_SIGINFO;

    if( sigaction( __sig, &sigact, reinterpret_cast<struct sigaction *>(NULL) ) != 0 ) {
        std::fprintf( stderr, "error setting signal handler for %d (%s)\n",__sig, strsignal(__sig) );
        exit( EXIT_FAILURE );
    }
}

void HandleException(std::exception_ptr eptr) // passing by value is ok
{
    try {
        if (eptr) {
            std::rethrow_exception(eptr);
        }
    } catch(const std::exception& e) {
        std::cout << "Caught exception \"" << e.what() << "\"\n";
    }
}
