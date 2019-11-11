#ifndef utils_ErrorHandling_h
#define utils_ErrorHandling_h

//Breaking some code design rules by including these SIG includes here, but makes things nicer for a developer//
#include <execinfo.h> //SIGSEV handler
#include <signal.h> //SIGSEV handler
#include <cxxabi.h> //SIGSEV handler
#include <cstdlib> //SIBABRT
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <ucontext.h>
#include <unistd.h>


/* This structure mirrors the one found in /usr/include/asm/ucontext.h */
typedef struct _sig_ucontext {
    unsigned long     uc_flags;
    struct ucontext   *uc_link;
    stack_t           uc_stack;
    struct sigcontext uc_mcontext;
    sigset_t          uc_sigmask;
} sig_ucontext_t;

void CritErrHdlr( int sig_num, siginfo_t * info, void * ucontext );

void InstallSignal( int __sig );

#include <iostream>
#include <string>
#include <exception>
#include <stdexcept>

void HandleException(std::exception_ptr eptr); // passing by value is ok



#endif //utils_ErrorHandling_h
