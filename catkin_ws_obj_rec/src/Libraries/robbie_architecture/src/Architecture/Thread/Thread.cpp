/*******************************************************************************
 *  Thread.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Thread.cpp 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#include <iostream>

#include "Thread.h"
#include <sstream>
#include "ros/ros.h"

using namespace std;

/**
 * Wrapper function that executes the function that should run in a separate
 * thread. This wrapper is necessary on the x86 platform when using the GNU
 * C++ compiler to work around a bug in its code generation (incorrect stack
 * alignment).
 *
 * @param instance A pointer to the Thread-object which run method will be executed when the POSIX-thread
 *                 is startet
 */
void threadWrapper ( Thread* instance )
{
    // Execute thread's run code
    instance->run();

}

void Thread::start(const char* name)
{
    // Start function as a new thread using a wrapper function
    int ret = pthread_create ( &m_Thread, NULL, reinterpret_cast<void* ( * ) ( void* ) > ( threadWrapper ), this );
    if ( ret != 0 )
    {
        ostringstream stream;
        stream << "pthread_create returned error " << ret;
        ROS_ERROR_STREAM( stream.str() );
    }
    else
    {
        if (!name) {
            name="UnnamedThread";
        }
        std::ostringstream s;
        s << "Created new Thread: \"" << name << "\" " << (int)m_Thread << " "  << std::hex << m_Thread << std::dec;
        ROS_INFO_STREAM( s.str() );
    }

}

void Thread::cancel()
{
    const int max_retry = 5;
    int retry = 0;
    int ret;

    do
    {
        /*    cout << retry << ".." << flush;
            try {
                ROS_INFO_STREAM( "Canceling thread..." )
                sleep ( 1 );
                ret = pthread_cancel ( m_Thread );
            } catch (...) {
                ROS_INFO_STREAM( "pthread_cancel has thrown n exception" )
                throw;
            }
            sleep(1);
            sleep ( 1 );
        		ret = pthread_cancel ( m_Thread );
            cout << ret << endl;*/
        ret = pthread_cancel ( m_Thread );
        if ( ret != 0 )
        {
            ostringstream stream;
            stream << "pthread_cancel returned error " << ret;
            ROS_ERROR_STREAM( stream.str() );
        }
        retry++;
        usleep ( 1000000 );
    }
    while ( ( ret != 0 ) && ( retry < max_retry ) );

    if (ret!=0) {
        ROS_ERROR( "Canceling thread failed." );
    } else {
         ROS_DEBUG_STREAM( "Thread " << m_Thread << " cancelled." );
    }


}

void Thread::join()
{
    ROS_INFO_STREAM( "Joining Thread..." );
    int ret = pthread_join ( m_Thread, NULL );
    if ( ret != 0 )
    {
        ostringstream stream;
        stream << "pthread_join returned error " << ret;
       ROS_ERROR_STREAM( stream.str() );
    }
}

void Thread::setCancelSettings()
{
    // Setup thread so that it can be canceled from outside
    pthread_setcancelstate ( PTHREAD_CANCEL_ENABLE, NULL );
    pthread_setcanceltype ( PTHREAD_CANCEL_DEFERRED, NULL );
}

void Thread::testCancel()
{
    pthread_testcancel();
}

#ifdef __TEST__

#include <iostream>

int main ( int argc, char **argv )
{
    std::cout << "UNITTEST" << std::endl;
    return 0;
}
#endif
