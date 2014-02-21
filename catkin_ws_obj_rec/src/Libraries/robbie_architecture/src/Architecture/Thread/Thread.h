/*******************************************************************************
 *  Thread.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Thread.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef THREAD_H
#define THREAD_H

#include "Types.h"  //Definitions for POSIX-Threads
#include "Mutex.h"

/**
 * Encapsulates the platform-specific details of a thread.
 *
 * Every thread of execution in an application is created by instantiating
 * an object of a class derived from the Thread class. These classes must
 * implement the run() method, which specifies the code of the thread.
 *
 * You may store data local to the thread in the instances of the derived
 * classes, but then you have to make sure, that the instance exists during
 * the whole time of execution. In other words: Do not create local variables
 * of a thread class!
 *
 * @note Currently, this implementation only supports POSIX threads.
 * @todo Add support for other thread libraries.
 *
 * Some error codes:
 * ESRCH    3
 * EINVAL  22
 * EDEADLK 35
 * EAGAIN  11 e.g. when the is "sleep"ing!
 */
class Thread
{

	public:

		virtual ~Thread() {};

		/**
		 * Start the execution of the thread.
		 * This method spawns a new thread which executes the run() method of the
		 * instance.
		 */
		void start(const char* name=0);

		/**
		 * Stops the threads.
		 */
		void cancel();

		/**
		 * Wait for the thread to terminate.
		 */
		void join();

		/**
		 * Test, if the thread was canceled.
		 */
		void testCancel();

		/**
		 * Sets the cancel settings of the thread.
		 */
		void setCancelSettings();

    /**
     * Code to execute in a new thread.
     * Derived classes have to override this method to specify the code that
     * will be executed in a new thread.
     */
    virtual void run() = 0;

	protected:

		/**
		 * Platform-specific handle for a thread.
		 */
		TThread m_Thread;
    

};

#endif
