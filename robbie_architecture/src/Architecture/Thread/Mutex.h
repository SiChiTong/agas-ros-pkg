/*******************************************************************************
 *  Mutex.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Mutex.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef MUTEX_H
#define MUTEX_H

#include "Types.h"

/**
 * Protects a section of code to be executed by only a single thread
 * at any given time.
 *
 * The Mutex class provides a synchonization object that can be used to
 * protect sections of code to be executed by only a single thread at any
 * given time. This code fragment is also called a <i>critical section</i>,
 * a mutex is also called a <i>semaphore</i>.
 *
 * Mutex's have a lock() and unlock() method; only one thread can continue
 * from the lock() until the unlock() method is called. During this time, any
 * other thread has to wait until the mutex becomes available.
 *
 * @note This implementation does not provide recursive mutexes.
 * @note Currently, this implementation supports only POSIX threads.
 */
class Mutex
{

	public:

		/**
		 * Constructor 
		 */
		Mutex();

		/**
		 * Destructor 
		 */
		virtual ~Mutex();

		/**
		 * Enter critical sections 
		 */
		void lock ();

		/**
		 * Leave critical sections 
		 */
		void unlock();

		/**
		 * Returns pointer of the mutex
		 * @return pointer of the mutex
		 */
		TMutex* getMutex();

	private:

		/**
		 * Platform-dependent mutex representation.
		 */
		TMutex* m_Mutex;
};

#endif
