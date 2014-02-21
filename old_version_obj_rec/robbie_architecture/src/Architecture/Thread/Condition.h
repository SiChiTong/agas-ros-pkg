/*******************************************************************************
 *  Condition.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Condition.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/


#ifndef CONDITION_H
#define CONDITION_H

#include "Types.h"
#include "Mutex.h"

/**
 * A synchronization object for one-to-one or one-to-many control
 * events.
 *
 * The Condition class provides a synchonization object for one-to-one and
 * one-to-many control events. Using this class, it is possible to pause the
 * execution of a thread until it gets a signal from another thread to
 * continue. This signal can be sent to only a single thread (one-to-one event)
 * via signal or to all threads waiting for the condition object (one-to-many)
 * via broadcast.
 *
 * Note that sometimes signals are issued by the operating system and not only
 * by other threads. So the criterion you are waiting for may not be satisfied.
 * However, this can be handled using a loop:
 * @code
 *    Condition condition;
 *    bool criterion;
 *    Mutex mutex;
 *    mutex.lock();
 *    ...
 *    while( !criterion){
 *      condition.wait(mutex);
 *    }
 *    ...
 *    mutex.unlock();
 * @endcode
 * The signaling thread should use code like the following:
 * @code
 *    criterion = true;
 *    condition.signal();   // or "condition.broadcast()"
 * @endcode
 */
class Condition
{

	public:

		/**
		 * Default constructor.
		 * Creates a new platform-dependent condition object.
		 */
		Condition();

		/**
		 * Destructor.
		 * Destroys the instance and releases all associated system resources.
		 */
		virtual ~Condition();

		/**
		 * Wait for a signal.
		 * Pauses the thread, unlock's the given \a mutex and waits for a signal
		 * before the execution is continued.
		 * @param mutex Mutex that is unlocked until the thread wakes up again
		 */
		void wait ( Mutex& mutex );


    /**
     * Wait for a signal until a given maximum time has passed
     * @param maxTime Maximal wait time [ms]
     * @see wait( Mutex& )
     */
    void wait( Mutex& mutex, unsigned int maxTime );

    /**
		 * Issue a signal and wake up a single thread waiting for this condition.
		 * This is useful for a one-to-one control event.
		 */
		void signal();

		/**
		 * Issue a signal and wake up all threads waiting for this condition.
     * This is useful for a one-to-many control event.
		 */
		void broadcast();

	private:

		/**
		 * Platform-dependent condition representation. 
		 */
		TCondition* m_Condition;

		/**
		 * This variable is set to true when a signal arrives. It serves as a reminder in the case that a signal arrives
		 * before a process starts waiting on it. After every wait call this variable will be set to false.
		 */
		bool m_SignalArrived;

		/**
		 * This mutex is needed for locking the condition while the variable m_SignalArrived is modified and read by
		 * another process. Without this mutex it would be possible that signals arrive before a process starts waiting
		 * on them.
		 */
		Mutex m_SignalArrivedMutex;

};

#endif
