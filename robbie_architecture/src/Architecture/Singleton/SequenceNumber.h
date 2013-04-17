/*******************************************************************************
 *  SequenceNumber.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: SequenceNumber.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef SEQUENCENUMBER_H
#define SEQUENCENUMBER_H

#include <time.h>
#include <assert.h>
#include <sys/time.h>

#include "../../Architecture/Thread/Mutex.h"

/**
 * @brief Singleton for generating unique numbers
 * @author (R5)
 */
class SequenceNumber
{

	public:

		/**
		 * Returns instance of the singleton
		 * @return instance of the singleton  
		 */
		static SequenceNumber* getInstance();

		/**
		 * Returns system-wide unique IDs for any kind of objects which than can be distinguished. This
		 * method is protected with a mutex and thus only allows one thread at a time to enter it.
		 * @return unique ID.
		 */
		unsigned long getSequenceNumber();

		/**
		 * Destructor
		 */
		virtual ~SequenceNumber();

	private:

		/**
		 * Private default Constructor (-->Singleton)
		 */
		SequenceNumber();

		/**
		 * Single instance of the SequenceNumber
		 */
		static SequenceNumber* instance;

		/**
		 * Number, that will be the next unique ID for a new message
		 */
		unsigned long m_NextValue;

		/**
		 * Mutex which is needed for incrementing m_NextValue
		 */
		Mutex m_NextValueMutex;
};

#endif


























