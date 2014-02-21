/*******************************************************************************
 *  Clock.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Clock.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef CLOCK_H
#define CLOCK_H

#include <time.h>
#include <assert.h>
#include <sys/time.h>
#include <string>
#include <sstream>

/**
 * @brief Millisecond clock
 * @author (R5)
 * This class provides a clock with millisecond accuracy. It is implemented as a singleton.
 * The timestamps returned by the method getTimestamp are relative to the creation time of the
 * first instance of this class.
 */
class Clock
{

	public:

		/**
		 * Returns instance of the singleton
		 * @return instance of the singleton  
		 */
		static Clock* getInstance();

    /**
     * Returns timestamp in ms (milliseconds) since the first instance of clock has been created
     * @return timestamp in ms
     */
    unsigned int getTimestamp();

    /**
     * Returns timestamp in ms (milliseconds) since the first instance of clock has been created
     * @return timestamp in ms
     */
    std::string getTimeString();
    std::string getTimeString( unsigned timestamp );

    /** @return String containing filename-compatible date string */
    static std::string getFilenameDateString();

		/**
		 * Destructor
		 */
		virtual ~Clock();

	private:

		/**
		 * Private default Constructor (-->Singleton)
		 */
		Clock();

		/**
		 * Single instance of the clock
		 */
		static Clock* instance;

		/**
		 * Creation of the clock
		 */
		timeval m_StartTime;
};
#endif
