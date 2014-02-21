/*******************************************************************************
 *  Condition.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Condition.cpp 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#include "Condition.h"

#include <sys/time.h>
#include <iostream>

using namespace std;


Condition::Condition()
{
	m_Condition = new TCondition;
	pthread_cond_init ( m_Condition, NULL );
	m_SignalArrived = false;
}


Condition::~Condition()
{
	pthread_cond_destroy ( m_Condition );
	delete m_Condition;
}


void Condition::wait ( Mutex& mutex )
{
	m_SignalArrivedMutex.lock();
	if ( m_SignalArrived )
	{
		m_SignalArrived = false;
		m_SignalArrivedMutex.unlock();
	}
	else
	{
		mutex.unlock();
		pthread_cond_wait ( m_Condition, m_SignalArrivedMutex.getMutex() );
		mutex.lock();
		m_SignalArrived = false;
		m_SignalArrivedMutex.unlock();
	}
}


void Condition::wait( Mutex& mutex, unsigned int maxTime )
{
  m_SignalArrivedMutex.lock();
  if ( m_SignalArrived || (maxTime==0) )
  {
    m_SignalArrived = false;
    m_SignalArrivedMutex.unlock();
  }
  else
  {
    struct timeval currentTime;
    gettimeofday ( &currentTime, NULL );

    long int sumUsec = currentTime.tv_usec + (maxTime*1000);
  
    struct timespec endTime;
    endTime.tv_sec = currentTime.tv_sec + ( sumUsec / 1000000 );
    endTime.tv_nsec = ( sumUsec % 1000000 ) * 1000;

    mutex.unlock();
    pthread_cond_timedwait ( m_Condition, m_SignalArrivedMutex.getMutex(), &endTime );
    mutex.lock();
    m_SignalArrived = false;
    m_SignalArrivedMutex.unlock();
  }
}


void Condition::signal()
{
	m_SignalArrivedMutex.lock();
	m_SignalArrived = true;
	pthread_cond_signal ( m_Condition );
	m_SignalArrivedMutex.unlock();
}


void Condition::broadcast()
{
	m_SignalArrivedMutex.lock();
	m_SignalArrived = true;
	pthread_cond_broadcast ( m_Condition );
	m_SignalArrivedMutex.unlock();
}

