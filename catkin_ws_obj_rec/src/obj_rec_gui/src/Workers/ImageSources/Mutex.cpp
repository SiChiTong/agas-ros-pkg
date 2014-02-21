/*******************************************************************************
 *  Mutex.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Mutex.cpp 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/


#include "Mutex.h"

Mutex::Mutex()
{
	m_Mutex = new TMutex;
	pthread_mutex_init ( m_Mutex, NULL );
}


Mutex::~Mutex()
{
	pthread_mutex_destroy ( m_Mutex );
	delete m_Mutex;
}


void Mutex::lock ()
{
	pthread_mutex_lock ( m_Mutex );
}


void Mutex::unlock()
{
	pthread_mutex_unlock ( m_Mutex );
}


TMutex* Mutex::getMutex()
{
	return m_Mutex;
}

#ifdef __TEST__

#include <iostream>

int main ( int argc, char **argv )
{
	std::cout << "UNITTEST" << std::endl;
	return 0;
}
#endif
