/*******************************************************************************
 *  SequenceNumber.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: SequenceNumber.cpp 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#include "SequenceNumber.h"

SequenceNumber* SequenceNumber::instance = 0;

SequenceNumber* SequenceNumber::getInstance()
{
	if ( instance == 0 )
	{
		instance = new SequenceNumber();
	}
	return instance;
}

SequenceNumber::SequenceNumber()
{
	m_NextValue = 0;
}

unsigned long SequenceNumber::getSequenceNumber()
{
	m_NextValueMutex.lock();
	m_NextValue++;
	m_NextValueMutex.unlock();
	return m_NextValue;
}


SequenceNumber::~SequenceNumber()
{}

#ifdef __TEST__

#include <iostream>

int main ( int argc, char **argv )
{
	std::cout << "UNITTEST" << std::endl;

	return 0;
}
#endif

