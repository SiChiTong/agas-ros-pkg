/*******************************************************************************
 *  Clock.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Clock.cpp 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#include "Clock.h"
#include <sstream>

using namespace std;

Clock* Clock::instance = 0;


Clock* Clock::getInstance()
{
	if ( instance == 0 )
	{
		instance = new Clock();
	}
	return instance;
}


Clock::Clock()
{
	gettimeofday ( &m_StartTime, NULL );
}


unsigned int Clock::getTimestamp()
{
	struct timeval now;
	gettimeofday ( &now, NULL );
	unsigned int delta_s = now.tv_sec - m_StartTime.tv_sec;
	int  delta_us = now.tv_usec - m_StartTime.tv_usec;
	unsigned int delta_t = ( delta_s * 1000 ) + ( delta_us / 1000 );
	return delta_t;
}

std::string Clock::getTimeString( unsigned timestamp )
{
  ostringstream outStream;
  outStream.setf ( ios::right, ios::adjustfield );
  outStream.fill ( '0' );
  
  //hours
  outStream.width ( 2 );
  outStream << timestamp/3600000 << ":";
  
  //minutes
  outStream.width ( 2 );
  outStream << timestamp/60000 % 60 << ":";
  
  //seconds
  outStream.width ( 2 );
  outStream << timestamp/1000 % 60 << ":";
  
  //milliseconds
  outStream.width ( 3 );
  outStream << timestamp % 1000;
  
  return outStream.str();
}

std::string Clock::getTimeString()
{
  return getTimeString( getTimestamp() );
}

std::string Clock::getFilenameDateString()
{
  stringstream dateStr;
  time_t timestamp = time( 0 );
  tm* now = localtime( &timestamp );
  int year = now->tm_year + 1900;
  int month = now->tm_mon + 1;
  int day = now->tm_mday;
  int hour = now->tm_hour;
  int min = now->tm_min;
  int sec = now->tm_sec;

  dateStr << year << ".";
  dateStr.fill( '0' );
  dateStr.width( 2 );
  dateStr << month << ".";
  dateStr.width( 2 );
  dateStr << day << "_";
  dateStr.width( 2 );
  dateStr << hour << "-";
  dateStr.width( 2 );
  dateStr << min << "-";
  dateStr.width( 2 );
  dateStr  << sec;
  return dateStr.str();
}


Clock::~Clock()
{}

#ifdef __TEST__

#include <iostream>

int main ( int argc, char **argv )
{
	std::cout << "UNITTEST" << std::endl;

	return 0;
}
#endif
