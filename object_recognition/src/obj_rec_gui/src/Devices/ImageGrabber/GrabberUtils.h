#ifndef GRABBER_UTILS
#define GRABBER_UTILS

/***********************************************************************
 * basic functionalities for the puma grabber
 *********************************************************************/

#include <string>
#include <vector>

/**
 * converts a number to a string
 */
std::string toString ( long _value );

/**
 * split a string into substrings by finding the given seperators
 *
 */
std::vector<std::string> splitString ( const std::string &_orginal, const std::string &_seperators );

#endif
