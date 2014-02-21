/*******************************************************************************
 *  String.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "String.h"

#define THIS String

using namespace std;

THIS::THIS()
{
}

THIS::~THIS()
{
}

string THIS::upCase( string text )
{
  for (size_t i=0;i<text.length();i++) {
    if (text[i] >= 'a' && text[i] <= 'z') {
      text[i] += ('A'-'a');
    }
  }
  return text;
}

string THIS::downCase( string text )
{
  for (size_t i=0;i<text.length();i++) {
    if (text[i] >= 'A' && text[i] <= 'Z') {
      text[i] -= ('A'-'a');
    }
  }
  return text;
}

std::vector<std::string> THIS::explode (const std::string& text, const std::string& separators, bool keepSeparators)
{
  size_t n     = text.length ();
  size_t start = text.find_first_not_of (separators);
  std::vector<std::string> words;

  while (start < n)
  {
    size_t stop = text.find_first_of (separators, start);
    if (stop > n) stop = n;
    if ( keepSeparators )
    {
      words.push_back (text.substr (start, stop-start+1));
    }
    else
    {
      words.push_back (text.substr (start, stop-start));
    }
    start = text.find_first_not_of (separators, stop+1);
  };
  return words;
}

std::string THIS::removeCharacters( std::string text, std::string characters )
{
  std::vector<std::string> parts = explode( text, characters, false );

  string result="";
  for ( unsigned i=0; i<parts.size(); i++ )
  {
    result += parts[i];
  }

  return result;
}



#undef THIS
