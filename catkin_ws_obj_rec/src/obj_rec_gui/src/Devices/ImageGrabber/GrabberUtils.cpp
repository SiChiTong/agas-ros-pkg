#include "GrabberUtils.h"

#include <sstream>
#include <ctype.h>
#include <algorithm>

//using namespace puma2;
using namespace std;


string toString(long _value)
{
  ostringstream o;
  o<< _value;
  return (o.str());
}


// in addition to STL find_first_of ..
template<class T1, class T2> T1 find_first_not_of ( T1 first1, T1 last1, T2 first2, T2 last2 )
{
  T2 mid2;
  for ( ; first1 != last1; first1++ )
  {
    for ( mid2 = first2; mid2 != last2; ++mid2 )
      if ( *first1 == *mid2 ) break;
    if ( mid2 == last2 ) return ( first1 );
  }
  return ( first1 );
}

vector<string> splitString(const string &_orginal, const string &_seperators)
{
  vector<string> elements;
  string::const_iterator endIt, beginIt = _orginal.begin();

  do
    {
      endIt = find_first_of(beginIt, _orginal.end(), _seperators.begin(), _seperators.end());
      if (beginIt != endIt)
	{
	 elements.push_back("");
	  copy(beginIt, endIt, back_inserter(elements.back()));
	}
      beginIt = find_first_not_of(endIt, _orginal.end(), _seperators.begin(), _seperators.end());

    } while (beginIt != _orginal.end());

  return elements;
}
