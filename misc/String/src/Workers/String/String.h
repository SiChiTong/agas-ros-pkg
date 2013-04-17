/*******************************************************************************
 *  String.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef String_H
#define String_H

#include <string>
#include <sstream>
#include <vector>
#include <iostream>

/**
 * @class  String
 * @brief  String help functions
 * @author David Gossow (RX)
 */
class String
{
	public:

		/** @brief The constructor */
		String();

		/** @brief The destructor */
		~String();

		static std::string upCase ( std::string text );

		static std::string downCase ( std::string text );

		static std::string removeCharacters ( std::string text, std::string characters );

		/** @brief splits a string into substrings at certain separators
		 * @param separators string containing all possible separators
		*/
		static std::vector<std::string> explode ( const std::string& text, const std::string& separators, bool keepSeparators = false );

		template<class C>
		inline static std::string implode ( C& container, const std::string& separator )
		{
			typename C::const_iterator it;
			typename C::const_iterator it2;
			std::string result = "";
			for ( it = container.begin() ; it != container.end(); ++it )
			{
				it2 = it;
				it2++;
				if ( it2 != container.end() )
				{
					result += *it + separator;
				}
				else
				{
					result += *it;
				}
			}
			return result;
		}

		template<class T>
		inline static std::vector<T> convertTo ( std::vector<std::string> strVec )
		{
			std::vector<T> tVec;
			tVec.resize ( strVec.size() );

			for ( unsigned i = 0; i < strVec.size(); i++ )
			{
				std::stringstream s;
				s << strVec[i];
				s >> tVec[i];
			}
			return tVec;
		}

		/// @brief convert a string to some kind of variable
		/// example: int i = String::convertTo<int>( "16" );
		template<class T>
		inline static T convertTo ( std::string str )
		{
			std::stringstream s;
			T t;
			s << str;
			s >> t;
			return t;
		}

		/// convert a variable of arbitrary type T to a string (must be writable to a stream)
		template<class T>
		inline static std::string toString ( T value )
		{
			std::ostringstream s;
			s << value;
			return s.str();
		}

	private:

};

#endif
