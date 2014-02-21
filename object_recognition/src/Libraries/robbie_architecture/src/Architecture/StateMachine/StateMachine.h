/*******************************************************************************
 *  StateMachine.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef StateMachine_H
#define StateMachine_H

#include <string>
#include <map>
#include <sstream>
#include <deque>
#include <typeinfo>

#include "ros/ros.h"
#include "../../Architecture/Singleton/Clock.h"

#define ADD_MACHINE_STATE( STATE_MACHINE, STATE ) { STATE_MACHINE.addState( STATE, #STATE ); }

/**
 * @class  StateMachine
 * @brief  Generic state machine implementation
 * @author David Gossow (RX)
 */
template<typename StateEnumT>
class StateMachine
{
  public:

    /** @brief The constructor */
    StateMachine( );

    /** @brief The destructor */
    ~StateMachine();

    /// @brief Set machine name
    void setName( std::string name ) { m_Name=name; }

    /// @brief Set maximal number of entries in state history
    void setHistoryLength( unsigned historyLength ) { m_HistoryLength=historyLength; }

    /// @brief Register a new state id and its description. Used by ADD_MACHINE_STATE()
    void addState( StateEnumT state, std::string name ) { m_StateNames[ state ] = name; }

    /// @return current machine state
    StateEnumT state() { return m_State; }

    /// @return previous state (starting at index 0)
    StateEnumT historyState( unsigned steps );

    /// @return textual description of machine name and current state
    std::string stateString() { return m_Name + ": " + stateString( m_State ); }

    /// @return textual description of given state
    std::string stateString( StateEnumT state );

    /// @brief Change the machine's state
    void setState( StateEnumT state );

    /// @brief Reverse state to previous one
    void setLastState();

    /// @brief Milliseconds since last state change
    unsigned timeSinceStateChange();

    /// @return extensive information about the machine's properties, history etc.
    std::string info();

  private:

    StateEnumT m_State;

    std::map< StateEnumT, std::string > m_StateNames;

    std::string m_Name;

    unsigned m_LastStateChange;

    std::deque<StateEnumT> m_StateHistory;
    unsigned m_HistoryLength;

};


template<typename StateEnumT> unsigned StateMachine<StateEnumT>::timeSinceStateChange( )
{
  return Clock::getInstance()->getTimestamp() - m_LastStateChange;
}


template<typename StateEnumT> StateMachine<StateEnumT>::StateMachine( )
{
  m_Name = "Unnamed State Machine";
  m_State = StateEnumT(0);
  m_LastStateChange = Clock::getInstance()->getTimestamp();
  m_HistoryLength=20;
}


template<typename StateEnumT> StateMachine<StateEnumT>::~StateMachine( )
{
}


template<typename StateEnumT> std::string StateMachine<StateEnumT>::stateString( StateEnumT state )
{
  if ( m_StateNames.find( state ) == m_StateNames.end() )
  {
    std::ostringstream stream;
    stream << "Invalid state (" << state << ")";
    return stream.str();
  }
  return m_StateNames[ state ];
}


template<typename StateEnumT> StateEnumT StateMachine<StateEnumT>::historyState( unsigned steps )
{
  if ( steps < m_StateHistory.size() )
  {
    return m_StateHistory[ m_StateHistory.size()-1-steps ];
  }
  else if ( m_StateHistory.size() != 0 )
  {
    return m_StateHistory.front();
  }
  else
  {
    return m_State;
  }
}


template<typename StateEnumT> void StateMachine<StateEnumT>::setState( StateEnumT state )
{
  if ( m_State != state )
  {

    if ( m_StateNames.find( state ) == m_StateNames.end() )
    {
      std::ostringstream stream;
      stream << "Invalid state (" << state << ")!" << std::endl << info();
      ROS_ERROR_STREAM( stream.str() );
    }

    std::ostringstream stream;
    stream << "Setting state of '" << m_Name << "' to " << stateString( state );
    stream << " after " << float(timeSinceStateChange())/1000.0 << "s";
    ROS_INFO_STREAM( stream.str() );

    m_StateHistory.push_back( m_State );
    if ( m_StateHistory.size() > m_HistoryLength )
    {
      m_StateHistory.pop_front();
    }

    m_State = state;
    m_LastStateChange = Clock::getInstance()->getTimestamp();
  }
}


template<typename StateEnumT> void StateMachine<StateEnumT>::setLastState()
{
  std::ostringstream stream;
  stream << "Reversing state of '" << m_Name << "' to " << m_StateHistory.back();
  stream << " after " << float(timeSinceStateChange())/1000.0 << "s";
  ROS_INFO_STREAM( stream.str() );

  m_State = m_StateHistory.back();
  m_StateHistory.pop_back();

  m_LastStateChange = Clock::getInstance()->getTimestamp();
}


template<typename StateEnumT> std::string StateMachine<StateEnumT>::info()
{
  std::ostringstream stream;
  stream << "State machine:     " << m_Name << std::endl;
  stream << "Current state:     " << stateString( m_State ) << std::endl;
  stream << "State type:        " << typeid( StateEnumT ).name() << std::endl;

  stream << "Registered states: ";
  typename std::map< StateEnumT, std::string >::iterator it;
  it=m_StateNames.begin();
  while ( it != m_StateNames.end() )
  {
    if ( it != m_StateNames.begin() ) { stream << ", "; }
    stream << it->second << "(" << it->first << ")";
    it++;
  };

  stream << "Previous states:   ";
  for( unsigned i=0; i<m_StateHistory.size(); i++ )
  {
    if ( i!=0 ) { stream << " -> "; }
    stream << stateString( m_StateHistory[i] );
  }

  return stream.str();
}

#endif
