/*******************************************************************************
 *  TwoWayThreadModule.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: TwoWayThreadModule.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/

#ifndef TWOWAYTHREADMODULE_H
#define TWOWAYTHREADMODULE_H

#include "Thread.h"
#include "Condition.h"
#include "ThreadQueue.h"

/**
 * @class  TwoWayThreadModule
 * @author Johannes Pellenz, David Gossow (RX)
 * @brief  generic template class for a thread with one outbound queue and multiple inbound queues
 *
 * Encapsulates the platform-specific details of a thread with an outbound
 * and an inbound queue. Such a thread is the basis for a dockable module
 * which is able to communicate with another instance by exchanging messages
 * which are pushed in the queues and can be of any type. The outbound queue
 * has the purpose to transport messages from the module to the controlling
 * instance (e.g. dispatcher) while the inbound queue transports messages
 * in the reverse direction.
 *
 * Every thread of execution in an application is created by instantiating
 * an object of a class derived from the Thread class. These classes must
 * implement the run() method, which specifies the code of the thread.
 * The type of the messages is set when using the template. Example:

 *
 */
template<class OutT, class InT>
class TwoWayThreadModule : public Thread
{

  public:

    /**
     * @brief The constructor. By default, one ingoing and one outgoing queue is created.
     * @note  sets his own condition object as the one for the inbound queue.
     */
    TwoWayThreadModule( size_t numInQueues ) : Thread()
    {

    }

    virtual ~TwoWayThreadModule()
    {}

    /**
     * Pushs an element in the outbound queue. The signaling of a waiting thread is done in the class
     * ThreadQueue and not here.
     * @param element the item which will be put in front of the outbound queue
     */
    void pushOut ( OutT element, size_t queueId )
    {
      m_OutboundQueue.push ( element );
    }

    /**
     * This method returns the first element of the outbound queue. If no element is present, it blocks until
     * a new is available. This behaviour is implemented in the class ThreadQueue.
     * @return the first element of the outbound queue
     */
    OutT popOut()
    {
      return ( m_OutboundQueue.pop() );
    }

    /**
     * @return the current number of elements in the outbound queue
     */
    int sizeOut()
    {
      return m_OutboundQueue.size();
    }

    /**
     * Pushs an element in the inbound queue. The signaling of a waiting thread is done in the class
     * ThreadQueue and not here.
     * @param element the item which will be put in front of the inbound queue
     *
     */
    void pushIn ( InT element )
    {
      m_InboundQueue.push ( element );
    }

    /**
     * This method returns the first element of the inbound queue. If no element is present, it blocks until
     * a new is available. This behaviour is implemented in the class ThreadQueue.
     * @return the first element of the inbound queue
     */
    InT popIn()
    {
      return ( m_InboundQueue.pop() );
    }

    /**
     * @return true, if the inbound queue contains no elements
     */
    bool emptyIn()
    {
      return m_InboundQueue.empty();
    }

    /**
     * @return the current number of elements in the inbound queue
     */
    int sizeIn()
    {
      return m_InboundQueue.size();
    }

  protected:

};

#endif
