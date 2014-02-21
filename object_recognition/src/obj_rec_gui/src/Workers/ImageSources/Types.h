/*******************************************************************************
 *  Types.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: R5; DevelTest: Date; Reviewer: R5; Review: Date; State: OK§
 *
 *  Additional information:
 *  $Id: Types.h 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/


#ifndef TYPES_H
#define TYPES_H


#define HAVE_PTHREAD_H
#define POSIX_THREADS

#ifdef HAVE_PTHREAD_H
#  include <pthread.h>
#endif


#ifdef POSIX_THREADS
typedef pthread_mutex_t TMutex;
typedef pthread_cond_t  TCondition;
typedef pthread_t       TThread;
#endif


/*----------------------------------------------------------------------------*/
/**
 *  \file  threads/Types.h
 *  \brief Defines platform-independent types for mutex, condition and thread
 *         variables.
 *
 *  This file provides definitions of platform-independent types that are used
 *  to implement the \ref Mutex, \ref Condition and \ref Thread classes. See
 *  the documentation of this classes for details.
 */
/*----------------------------------------------------------------------------*/


#endif
