/**
 * @file    PumaException.h
 * @brief   Contains PumaException.
 *
 * (c) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $ 
 */

#ifndef PUMA_EXCEPTION_H
#define PUMA_EXCEPTION_H

#include <exception>
#include <string>

namespace puma2 {

/**
 * @class   PumaException
 * @brief   Exception base class for any type of exception within the PUMA libs.
 * @author  Detlev Droege
 * @date    Februar 2007
 */

class PumaException : public std::exception {
  public:
    /**
     * severity of exceptions.
     */
    typedef enum {
      inaccuracy,
      faulty,
      ignorable,
      intolerable
    } exceptionSeverity;

  private:
    /**
     * message text.
     */
    std::string	message;
    exceptionSeverity severity;
    
  public:

    /**
     * Default constructor.
     */
    PumaException();

    /**
     * Default constructor.
     * @param severity   The severity code for this exception
     * @param msg        A description text for this exception
     */
    PumaException(exceptionSeverity severity, std::string &msg);

    /**
     * Destructor
     */
    virtual ~PumaException() throw ();

    /**
     * Return a information string about the exception
     */
    std::string description() const;
};

}
#endif /* PUMA_EXCEPTION_H */
