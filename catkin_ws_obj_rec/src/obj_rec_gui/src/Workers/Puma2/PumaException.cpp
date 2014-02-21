/**
 * @file    PumaException.cc
 * @brief   Contains PumaException.
 *
 * (c) 2007 AG Aktives Sehen <agas@uni-koblenz.de>, Universitaet Koblenz-Landau
 * $Id: $ 
 */

#include "PumaException.h"
#include <iostream>

using namespace std;
using namespace puma2;

PumaException::PumaException()
{
}

PumaException::PumaException(exceptionSeverity sev, std::string &msg)
{
  severity = sev;
  message = msg;
  cout << endl << this->description() << endl;
}

PumaException::~PumaException() throw () {};

std::string  PumaException::description() const
{ 
  std::string txt = "PUMA Exception: " + message; 
  return txt;  
}
