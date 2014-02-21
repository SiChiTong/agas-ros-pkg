/*******************************************************************************
 *  ImageSources.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $ 
 *******************************************************************************/

#include "ImageSources.h"

#define THIS ImageSources

std::map<THIS::SourceId,std::string> THIS::sourceDesc=std::map<THIS::SourceId,std::string>();
Mutex THIS::mutex=Mutex();

std::map< THIS::SourceId, std::string > THIS::getSourceDesc()
{
  initSourceDesc();
  mutex.lock();
  std::map<SourceId,std::string> descCopy=sourceDesc;
  mutex.unlock();
  return descCopy;
}

std::string THIS::getSourceDesc( THIS::SourceId id )
{
  initSourceDesc();
  mutex.lock();
  std::string desc;
  std::map< SourceId, std::string >::iterator descItem=sourceDesc.find(id);
  if (descItem==sourceDesc.end()) { desc="Unknown image source"; }
  else { desc=descItem->second; }
  mutex.unlock();
  return desc;
};

void THIS::initSourceDesc() {
  mutex.lock();
  if (sourceDesc.empty()) {

    fillSourceDesc();

    std::ostringstream stream;
    stream << "Registered image sources:" << std::endl;
    stream.setf ( std::ios::left, std::ios::adjustfield );
    stream.width(7);
    stream << "Id" << "Description" << std::endl;
    std::map< ImageSources::SourceId, std::string >::iterator it;
    for ( it=sourceDesc.begin(); it!=sourceDesc.end(); it++ )
    {
      stream.width(7);
      stream << it->first << it->second << std::endl;
    }
    // TRACE_SYSTEMINFO(stream.str()); // TODO use ros
  }
  mutex.unlock();
}

#undef THIS
