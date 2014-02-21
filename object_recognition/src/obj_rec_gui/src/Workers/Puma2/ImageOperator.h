#ifndef ImageOperator_H
#define ImageOperator_H

#include "ImageSuperClasses.h"

namespace puma2 {

/**
 * @class ImageOperator
 * @brief Image operator for any image operator classes
 */

class ImageOperator 
{
  public:

    /**
     * Default constructor.
     */
    ImageOperator();

    /**
     * Destructor
     */
    virtual ~ImageOperator();
};

}

#endif
