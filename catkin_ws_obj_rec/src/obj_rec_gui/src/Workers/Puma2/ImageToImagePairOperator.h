/*******************************************************************************
 *  ImageToImagePairOperator.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ImageToImagePairOperator.h 24370 2008-04-16 12:36:38Z pietsch $
 *******************************************************************************/

#ifndef ImageToImagePairOperator_H
#define ImageToImagePairOperator_H

#include <iostream>
#include "ImageOperator.h"

namespace puma2
{

  /**
   * @class ImageOperator
   * @brief Image operator for any triple of image classes (in, (out1,out2))
   * @author David Gossow (RX)
   */

  template <class InputClass, class OutputClass1, class OutputClass2 > class ImageToImagePairOperator
        : public ImageOperator
  {
    public:

      /**
       * Default constructor.
       */
      ImageToImagePairOperator();

      /**
       * Destructor
       */
      virtual ~ImageToImagePairOperator();

      /// interface that will call apply -- can be overwritten in derived class
      virtual void operator() ( const InputClass & inputImage, OutputClass1 & outputImage1, OutputClass2 & outputImage2 );

    protected:
      /// must be defined in derived class
      virtual void apply ( const InputClass & inputImage, OutputClass1 & outputImage1, OutputClass2 & outputImage2 ) = 0;

      /**
      * test and adjust size of output image
      *
      * Default: sizes must be equal for input and output
      *
      * if output image has zero size, it will be allocated to have
      * the same size as the input image.
      *
      * other behaviour can be defined for derived operators which only
      * have to redefine this method
      *
      * The method will throw an exception if input and output image are the
      * same. This can be overwritten in a derived class,
      * if inplace operation is possible.
      */
      virtual void checkArgument ( const InputClass & inputImage, OutputClass1 & outputImage1, OutputClass2 & outputImage2 );

    public:

      static  void checkImageArgument ( const InputClass & inputImage, OutputClass1 & outputImage1, OutputClass2 & outputImage2 );

  };


  template <class InputClass, class OutputClass1, class OutputClass2>
  ImageToImagePairOperator<InputClass, OutputClass1, OutputClass2>::ImageToImagePairOperator()
  {
  }

  template <class InputClass, class OutputClass1, class OutputClass2>
  ImageToImagePairOperator<InputClass, OutputClass1, OutputClass2>::~ImageToImagePairOperator()
  {
  };

  template <class InputClass, class OutputClass1, class OutputClass2>
  void ImageToImagePairOperator<InputClass, OutputClass1, OutputClass2>::operator()
  ( const InputClass & inputImage, OutputClass1 & outputImage1, OutputClass2 & outputImage2 )
  {
    checkArgument ( inputImage, outputImage1, outputImage2 );   // virtual - can be overwritten
    apply ( inputImage, outputImage1, outputImage2 );    // virtual - must be overwritten
  }


  template <class InputClass, class OutputClass1, class OutputClass2>
  void ImageToImagePairOperator<InputClass, OutputClass1, OutputClass2>::checkArgument
  ( const InputClass & inputImage, OutputClass1 & outputImage1, OutputClass2 & outputImage2 )
  {
    checkImageArgument ( inputImage, outputImage1, outputImage2 );
  }

  template <class InputClass, class OutputClass1, class OutputClass2>
  void ImageToImagePairOperator<InputClass, OutputClass1, OutputClass2>::checkImageArgument
  ( const InputClass & inputImage, OutputClass1 & outputImage1, OutputClass2 & outputImage2 )
  {
    if ( outputImage1.getWidth() != 0 )
    {
      if ( ( outputImage1.getWidth() != outputImage2.getWidth() )  || ( outputImage1.getHeight() != outputImage2.getHeight() ) ||
           ( outputImage1.getWidth() != inputImage.getWidth() )  || ( outputImage1.getHeight() != inputImage.getHeight() ) )
      {
        throw "Image size missmatch";
      }
    }
    else
    {
      if ( ( reinterpret_cast<const void*> ( &inputImage ) == reinterpret_cast<const void*> ( &outputImage1 ) ) ||
           ( reinterpret_cast<const void*> ( &inputImage ) == reinterpret_cast<const void*> ( &outputImage2 ) ) )
      {
        throw "Inplace operation of image operator not possible";
      }
      outputImage1.resize ( inputImage.getWidth(), inputImage.getHeight() );
      outputImage2.resize ( inputImage.getWidth(), inputImage.getHeight() );
    }
  }

} // Namespace

#endif
