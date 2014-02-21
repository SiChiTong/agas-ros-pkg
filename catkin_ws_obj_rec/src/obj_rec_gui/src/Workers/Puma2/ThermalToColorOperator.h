/*******************************************************************************
 *  ThermalToColorOperator.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ThermalToColorOperator.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef ThermalToColorOperator_H
#define ThermalToColorOperator_H

#include "ImageToImageOperator.h"
#include "ColorImageUV8.h"
#include "../../Workers/Puma2/GrayLevelImage8.h"
#include "../../Workers/Puma2/ColorImageRGB8.h"

namespace puma2 {

/**
 * @class ThermalToColorOperator
 * @brief Converts a GrayLevelImage containing temperature information to RGB
 */
class ThermalToColorOperator :
	public ImageToImageOperator<GrayLevelImage8,ColorImageRGB8>
{
  public:

    /** @brief Default constructor/destructor */
    ThermalToColorOperator() {};
    virtual ~ThermalToColorOperator() {};

    /** @brief Create, apply, delete */
    ThermalToColorOperator(const GrayLevelImage8& constThermalImage, ColorImageRGB8& imageRGB, int thermalMin=20, int thermalMax=50);

    /// Do the real work of the operator
  	virtual void apply(const GrayLevelImage8& thermalImage, ColorImageRGB8& imageRGB);

  private:

    int m_ThermalMin, m_ThermalMax;

};

}

#endif
