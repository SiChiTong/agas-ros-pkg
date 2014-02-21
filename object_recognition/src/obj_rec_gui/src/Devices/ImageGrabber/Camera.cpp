/*******************************************************************************
*  Camera.cpp
*
*  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
*           Universitaet Koblenz-Landau
*
*  Information on Code Review state:
*  §Author:DG DevelTest: Reviewer: Review: Date: State: §
*
*  Additional information:
*  $Id: Camera.cpp 44313 2011-04-06 22:46:28Z agas $
******************************************************************************/

#include <stdlib.h>
#include "Camera.h"

#include "../../Workers/String/String.h"


#include <sstream>
#include <assert.h>
#include <math.h>


#define THIS Camera

using namespace std;

THIS::THIS( std::string deviceDescription, int formatId, int subFormatId, float horizontalViewAngle, std::string customProperties, bool rotateImage )
{
    m_DeviceDescription=deviceDescription;
    m_FormatId=formatId;
    m_SubFormatId=subFormatId;
    m_HorizontalViewAngle = horizontalViewAngle;
		m_RotateImage = rotateImage;

    std::vector<std::string> pairs=String::explode( customProperties, ";" );
    for ( unsigned i=0; i<pairs.size(); i++ )
    {
        vector<string> values=String::explode( pairs[i], "=" );
        assert( values.size() == 2 );
        string leftVal=values[0];
        double rightVal=atof( values[1].c_str() );
        m_CustomProperties[ leftVal ] = rightVal;
    }
}

float THIS::getHorizontalViewAngle(float diagonalViewAngle)
{
    float x = 8 / 5 * tan(diagonalViewAngle / 2.0);
    float hViewAngle = 2.0 * atan( x / 2.0 );
    return hViewAngle;
}

#undef THIS
