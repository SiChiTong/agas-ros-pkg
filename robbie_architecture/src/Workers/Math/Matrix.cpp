/*******************************************************************************
 *  Matrix.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $ 
 *******************************************************************************/

#include "Matrix.h"

#define THIS Matrix

THIS THIS::transpose() {
    Matrix newMatrix(m_Columns, m_Lines);
    for (unsigned i = 0; i < m_Lines*m_Columns; i++) {
        newMatrix.setValue(i/m_Columns, i%m_Lines, m_Values[i/m_Lines * i%m_Columns]);
    }
    return newMatrix;
}

#undef THIS
