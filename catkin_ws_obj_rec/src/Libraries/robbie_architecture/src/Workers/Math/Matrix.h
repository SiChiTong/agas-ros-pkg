/*******************************************************************************
 *  Matrix.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef MATRIX_H
#define MATRIX_H

#include "mat2.h"
#include "mat3.h"

/**
 * @class  Matrix
 * @brief  This class describes a Matrix
 * @author Michael Dahl
 */
class Matrix
{
  public:

    /** @brief The constructor */
    Matrix(unsigned lines, unsigned columns){
        m_Lines = lines;
        m_Columns = columns;
        m_Values = new double[lines*columns];
        for (unsigned i = 0; i< lines*columns; i++) {
            m_Values[i] = 0;
        }
    }

    Matrix(const CMat2 matrix2) {
        m_Lines = 2;
        m_Columns = 2;
        m_Values = new double[4];
        for(unsigned i = 0; i < m_Lines * m_Columns; i++) {
            m_Values[i] = matrix2.valueAt(i);
        }
    }

    Matrix(const CMat3 matrix3) {
        m_Lines = 3;
        m_Columns = 3;
        m_Values = new double[9];
        for(unsigned i = 0; i < m_Lines * m_Columns; i++) {
            m_Values[i] = matrix3.valueAt(i);
        }
    }

    /** @brief The destructor */
    ~Matrix() {
        delete[] m_Values;
    }

    inline double getValue(int line, int column) const{
        return m_Values[line*m_Columns  + column];
    }

    inline void setValue(int line, int column, double value) {
        m_Values[line*m_Columns + column] = value;
    }

    Matrix transpose();

    Matrix operator *(double scalar){
        Matrix returnMatrix(m_Lines, m_Columns);
        for (unsigned i = 0; i < m_Lines * m_Columns; i++) {
            returnMatrix.setValue(i/m_Lines, i%m_Lines, this->getValue(i/m_Lines, i%m_Lines) * scalar);
        }
        return returnMatrix;
    }

    Matrix operator /(double scalar){
        Matrix returnMatrix(m_Lines, m_Columns);
        for (unsigned i = 0; i < m_Lines * m_Columns; i++) {
            returnMatrix.setValue(i/m_Lines, i%m_Lines, this->getValue(i/m_Lines, i%m_Lines) / scalar);
        }
        return returnMatrix;
    }

    Matrix operator *(const Matrix rhs){
        Matrix returnMatrix(m_Lines, m_Columns);
        for (unsigned i = 0; i < m_Lines * m_Columns; i++) {
            returnMatrix.setValue(i/m_Lines, i%m_Lines, this->getValue(i/m_Lines, i%m_Lines) * rhs.getValue(i%m_Lines, i/m_Lines));
        }
        return returnMatrix;
    }

    Matrix operator +(const Matrix rhs){
        Matrix returnMatrix(m_Lines, m_Columns);
        for (unsigned i = 0; i < m_Lines * m_Columns; i++) {
            returnMatrix.setValue(i/m_Lines, i%m_Lines, this->getValue(i/m_Lines, i%m_Lines) + rhs.getValue(i/m_Lines, i%m_Lines));
        }
        return returnMatrix;
    }

    Matrix operator -(const Matrix rhs){
        Matrix returnMatrix(m_Lines, m_Columns);
        for (unsigned i = 0; i < m_Lines * m_Columns; i++) {
            returnMatrix.setValue(i/m_Lines, i%m_Lines, this->getValue(i/m_Lines, i%m_Lines) - rhs.getValue(i/m_Lines, i%m_Lines));
        }
        return returnMatrix;
    }

  private:

  unsigned int m_Lines;
  unsigned int m_Columns;
  double* m_Values;
};

#endif
