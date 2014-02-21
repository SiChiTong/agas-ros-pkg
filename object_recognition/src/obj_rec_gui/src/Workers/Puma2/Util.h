#ifndef Util_H
#define Util_H

#include <cmath>

static const double M_PI2 = 2 * M_PI;

/**
 * Computes the faculty of the given input number. The input number has to take
 * a value between 0 and 12. Otherwise, the application will quit.
 * 
 * \param n The number which faculty shall be computed.
 * \return The faculty of the input number.
 **/
inline int fac(int n)
{
  assert( n >= 0 );
  assert( n <= 12 );

  int static const fac[13] = {
    1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880, 3628800, 39916800,
    479001600};

  return fac[n];
}

/**
 * Converts float to integer, considering a correct roundung policy.
 *
 * \param f A float value.
 * \return A corresponding integer value.
 **/
inline int ftoi(float f)
{
  return static_cast<int>( f + ( (f > 0.0f) ? (0.5f): (-0.5f) ) );
}

/**
 * Returns the minimum of two given integer numbers.
 * 
 * \param a An arbitrary number.
 * \param b Another arbitrary number.
 * \return The minimum of the two input numbers.
 **/
inline int min(int a, int b)
{
  return (a<b)?a:b;
}

/**
 * Returns the maximum of two given integer numbers.
 * 
 * \param a An arbitrary number.
 * \param b Another arbitrary number.
 * \return The maximum of the two input numbers.
 **/
inline int max(int a, int b)
{
  return (a>b)?a:b;
}

#endif
