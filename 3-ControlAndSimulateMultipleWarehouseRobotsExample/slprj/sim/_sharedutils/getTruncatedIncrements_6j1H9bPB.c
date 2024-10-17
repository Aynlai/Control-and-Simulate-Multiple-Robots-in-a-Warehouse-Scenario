#include "rtwtypes.h"
#include "getTruncatedIncrements_6j1H9bPB.h"
#include "mwmathutil.h"
#include <math.h>
#include "rt_nonfinite.h"

void getTruncatedIncrements_6j1H9bPB(real_T x0, real_T x1, real_T b_y0, real_T
  b_y1, boolean_T *endClipped, real_T *n, real_T *xInc, real_T *yInc, real_T
  *dtx, real_T *dty, real_T *txNext, real_T *tyNext)
{
  real_T absx;
  real_T b_absx;
  real_T b_dtx_tmp;
  real_T b_dty_tmp;
  real_T b_dx;
  real_T dna;
  real_T dx;
  real_T nx;
  real_T x1Floor;
  int32_T b_exponent;
  int32_T b_xIncSign;
  int32_T exponent;
  int32_T xIncSign;
  boolean_T tmp;
  boolean_T xClipped;
  boolean_T yClipped;
  nx = 0.0;
  dx = x1 - x0;
  xIncSign = -1;
  dna = muDoubleScalarFloor(x0);
  x1Floor = muDoubleScalarFloor(x1);
  if (muDoubleScalarRound(x0) - dna > 0.0) {
    xIncSign = 1;
  }

  b_dtx_tmp = muDoubleScalarAbs(dx);
  *dtx = 1.0 / b_dtx_tmp;
  absx = muDoubleScalarAbs(x1);
  if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
    absx = (rtNaN);
  } else if (absx < 4.4501477170144028E-308) {
    absx = 4.94065645841247E-324;
  } else {
    frexp(absx, &exponent);
    absx = ldexp(1.0, exponent - 53);
  }

  if (b_dtx_tmp <= 2.0 * absx) {
    *xInc = xIncSign;
    *txNext = *dtx;
  } else if (dx > 0.0) {
    *xInc = 1.0;
    *txNext = ((dna + 1.0) - x0) * *dtx;
    nx = x1Floor - dna;
  } else {
    *xInc = -1.0;
    *txNext = (x0 - dna) * *dtx;
    nx = dna - x1Floor;
  }

  xIncSign = (int32_T)muDoubleScalarMin(muDoubleScalarMax(0.0, dna), 60.0);
  exponent = (int32_T)muDoubleScalarMin(muDoubleScalarMax(0.0, x1Floor), 60.0);
  dx = 0.0;
  b_dx = b_y1 - b_y0;
  b_xIncSign = -1;
  b_dtx_tmp = muDoubleScalarFloor(b_y0);
  absx = muDoubleScalarFloor(b_y1);
  if (muDoubleScalarRound(b_y0) - b_dtx_tmp > 0.0) {
    b_xIncSign = 1;
  }

  b_dty_tmp = muDoubleScalarAbs(b_dx);
  *dty = 1.0 / b_dty_tmp;
  b_absx = muDoubleScalarAbs(b_y1);
  if (muDoubleScalarIsInf(b_absx) || muDoubleScalarIsNaN(b_absx)) {
    b_absx = (rtNaN);
  } else if (b_absx < 4.4501477170144028E-308) {
    b_absx = 4.94065645841247E-324;
  } else {
    frexp(b_absx, &b_exponent);
    b_absx = ldexp(1.0, b_exponent - 53);
  }

  if (b_dty_tmp <= 2.0 * b_absx) {
    *yInc = b_xIncSign;
    *tyNext = *dty;
  } else if (b_dx > 0.0) {
    *yInc = 1.0;
    *tyNext = ((b_dtx_tmp + 1.0) - b_y0) * *dty;
    dx = absx - b_dtx_tmp;
  } else {
    *yInc = -1.0;
    *tyNext = (b_y0 - b_dtx_tmp) * *dty;
    dx = b_dtx_tmp - absx;
  }

  b_exponent = (int32_T)muDoubleScalarMin(muDoubleScalarMax(0.0, b_dtx_tmp),
    60.0);
  b_xIncSign = (int32_T)muDoubleScalarMin(muDoubleScalarMax(0.0, absx), 60.0);
  xClipped = ((dna != xIncSign) || (x1Floor != exponent));
  yClipped = ((b_dtx_tmp != b_exponent) || (absx != b_xIncSign));
  *endClipped = ((x1Floor != exponent) || (absx != b_xIncSign));
  tmp = !yClipped;
  if (xClipped || (!tmp)) {
    if (xClipped && yClipped) {
      dna = muDoubleScalarAbs(dna - (real_T)xIncSign) + muDoubleScalarAbs
        (x1Floor - (real_T)exponent);
      x1Floor = dna / nx;
      b_dtx_tmp = muDoubleScalarAbs(b_dtx_tmp - (real_T)b_exponent) +
        muDoubleScalarAbs(absx - (real_T)b_xIncSign);
      absx = b_dtx_tmp / dx;
      if (muDoubleScalarAbs(x1Floor) >= muDoubleScalarAbs(absx)) {
        dx = muDoubleScalarCeil((1.0 - x1Floor) * dx);
        nx -= dna;
      } else {
        nx = muDoubleScalarCeil((1.0 - absx) * nx);
        dx -= b_dtx_tmp;
      }
    } else if (xClipped && tmp) {
      dna = muDoubleScalarAbs(dna - (real_T)xIncSign) + muDoubleScalarAbs
        (x1Floor - (real_T)exponent);
      dx = muDoubleScalarCeil((1.0 - dna / nx) * dx);
      nx -= dna;
    } else {
      dna = muDoubleScalarAbs(b_dtx_tmp - (real_T)b_exponent) +
        muDoubleScalarAbs(absx - (real_T)b_xIncSign);
      nx = muDoubleScalarCeil((1.0 - dna / dx) * nx);
      dx -= dna;
    }
  }

  *n = nx + dx;
}
