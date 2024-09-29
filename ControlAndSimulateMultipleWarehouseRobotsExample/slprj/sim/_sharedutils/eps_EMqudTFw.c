#include "rtwtypes.h"
#include "eps_EMqudTFw.h"
#include "mwmathutil.h"
#include <math.h>
#include "rt_nonfinite.h"

real_T eps_EMqudTFw(real_T x)
{
  real_T absx;
  real_T r;
  int32_T exponent;
  absx = muDoubleScalarAbs(x);
  if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
    r = (rtNaN);
  } else if (absx < 4.4501477170144028E-308) {
    r = 4.94065645841247E-324;
  } else {
    frexp(absx, &exponent);
    r = ldexp(1.0, exponent - 53);
  }

  return r;
}
