#include "rtwtypes.h"
#include "maximum_w0hsPrr7.h"
#include "mwmathutil.h"

real_T maximum_w0hsPrr7(const real_T x[4])
{
  real_T ex;
  real_T x_p;
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  if (!muDoubleScalarIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!muDoubleScalarIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 5; k++) {
      x_p = x[k - 1];
      if (ex < x_p) {
        ex = x_p;
      }
    }
  }

  return ex;
}
