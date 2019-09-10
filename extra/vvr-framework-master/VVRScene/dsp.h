#ifndef DSP_H
#define DSP_H

#include "vvrscenedll.h"
#include <iostream>
#include <vector>

namespace vvr {
namespace dsp {

typedef std::vector<double> Signal;

Signal VVRScene_API smooth(const Signal &in, int window_size);

Signal VVRScene_API diff(const Signal &in, int stride = 1);

Signal VVRScene_API threshold(const Signal &in, double threshold);

Signal VVRScene_API consecutiveThreshold(const Signal &in, int max_cons_vals);

unsigned VVRScene_API detectZero(const Signal &samples, unsigned offset, double tolerance = 0, bool reverse_dir = false);

unsigned VVRScene_API detectNonZero(const Signal &signal, unsigned offset, double tolerance = 0, bool reverse_dir = false);

double VVRScene_API interpLinear(double x0, double x1, double y0, double y1, double x);

double VVRScene_API interpSmooth_0_1(int i, int imax);

}
}

#endif
