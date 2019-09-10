#include "dsp.h"
#include <algorithm>
#include <cmath>

using namespace std;

vvr::dsp::Signal vvr::dsp::smooth(const Signal &in, int window_size)
{
    Signal out(in.size());

    if (window_size % 2 == 0) window_size++; // we need odd size

    const int halfwin = window_size >> 1;

    long double sum;

    for (int i = halfwin; i < in.size() - halfwin; i++)
    {
        sum=0;

        for (int j = i - halfwin; j <= i + halfwin; j++)
            sum += in[j];

        out[i] = sum / window_size;
    }

    return out;
}

vvr::dsp::Signal vvr::dsp::diff(const Signal &in, int stride)
{
    Signal out(in.size());

    if (stride == 0) stride = 1; // Stride cannot be zero

    for (int i=0; i<stride; i++) {
        out[i] = ::fabs(in[i] - in[0]) / (i+1);
    }

    for (int i=stride; i<in.size(); i++) {
        out[i] = ::fabs(in[i] - in[i-stride]);
    }

    if (stride > 1) {
        for (int i = stride; i < in.size(); i++) {
            out[i] /= stride;
        }
    }

    return out;
}

vvr::dsp::Signal vvr::dsp::threshold(const Signal &in, double threshold)
{
    Signal out(in);

    for (int i=0; i<in.size(); i++)
    {
        if (in[i] <= threshold)
            out[i] = 0;
        else
            out[i] = 1;
    }

    return out;
}

vvr::dsp::Signal vvr::dsp::consecutiveThreshold(const Signal &in, int max_cons_vals)
{
    Signal out(in);

    int zero_counter = 0;

    for (int i=0; i<in.size(); i++)
    {
        if (!in[i]) zero_counter++;
        else zero_counter = 0;
        if (zero_counter < max_cons_vals)
            out[i] = 1;
        else 
            out[i] = 0;
    }

    return out;
}

double vvr::dsp::interpSmooth_0_1 (int i, int imax)
{
    return sin(1.57079632679489661923 * i / imax);
}

double vvr::dsp::interpLinear (double x0, double x1, double y0, double y1, double x)
{
    const double dx = x1-x0;
    const double dy = y1-y0;
    const double a = dy/dx;
    return y0 + a * (x-x0);
}

unsigned vvr::dsp::detectNonZero(const Signal &signal, unsigned offset, double tolerance, bool reverse_dir)
{
    if (!reverse_dir)
    {
        for (unsigned i=offset; i<signal.size(); i++)
        {
            if (::fabs(signal[i]) > tolerance)
                return i;
        }
    }
    else
    {
        for (int i=offset; i>=0; i--)
        {
            if (::fabs(signal[i]) > tolerance)
                return i;
        }
    }

    return offset;
}

unsigned vvr::dsp::detectZero(const Signal &signal, unsigned offset, double tolerance, bool reverse_dir)
{
    if (!reverse_dir)
    {
        for (unsigned i=offset; i<signal.size(); i++)
        {
            if (::fabs(signal[i]) <= tolerance)
                return i;
        }
    }
    else
    {
        for (int i=offset; i>=0; i--)
        {
            if (::fabs(signal[i]) <= tolerance)
                return i;
        }
    }

    return offset;
}
