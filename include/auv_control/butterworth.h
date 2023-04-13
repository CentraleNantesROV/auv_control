#ifndef AUV_CONTROL_BUTTERWORTH_H
#define AUV_CONTROL_BUTTERWORTH_H

#include <array>
#include <vector>
#include <math.h>

// this class implements a 2nd-order Butterworth low-pass filter

class Butterworth
{
public:
  Butterworth() {}

  Butterworth(double frequency, double dt)
  {
    init(frequency, dt);
  }

  void init(double frequency, double dt)
  {
    const double ita =1.0/ tan(M_PI*frequency*dt);
    const double q=sqrt(2.0);

    b[0] = 1.0 / (1.0 + q*ita + ita*ita);
    b[1]= 2*b[0];
    b[2]= b[0];
    a = {2.0 * (ita*ita - 1.0) * b[0], -(1.0 - q*ita + ita*ita) * b[0]};
  }

  double filter(double v)
  {
    // swap inputs
    x[2] = x[1];
    x[1] = x[0];
    x[0] = v;

    // compute output
    v = b[2]*x[2] +
        b[1]*x[1] + a[1]*y[1] +
        b[0]*x[0] + a[0]*y[0];

    // swap output
    y[1] = y[0];
    y[0] = v;

    return v;
  }

protected:
  std::array<double,3> a, b, x;
  std::array<double,2> y;
};


// same but for nD-variables (std::vectors, Eigen lib vectors...)

class Butterworth_nD
{
public:
  // n identical filters
  Butterworth_nD() {}
  Butterworth_nD(uint size, double frequency, double dt)
  {
    init(size, frequency, dt);
  }
  inline void init(uint size, double frequency, double dt)
  {
    filters = std::vector<Butterworth>(size, Butterworth(frequency, dt));
  }

  // deduce n from given frequencies
  Butterworth_nD(std::vector<double> frequencies, double dt)
  {
    filters.clear();
    for(auto f: frequencies)
      filters.push_back(Butterworth(f, dt));
  }

  template <class T>
  void filter(T &var)
  {
    for(unsigned int i=0;i<var.size();++i)
      var[i] = filters[i].filter(var[i]);
  }

protected:
  std::vector<Butterworth> filters;
};

#endif // BUTTERWORTH_H
