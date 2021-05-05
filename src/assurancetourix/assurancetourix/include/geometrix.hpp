#ifndef GEOMETRIX_HPP
#define GEOMETRIX_HPP

#include "assurancetourix.hpp"

class Assurancetourix;
class Geometrix {
public:
  Geometrix(Assurancetourix* node);
  ~Geometrix();
private:

  Assurancetourix* node;
};

#endif
