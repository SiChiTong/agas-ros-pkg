#include "GrayLevelImage16.h"

using namespace puma2;

GrayLevelImage16::GrayLevelImage16(int x, int y) : SingleElementImage<unsigned short>(x,y) {
  setupImageBaseVariables();
};

GrayLevelImage16::GrayLevelImage16(int x, int y, GrayLevelImage16 * m, int xo, int yo) : SingleElementImage<unsigned short>(x,y,m,xo,yo) {
  setupImageBaseVariables();
};
