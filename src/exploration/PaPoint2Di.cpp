#include "PaPoint2Di.h"

#include <math.h>


double PaPoint2Di::dst(const PaPoint2Di& a, const PaPoint2Di& b){
    return sqrt(pow(a._x - b._x,2) + pow(a._y - b._y,2));
}

PaPoint2Di::PaPoint2Di(int x, int y){
    _x = x;
    _y = y;
}

bool PaPoint2Di::operator==(const PaPoint2Di& other) const {
  return (this->_x == other.getX()) && (this->_y == other.getY());
}

bool PaPoint2Di::operator!=(const PaPoint2Di& other) const {
  return ! (*this == other);
}


std::string PaPoint2Di::to_string() const{
    std::string output;
    output = "(" + std::to_string(_x) + ", " + std::to_string(_y) + ")";
    return output;
}
