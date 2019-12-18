#include "PaPoint2Dd.h"

#include <math.h>


double PaPoint2Dd::dst(const PaPoint2Dd& a, const PaPoint2Dd& b){
    return sqrt(pow(a._x - b._x,2) + pow(a._y - b._y,2));
}

PaPoint2Dd::PaPoint2Dd(int x, int y){
    _x = x;
    _y = y;
}

bool PaPoint2Dd::operator==(const PaPoint2Dd& other) const {
  return (this->_x == other.getX()) && (this->_y == other.getY());
}

bool PaPoint2Dd::operator!=(const PaPoint2Dd& other) const {
  return ! (*this == other);
}


std::string PaPoint2Dd::to_string() const{
    std::string output;
    output = "(" + std::to_string(_x) + ", " + std::to_string(_y) + ")";
    return output;
}
