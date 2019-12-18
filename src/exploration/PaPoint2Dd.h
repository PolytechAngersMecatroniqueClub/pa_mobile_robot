#ifndef PAPOINT2DI_H
#define PAPOINT2DI_H

#include <string>

class PaPoint2Dd {

    private:
        double _x;
        double _y;
    
    public:
        PaPoint2Dd(int x=0, int y=0);
        double getX() const { return _x; }
        double getY() const{ return _y; }
        void setX(int x){ _x = x; }
        void setY(int y){ _y = y; }

        bool operator==(const PaPoint2Dd& other) const;
        bool operator!=(const PaPoint2Dd& other) const;

        std::string to_string() const;


        static double dst(const PaPoint2Dd& a, const PaPoint2Dd& b);
};

#endif
