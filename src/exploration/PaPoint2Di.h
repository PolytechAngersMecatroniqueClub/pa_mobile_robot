#ifndef PAPOINT2DI_H
#define PAPOINT2DI_H

#include <string>

class PaPoint2Di {

    private:
        int _x;
        int _y;
    
    public:
        PaPoint2Di(int x=0, int y=0);
        int getX() const { return _x; }
        int getY() const{ return _y; }
        void setX(int x){ _x = x; }
        void setY(int y){ _y = y; }

        bool operator==(const PaPoint2Di& other) const;
        bool operator!=(const PaPoint2Di& other) const;

        std::string to_string() const;


        static double dst(const PaPoint2Di& a, const PaPoint2Di& b);
};

#endif
