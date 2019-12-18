#ifndef NODE_H
#define NODE_H

#include "PaPoint2Di.h"

class PaNode {

    private:
        bool _walkable;
        PaPoint2Di _position;
        PaPoint2Di _parent_position;
        double _s_cost;
        double _h_cost;
        double _f_cost;
        bool _opened;
        bool _closed;
    
    public:
        PaNode(bool walkable = false, const PaPoint2Di& position = PaPoint2Di(), double s_cost = 0, double h_cost = 0, const PaPoint2Di& parent = PaPoint2Di());

        bool operator<(const PaNode& other) const;

        void update_costs(double s_cost, double h_cost, const PaPoint2Di& parent_position);

        void setPosition(const PaPoint2Di& position){ _position = position; }
        void setPosition(int x, int y){ _position = PaPoint2Di(x, y); }
        const PaPoint2Di& getPosition() const { return _position; }


        void setParentPosition(const PaPoint2Di& parent_position){ _parent_position = parent_position; }
        const PaPoint2Di& getParentPosition() const { return _parent_position; }

        void setWalkable(bool walkable){ _walkable = walkable; }
        bool isWalkable() const { return _walkable; }

        void setScost(double s_cost){ _s_cost = s_cost; }
        double getScost() const{ return _s_cost; }

        void setHcost(double h_cost){ _h_cost = h_cost; }
        void setFcost(double f_cost){ _f_cost = f_cost; }

        void setClosed(bool closed) { _closed = closed; }
        bool isClosed() const { return _closed; }

        void setOpened(bool opened) { _opened = opened; }
        bool isOpened() const { return _opened; }
        
};

#endif
