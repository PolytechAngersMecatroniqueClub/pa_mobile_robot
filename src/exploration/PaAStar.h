#ifndef PAASTAR_H
#define PAASTAR_H

#include "PaNode.h"
#include "PaPoint2Di.h"
#include "PaExplorationMap.h"

#include <vector>

class PaAStar {

    private:
        std::vector<PaNode> _nodes;
        std::vector <PaPoint2Di> _opened;
        std::vector <PaPoint2Di> _closed;
        const PaExplorationMap* _pmap;
        
        void _reset();

    
    public:
        PaAStar(const PaExplorationMap* pmap);
        int heuristic(const PaPoint2Di& position, const PaPoint2Di& target);
        void update_node(const PaPoint2Di& pos_parent, int dx, int dy, const PaPoint2Di& target);
        std::vector<PaPoint2Di> find_path(const PaPoint2Di& start, const PaPoint2Di& target);
};

#endif
