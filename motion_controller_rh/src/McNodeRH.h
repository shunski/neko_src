// McNodeRH.h
#ifndef MCNODERH_H
#define MCNODERH_H

#include <jcatty_simple_struct/McNode.h>

class McNodeRH : public McNode
{
    public:
        McNodeRH()
            McNode("rhLegCommand", "rhCurrentState", "rhProcessedInfo", "rhLocomotionAction")
        {}
};
#endif
