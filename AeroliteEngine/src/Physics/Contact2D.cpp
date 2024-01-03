#include "Contact2D.h"
#include <iostream>

namespace Aerolite {

    void Aerolite::Contact2D::ResolvePenetration(void)
    {
        if(a->IsStatic() && b->IsStatic()) return;
        
        float da = depth / (a->invMass + b->invMass) * a->invMass;
        float db = depth / (a->invMass + b->invMass) * b->invMass;
        
        a->position -= (normal * da);
        b->position += (normal * db);
    }
}
