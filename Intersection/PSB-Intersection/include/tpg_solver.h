#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <math.h>
#include <iostream>
#include <ilcplex/ilocplex.h>

#include "tgp_node.h"
#include "berstein.h"
#include "Common.h"

class TGP_Solver{
public:
    TGP_Solver();
    bool TGP2MILP(int n_points, double t_length);
private:

};