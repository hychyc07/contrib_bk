// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _COMBINATORICS__H_
#define _COMBINATORICS__H_

#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

int permutations(int n, int r);
int factorial(int n);
std::string i2s(int& number);
int s2i(std::string s);
std::string getPermutationMask(int n, int p);

template <typename BiIterator>
bool next_combination(BiIterator n_begin, BiIterator n_end, BiIterator r_begin, BiIterator r_end);

#endif
