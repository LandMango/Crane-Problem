///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>
#include <math.h>

#include "cranes_types.hpp"

namespace cranes {

// Solve the crane unloading problem for the given grid, using an exhaustive
// optimization algorithm.
//
// This algorithm is expected to run in exponential time, so the grid's
// width+height must be small enough to fit in a 64-bit int; this is enforced
// with an assertion.
//
// The grid must be non-empty.
path crane_unloading_exhaustive(const grid& setting) {

  // grid must be non-empty.
  assert(setting.rows() > 0);
  assert(setting.columns() > 0);

  // Compute maximum path length, and check that it is legal.
  const size_t max_steps = setting.rows() + setting.columns() - 2;
  assert(max_steps < 64);

  // TODO: implement the exhaustive search algorithm, then delete this
  // comment.
  path best(setting);
  for (size_t steps = 0; steps <= max_steps; steps++) {
      for (uint64_t bits = 0; bits <= pow(2, steps) - 1; bits++) {
          path candidate(setting);
          bool valid = true;
          for (uint64_t k = 0; k <= steps - 1; k++) {
              step_direction step_direction;
              int bit = (bits >> k) & 1;
              if (bit == 1) {
                  step_direction = STEP_DIRECTION_EAST;
              }
              else {
                  step_direction = STEP_DIRECTION_SOUTH;
              }
              if (candidate.is_step_valid(step_direction)) {
                  candidate.add_step(step_direction);
              }
              else {
                  valid = false;
                  break;
              }
          }

          if (valid) {
              if (candidate.total_cranes() > best.total_cranes()) {
                  best = candidate;
              }
          }
      }
  }

  return best;
}

// Solve the crane unloading problem for the given grid, using a dynamic
// programming algorithm.
//
// The grid must be non-empty.
//path crane_unloading_dyn_prog(const grid& setting) {
path crane_unloading_dyn_prog(const grid& setting) 
{

    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);


    using cell_type = std::optional<path>;

    std::vector<std::vector<cell_type> > A(setting.rows(),
        std::vector<cell_type>(setting.columns()));

    A[0][0] = path(setting);
    assert(A[0][0].has_value());

    for (coordinate r = 0; r < setting.rows(); ++r) 
    {
        for (coordinate c = 0; c < setting.columns(); ++c) 
        {

            if (setting.get(r, c) == CELL_BUILDING) {
                A[r][c].reset();
                continue;
            }

            cell_type from_above = std::nullopt;
            cell_type from_left = std::nullopt;

            // TODO: implement the dynamic programming algorithm, then delete this
            // comment.

            if (r > 0 && A[r - 1][c].has_value()) 
            {
                from_above = A[r - 1][c];
                if (from_above->is_step_valid(STEP_DIRECTION_SOUTH)) 
                {
                    from_above->add_step(STEP_DIRECTION_SOUTH);
                }
            }

            if (c > 0 && A[r][c - 1].has_value()) 
            {
                from_left = A[r][c - 1];
                if (from_left->is_step_valid(STEP_DIRECTION_EAST)) 
                {
                    from_left->add_step(STEP_DIRECTION_EAST);
                }
            }
            if (from_above.has_value() && from_left.has_value()) {
                if (from_above->total_cranes() > from_left->total_cranes()) 
                {
                    A[r][c] = from_above;
                }
                else {
                    A[r][c] = from_left;
                }
            }
            else if (from_left.has_value()) 
            {
                A[r][c] = from_left;
            }
            else if (from_above.has_value()) 
            {
                A[r][c] = from_above;
            }
        }
    }
  
    cell_type* best = &(A[0][0]);
    assert(best->has_value());
    for (coordinate r = 0; r < setting.rows(); ++r) 
    {
        for (coordinate c = 0; c < setting.columns(); ++c) 
        {
            if (A[r][c].has_value() && A[r][c]->total_cranes() > (*best)->total_cranes()) 
            {
                best = &(A[r][c]);
            }
        }
    }
           assert(best->has_value());
            //  //   std::cout << "total cranes" << (**best).total_cranes() << std::endl;

            return **best;
        }

    
}
