/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "absl/memory/memory.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

ProbabilityGrid::ProbabilityGrid(const MapLimits& limits)
    : limits_(limits) {
      int size = limits_.cell_limits_.num_x_cells * limits_.cell_limits_.num_y_cells;
      lookupTable_ = new std::vector<uint8>[size]();

    }


// Returns the probability of the cell with 'cell_index'.
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return lookupTable_[cell_index];
}


}  // namespace mapping
}  // namespace cartographer
