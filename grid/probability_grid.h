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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"

namespace cartographer {
namespace mapping {

// Represents a 2D grid of probabilities.
class ProbabilityGrid : public Grid2D {
 public:
  explicit ProbabilityGrid(const MapLimits& limits);

  ~ProbabilityGrid(){
    if(lookupTable_ != NULL)
      delete lookupTable_;
  }


  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i& cell_index) const;


 private:
  MapLimits limits_;
  std::vector<uint8>* lookupTable_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
