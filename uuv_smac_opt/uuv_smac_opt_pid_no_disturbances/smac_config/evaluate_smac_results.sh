#!/usr/bin/env bash
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Be aware that the input directory should be set as the directory where the SMAC results folders was created, being
# in this case ./ for a SMAC folder stored as ./smac_output_<timestamp>
rosrun uuv_evaluation evaluate_smac_best_results.py --input_dir ./ --output_dir ./best_results --simulate 1

DIRECTORY=./comparative_analysis
if [ ! -d "$DIRECTORY" ]; then
  mkdir -p ${DIRECTORY}
fi

# Set the folder with all partial results from SMAC as an input for the comparison
rosrun uuv_evaluation run_best_worst_comparison.py --input_dir ./best_results --input_dir_labels "SMAC PID" --output_dir ${DIRECTORY} --config_file analysis_configuration.yaml
