#!/usr/bin/env python
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

from __future__ import division
import argparse
import numpy as np
import os
import yaml
import sys
from bag_evaluation import Evaluation

import roslib
import rospy
roslib.load_manifest('uuv_evaluation')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze bag file")
    parser.add_argument("bagfile", type=str)
    parser.add_argument("output_dir", type=str)

    args = parser.parse_args(rospy.myargv()[1:])

    sim_eval = Evaluation(args.bagfile, args.output_dir)

    sim_eval.compute_kpis()
    sim_eval.save_evaluation()
