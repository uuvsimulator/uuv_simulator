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

PKG = 'uuv_evaluation'
NAME = 'test_evaluate_trajectory'

import rospy
import rostest
from rospkg import RosPack
import unittest
import yaml
import os
import shutil
from parameter_opt import SimulationRunner
from bag_evaluation import Evaluation

import roslib; roslib.load_manifest(PKG)

# PID controller parameters to test the simulation wrapper
PARAMS = dict(Kp=[11993.888, 11993.888, 11993.888, 19460.069, 19460.069, 19460.069],
              Kd=[9077.459, 9077.459, 9077.459, 18880.925, 18880.925, 18880.925],
              Ki=[321.417, 321.417, 321.417, 2096.951, 2096.951, 2096.951])

ROSPACK_INST = RosPack()
ROOT_PATH = os.path.join(ROSPACK_INST.get_path(PKG), 'test')
TASK = os.path.join(ROOT_PATH, 'example_task.yaml')
RESULTS_DIR = os.path.join(ROOT_PATH, 'results')
ROSBAG = os.path.join(ROOT_PATH, 'recording.bag')

class TestEvaluateTrajectory(unittest.TestCase):
    def setUp(self):
        if not os.path.isdir(RESULTS_DIR):
            os.makedirs(RESULTS_DIR)
            
    def tearDown(self):
        if os.path.isdir(RESULTS_DIR):
            shutil.rmtree(RESULTS_DIR)
    
    def test_generate_kpis(self):        
        self.assertIn('recording.bag', os.listdir(ROOT_PATH),'recording.bag cannot be found')

        sim_eval = Evaluation(ROSBAG, RESULTS_DIR)        
        sim_eval.compute_kpis()

        self.assertTrue(type(sim_eval.get_kpis()) == dict, 'KPIs structure is not a dictionary')

    def test_store_kpis(self):
        self.assertIn('recording.bag', os.listdir(ROOT_PATH),'recording.bag cannot be found')

        sim_eval = Evaluation(ROSBAG, RESULTS_DIR)        
        sim_eval.compute_kpis()
        sim_eval.save_kpis()

        self.assertIn('computed_kpis.yaml', os.listdir(RESULTS_DIR), 'KPIs were not stored in file computed_kpis.yaml')
        self.assertIn('kpi_labels.yaml', os.listdir(RESULTS_DIR), 'KPIs labels were not stored in file kpis_labels.yaml')

    def test_store_images(self):
        self.assertIn('recording.bag', os.listdir(ROOT_PATH),'recording.bag cannot be found')

        sim_eval = Evaluation(ROSBAG, RESULTS_DIR)  
        sim_eval.compute_kpis()
        sim_eval.save_evaluation()

        pdf_files = list()
        for f in os.listdir(RESULTS_DIR):
            if '.pdf' in f:
                pdf_files.append(f)

        self.assertGreater(len(pdf_files), 0, 'PDF files were not generated')

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestEvaluateTrajectory)