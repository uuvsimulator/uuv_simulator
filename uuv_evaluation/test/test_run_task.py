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
NAME = 'test_run_task'

import rospy
import rostest
import subprocess
from rospkg import RosPack
import unittest
import yaml
import time
import os
import shutil
import numpy as np
from parameter_opt import SimulationRunner

import roslib; roslib.load_manifest(PKG)

# PID controller parameters to test the simulation wrapper
PARAMS = dict(Kp=[11993.888, 11993.888, 11993.888, 19460.069, 19460.069, 19460.069],
              Kd=[9077.459, 9077.459, 9077.459, 18880.925, 18880.925, 18880.925],
              Ki=[321.417, 321.417, 321.417, 2096.951, 2096.951, 2096.951])

ROSPACK_INST = RosPack()
ROOT_PATH = os.path.join(ROSPACK_INST.get_path(PKG), 'test')
OCEAN_WORLD_TASK = os.path.join(ROOT_PATH, 'example_start_ocean_world.yaml')
RESULTS_DIR = os.path.join(ROOT_PATH, 'results')

class TestRunTask(unittest.TestCase):
    def tearDown(self):
        if os.path.isdir(RESULTS_DIR):
            shutil.rmtree(RESULTS_DIR)

    def test_create_task(self):        
        runner = SimulationRunner(PARAMS, OCEAN_WORLD_TASK, RESULTS_DIR, True)
        runner.run(PARAMS)

        self.assertIn('results', os.listdir(ROOT_PATH),'Root results directory was not generated')
        self.assertTrue(os.path.isdir(runner.current_sim_results_dir), 'Results directory does not exist')        
        self.assertIn('ros', os.listdir(runner.current_sim_results_dir),'ROS_HOME not correctly changed to the current directory')
        self.assertIn('recording.bag', os.listdir(runner.current_sim_results_dir),'recording.bag cannot be found')
        del runner

    def test_params(self):
        runner = SimulationRunner(PARAMS, OCEAN_WORLD_TASK, RESULTS_DIR, True)
        runner.run(PARAMS)
        self.assertIn('params_0.yml', os.listdir(runner.current_sim_results_dir),'Parameter file created correctly')

        with open(os.path.join(runner.current_sim_results_dir, 'params_0.yml'), 'r') as c_file:
            params = yaml.load(c_file)

        for tag in PARAMS:
            self.assertIn(tag, params, 'Tag <%s> does not exist in the stored parameter list' % tag)
            self.assertEqual(PARAMS[tag], params[tag], 'List of parameters for tag <%s> is different' % tag)
        del runner

    def test_task_process_timeout(self):
        runner = SimulationRunner(PARAMS, OCEAN_WORLD_TASK, RESULTS_DIR, True)
        success = runner.run(PARAMS, timeout=2)
        self.assertFalse(success, 'Simulation runner must return False when the timeout is triggered, flag=' + str(success))
        self.assertTrue(runner.process_timeout_triggered, 'Simulation runner did not set the flag process_timeout_triggered')
        del runner

    def test_batch_run_task(self):
        for _ in range(3):            
            runner = SimulationRunner(dict(), OCEAN_WORLD_TASK, RESULTS_DIR, True)
            success = runner.run(dict(), timeout=30)            
            self.assertTrue(success, 'Simulation returned with error, flag=%s, timeout=%s' % (str(success), str(runner.process_timeout_triggered)))
            del runner            

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestRunTask)