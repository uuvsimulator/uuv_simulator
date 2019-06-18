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
from __future__ import print_function
import xmltodict
import rospy
import rospkg
import unittest
import os
import jinja2
import random
from collections import OrderedDict

PKG = 'uuv_assistants'
NAME = 'test_templates' 

ROSPKG = rospkg.RosPack()

import roslib
roslib.load_manifest(PKG)

ENV = jinja2.Environment(
    loader=jinja2.FileSystemLoader([
        os.path.join(ROSPKG.get_path('uuv_assistants'), 'templates', 'robot_description'),
        os.path.join(ROSPKG.get_path('uuv_assistants'), 'templates', 'robot_description', 'modules')]))

def cuboid_inertia(mass, x, y, z):
    return dict(
        mass=mass,
        ixx=1. / 12. * mass * (y**2 + z**2),
        iyy=1. / 12. * mass * (x**2 + z**2),
        izz=1. / 12. * mass * (y**2 + x**2),
        ixy=0,
        ixz=0,
        iyz=0)

def cylinder_inertia(mass, radius, length):
    return dict(
        mass=mass,
        ixx=1. / 12. * mass * (3 * radius**2 + length**2),
        iyy=1. / 12. * mass * (3 * radius**2 + length**2),
        izz=0.5 * mass * radius**2,
        ixy=0,
        ixz=0,
        iyz=0)

def ellipsoid_inertia(mass, a, b, c):
    return dict(
        mass=mass,
        ixx=1. / 5. * mass * (b**2 + c**2),
        iyy=1. / 5. * mass * (a**2 + c**2),
        izz=1. / 5. * mass * (a**2 + b**2),
        ixy=0,
        ixz=0,
        iyz=0)

def sphere_inertia(mass, radius):
    return dict(
        mass=mass,
        ixx=2. / 5. * mass * radius**2,
        iyy=2. / 5. * mass * radius**2,
        izz=2. / 5. * mass * radius**2,
        ixy=0,
        ixz=0,
        iyz=0)

class TestTemplates(unittest.TestCase):        
    def test_cuboid_inertial(self):
        header_str = """{% import 'inertias.urdf.jinja' as inertias %}\n"""
    
        for _ in range(10):
            mass = random.random()
            x = random.random()
            y = random.random()
            z = random.random()
            
            info_str = '{{ inertias.cuboid(' + \
                '{}, {}, {}, {}'.format(mass, x, y, z) + \
                    ')}}' 
            template_str = header_str + info_str     

            template = ENV.from_string(template_str)
            data = xmltodict.parse(template.render())
            inertia = cuboid_inertia(
                mass, x, y, z)

            self.assertIn('inertial', data)
            self.assertIn('mass', data['inertial'])
            
            for tag in inertia:
                if tag == 'mass':
                    self.assertAlmostEqual(
                        float(data['inertial']['mass']['@value']), 
                        inertia['mass'])
                else: 
                    self.assertAlmostEqual(
                        float(data['inertial']['inertia']['@' + tag]), 
                        inertia[tag])

    def test_ellipsoid_inertial(self):
        header_str = """{% import 'inertias.urdf.jinja' as inertias %}\n"""
            
        for _ in range(10):
            mass = random.random()
            a = random.random()
            b = random.random()
            c = random.random()
            
            info_str = '{{ inertias.ellipsoid(' + \
                '{}, {}, {}, {}'.format(mass, a, b, c) + \
                    ')}}' 
            template_str = header_str + info_str     

            template = ENV.from_string(template_str)
            data = xmltodict.parse(template.render())
            inertia = ellipsoid_inertia(
                mass, a, b, c)

            self.assertIn('inertial', data)
            self.assertIn('mass', data['inertial'])
            
            for tag in inertia:
                if tag == 'mass':
                    self.assertAlmostEqual(
                        float(data['inertial']['mass']['@value']), 
                        inertia['mass'])
                else: 
                    self.assertAlmostEqual(
                        float(data['inertial']['inertia']['@' + tag]), 
                        inertia[tag])

    def test_cylinder_inertial(self):
        header_str = """{% import 'inertias.urdf.jinja' as inertias %}\n"""
            
        for _ in range(10):
            mass = random.random()
            radius = random.random()
            length = random.random()
            
            info_str = '{{ inertias.cylinder(' + \
                '{}, {}, {}'.format(mass, radius, length) + \
                    ')}}' 
            template_str = header_str + info_str     

            template = ENV.from_string(template_str)
            data = xmltodict.parse(template.render())
            inertia = cylinder_inertia(
                mass, radius, length)

            self.assertIn('inertial', data)
            self.assertIn('mass', data['inertial'])
            
            for tag in inertia:
                if tag == 'mass':
                    self.assertAlmostEqual(
                        float(data['inertial']['mass']['@value']), 
                        inertia['mass'])
                else: 
                    self.assertAlmostEqual(
                        float(data['inertial']['inertia']['@' + tag]), 
                        inertia[tag])

    def test_sphere_inertial(self):
        header_str = """{% import 'inertias.urdf.jinja' as inertias %}\n"""
            
        for _ in range(10):
            mass = random.random()
            radius = random.random()
            
            info_str = '{{ inertias.sphere(' + \
                '{}, {}'.format(mass, radius) + \
                    ')}}' 
            template_str = header_str + info_str     

            template = ENV.from_string(template_str)
            data = xmltodict.parse(template.render())
            inertia = sphere_inertia(
                mass, radius)

            self.assertIn('inertial', data)
            self.assertIn('mass', data['inertial'])
            
            for tag in inertia:
                if tag == 'mass':
                    self.assertAlmostEqual(
                        float(data['inertial']['mass']['@value']), 
                        inertia['mass'])
                else: 
                    self.assertAlmostEqual(
                        float(data['inertial']['inertia']['@' + tag]), 
                        inertia[tag])

    def test_colors(self):
        fixed_colors = dict(
            blue=[0, 0, 1, 1],
            red=[1, 0, 0, 1],
            black=[0, 0, 0, 1],
            white=[1, 1, 1, 1],
            gray=[0.58, 0.58, 0.58, 1]
        )

        header_str = """{% import 'materials.urdf.jinja' as materials %}\n"""
            
        for tag in fixed_colors:
            info_str = '{{ materials.' + '{}'.format(tag) + ' }}'
            template_str = header_str + info_str
            template = ENV.from_string(template_str)
            data = xmltodict.parse(template.render())

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestTemplates)
