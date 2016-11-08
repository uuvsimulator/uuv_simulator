# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from urdf_parser_py.xml_reflection.basics import *
import urdf_parser_py.xml_reflection as xmlr

# What is the scope of plugins? Model, World, Sensor?

xmlr.start_namespace('sdf')

class Pose(xmlr.Object):
	def __init__(self, vec=None, extra=None):
		self.xyz = None
		self.rpy = None
		if vec is not None:
			assert isinstance(vec, list)
			count = len(vec)
			if len == 3:
				xyz = vec
			else:
				self.from_vec(vec)
		elif extra is not None:
			assert xyz is None, "Cannot specify 6-length vector and 3-length vector"
			assert len(extra) == 3, "Invalid length"
			self.rpy = extra

	def from_vec(self, vec):
		assert len(vec) == 6, "Invalid length"
		self.xyz = vec[:3]
		self.rpy = vec[3:6]

	def as_vec(self):
		xyz = self.xyz if self.xyz else [0, 0, 0]
		rpy = self.rpy if self.rpy else [0, 0, 0]
		return xyz + rpy

	def read_xml(self, node):
		# Better way to do this? Define type?
		vec = get_type('vector6').read_xml(node)
		self.load_vec(vec)

	def write_xml(self, node):
		vec = self.as_vec()
		get_type('vector6').write_xml(node, vec)

	def check_valid(self):
		assert self.xyz is not None or self.rpy is not None

name_attribute = xmlr.Attribute('name', str)
pose_element = xmlr.Element('pose', Pose, False)

class Entity(xmlr.Object):
	def __init__(self, name = None, pose = None):
		self.name = name
		self.pose = pose

xmlr.reflect(Entity, params = [
	name_attribute,
	pose_element
	])


class Inertia(xmlr.Object):
	KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

	def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
		self.ixx = ixx
		self.ixy = ixy
		self.ixz = ixz
		self.iyy = iyy
		self.iyz = iyz
		self.izz = izz

	def to_matrix(self):
		return [
			[self.ixx, self.ixy, self.ixz],
			[self.ixy, self.iyy, self.iyz],
			[self.ixz, self.iyz, self.izz]]

xmlr.reflect(Inertia, params = [xmlr.Element(key, float) for key in Inertia.KEYS])

# Pretty much copy-paste... Better method?
# Use multiple inheritance to separate the objects out so they are unique?
class Inertial(xmlr.Object):
	def __init__(self, mass = 0.0, inertia = None, pose=None):
		self.mass = mass
		self.inertia = inertia
		self.pose = pose

xmlr.reflect(Inertial, params = [
	xmlr.Element('mass', float),
	xmlr.Element('inertia', Inertia),
	pose_element
	])


class Link(Entity):
	def __init__(self, name = None, pose = None, inertial = None, kinematic = False):
		Entity.__init__(self, name, pose)
		self.inertial = inertial
		self.kinematic = kinematic

xmlr.reflect(Link, parent_cls = Entity, params = [
	xmlr.Element('inertial', Inertial),
	xmlr.Attribute('kinematic', bool, False),
	xmlr.AggregateElement('visual', Visual, var = 'visuals'),
	xmlr.AggregateElement('collision', Collision, var = 'collisions')
	])


class Model(Entity):
	def __init__(self, name = None, pose=None):
		Entity.__init__(self, name, pose)
		self.links = []
		self.joints = []
		self.plugins = []

xmlr.reflect(Model, parent_cls = Entity, params = [
	xmlr.AggregateElement('link', Link, var = 'links'),
	xmlr.AggregateElement('joint', Joint, var = 'joints'),
	xmlr.AggregateElement('plugin', Plugin, var = 'plugins')
	])

xmlr.end_namespace('sdf')
