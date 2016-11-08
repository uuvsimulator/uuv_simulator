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

import string
import yaml
import collections
from lxml import etree

# Different implementations mix well it seems
# @todo Do not use this?
from xml.etree.ElementTree import ElementTree

def xml_string(rootXml, addHeader = True):
    # Meh
    xmlString = etree.tostring(rootXml, pretty_print = True)
    if addHeader:
        xmlString = '<?xml version="1.0"?>\n' + xmlString
    return xmlString

def dict_sub(obj, keys):
    return dict((key, obj[key]) for key in keys)

def node_add(doc, sub):
    if sub is None:
        return None
    if type(sub) == str:
        return etree.SubElement(doc, sub)
    elif isinstance(sub, etree._Element):
        doc.append(sub) # This screws up the rest of the tree for prettyprint...
        return sub
    else:
        raise Exception('Invalid sub value')

def pfloat(x):
    return str(x).rstrip('.')

def xml_children(node):
    children = node.getchildren()
    def predicate(node):
        return not isinstance(node, etree._Comment)
    return list(filter(predicate, children))

def isstring(obj):
    try:
        return isinstance(obj, basestring)
    except NameError:
        return isinstance(obj, str)

def to_yaml(obj):
    """ Simplify yaml representation for pretty printing """
    # Is there a better way to do this by adding a representation with yaml.Dumper?
    # Ordered dict: http://pyyaml.org/ticket/29#comment:11
    if obj is None or isstring(obj):
        out = str(obj)
    elif type(obj) in [int, float, bool]:
        return obj
    elif hasattr(obj, 'to_yaml'):
        out = obj.to_yaml()
    elif isinstance(obj, etree._Element):
    	out = etree.tostring(obj, pretty_print = True)
    elif type(obj) == dict:
        out = {}
        for (var, value) in obj.items():
            out[str(var)] = to_yaml(value)
    elif hasattr(obj, 'tolist'):
        # For numpy objects
        out = to_yaml(obj.tolist())
    elif isinstance(obj, collections.Iterable):
        out = [to_yaml(item) for item in obj]
    else:
        out = str(obj)
    return out

class SelectiveReflection(object):
	def get_refl_vars(self):
		return list(vars(self).keys())

class YamlReflection(SelectiveReflection):
	def to_yaml(self):
		raw = dict((var, getattr(self, var)) for var in self.get_refl_vars())
		return to_yaml(raw)

	def __str__(self):
		return yaml.dump(self.to_yaml()).rstrip() # Good idea? Will it remove other important things?
