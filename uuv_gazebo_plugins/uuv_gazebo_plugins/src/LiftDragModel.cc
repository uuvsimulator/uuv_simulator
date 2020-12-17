// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/gazebo.hh>

#include <uuv_gazebo_plugins/LiftDragModel.hh>

namespace gazebo {

/////////////////////////////////////////////////
bool LiftDrag::CheckForElement(sdf::ElementPtr _sdf,
                               const std::string &element)
{
  if (!_sdf->HasElement(element))
  {
    std::cerr << " LiftDrag: Missing required element: "
              << element
              << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
LiftDrag* LiftDragFactory::CreateLiftDrag(
    sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("type"))
  {
    std::cerr << "liftdrag does not have a type element" << std::endl;
    return NULL;
  }

  std::string identifier = _sdf->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create LiftDrag with unknown identifier: "
              << identifier << std::endl;
    return NULL;
  }

  return creators_[identifier](_sdf);
}

/////////////////////////////////////////////////
LiftDragFactory& LiftDragFactory::GetInstance()
{
  static LiftDragFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool LiftDragFactory::RegisterCreator(const std::string& _identifier,
                                      LiftDragCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering LiftDrag with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;

  std::cout << "Registered LiftDrag type " << _identifier << std::endl;
  return true;
}

/////////////////////////////////////////////////
const std::string LiftDragQuadratic::IDENTIFIER = "Quadratic";
REGISTER_LIFTDRAG_CREATOR(LiftDragQuadratic,
                          &LiftDragQuadratic::create)

/////////////////////////////////////////////////
LiftDrag* LiftDragQuadratic::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("lift_constant"))
  {
    std::cerr << "LiftDragQuadratic: expected element lift_constant"
              << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("drag_constant"))
  {
    std::cerr << "LiftDragQuadratic: expected element drag_constant"
              << std::endl;
    return NULL;
  }

  gzmsg << "Lift constant= " << _sdf->Get<double>("lift_constant") << std::endl;
  gzmsg << "Drag constant= " << _sdf->Get<double>("drag_constant") << std::endl;

  return new LiftDragQuadratic(_sdf->Get<double>("lift_constant"),
                               _sdf->Get<double>("drag_constant"));
}

/////////////////////////////////////////////////
ignition::math::Vector3d LiftDragQuadratic::compute(
  const ignition::math::Vector3d &_velL)
{
  ignition::math::Vector3d velL = _velL;
  double angle = atan2(_velL.Y(), _velL.X());

  if (angle > M_PI_2)
  {
    angle -= M_PI;
    velL = -_velL;
  }
  else if (angle < -M_PI_2)
  {
    angle += M_PI;
    velL = -_velL;
  }

  double u = velL.Length();
  double u2  = u * u;
  double du2 = angle * u2;

  double drag = angle * du2 * this->dragConstant;
  double lift = du2 * this->liftConstant;

  ignition::math::Vector3d liftDirectionL =
    -ignition::math::Vector3d::UnitZ.Cross(_velL).Normalize();
  ignition::math::Vector3d dragDirectionL = -_velL;

  return lift*liftDirectionL + drag*dragDirectionL.Normalize();
}

/////////////////////////////////////////////////
bool LiftDragQuadratic::GetParam(std::string _tag, double& _output)
{
  _output = 0.0;
  if (!_tag.compare("drag_constant"))
    _output = this->dragConstant;
  else if (!_tag.compare("lift_constant"))
    _output = this->liftConstant;
  else
    return false;

  gzmsg << "LiftDragQuadratic::GetParam <" << _tag << ">=" << _output <<
    std::endl;
  return true;
}

/////////////////////////////////////////////////
std::map<std::string, double> LiftDragQuadratic::GetListParams()
{
  std::map<std::string, double> params;
  params["drag_constant"] = this->dragConstant;
  params["lift_constant"] = this->liftConstant;
  return params;
}

/////////////////////////////////////////////////
const std::string LiftDragTwoLines::IDENTIFIER = "TwoLines";
REGISTER_LIFTDRAG_CREATOR(LiftDragTwoLines,
                          &LiftDragTwoLines::create)

/////////////////////////////////////////////////
LiftDrag* LiftDragTwoLines::create(sdf::ElementPtr _sdf)
{
  if (LiftDrag::CheckForElement(_sdf, "area") &&
      LiftDrag::CheckForElement(_sdf, "fluid_density") &&
      LiftDrag::CheckForElement(_sdf, "a0") &&
      LiftDrag::CheckForElement(_sdf, "alpha_stall") &&
      LiftDrag::CheckForElement(_sdf, "cla") &&
      LiftDrag::CheckForElement(_sdf, "cla_stall") &&
      LiftDrag::CheckForElement(_sdf, "cda") &&
      LiftDrag::CheckForElement(_sdf, "cda_stall"))
  {
    return new LiftDragTwoLines(_sdf->Get<double>("area"),
                                _sdf->Get<double>("fluid_density"),
                                _sdf->Get<double>("a0"),
                                _sdf->Get<double>("alpha_stall"),
                                _sdf->Get<double>("cla"),
                                _sdf->Get<double>("cla_stall"),
                                _sdf->Get<double>("cda"),
                                _sdf->Get<double>("cda_stall"));
  }
  else
    return NULL;
}

/////////////////////////////////////////////////
ignition::math::Vector3d LiftDragTwoLines::compute(const ignition::math::Vector3d &_velL)
{
  // The following computations are based on the LiftDragPlugin included
  // in Gazebo simulator 7.0 (http://gazebosim.org)
  // Copyright (C) 2012-2016 Open Source Robotics Foundation
  // This source code is licensed under the Apache-2.0 License found in
  // the open_source_licenses.txt file in the root directory of this source
  // tree.
  ignition::math::Vector3d velL = _velL;
  double angle = atan2(_velL.Y(), _velL.X());

  // Make sure angle is in [-pi/2, pi/2]
  if (angle > M_PI_2)
  {
    angle -= M_PI;
    velL = -_velL;
  }
  else if (angle < -M_PI_2)
  {
    angle += M_PI;
    velL = -_velL;
  }

  double alpha = angle + this->a0;

  // Again, normalize to within +/-90 deg
  while (fabs(alpha) > 0.5 * M_PI)
  {
    alpha = alpha > 0 ? alpha - M_PI : alpha + M_PI;
  }

  double u = velL.Length();

  // Compute dynamic pressure:
  double q = 0.5 * this->fluidDensity * u * u;

  // Compute lift&drag coefficients:
  double cl, cd;
  if (alpha > this->alphaStall)
  {
    // stall
    double diffAlpha = alpha - this->alphaStall;
    cl = this->cla*this->alphaStall +
        this->claStall * diffAlpha;
    cd = this->cda*this->alphaStall +
        this->cdaStall * diffAlpha;
  }
  else if (alpha < -this->alphaStall)
  {
    // stall
    double sumAlpha = alpha + this->alphaStall;
    cl = -this->cla * this->alphaStall +
        this->claStall * sumAlpha;
    cd = this->cda * this->alphaStall +
        -this->cdaStall * sumAlpha;
  }
  else
  {
    // no stall
    // angle of attack > -a0
    if (alpha > 0)
    {
        cd = this->cda * alpha;
        cl = this->cla * alpha;
    }
    // angle of attack <= -a0
    else
    {
        cd = -this->cda * alpha;
        cl = this->cla * alpha;
    }
  }

  double lift = cl*q*this->area;
  double drag = cd*q*this->area;

  ignition::math::Vector3d liftDirectionL = -ignition::math::Vector3d::UnitZ.Cross(_velL).Normalize();
  ignition::math::Vector3d dragDirectionL = -_velL;

  return lift*liftDirectionL + drag*dragDirectionL.Normalize();
}

/////////////////////////////////////////////////
bool LiftDragTwoLines::GetParam(std::string _tag, double& _output)
{
  _output = 0.0;
  if (!_tag.compare("area"))
    _output = this->area;
  else if (!_tag.compare("fluid_density"))
    _output = this->fluidDensity;
  else if (!_tag.compare("a0"))
    _output = this->a0;
  else if (!_tag.compare("alpha_stall"))
    _output = this->alphaStall;
  else if (!_tag.compare("cla"))
    _output = this->cla;
  else if (!_tag.compare("cla_stall"))
    _output = this->claStall;
  else if (!_tag.compare("cda"))
    _output = this->cda;
  else if (!_tag.compare("cda_stall"))
    _output = this->cdaStall;
  else
    return false;

  gzmsg << "LiftDragQuadratic::GetParam <" << _tag << ">=" << _output <<
    std::endl;
  return true;
}

/////////////////////////////////////////////////
std::map<std::string, double> LiftDragTwoLines::GetListParams()
{
  std::map<std::string, double> params;
  params["area"] = this->area;
  params["fluid_density"] = this->fluidDensity;
  params["a0"] = this->a0;
  params["alpha_stall"] = this->alphaStall;
  params["cla"] = this->cla;
  params["cla_stall"] = this->claStall;
  params["cda"] = this->cda;
  params["cda_stall"] = this->cdaStall;
  return params;
}

}
