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

#include <uuv_gazebo_plugins/Dynamics.hh>

namespace gazebo {

/////////////////////////////////////////////////
void Dynamics::Reset()
{
  this->prevTime = -10.;
  this->state = 0.;
}

/////////////////////////////////////////////////
Dynamics* DynamicsFactory::CreateDynamics(
    sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("type"))
  {
    std::cerr << "dynamics does not have a type element" << std::endl;
    return NULL;
  }

  std::string identifier = _sdf->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create ThrusterDynamics with unknown identifier: "
              << identifier << std::endl;
    return NULL;
  }

  return creators_[identifier](_sdf);
}

/////////////////////////////////////////////////
DynamicsFactory& DynamicsFactory::GetInstance()
{
  static DynamicsFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool DynamicsFactory::RegisterCreator(const std::string& _identifier,
                               DynamicsCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering ThrusterDynamics with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;

  std::cout << "Registered ThrusterDynamics type " << _identifier << std::endl;
  return true;
}

/////////////////////////////////////////////////
const std::string DynamicsZeroOrder::IDENTIFIER = "ZeroOrder";
REGISTER_DYNAMICS_CREATOR(DynamicsZeroOrder,
                          &DynamicsZeroOrder::create)

/////////////////////////////////////////////////
Dynamics* DynamicsZeroOrder::create(sdf::ElementPtr _sdf)
{
    return new DynamicsZeroOrder();
}

/////////////////////////////////////////////////
double DynamicsZeroOrder::update(double _cmd, double _t)
{
  return _cmd;  // no dynamics
}

/////////////////////////////////////////////////
const std::string DynamicsFirstOrder::IDENTIFIER = "FirstOrder";
REGISTER_DYNAMICS_CREATOR(DynamicsFirstOrder,
                          &DynamicsFirstOrder::create)

/////////////////////////////////////////////////
Dynamics* DynamicsFirstOrder::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("timeConstant"))
  {
    std::cerr << "DynamicsFirstOrder: expected element time_constant"
                 << std::endl;
    return NULL;
  }
  double tau = _sdf->Get<double>("timeConstant");
  return new DynamicsFirstOrder(tau);
}

/////////////////////////////////////////////////
double DynamicsFirstOrder::update(double _cmd, double _t)
{
  if (prevTime < 0)
  {
    prevTime = _t;
    return state;
  }

  double dt = _t - prevTime;

  double alpha = std::exp(-dt/tau);
  state = state*alpha + (1.0 - alpha)*_cmd;

  prevTime = _t;

  return state;
}

/////////////////////////////////////////////////
DynamicsFirstOrder::DynamicsFirstOrder(double _tau)
  : Dynamics(), tau(_tau)
{
}

/////////////////////////////////////////////////
const std::string ThrusterDynamicsYoerger::IDENTIFIER = "Yoerger";
REGISTER_DYNAMICS_CREATOR(ThrusterDynamicsYoerger,
                          &ThrusterDynamicsYoerger::create)

/////////////////////////////////////////////////
Dynamics* ThrusterDynamicsYoerger::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("alpha"))
  {
    std::cerr << "ThrusterDynamicsYoerger: expected element alpha"
                 << std::endl;
    return NULL;
  }
  double alpha = _sdf->Get<double>("alpha");

  if (!_sdf->HasElement("beta"))
  {
    std::cerr << "ThrusterDynamicsYoerger: expected element beta"
                 << std::endl;
    return NULL;
  }
  double beta = _sdf->Get<double>("beta");
  return new ThrusterDynamicsYoerger(alpha, beta);
}

/////////////////////////////////////////////////
double ThrusterDynamicsYoerger::update(double _cmd, double _t)
{
  if (prevTime < 0)
  {
    prevTime = _t;
    return state;
  }

  double dt = _t - prevTime;

  state += dt*(beta*_cmd - alpha*state*std::abs(state));

  return state;
}

/////////////////////////////////////////////////
ThrusterDynamicsYoerger::ThrusterDynamicsYoerger(double _alpha, double _beta)
  : Dynamics(), alpha(_alpha), beta(_beta)
{
}


/////////////////////////////////////////////////
const std::string ThrusterDynamicsBessa::IDENTIFIER = "Bessa";
REGISTER_DYNAMICS_CREATOR(ThrusterDynamicsBessa,
                          &ThrusterDynamicsBessa::create)

/////////////////////////////////////////////////
Dynamics* ThrusterDynamicsBessa::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("Jmsp"))
  {
    std::cerr << "ThrusterDynamicsBessa: expected element Jmsp"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("Kv1"))
  {
    std::cerr << "ThrusterDynamicsBessa: expected element Kv1"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("Kv2"))
  {
    std::cerr << "ThrusterDynamicsBessa: expected element Kv2"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("Kt"))
  {
    std::cerr << "ThrusterDynamicsBessa: expected element Kt"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("Rm"))
  {
    std::cerr << "ThrusterDynamicsBessa: expected element Rm"
                 << std::endl;
    return NULL;
  }
  return new ThrusterDynamicsBessa(_sdf->Get<double>("Jmsp"),
                                   _sdf->Get<double>("Kv1"),
                                   _sdf->Get<double>("Kv2"),
                                   _sdf->Get<double>("Kt"),
                                   _sdf->Get<double>("Rm"));
}

/////////////////////////////////////////////////
double ThrusterDynamicsBessa::update(double _cmd, double _t)
{
  if (prevTime < 0)
  {
    prevTime = _t;
    return state;
  }

  double dt = _t - prevTime;

  state += dt*(_cmd*Kt/Rm - Kv1*state
                - Kv2*state*std::abs(state))/Jmsp;

  return state;
}

/////////////////////////////////////////////////
ThrusterDynamicsBessa::ThrusterDynamicsBessa(double _Jmsp, double _Kv1,
                                             double _Kv2, double _Kt,
                                             double _Rm) :
  Dynamics(), Jmsp(_Jmsp), Kv1(_Kv1), Kv2(_Kv2), Kt(_Kt), Rm(_Rm)
{
}
}
