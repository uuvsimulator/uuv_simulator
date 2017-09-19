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
#include <uuv_gazebo_plugins/HydrodynamicModel.hh>

namespace gazebo
{
/////////////////////////////////////////////////
HydrodynamicModel::HydrodynamicModel(sdf::ElementPtr _sdf,
    physics::LinkPtr _link) : BuoyantObject(_link)
{
  GZ_ASSERT(_link != NULL, "Invalid link pointer");

  // Initialize filtered acceleration & last velocity
  this->filteredAcc.setZero();
  this->lastVelRel.setZero();

  // Set volume
  if (_sdf->HasElement("volume"))
    this->volume = _sdf->Get<double>("volume");


  // Reading the information for the metacentric width and length in the case
  // that the model is a surface vessel or floating object
  if (_sdf->HasElement("metacentric_width") &&
      _sdf->HasElement("metacentric_length"))
  {
    this->metacentricWidth = _sdf->Get<double>("metacentric_width");
    this->metacentricLength = _sdf->Get<double>("metacentric_length");
    this->isSurfaceVessel = true;
    if (_sdf->HasElement("water_level_plane_area"))
      this->waterLevelPlaneArea = _sdf->Get<double>("water_level_plane_area");
    else
      this->waterLevelPlaneArea = 0.0;

    gzmsg << "Surface vessel parameters" << std::endl;
    gzmsg << "metacentric_width=" << this->metacentricWidth << std::endl;
    gzmsg << "metacentric_length=" << this->metacentricLength << std::endl;
    gzmsg << "water_level_plane_area=" << this->waterLevelPlaneArea << std::endl;
  }
  else
  {
    this->metacentricWidth = 0.0;
    this->metacentricLength = 0.0;
    this->waterLevelPlaneArea = 0.0;
    this->isSurfaceVessel = false;
  }

  // Get the center of buoyancy
  std::vector<double> cob = {0, 0, 0};
  if (_sdf->HasElement("center_of_buoyancy"))
  {
    cob = Str2Vector(_sdf->Get<std::string>("center_of_buoyancy"));
    this->SetCoB(math::Vector3(cob[0], cob[1], cob[2]));
  }
  // FIXME(mam0box) This is a work around the problem of the invalid bounding
  // box returned by Gazebo
  if (_sdf->HasElement("box"))
  {
    sdf::ElementPtr sdfModel = _sdf->GetElement("box");
    if (sdfModel->HasElement("width") && sdfModel->HasElement("length") &&
        sdfModel->HasElement("height"))
    {
      double width = sdfModel->Get<double>("width");
      double length = sdfModel->Get<double>("length");
      double height = sdfModel->Get<double>("height");
      math::Box boundingBox = math::Box(math::Vector3(-width/2,
                                                      -length/2,
                                                      -height/2),
                                        math::Vector3(width/2,
                                                      length/2,
                                                      height/2));
      // Setting the the bounding box from the given dimensions
      this->SetBoundingBox(boundingBox);
    }
  }

  // If neutrally buoyant is given, then calculate restoring
  // force to cancel out the gravitational force
  if (_sdf->HasElement("neutrally_buoyant"))
  {
    if (_sdf->Get<bool>("neutrally_buoyant"))
      this->SetNeutrallyBuoyant();
  }

  // Initialize Reynolds number with zero (will not always be used)
  this->Re = 0;

  // Initialize temperature (not used by all models)
  this->temperature = 0;
}

/////////////////////////////////////////////////
void HydrodynamicModel::ComputeAcc(Eigen::Vector6d _velRel, double _time,
                                  double _alpha)
{
  // Compute Fossen's nu-dot numerically. We have to do this for now since
  // Gazebo reports angular accelerations that are off by orders of magnitude.
  double dt = _time - lastTime;

  if (dt <= 0.0)  // Extra caution to prevent division by zero
    return;

  Eigen::Vector6d acc = (_velRel - this->lastVelRel) / dt;

  // TODO  We only have access to the acceleration of the previous simulation
  //       step. The added mass will induce a strong force/torque counteracting
  //       it in the current simulation step. This can lead to an oscillating
  //       system.
  //       The most accurate solution would probably be to first compute the
  //       latest acceleration without added mass and then use this to compute
  //       added mass effects. This is not how gazebo works, though.
  this->filteredAcc = (1.0 - _alpha) * this->filteredAcc + _alpha * acc;

  lastTime = _time;
  this->lastVelRel = _velRel;
}

/////////////////////////////////////////////////
math::Vector3 HydrodynamicModel::ToNEDConvention(math::Vector3 _vec)
{
  math::Vector3 output = _vec;
  output.y = -1 * output.y;
  output.z = -1 * output.z;
  return output;
}

/////////////////////////////////////////////////
math::Vector3 HydrodynamicModel::FromNEDConvention(math::Vector3 _vec)
{
  return this->ToNEDConvention(_vec);
}

/////////////////////////////////////////////////
bool HydrodynamicModel::CheckParams(sdf::ElementPtr _sdf)
{
  if (this->params.empty()) return true;

  for (auto tag : this->params)
  {
    if (!_sdf->HasElement(tag))
      {
        gzerr << "Hydrodynamic model: Expected element " <<
           tag << std::endl;
        return false;
      }
  }

  return true;
}

/////////////////////////////////////////////////
HydrodynamicModel * HydrodynamicModelFactory::CreateHydrodynamicModel(
    sdf::ElementPtr _sdf, physics::LinkPtr _link)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");
  sdf::ElementPtr sdfModel = _sdf->GetElement("hydrodynamic_model");
  if (!sdfModel->HasElement("type"))
  {
    std::cerr << "Model has no type" << std::endl;
    return NULL;
  }

  std::string identifier = sdfModel->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create HydrodynamicModel with unknown identifier: "
              << identifier << std::endl;
    return NULL;
  }

  return creators_[identifier](_sdf, _link);
}

/////////////////////////////////////////////////
HydrodynamicModelFactory& HydrodynamicModelFactory::GetInstance()
{
  static HydrodynamicModelFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool HydrodynamicModelFactory::RegisterCreator(const std::string& _identifier,
                               HydrodynamicModelCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering HydrodynamicModel with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;

  std::cout << "Registered HydrodynamicModel type " << _identifier << std::endl;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Fossen's robot-like equations of motion for underwater vehicles
//////////////////////////////////////////////////////////////////////////

const std::string HMFossen::IDENTIFIER = "fossen";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMFossen,
                                   &HMFossen::create);

/////////////////////////////////////////////////
HydrodynamicModel* HMFossen::create(sdf::ElementPtr _sdf,
                                    physics::LinkPtr _link)
{
  return new HMFossen(_sdf, _link);
}

/////////////////////////////////////////////////
HMFossen::HMFossen(sdf::ElementPtr _sdf,
                   physics::LinkPtr _link)
                  : HydrodynamicModel(_sdf, _link)
{
  std::vector<double> addedMass(36, 0.0);
  std::vector<double> linDampCoef(6, 0.0);
  std::vector<double> linDampForward(6, 0.0);
  std::vector<double> quadDampCoef(6, 0.0);

  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");
  // Load added-mass coefficients, if provided. Otherwise, the added-mass
  // matrix is set to zero
  if (modelParams->HasElement("added_mass"))
    addedMass = Str2Vector(modelParams->Get<std::string>("added_mass"));
  else
    gzmsg << "HMFossen: Using added mass NULL" << std::endl;

  this->params.push_back("added_mass");

  // Load linear damping coefficients, if provided. Otherwise, the linear
  // damping matrix is set to zero
  if (modelParams->HasElement("linear_damping"))
    linDampCoef = Str2Vector(modelParams->Get<std::string>("linear_damping"));
  else
    gzmsg << "HMFossen: Using linear damping NULL" << std::endl;

  this->params.push_back("linear_damping");

  // Load linear damping coefficients that described the damping forces
  // proportional to the forward speed only, if provided. Otherwise, the linear
  // damping matrix is set to zero
  if (modelParams->HasElement("linear_damping_forward_speed"))
    linDampForward = Str2Vector(
      modelParams->Get<std::string>("linear_damping_forward_speed"));
  else
    gzmsg << "HMFossen: Using linear damping for forward speed NULL"
      << std::endl;

  this->params.push_back("linear_damping_forward_speed");

  // Load nonlinear quadratic damping coefficients, if provided. Otherwise,
  // the nonlinear quadratic damping matrix is set to zero
  if (modelParams->HasElement("quadratic_damping"))
    quadDampCoef = Str2Vector(
        modelParams->Get<std::string>("quadratic_damping"));
  else
    gzmsg << "HMFossen: Using quad damping NULL" << std::endl;

  this->params.push_back("quadratic_damping");

  // Adding the volume
  this->params.push_back("volume");

  GZ_ASSERT(addedMass.size() == 36,
            "Added-mass coefficients vector must have 36 elements");
  GZ_ASSERT(linDampCoef.size() == 6 || linDampCoef.size() == 36,
            "Linear damping coefficients vector must have 6 elements for a "
            "diagonal matrix or 36 elements for a full matrix");
  GZ_ASSERT(linDampForward.size() == 6 || linDampForward.size() == 36,
            "Linear damping coefficients proportional to the forward speed "
            "vector must have 6 elements for a diagonal matrix or 36 elements"
            " for a full matrix");
  GZ_ASSERT(quadDampCoef.size() == 6 || quadDampCoef.size() == 36,
            "Quadratic damping coefficients vector must have 6 elements for a "
            "diagonal matrix or 36 elements for a full matrix");

  this->DLin.setZero();
  this->DNonLin.setZero();
  this->DLinForwardSpeed.setZero();

  for (int row = 0; row < 6; row++)
    for (int col = 0; col < 6; col++)
    {
      // Set added-mass coefficient
      this->Ma(row, col) = addedMass[6*row+col];
      // Set the linear damping matrix if a full matrix was provided
      if (linDampCoef.size() == 36)
        this->DLin(row, col) = linDampCoef[6*row+col];
      if (quadDampCoef.size() == 36)
        this->DNonLin(row, col) = quadDampCoef[6*row+col];
      if (linDampForward.size() == 36)
        this->DLinForwardSpeed(row, col) = linDampForward[6*row+col];
    }

  // In the case the linear damping matrix was set as a diagonal matrix
  for (int i = 0; i < 6; i++)
  {
    if (linDampCoef.size() == 6)
      this->DLin(i, i) = linDampCoef[i];
    if (quadDampCoef.size() == 6)
      this->DNonLin(i, i) = quadDampCoef[i];
    if (linDampForward.size() == 6)
      this->DLinForwardSpeed(i, i) = linDampForward[i];
  }

  // Store damping coefficients
  this->linearDampCoef = linDampCoef;
  this->quadDampCoef = quadDampCoef;
}

/////////////////////////////////////////////////
void HMFossen::ApplyHydrodynamicForces(
  double _time, const math::Vector3 &_flowVelWorld)
{
  // Link's pose
  const math::Pose pose = this->link->GetWorldPose();

  // Compute velocities in body frame (usually called q or nu)
  const math::Vector3 linVel = this->link->GetRelativeLinearVel();
  const math::Vector3 angVel = this->link->GetRelativeAngularVel();

  // Compute accelerations in body frame (\dot q or \dot nu):
  math::Quaternion rotWorld2Body = pose.rot.GetInverse();

  // Transform the flow velocity to the BODY frame
  math::Vector3 flowVel = pose.rot.GetInverse().RotateVector(_flowVelWorld);

  Eigen::Vector6d velRel, acc;
  // Compute the relative velocity
  velRel = EigenStack(
    this->ToNEDConvention(linVel - flowVel),
    this->ToNEDConvention(angVel));

  // Update added Coriolis matrix
  this->ComputeAddedCoriolisMatrix(velRel, this->Ma, this->Ca);

  // Update damping matrix
  this->ComputeDampingMatrix(velRel, this->D);

  // Filter acceleration (see issue explanation above)
  this->ComputeAcc(velRel, _time, 0.3);

  // We can now compute the additional forces/torques due to thisdynamic
  // effects based on Eq. 8.136 on p.222 of Fossen: Handbook of Marine Craft ...

  // Damping forces and torques
  Eigen::Vector6d damping = -this->D * velRel;

  // Added-mass forces and torques
  Eigen::Vector6d added = -this->Ma * this->filteredAcc;

  // Added Coriolis term
  Eigen::Vector6d cor = -this->Ca * velRel;

  // All additional (compared to standard rigid body) Fossen terms combined.
  Eigen::Vector6d tau = damping + added + cor;

  GZ_ASSERT(!std::isnan(tau.norm()), "Hydrodynamic forces vector is nan");

  if (!std::isnan(tau.norm()))
  {
    // Convert the forces and moments back to Gazebo's reference frame
    math::Vector3 hydForce =
      this->FromNEDConvention(Vec3dToGazebo(tau.head<3>()));
    math::Vector3 hydTorque =
      this->FromNEDConvention(Vec3dToGazebo(tau.tail<3>()));

    // Forces and torques are also wrt link frame
    this->link->AddRelativeForce(hydForce);
    this->link->AddRelativeTorque(hydTorque);
  }

  this->ApplyBuoyancyForce();

  if ( this->debugFlag )
  {
    // Store intermediate results for debugging purposes
    this->StoreVector(UUV_DAMPING_FORCE, Vec3dToGazebo(damping.head<3>()));
    this->StoreVector(UUV_DAMPING_TORQUE, Vec3dToGazebo(damping.tail<3>()));

    this->StoreVector(UUV_ADDED_MASS_FORCE, Vec3dToGazebo(added.head<3>()));
    this->StoreVector(UUV_ADDED_MASS_TORQUE, Vec3dToGazebo(added.tail<3>()));

    this->StoreVector(UUV_ADDED_CORIOLIS_FORCE, Vec3dToGazebo(cor.head<3>()));
    this->StoreVector(UUV_ADDED_CORIOLIS_TORQUE, Vec3dToGazebo(cor.tail<3>()));
  }
}

/////////////////////////////////////////////////
void HMFossen::ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                          const Eigen::Matrix6d& _Ma,
                                          Eigen::Matrix6d &_Ca) const
{
  // This corresponds to eq. 6.43 on p. 120 in
  // Fossen, Thor, "Handbook of Marine Craft and Hydrodynamics and Motion
  // Control", 2011
  Eigen::Vector6d ab = _Ma * _vel;
  Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
  _Ca << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1 * CrossProductOperator(ab.tail<3>());
}

/////////////////////////////////////////////////
void HMFossen::ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                    Eigen::Matrix6d &_D) const
{
  // From Antonelli 2014: the viscosity of the fluid causes
  // the presence of dissipative drag and lift forces on the
  // body. A common simplification is to consider only linear
  // and quadratic damping terms and group these terms in a
  // matrix Drb

  _D.setZero();

  _D = -1 * this->DLin - _vel[0] * this->DLinForwardSpeed;

  // Nonlinear damping matrix is considered as a diagonal matrix
  for (int i = 0; i < 6; i++)
  {
    _D(i, i) += -1 * this->DNonLin(i, i) * std::fabs(_vel[i]);
  }
}

/////////////////////////////////////////////////
bool HMFossen::GetParam(std::string _tag, std::vector<double>& _output)
{
  _output = std::vector<double>();
  if (!_tag.compare("added_mass"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->Ma(i, j));
  }
  else if (!_tag.compare("linear_damping"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DLin(i, j));
  }
  else if (!_tag.compare("linear_damping_forward_speed"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DLinForwardSpeed(i, j));
  }
  else if (!_tag.compare("quadratic_damping"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DNonLin(i, j));
  }
  else if (!_tag.compare("center_of_buoyancy"))
  {
    _output.push_back(this->centerOfBuoyancy.x);
    _output.push_back(this->centerOfBuoyancy.y);
    _output.push_back(this->centerOfBuoyancy.z);
  }
  else
    return false;
  return true;
}

/////////////////////////////////////////////////
bool HMFossen::GetParam(std::string _tag, double& _output)
{
  _output = -1.0;
  if (!_tag.compare("volume"))
    _output = this->volume;
  else if (!_tag.compare("fluid_density"))
    _output = this->fluidDensity;
  else if (!_tag.compare("bbox_height"))
    _output = this->boundingBox.GetZLength();
  else if (!_tag.compare("bbox_width"))
    _output = this->boundingBox.GetYLength();
  else if (!_tag.compare("bbox_length"))
    _output = this->boundingBox.GetXLength();
  else
  {
    _output = -1.0;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void HMFossen::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("all"))
  {
    for (auto tag : this->params)
      this->Print(tag);
    return;
  }
  if (!_message.empty())
    std::cout << _message << std::endl;
  else
    std::cout << this->link->GetModel()->GetName() << "::"
      << this->link->GetName() << "::" << _paramName
      << std::endl;
  if (!_paramName.compare("added_mass"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->Ma(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("linear_damping"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DLin(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("linear_damping_forward_speed"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DLinForwardSpeed(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("quadratic_damping"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DNonLin(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("volume"))
  {
    std::cout << std::setw(12) << this->volume << " m^3" << std::endl;
  }
}

//////////////////////////////////////////////////////////////////////////
// Hydrodynamic model for a sphere
//////////////////////////////////////////////////////////////////////////

const std::string HMSphere::IDENTIFIER = "sphere";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMSphere,
                                   &HMSphere::create);

/////////////////////////////////////////////////
HydrodynamicModel* HMSphere::create(sdf::ElementPtr _sdf,
                                    physics::LinkPtr _link)
{
  return new HMSphere(_sdf, _link);
}

/////////////////////////////////////////////////
HMSphere::HMSphere(sdf::ElementPtr _sdf,
                   physics::LinkPtr _link)
                   : HMFossen(_sdf, _link)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("radius"))
    this->radius = modelParams->Get<double>("radius");
  else
  {
    gzmsg << "HMSphere: Using the smallest length of bounding box as radius"
          << std::endl;
    this->radius = std::min(this->boundingBox.GetXLength(),
                            std::min(this->boundingBox.GetYLength(),
                                     this->boundingBox.GetZLength()));
  }
  gzmsg << "HMSphere::radius=" << this->radius << std::endl;
  gzmsg << "HMSphere: Computing added mass" << std::endl;

  this->params.push_back("radius");
  // Reynolds number for subcritical flow
  // Reference:
  //    - MIT Marine Hydrodynamic (Lecture Notes)
  // TODO Consider also critical flow
  this->Re = 3e5;

  // Drag coefficient for a sphere in subcritical flow
  // Reference:
  //    - MIT Marine Hydrodynamic (Lecture Notes)
  this->Cd = 0.5;

  // Area of the cross section
  this->areaSection = PI * std::pow(this->radius, 2.0);

  // See derivation in MIT's Marine Hydrodynamics lecture notes
  // The sphere has the same projected area for X, Y and Z

  // TODO Interpolate temperatures in look-up table
  double sphereMa = -2.0 / 3.0 * this->fluidDensity * PI * \
                   std::pow(this->radius, 3.0);
  // At the moment, only pressure drag is calculated, no skin friction drag
  double Dq = -0.5 * this->fluidDensity * this->Cd * this->areaSection;

  for (int i = 0; i < 3; i++)
  {
    // Setting the added mass
    this->Ma(i, i) = -sphereMa;
    // Setting the pressure drag
    this->quadDampCoef[i] = Dq;
  }
}

/////////////////////////////////////////////////
void HMSphere::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("all"))
  {
    for (auto tag : this->params)
      this->Print(tag);
    return;
  }
  if (!_message.empty())
    std::cout << _message << std::endl;
  else
    std::cout << this->link->GetModel()->GetName() << "::"
      << this->link->GetName() << "::" << _paramName
      << std::endl;
  if (!_paramName.compare("radius"))
    std::cout << std::setw(12) << this->radius << std::endl;
  else
    HMFossen::Print(_paramName, _message);
}

//////////////////////////////////////////////////////////////////////////
// Hydrodynamic model for a cylinder
//////////////////////////////////////////////////////////////////////////

const std::string HMCylinder::IDENTIFIER = "cylinder";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMCylinder,
                                   &HMCylinder::create);

/////////////////////////////////////////////////
HydrodynamicModel* HMCylinder::create(sdf::ElementPtr _sdf,
                                      physics::LinkPtr _link)
{
  return new HMCylinder(_sdf, _link);
}

/////////////////////////////////////////////////
HMCylinder::HMCylinder(sdf::ElementPtr _sdf,
                       physics::LinkPtr _link)
                       : HMFossen(_sdf, _link)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("radius"))
    this->radius = modelParams->Get<double>("radius");
  else
  {
    gzmsg << "HMCylinder: Using the smallest length of bounding box as radius"
          << std::endl;
    this->radius = std::min(this->boundingBox.GetXLength(),
                            std::min(this->boundingBox.GetYLength(),
                                     this->boundingBox.GetZLength()));
  }
  gzmsg << "HMCylinder::radius=" << this->radius << std::endl;

  if (modelParams->HasElement("length"))
    this->length = modelParams->Get<double>("length");
  else
  {
      gzmsg << "HMCylinder: Using the biggest length of bounding box as length"
            << std::endl;
      this->length = std::max(this->boundingBox.GetXLength(),
                              std::max(this->boundingBox.GetYLength(),
                                       this->boundingBox.GetZLength()));
  }
  gzmsg << "HMCylinder::length=" << this->length << std::endl;

  this->dimRatio = this->length / (2* this->radius);

  gzmsg << "HMCylinder::dimension_ratio=" << this->dimRatio << std::endl;

  // Approximation of drag coefficients
  // Reference: http://www.mech.pk.edu.pl/~m52/pdf/fm/R_09.pdf
  // For the circular profile
  if (this->dimRatio <= 1) this->cdCirc = 0.91;
  else if (this->dimRatio > 1 && this->dimRatio <= 2) this->cdCirc = 0.85;
  else if (this->dimRatio > 2 && this->dimRatio <= 4) this->cdCirc = 0.87;
  else if (this->dimRatio > 4 && this->dimRatio <= 7) this->cdCirc = 0.99;

  // For the rectagular profile
  if (this->dimRatio <= 1) this->cdLength = 0.63;
  else if (this->dimRatio > 1 && this->dimRatio <= 2) this->cdLength = 0.68;
  else if (this->dimRatio > 2 && this->dimRatio <= 5) this->cdLength = 0.74;
  else if (this->dimRatio > 5 && this->dimRatio <= 10) this->cdLength = 0.82;
  else if (this->dimRatio > 10 && this->dimRatio <= 40) this->cdLength = 0.98;
  else if (this->dimRatio > 40) this->cdLength = 0.98;

  if (modelParams->HasElement("axis"))
  {
    this->axis = modelParams->Get<std::string>("axis");
    GZ_ASSERT(this->axis.compare("i") == 0 ||
              this->axis.compare("j") == 0 ||
              this->axis.compare("k") == 0, "Invalid axis of rotation");
  }
  else
  {
    gzmsg << "HMCylinder: Using the direction of biggest length as axis"
          << std::endl;
    double maxLength = std::max(this->boundingBox.GetXLength(),
                                std::max(this->boundingBox.GetYLength(),
                                         this->boundingBox.GetZLength()));
    if (maxLength == this->boundingBox.GetXLength())
      this->axis = "i";
    else if (maxLength == this->boundingBox.GetYLength())
      this->axis = "j";
    else
      this->axis = "k";
  }
  gzmsg << "HMCylinder::rotation_axis=" << this->axis << std::endl;

  // Calculating the added mass and damping for the cylinder
  // Calculate added mass coefficients for the cylinder along its length
  double MaLength = -this->fluidDensity * PI *
                    std::pow(this->radius, 2.0) * this->length;

  // Calculate the added mass coefficients for the circular area
  double MaCirc = -this->fluidDensity * PI * std::pow(this->radius, 2.0);

  // Calculating added mass torque coefficients
  // Reference: Schjolberg, 1994 (Modelling and Control of Underwater-Vehicle
  // Manipulator System)
  double MaLengthTorque = (-1.0/12.0) * this->fluidDensity * PI \
                * std::pow(this->radius, 2.0) * std::pow(this->length, 3.0);

  // Calculate drag forces and moments
  // At the moment, only pressure drag is calculated, no skin friction drag
  double DCirc = -0.5 * this->cdCirc * PI * std::pow(this->radius, 2.0) \
                    * this->fluidDensity;
  double DLength = -0.5 * this->cdLength * this->radius * this->length \
                    * this->fluidDensity;

  if (this->axis.compare("i") == 0)
  {
      this->Ma(0, 0) = -MaCirc;
      this->Ma(1, 1) = -MaLength;
      this->Ma(2, 2) = -MaLength;

      this->Ma(4, 4) = -MaLengthTorque;
      this->Ma(5, 5) = -MaLengthTorque;

      this->DNonLin(0, 0) = DCirc;
      this->DNonLin(1, 1) = DLength;
      this->DNonLin(2, 2) = DLength;
  }
  else if (this->axis.compare("j") == 0)
  {
      this->Ma(0, 0) = -MaLength;
      this->Ma(1, 1) = -MaCirc;
      this->Ma(2, 2) = -MaLength;

      this->Ma(3, 3) = -MaLengthTorque;
      this->Ma(5, 5) = -MaLengthTorque;

      this->DNonLin(0, 0) = DLength;
      this->DNonLin(1, 1) = DCirc;
      this->DNonLin(2, 2) = DLength;
  }
  else
  {
      this->Ma(0, 0) = -MaLength;
      this->Ma(1, 1) = -MaLength;
      this->Ma(2, 2) = -MaCirc;

      this->Ma(3, 3) = -MaLengthTorque;
      this->Ma(4, 4) = -MaLengthTorque;

      this->DNonLin(0, 0) = DLength;
      this->DNonLin(1, 1) = DLength;
      this->DNonLin(2, 2) = DCirc;
  }
}

/////////////////////////////////////////////////
void HMCylinder::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("radius"))
  {
    if (!_message.empty())
      gzmsg << this->link->GetName() << std::endl;
    std::cout << std::setw(12) << this->radius << std::endl;
  }
  else if (!_paramName.compare("length"))
  {
    if (!_message.empty())
      gzmsg << _message << std::endl;
    std::cout << std::setw(12) << this->length << std::endl;
  }
  else
    HMFossen::Print(_paramName, _message);
}

//////////////////////////////////////////////////////////////////////////
// Hydrodynamic model for a spheroid (STILL IN DEVELOPMENT, DON'T USE IT)
//////////////////////////////////////////////////////////////////////////
const std::string HMSpheroid::IDENTIFIER = "spheroid";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMSpheroid,
                                   &HMSpheroid::create);

/////////////////////////////////////////////////
HydrodynamicModel* HMSpheroid::create(sdf::ElementPtr _sdf,
                                      physics::LinkPtr _link)
{
  return new HMSpheroid(_sdf, _link);
}

/////////////////////////////////////////////////
HMSpheroid::HMSpheroid(sdf::ElementPtr _sdf,
                       physics::LinkPtr _link)
                       : HMFossen(_sdf, _link)
{
  gzerr << "Hydrodynamic model for a spheroid is still in development!"
    << std::endl;
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("radius"))
    this->radius = modelParams->Get<double>("radius");
  else
  {
    gzmsg << "HMSpheroid: Using the smallest length of bounding box as radius"
          << std::endl;
    this->radius = std::min(this->boundingBox.GetXLength(),
                            std::min(this->boundingBox.GetYLength(),
                                     this->boundingBox.GetZLength()));
  }
  GZ_ASSERT(this->radius > 0, "Radius cannot be negative");
  gzmsg << "HMSpheroid::radius=" << this->radius << std::endl;

  if (modelParams->HasElement("length"))
    this->length = modelParams->Get<double>("length");
  else
  {
      gzmsg << "HMSpheroid: Using the biggest length of bounding box as length"
            << std::endl;
      this->length = std::max(this->boundingBox.GetXLength(),
                              std::max(this->boundingBox.GetYLength(),
                                       this->boundingBox.GetZLength()));
  }
  GZ_ASSERT(this->length > 0, "Length cannot be negative");
  gzmsg << "HMSpheroid::length=" << this->length << std::endl;

  // Eccentricity
  double ecc = std::sqrt(1 -
               std::pow(this->radius / this->length, 2.0));

  gzmsg << "ecc=" << ecc << std::endl;

  double ln = std::log((1 + ecc) / (1 - ecc));
  double alpha = 2 * (1 - std::pow(ecc, 2.0)) / std::pow(ecc, 3.0);

  alpha *= (0.5 * ln - ecc);

  double beta = 1 / std::pow(ecc, 2.0) - \
                (1 - std::pow(ecc, 2.0) / (2 * std::pow(ecc, 3.0))) * ln;

  gzmsg << "alpha=" << alpha << std::endl;
  gzmsg << "beta=" << beta << std::endl;

  double mass = this->link->GetInertial()->GetMass();

  this->Ma(0, 0) = mass * alpha / (2 - alpha);
  this->Ma(1, 1) = mass * beta / (2 - beta);
  this->Ma(2, 2) = this->Ma(1, 1);
  this->Ma(3, 3) = 0;

  double ba_minus = std::pow(this->radius, 2.0) - std::pow(this->length, 2.0);
  double ba_plus = std::pow(this->radius, 2.0) + std::pow(this->length, 2.0);
  this->Ma(4, 4) = -0.2 * mass * std::pow(ba_minus, 2.0) * (alpha - beta);
  this->Ma(4, 4) /= (2 * ba_minus - ba_plus * (alpha - beta));

  this->Ma(5, 5) = this->Ma(4, 4);
}

/////////////////////////////////////////////////
void HMSpheroid::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("radius"))
  {
    if (!_message.empty())
      gzmsg << this->link->GetName() << std::endl;
    std::cout << std::setw(12) << this->radius << std::endl;
  }
  else if (!_paramName.compare("length"))
  {
    if (!_message.empty())
      gzmsg << _message << std::endl;
    std::cout << std::setw(12) << this->length << std::endl;
  }
  else
    HMFossen::Print(_paramName, _message);
}

//////////////////////////////////////////////////////////////////////////
// Hydrodynamic model for a box (STILL IN DEVELOPMENT, DON'T USE IT)
//////////////////////////////////////////////////////////////////////////

const std::string HMBox::IDENTIFIER = "box";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMBox,
                                   &HMBox::create);

/////////////////////////////////////////////////
HydrodynamicModel* HMBox::create(sdf::ElementPtr _sdf,
                                 physics::LinkPtr _link)
{
  return new HMBox(_sdf, _link);
}

/////////////////////////////////////////////////
HMBox::HMBox(sdf::ElementPtr _sdf,
             physics::LinkPtr _link)
             : HMFossen(_sdf, _link)
{
  gzerr << "Hydrodynamic model for box is still in development!" << std::endl;

  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("cd"))
    this->Cd = modelParams->Get<double>("cd");
  else
  {
    gzmsg << "HMBox: Using 1 as drag coefficient"
          << std::endl;
    this->Cd = 1;
  }

  GZ_ASSERT(modelParams->HasElement("length"), "Length of the box is missing");
  GZ_ASSERT(modelParams->HasElement("width"), "Width of the box is missing");
  GZ_ASSERT(modelParams->HasElement("height"), "Height of the box is missing");

  this->length = modelParams->Get<double>("length");
  this->width = modelParams->Get<double>("width");
  this->height = modelParams->Get<double>("height");

  // Calculate drag force coefficients
  this->quadDampCoef[0] = -0.5 * this->Cd * this->width * this->height \
                    * this->fluidDensity;
  this->quadDampCoef[1] = -0.5 * this->Cd * this->length * this->height \
                    * this->fluidDensity;
  this->quadDampCoef[2] = -0.5 * this->Cd * this->width * this->length \
                    * this->fluidDensity;
}

/////////////////////////////////////////////////
void HMBox::Print(std::string _paramName, std::string _message)
{
    if (!_paramName.compare("length"))
    {
      if (!_message.empty())
        gzmsg << _message << std::endl;
      std::cout << std::setw(12) << this->length << std::endl;
    }
    else if (!_paramName.compare("width"))
    {
      if (!_message.empty())
        gzmsg << _message << std::endl;
      std::cout << std::setw(12) << this->width << std::endl;
    }
    else if (!_paramName.compare("height"))
    {
      if (!_message.empty())
        gzmsg << _message << std::endl;
      std::cout << std::setw(12) << this->height << std::endl;
    }
    else
        HMFossen::Print(_paramName, _message);
}
}
