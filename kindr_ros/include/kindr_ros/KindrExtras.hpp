/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2020, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef KINDR_EXTRAS_H
#define KINDR_EXTRAS_H
#include <kindr/Core>
#include <ros/duration.h>

namespace kindr {

typedef kindr::HomogeneousTransformationPosition3RotationQuaternionD MatrixHom;

template<typename KindrType>
inline std::vector<typename KindrType::Implementation> kindrMatVectorToEigen(const std::vector<KindrType> &vector)
{
  std::vector<typename KindrType::Implementation> ret;
  ret.reserve(vector.size());
  for (const auto &m : vector)
  {
    ret.push_back(m.toImplementation());
  }
  return ret;
}

inline std::vector<Eigen::Isometry3d> kindrMatVectorToEigen(const std::vector<kindr::MatrixHom> &vector)
{
  std::vector<Eigen::Isometry3d> ret;
  ret.reserve(vector.size());
  for (const auto &m : vector)
  {
    ret.push_back(Eigen::Isometry3d(m.getTransformationMatrix()));
  }
  return ret;
}



template<typename PrimType_>
class Seconds
{
public:
  inline explicit Seconds(double seconds)
    : impl_(seconds)
  {}
  PrimType_ impl_;
};
typedef Seconds<double> SecondsD;
typedef Seconds<float> SecondsF;

//! \brief Angle Vector missing from kindr, we make no assumption on how to apply the rotation
template <typename PrimType_, int Dimension_>
using Angle = Vector<PhysicalType::Angle, PrimType_, Dimension_>;
typedef Angle<double, 3> Angle3D;

typedef Time<double, 1> Time1D;
typedef Velocity<double, 1> Velocity1D;
typedef Position<double, 1> Position1D;
typedef Acceleration<double, 1> Acceleration1D;
typedef Jerk<double, 1> Jerk1D;
typedef Force<double, 1> Force1D;
typedef Momentum<double, 1> Momentum1D;
typedef AngularJerk<double, 1> AngularJerk1D;
typedef AngularAcceleration<double, 1> AngularAcceleration1D;
typedef AngularVelocity<double, 1> AngularVelocity1D;
typedef Torque<double, 1> Torque1D;
typedef AngularMomentum<double, 1> AngularMomentum1D;
typedef Angle<double, 1> Angle1D;


typedef Time<double, 2> Time2D;
typedef Velocity<double, 2> Velocity2D;
typedef Position<double, 2> Position2D;
typedef Acceleration<double, 2> Acceleration2D;
typedef Jerk<double, 2> Jerk2D;
typedef Force<double, 2> Force2D;
typedef Momentum<double, 2> Momentum2D;
typedef AngularJerk<double, 2> AngularJerk2D;
typedef AngularAcceleration<double, 2> AngularAcceleration2D;
typedef AngularVelocity<double, 2> AngularVelocity2D;
typedef Torque<double, 2> Torque2D;
typedef AngularMomentum<double, 2> AngularMomentum2D;
typedef Angle<double, 2> Angle2D;


typedef Time<double, Eigen::Dynamic> TimeXD;
typedef Velocity<double, Eigen::Dynamic> VelocityXD;
typedef Position<double, Eigen::Dynamic> PositionXD;
typedef Acceleration<double, Eigen::Dynamic> AccelerationXD;
typedef Jerk<double, Eigen::Dynamic> JerkXD;
typedef Force<double, Eigen::Dynamic> ForceXD;
typedef Momentum<double, Eigen::Dynamic> MomentumXD;
typedef AngularJerk<double, Eigen::Dynamic> AngularJerkXD;
typedef AngularAcceleration<double, Eigen::Dynamic> AngularAccelerationXD;
typedef AngularVelocity<double, Eigen::Dynamic> AngularVelocityXD;
typedef Torque<double, Eigen::Dynamic> TorqueXD;
typedef AngularMomentum<double, Eigen::Dynamic> AngularMomentumXD;
typedef Angle<double, Eigen::Dynamic> AngleXD;

}
namespace kindr {

// This is specialized by the macros below
template <class Base>
class DerivativeType
{
};

// Allows that BASE / Time = DERIVATIVE and DERIVATIVE * TIME = BASE
#define KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(DERIVATIVE, BASE) \
  inline DERIVATIVE operator/(const BASE &a, const ros::Duration &dt) \
{ \
  return  DERIVATIVE(a.operator/(dt.toSec())); \
} \
inline BASE operator*(const DERIVATIVE &a,  const ros::Duration &dt) \
{ \
  return  BASE(a.operator*(dt.toSec())); \
} \
  inline DERIVATIVE operator/(const BASE &a, const SecondsD &dt) \
{ \
  return  DERIVATIVE(a.operator/(dt.impl_)); \
} \
inline BASE operator*(const DERIVATIVE &a,  const SecondsD &dt) \
{ \
  return  BASE(a.operator*(dt.impl_)); \
} \
template<> \
class DerivativeType<BASE>\
{ \
public: \
typedef DERIVATIVE type; \
};


//https://stackoverflow.com/questions/44268316/passing-a-template-type-into-a-macro
#define COMMA ,
#define KINDR_SPECIALIZE_DIMENSION(PrimType_, Dimension_) \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::Velocity<PrimType_ COMMA Dimension_>, kindr::Position<PrimType_ COMMA Dimension_>) \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::Acceleration<PrimType_ COMMA Dimension_>, kindr::Velocity<PrimType_ COMMA Dimension_>) \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::Jerk<PrimType_ COMMA Dimension_>, kindr::Acceleration<PrimType_ COMMA Dimension_>) \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::Force<PrimType_ COMMA Dimension_>, kindr::Momentum<PrimType_ COMMA Dimension_>) \
 \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::AngularJerk<PrimType_ COMMA Dimension_>, kindr::AngularAcceleration<PrimType_ COMMA Dimension_>) \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::AngularAcceleration<PrimType_ COMMA Dimension_>, kindr::AngularVelocity<PrimType_ COMMA Dimension_>) \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::AngularVelocity<PrimType_ COMMA Dimension_>, kindr::Angle<PrimType_ COMMA Dimension_>) \
KINDR_PAL_SPECIALIZE_PHYS_QUANT_TIME_DERIVATIVE(kindr::Torque<PrimType_ COMMA Dimension_>, kindr::AngularMomentum<PrimType_ COMMA Dimension_>)

KINDR_SPECIALIZE_DIMENSION(double, 1)
KINDR_SPECIALIZE_DIMENSION(double, 2)
KINDR_SPECIALIZE_DIMENSION(double, 3)
KINDR_SPECIALIZE_DIMENSION(double, Eigen::Dynamic)

#undef COMMA
} /* namespace kindr */


#endif // KINDR_EXTRAS_H
