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
#include "kindr_ros/KindrExtras.hpp"
#include "kindr/common/gtest_eigen.hpp"

/**
 * @brief Test and example about how to convert between types derivatives multiypling/dividing by time
 */
template<int Dimensions>
void TimeDerivativeTestImpl()
{
  // Both Duration types here can be interchanged
  ros::Duration ros_dt(2.0);
  kindr::SecondsD kindr_dt(2.0);

  // Linear Distance Types
  {
    kindr::Position<double, Dimensions> p;
    p.setRandom();
    kindr::Velocity<double, Dimensions> v = p/ros_dt;
    kindr::Acceleration<double, Dimensions> a = v/ros_dt;
    kindr::Acceleration<double, Dimensions> a1 = p/kindr_dt/ros_dt;
    kindr::Jerk<double, Dimensions> j = a/ros_dt;
    kindr::Jerk<double, Dimensions> j1 = v/ros_dt/ros_dt;
    kindr::Jerk<double, Dimensions> j2 = p/ros_dt/ros_dt/ros_dt;

    kindr::Position<double, Dimensions> p1 = v*kindr_dt;
    kindr::Position<double, Dimensions> p2 = a*ros_dt*kindr_dt;
    kindr::Position<double, Dimensions> p3 = j*kindr_dt*ros_dt*kindr_dt;

    kindr::Velocity<double, Dimensions> v1 = a*ros_dt;
    kindr::Velocity<double, Dimensions> v2 = j*kindr_dt*ros_dt;

    kindr::Acceleration<double, Dimensions> a2 = j*kindr_dt;

    kindr::Acceleration<double, Dimensions> a3 = a2 * 1.0; //Using just a scalar won't change the type
    kindr::Jerk<double, Dimensions> j3 = j2 / 1.0; //Using just a scalar won't change the type


    KINDR_ASSERT_DOUBLE_MX_EQ(p.toImplementation(), p1.toImplementation(), 1e-6, "")
    KINDR_ASSERT_DOUBLE_MX_EQ(p1.toImplementation(), p2.toImplementation(), 1e-6, "")
    KINDR_ASSERT_DOUBLE_MX_EQ(p2.toImplementation(), p3.toImplementation(), 1e-6, "")

    KINDR_ASSERT_DOUBLE_MX_EQ(v.toImplementation(), v1.toImplementation(), 1e-6, "")
    KINDR_ASSERT_DOUBLE_MX_EQ(v1.toImplementation(), v2.toImplementation(), 1e-6, "")


    KINDR_ASSERT_DOUBLE_MX_EQ(a.toImplementation(), a1.toImplementation(), 1e-6, "")
    KINDR_ASSERT_DOUBLE_MX_EQ(a1.toImplementation(), a2.toImplementation(), 1e-6, "")
    KINDR_ASSERT_DOUBLE_MX_EQ(a2.toImplementation(), a3.toImplementation(), 1e-6, "")


    KINDR_ASSERT_DOUBLE_MX_EQ(j.toImplementation(), j1.toImplementation(), 1e-6, "")
    KINDR_ASSERT_DOUBLE_MX_EQ(j1.toImplementation(), j2.toImplementation(), 1e-6, "")
    KINDR_ASSERT_DOUBLE_MX_EQ(j2.toImplementation(), j3.toImplementation(), 1e-6, "")
  }
  // Same for Angular Distance Types
  {
    kindr::Angle<double, Dimensions> p;
    kindr::AngularVelocity<double, Dimensions> v = p/ros_dt;
    kindr::AngularAcceleration<double, Dimensions> a = v/ros_dt;
    kindr::AngularJerk<double, Dimensions> j = a/ros_dt;
  }

  {
    kindr::Momentum<double, Dimensions> m;
    kindr::Force<double, Dimensions> f = m/ros_dt;
  }

  {
    kindr::AngularMomentum<double, Dimensions> am;
    kindr::Torque<double, Dimensions> t = am/ros_dt;
  }
}

TEST(KindrExtrasTest, TimeDerivativeTest)
{
  TimeDerivativeTestImpl<1>();
  TimeDerivativeTestImpl<2>();
  TimeDerivativeTestImpl<3>();
  TimeDerivativeTestImpl<Eigen::Dynamic>();
}

TEST(KindrExtrasTest, RotationsTest)
{
  // Define the rotation in the notation that best suits you
  kindr::RotationQuaternionD rquat(0.070592885899994171, 0.70357419257695242, -0.070592885899994171, 0.70357419257695);

  // And easiliy convert it to other derivates of kindr::RotationBase
  kindr::EulerAnglesRpyD rpy(rquat);
  kindr::EulerAnglesYprD ypr(rpy);
  kindr::RotationVectorD rvect(ypr);
  kindr::RotationMatrixD rmat(rvect);

  kindr::RotationQuaternionD rquat2(rmat);
  KINDR_ASSERT_DOUBLE_MX_EQ(rquat.toImplementation().coeffs(), rquat2.toImplementation().coeffs(), 1e-6, "")
}

TEST(KindrExtrasTest, TransformsTest)
{
  kindr::Position3D mat_transl(0.4, 3.2, -2.3);
  const kindr::RotationQuaternionD mat_rot(kindr::EulerAnglesRpyD(M_PI, 0., 0.));
  kindr::MatrixHom mat(mat_transl,
                       mat_rot);
  kindr::Position3D p(1.0, 1.0, 1.0);
  kindr::EulerAnglesRpyD r(.0, .0, M_PI);

  // Transform a position
  const kindr::Position3D p2 = mat.transform(p);
  const kindr::Position3D rotated_p(1., -1., -1.);
  KINDR_ASSERT_DOUBLE_MX_EQ((mat_transl + rotated_p).toImplementation(), p2.toImplementation(), 1e-6, "")

  kindr::MatrixHom mat2(p,
                        kindr::RotationQuaternionD(r));

  // Concatenate matrices
  kindr::MatrixHom mat3 = mat * mat2;
  KINDR_ASSERT_DOUBLE_MX_EQ((mat_transl + rotated_p).toImplementation(),
                            mat3.getPosition().toImplementation(), 1e-6, "")

  KINDR_ASSERT_DOUBLE_MX_EQ((mat_rot * mat2.getRotation()).toImplementation().coeffs(),
                            mat3.getRotation().toImplementation().coeffs(), 1e-6, "")
}

