/*
 * Copyright (c) 2016, TNO IVS Helmond.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/robot_localization_estimator.h"
#include "robot_localization/ekf.h"
#include "robot_localization/ukf.h"

namespace RobotLocalization
{
RobotLocalizationEstimator::RobotLocalizationEstimator(unsigned int buffer_capacity,
                                                       FilterType filter_type,
                                                       const std::vector<double>& filter_args)
{
  state_buffer_.set_capacity(buffer_capacity);
  if ( filter_type == FilterTypes::EKF )
  {
    filter_ = new Ekf;
  }
  else if ( filter_type == FilterTypes::UKF )
  {
    filter_ = new Ukf(filter_args);
  }

  filter_->setProcessNoiseCovariance(Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE));
}

RobotLocalizationEstimator::~RobotLocalizationEstimator()
{
  delete filter_;
}

void RobotLocalizationEstimator::setState(const EstimatorState& state)
{
  // If newly received state is newer than any in the buffer, push back
  if ( state_buffer_.empty() || state.time_stamp > state_buffer_.back().time_stamp )
  {
    state_buffer_.push_back(state);
  }
  // If it is older, put it in the right position
  else
  {
    for ( boost::circular_buffer<EstimatorState>::iterator it = state_buffer_.begin(); it != state_buffer_.end(); ++it )
    {
      if ( state.time_stamp < it->time_stamp )
      {
        state_buffer_.insert(it, state);
        return;
      }
    }
  }
}

EstimatorResult RobotLocalizationEstimator::getState(const double time,
                                                     EstimatorState& state) const
{
  // If there's nothing in the buffer, there's nothing to give.
  if ( state_buffer_.size() == 0 )
  {
    return EstimatorResults::EmptyBuffer;
  }

  // Set state to the most recent one for now
  state = state_buffer_.back();

  // Go through buffer from new to old
  EstimatorState last_state_before_time = state_buffer_.front();
  EstimatorState next_state_after_time = state_buffer_.back();
  bool previous_state_found = false;
  bool next_state_found = false;

  for (boost::circular_buffer<EstimatorState>::const_reverse_iterator it = state_buffer_.rbegin();
       it != state_buffer_.rend(); ++it)
  {
    /* If the time stamp of the current state from the buffer is
       * older than the requested time, store it as the last state
       * before the requested time. If it is younger, save it as the
       * next one after, and go on to find the last one before.
       */
    if ( it->time_stamp <= time )
    {
      last_state_before_time = *it;
      previous_state_found = true;
      break;
    }
    else
    {
      next_state_after_time = *it;
      next_state_found = true;
    }
  }

  // If we found a previous state and a next state, we can do interpolation
  if ( previous_state_found && next_state_found )
  {
    interpolate(last_state_before_time, next_state_after_time, time, state);
    return EstimatorResults::Interpolation;
  }

  // If only a previous state is found, we can do extrapolation into the future
  else if ( previous_state_found )
  {
    extrapolate(last_state_before_time, time, state);
    return EstimatorResults::ExtrapolationIntoFuture;
  }

  // If only a next state is found, we'll have to extrapolate into the past.
  else if ( next_state_found )
  {
    extrapolate(next_state_after_time, time, state);
    return EstimatorResults::ExtrapolationIntoPast;
  }

  else
  {
    return EstimatorResults::Failed;
  }
}

void RobotLocalizationEstimator::setBufferCapacity(const int capacity)
{
  state_buffer_.set_capacity(capacity);
}

void RobotLocalizationEstimator::clearBuffer()
{
  state_buffer_.clear();
}

unsigned int RobotLocalizationEstimator::getBufferCapacity() const
{
  return state_buffer_.capacity();
}

unsigned int RobotLocalizationEstimator::getSize() const
{
  return state_buffer_.size();
}

void RobotLocalizationEstimator::extrapolate(const EstimatorState& boundary_state,
                                             const double requested_time,
                                             EstimatorState& state_at_req_time) const
{
  // Set up the filter with the boundary state
  filter_->setState(boundary_state.state);
  filter_->setEstimateErrorCovariance(boundary_state.covariance);

  // Calculate how much time we need to extrapolate into the future (negative if into the past)
  double delta = requested_time - boundary_state.time_stamp;

  // Use the filter to predict
  filter_->predict(boundary_state.time_stamp, delta);

  // Get the predicted state from the filter
  state_at_req_time.time_stamp = requested_time;
  state_at_req_time.state = filter_->getPredictedState();
  state_at_req_time.covariance = filter_->getEstimateErrorCovariance();

  return;
}

void RobotLocalizationEstimator::interpolate(const EstimatorState& given_state_1,
                                             const EstimatorState& given_state_2,
                                             const double requested_time,
                                             EstimatorState& state_at_req_time) const
{
  /*
   * We are going to estimate the state at t_3, between two given states, at t_1 and t_2. This can be done by
   * interpolation. As the positions, velocities and accelerations in x, y and z are fixed on both sides, we have 6
   * constraints to the spatial trajectory. To be able to meet these constraints, we must adopt a 6th order polynomial
   * trajectory. So we not only need position, velocity and acceleration terms, but also jerk, snap, and crackle, as
   * these higher time derivatives of position are called. By equating the predicted position, velocity and acceleration
   * from t_1 to t_2 polynomial model, with the actual position, velocity and acceleration at t_2, we can calculate the
   * coefficients of the polynomial. Polynomial model can then be used to estimate the state variables at t_3.
   *
   * In the rotational DOFs, we only have a given position and velocity, so we can model this motion with 4th order
   * polynomial. The rest of the calculation is similar to the position interpolation, we only have to cope with
   * wrapping angles.
   */

  double dt = given_state_2.time_stamp - given_state_1.time_stamp;
  double dt_sq = dt*dt;
  double dt_cubed = dt_sq*dt;
  double dt_quart = dt_cubed*dt;
  double dt_quint = dt_quart*dt;

  const double one_sixth = 1/6.;
  const double one_twentyfourth = 1/24.;

  Eigen::Vector3d x_1;
  x_1 << given_state_1.state(StateMemberX),
         given_state_1.state(StateMemberY),
         given_state_1.state(StateMemberZ);

  Eigen::Vector3d x_2;
  x_2 << given_state_2.state(StateMemberX),
         given_state_2.state(StateMemberY),
         given_state_2.state(StateMemberZ);

  Eigen::Vector3d v_1;
  v_1 << given_state_1.state(StateMemberVx),
         given_state_1.state(StateMemberVy),
         given_state_1.state(StateMemberVz);

  Eigen::Vector3d a_1;
  a_1 << given_state_1.state(StateMemberAx),
         given_state_1.state(StateMemberAy),
         given_state_1.state(StateMemberAz);

  // x(t) = F x(t_1) + G j. Where j is a modeled disturbance vector
  Eigen::Matrix<double, 9, 9> G;  // The matrix mapping the disturbance to the state variables
  Eigen::Matrix<double, 9, 1> j;  // The disturbance vector containing the estimated jerk, snap and crackle

  // Fill G
  Eigen::Matrix3d g_11 = Eigen::Matrix3d::Identity() * one_sixth*dt_cubed;
  Eigen::Matrix3d g_22 = g_11;
  Eigen::Matrix3d g_33 = g_11;

  Eigen::Matrix3d g_12 = Eigen::Matrix3d::Identity() * one_twentyfourth*dt_quart;
  Eigen::Matrix3d g_23 = g_12;

  Eigen::Matrix3d g_13 = Eigen::Matrix3d::Identity() * 1/120.*dt_quint;

  Eigen::Matrix3d g_21 = Eigen::Matrix3d::Identity() * 0.5*dt_sq;
  Eigen::Matrix3d g_32 = g_21;

  Eigen::Matrix3d g_31 = Eigen::Matrix3d::Identity() * dt;

  // Assign blocks in G


  /*
   * TODO: Right now, we only extrapolate from the last known state before the requested time. But as the state after
   * the requested time is also known, we may want to perform interpolation between states.
   */
  extrapolate(given_state_1, requested_time, state_at_req_time);
  return;
}

}  // namespace RobotLocalization
