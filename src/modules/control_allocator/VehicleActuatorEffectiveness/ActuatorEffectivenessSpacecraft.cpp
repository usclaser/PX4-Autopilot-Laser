/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ActuatorEffectivenessSpacecraft.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

ActuatorEffectivenessSpacecraft::ActuatorEffectivenessSpacecraft(ModuleParams *parent)
	: ModuleParams(parent),
	  _sc_thrusters(this)
{
}

bool
ActuatorEffectivenessSpacecraft::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Thrusters
	const bool thrusters_added_successfully = _sc_thrusters.addActuators(configuration);

	return thrusters_added_successfully;
}

void ActuatorEffectivenessSpacecraft::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	(void)actuator_min;
	(void)actuator_max;

	// Spacecraft thrusters are often solenoid-driven. For these actuators, "no commanded wrench"
	// should produce a true "valves closed" output even while armed. PX4's output mapping treats
	// finite actuator_motors values (including 0) as an active command, which maps to the midpoint
	// of PWM min..max. However, NaN actuator setpoints propagate through ControlAllocator and are
	// converted to NaN in actuator_motors, which the mixer/output layer interprets as "use disarmed
	// value" for that channel.
	//
	// Therefore: when the requested torque+thrust setpoint is essentially zero, force all spacecraft
	// thrusters to NaN so MAIN outputs remain at their disarmed values instead of ramping to a center
	// PWM on arm.
	if (matrix_index != 0) {
		return;
	}

	const float torque_norm = math::max(math::max(fabsf(control_sp(0)), fabsf(control_sp(1))), fabsf(control_sp(2)));
	const float thrust_norm = math::max(math::max(fabsf(control_sp(3)), fabsf(control_sp(4))), fabsf(control_sp(5)));

	// Deadband: cover numerical noise from allocation and setpoint publishing.
	static constexpr float IDLE_WRENCH_EPS = 1e-4f;

	if (torque_norm <= IDLE_WRENCH_EPS && thrust_norm <= IDLE_WRENCH_EPS) {
		const int n = _sc_thrusters.geometry().num_rotors;

		const int m = math::min(n, static_cast<int>(actuator_sp.size()));

		for (int i = 0; i < m; i++) {
			actuator_sp(i) = NAN;
		}
	}
}
