/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @brief Triggered parachute for a droppable payload (SITL only)
 *
 * The canopy is a real link, jointed to the payload from the moment the model
 * loads, so nothing has to be spawned or jointed at runtime - gz-sim cannot
 * create a joint between two independently spawned models from outside a
 * system, and DetachableJoint's <attach_topic> only re-attaches a joint whose
 * ends both existed at load time. What is gated here is the AERODYNAMICS: the
 * canopy rides along as dead mass producing no force until the trigger fires.
 *
 * Sequence:
 *   release (uORB gripper -> gz_bridge -> .../detachable_joint/detach)
 *     -> free fall for <deploy_delay> seconds of SIM time
 *     -> canopy inflates over <open_time>
 *     -> steady descent
 *
 * Drag is a plain quadratic  F = -0.5 * rho * cd * A * |v| * v  applied at
 * <cp> in the canopy link frame. It is deliberately NOT the stock
 * gz-sim-lift-drag-system: that models a wing as cd = cda * alpha and has a
 * singularity at exactly alpha = 90 deg, which is precisely where a canopy in
 * vertical descent sits - it produces literally zero force there, and the
 * canopy aerodynamically parks itself on that point. See
 * .worklog/payload-parachute.md for the measured evidence.
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace custom
{

class ParachuteSystem :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &info,
		       gz::sim::EntityComponentManager &ecm) final;

private:
	enum class State {
		Stowed,		///< carried; canopy inert
		Armed,		///< released, counting down to deploy
		Deploying,	///< canopy inflating
		Deployed	///< full area
	};

	void onRelease(const gz::msgs::Empty &);
	void onDeploy(const gz::msgs::Empty &);
	void applyDrag(gz::sim::EntityComponentManager &ecm, double area);

	/// Ask the world to create the canopy visual. Async: the entity shows up in
	/// the ECM some updates later, which trackCanopyVisual() waits for.
	void spawnCanopyVisual(const gz::sim::EntityComponentManager &ecm);

	/// Keep the spawned visual on the canopy link. The visual model is static,
	/// so nothing else writes its pose and we can own it outright.
	void trackCanopyVisual(gz::sim::EntityComponentManager &ecm);

	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _canopy_entity{gz::sim::kNullEntity};
	gz::sim::Link _canopy_link{gz::sim::kNullEntity};

	// Aerodynamics.
	double _cd{1.75};
	double _area{0.12};
	double _rho{1.2041};
	gz::math::Vector3d _cp{0.0, 0.0, 0.0};

	// Trigger.
	double _deploy_delay{0.5};
	double _open_time{0.2};

	State _state{State::Stowed};
	double _release_time{0.0};
	double _deploy_time{0.0};

	// Cosmetic. The canopy mesh is NOT part of the payload model - it is spawned
	// on deploy and pose-driven from here. Runtime Transparency / VisualCmd
	// changes do not reach the gz-sim 8 renderer, so a visual hidden inside the
	// payload stays visible on the drone for the whole flight.
	std::string _canopy_model{"parachute_canopy"};
	std::string _visual_name;
	std::string _world_name;
	gz::sim::Entity _visual_entity{gz::sim::kNullEntity};
	bool _spawn_requested{false};

	// Written from gz-transport callback threads, read in PreUpdate.
	std::atomic<bool> _release_received{false};
	std::atomic<bool> _deploy_received{false};

	gz::transport::Node _node;
	bool _valid{false};
};

} // namespace custom
