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

#include "ParachuteSystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include <gz/msgs/Utility.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	ParachuteSystem,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate
)

void ParachuteSystem::Configure(const gz::sim::Entity &entity,
				const std::shared_ptr<const sdf::Element> &sdf,
				gz::sim::EntityComponentManager &ecm,
				gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);

	if (!_model.Valid(ecm)) {
		gzerr << "[ParachuteSystem] must be attached to a model entity" << std::endl;
		return;
	}

	const std::string canopy_link = sdf->Get<std::string>("canopy_link", "canopy").first;
	_canopy_entity = _model.LinkByName(ecm, canopy_link);

	if (_canopy_entity == gz::sim::kNullEntity) {
		gzerr << "[ParachuteSystem] canopy_link \"" << canopy_link
		      << "\" not found in model \"" << _model.Name(ecm)
		      << "\" - the parachute will not generate any force" << std::endl;
		return;
	}

	_canopy_link = gz::sim::Link(_canopy_entity);
	// Without this the link reports no velocity and the drag term is never computed.
	_canopy_link.EnableVelocityChecks(ecm, true);

	_cd            = sdf->Get<double>("cd", _cd).first;
	_area          = sdf->Get<double>("area", _area).first;
	_rho           = sdf->Get<double>("air_density", _rho).first;
	_cp            = sdf->Get<gz::math::Vector3d>("cp", _cp).first;
	_deploy_delay  = sdf->Get<double>("deploy_delay", _deploy_delay).first;
	_open_time     = sdf->Get<double>("open_time", _open_time).first;
	// Empty disables the visual entirely (drag still works - useful headless).
	_canopy_model = sdf->Get<std::string>("canopy_model", _canopy_model).first;

	// open_time divides the inflation ramp; a zero would be a division by zero.
	_open_time = std::max(_open_time, 1e-3);

	// The payload is normally a nested model inside the carrier, and the release
	// signal is published against the CARRIER's name (gz_bridge builds it from
	// the spawned model instance, which carries the _N suffix). Derive it the
	// same way rather than hardcoding, so multi-vehicle SITL doesn't cross-trigger.
	const gz::sim::Entity top = gz::sim::topLevelModel(entity, ecm);
	std::string top_name = _model.Name(ecm);

	if (top != gz::sim::kNullEntity) {
		const auto *name_comp = ecm.Component<gz::sim::components::Name>(top);

		if (name_comp != nullptr) {
			top_name = name_comp->Data();
		}
	}

	// Unique per drop so several vehicles (or several runs in one world) do not
	// collide on the visual's name.
	_visual_name = top_name + "_canopy";

	const gz::sim::Entity world_entity = gz::sim::worldEntity(ecm);

	if (world_entity != gz::sim::kNullEntity) {
		_world_name = gz::sim::World(world_entity).Name(ecm).value_or("");
	}

	if (_canopy_model.empty() || _world_name.empty()) {
		gzwarn << "[ParachuteSystem] no canopy visual will be spawned"
		       << (_world_name.empty() ? " (world name unavailable)" : " (canopy_model empty)")
		       << "; drag is unaffected" << std::endl;
	}

	const std::string release_topic =
		sdf->Get<std::string>("release_topic",
				      "/model/" + top_name + "/detachable_joint/detach").first;
	const std::string deploy_topic =
		sdf->Get<std::string>("deploy_topic",
				      "/model/" + top_name + "/parachute/deploy").first;

	// Release only ARMS the timer. Deploy is the manual override that skips it -
	// useful for testing the canopy without flying a drop.
	if (!_node.Subscribe(release_topic, &ParachuteSystem::onRelease, this)) {
		gzwarn << "[ParachuteSystem] could not subscribe to release topic "
		       << release_topic << std::endl;
	}

	if (!_node.Subscribe(deploy_topic, &ParachuteSystem::onDeploy, this)) {
		gzwarn << "[ParachuteSystem] could not subscribe to deploy topic "
		       << deploy_topic << std::endl;
	}

	_valid = true;

	gzmsg << "[ParachuteSystem] armed on " << _model.Name(ecm)
	      << ": canopy \"" << canopy_link << "\", cd " << _cd
	      << ", area " << _area << " m^2, deploy " << _deploy_delay
	      << " s after release, inflating over " << _open_time << " s" << std::endl;
	gzmsg << "[ParachuteSystem]   release topic: " << release_topic << std::endl;
	gzmsg << "[ParachuteSystem]   deploy topic:  " << deploy_topic << std::endl;
}

void ParachuteSystem::onRelease(const gz::msgs::Empty &)
{
	_release_received = true;
}

void ParachuteSystem::onDeploy(const gz::msgs::Empty &)
{
	_deploy_received = true;
}

void ParachuteSystem::PreUpdate(const gz::sim::UpdateInfo &info,
				gz::sim::EntityComponentManager &ecm)
{
	if (!_valid || info.paused) {
		return;
	}

	const double t = std::chrono::duration<double>(info.simTime).count();

	switch (_state) {
	case State::Stowed:
		if (_deploy_received) {
			_state = State::Deploying;
			_deploy_time = t;
			spawnCanopyVisual(ecm);
			gzmsg << "[ParachuteSystem] deploy commanded at t=" << t << " s" << std::endl;

		} else if (_release_received) {
			_state = State::Armed;
			_release_time = t;
			gzmsg << "[ParachuteSystem] released at t=" << t << " s, free fall for "
			      << _deploy_delay << " s" << std::endl;
		}

		break;

	case State::Armed:
		if (_deploy_received || (t - _release_time) >= _deploy_delay) {
			_state = State::Deploying;
			_deploy_time = t;
			spawnCanopyVisual(ecm);
			gzmsg << "[ParachuteSystem] canopy deploying at t=" << t << " s" << std::endl;
		}

		break;

	case State::Deploying:
		if ((t - _deploy_time) >= _open_time) {
			_state = State::Deployed;
			gzmsg << "[ParachuteSystem] canopy fully inflated at t=" << t << " s" << std::endl;
		}

		break;

	case State::Deployed:
		break;
	}

	if (_state != State::Deploying && _state != State::Deployed) {
		return;
	}

	// Inflation ramp. Squared because the frontal area grows with the square of
	// the canopy radius, which keeps the opening shock from being a step input.
	double frac = 1.0;

	if (_state == State::Deploying) {
		frac = std::clamp((t - _deploy_time) / _open_time, 0.0, 1.0);
	}

	applyDrag(ecm, _area * frac * frac);
	trackCanopyVisual(ecm);
}

void ParachuteSystem::applyDrag(gz::sim::EntityComponentManager &ecm, double area)
{
	const auto vel = _canopy_link.WorldLinearVelocity(ecm);
	const auto omega = _canopy_link.WorldAngularVelocity(ecm);
	const auto pose = _canopy_link.WorldPose(ecm);

	if (!vel.has_value() || !omega.has_value() || !pose.has_value()) {
		return;
	}

	// <cp> is the offset from the link's CENTRE OF MASS to the canopy's
	// aerodynamic centre.
	const gz::math::Vector3d arm = pose->Rot().RotateVector(_cp);

	// Airspeed AT the canopy, not at the centre of mass. The omega x arm term is
	// what damps the riser pendulum: as the canopy swings, its own motion through
	// the air opposes the swing, exactly as a real canopy does. Without it the
	// pendulum is undamped, and on a payload with realistic (i.e. small)
	// rotational inertia it winds up until the solver diverges.
	const gz::math::Vector3d vel_cp = vel.value() + omega->Cross(arm);
	const double speed = vel_cp.Length();

	if (speed < 0.01) {
		return;
	}

	const gz::math::Vector3d force = -0.5 * _rho * _cd * area * speed * vel_cp;

	// AddWorldWrench applies the force at the centre of mass, so the offset has to
	// be converted into an explicit torque. This is the ONLY lever - the canopy
	// link's inertial pose must therefore sit at the link origin, or the offset
	// gets counted twice and the restoring torque doubles.
	_canopy_link.AddWorldWrench(ecm, force, arm.Cross(force));
}


void ParachuteSystem::spawnCanopyVisual(const gz::sim::EntityComponentManager &ecm)
{
	if (_spawn_requested || _canopy_model.empty() || _world_name.empty()) {
		return;
	}

	_spawn_requested = true;

	// Spawn it already on the canopy link, so it does not appear at the origin
	// for the one or two updates before trackCanopyVisual() takes over.
	const auto pose = _canopy_link.WorldPose(ecm);

	gz::msgs::EntityFactory req;
	req.set_sdf_filename("model://" + _canopy_model);
	req.set_name(_visual_name);
	req.set_allow_renaming(false);

	if (pose.has_value()) {
		gz::msgs::Set(req.mutable_pose(), pose.value());
	}

	// Fire and forget: this runs on the sim thread, so it must not block. The
	// entity turns up in the ECM a few updates later.
	// Spelled out as a std::function because Node::Request cannot deduce ReplyT
	// from a bare lambda.
	std::function<void(const gz::msgs::Boolean &, const bool)> on_reply =
	[this](const gz::msgs::Boolean & rep, const bool result) {
		if (!result || !rep.data()) {
			gzwarn << "[ParachuteSystem] could not spawn canopy visual \""
			       << _visual_name << "\"" << std::endl;
		}
	};

	const std::string service = "/world/" + _world_name + "/create";
	_node.Request(service, req, on_reply);
}

void ParachuteSystem::trackCanopyVisual(gz::sim::EntityComponentManager &ecm)
{
	if (!_spawn_requested || _canopy_model.empty()) {
		return;
	}

	if (_visual_entity == gz::sim::kNullEntity) {
		// The create request is async, so keep looking until it lands.
		_visual_entity = ecm.EntityByComponents(gz::sim::components::Name(_visual_name),
							gz::sim::components::Model());

		if (_visual_entity == gz::sim::kNullEntity) {
			return;
		}

		gzmsg << "[ParachuteSystem] canopy visual \"" << _visual_name << "\" attached"
		      << std::endl;
	}

	const auto pose = _canopy_link.WorldPose(ecm);

	if (!pose.has_value()) {
		return;
	}

	// SetWorldPoseCmd, NOT a direct write to components::Pose. The canopy model is
	// deliberately non-static (see parachute_canopy/model.sdf): gz-sim leaves
	// static models out of dynamic_pose/info, which is the per-iteration stream
	// the GUI follows, so a static canopy renders frozen in mid-air no matter how
	// correctly its Pose component tracks. Going through the physics teleport
	// keeps it in that stream.
	gz::sim::Model(_visual_entity).SetWorldPoseCmd(ecm, pose.value());
}
