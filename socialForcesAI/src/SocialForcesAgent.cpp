  
//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SocialForcesAgent.h"
#include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"
// #include <math.h>

// #include "util/Geometry.h"

/// @file SocialForcesAgent.cpp
/// @brief Implements the SocialForcesAgent class.

#undef min
#undef max

#define AGENT_MASS 1.0f

using namespace Util;
using namespace SocialForcesGlobals;
using namespace SteerLib;

// #define _DEBUG_ENTROPY 1

// gotta define static variables in C++ for some reason
Point SocialForcesAgent::evade = Point(0, 0, 0);
Point SocialForcesAgent::pursue = Point(0, 0, 0);
bool SocialForcesAgent::pursueIsLive = true;
float SocialForcesAgent::elapsed = 0;
Util::Vector up(0, 1, 0);

	// Select 
	// only 
	// one
	// |
	// |
	// |
	// |
	// |
	// |
	// V
bool pursueEvade = false;
bool expandingSpiral = false;
bool followLeader = false;


SocialForcesAgent::SocialForcesAgent()
{
	_SocialForcesParams.sf_acceleration = sf_acceleration;
	_SocialForcesParams.sf_personal_space_threshold = sf_personal_space_threshold;
	_SocialForcesParams.sf_agent_repulsion_importance = sf_agent_repulsion_importance;
	_SocialForcesParams.sf_query_radius = sf_query_radius;
	_SocialForcesParams.sf_body_force = sf_body_force;
	_SocialForcesParams.sf_agent_body_force = sf_agent_body_force;
	_SocialForcesParams.sf_sliding_friction_force = sf_sliding_friction_force;
	_SocialForcesParams.sf_agent_b = sf_agent_b;
	_SocialForcesParams.sf_agent_a = sf_agent_a;
	_SocialForcesParams.sf_wall_b = sf_wall_b;
	_SocialForcesParams.sf_wall_a = sf_wall_a;
	_SocialForcesParams.sf_max_speed = sf_max_speed;

	_enabled = false;
}

SocialForcesAgent::~SocialForcesAgent()
{
	// std::cout << this << " is being deleted" << std::endl;
	/*
	if (this->enabled())
	{
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		// getSimulationEngine()->getSpatialDatabase()->removeObject( this, bounds);
	}*/
	// std::cout << "Someone is removing an agent " << std::endl;
}

SteerLib::EngineInterface * SocialForcesAgent::getSimulationEngine()
{
	return _gEngine;
}

void SocialForcesAgent::setParameters(Behaviour behave)
{
	this->_SocialForcesParams.setParameters(behave);
}

void SocialForcesAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled == true);

	// if (true /* follow leader */ && id() == 0 && pursueIsLive) {
	// 	return;
	// }

	if (id() == 0)
		pursueIsLive = false;
	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;
}

void SocialForcesAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// std::cout << id() << ": reset" << std::endl;
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	// std::cout << "resetting agent " << this << std::endl;
	_waypoints.clear();
	_midTermPath.clear();

	// AgentGoalInfo fGoal;

	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);
	std::vector<AgentGoalInfo> goals;

	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
	*/
	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;
	// std::cout << "inital colour of agent " << initialConditions.color << std::endl;
	if ( initialConditions.colorSet == true )
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		getSimulationEngine()->getSpatialDatabase()->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		// std::cout << "new position is " << _position << std::endl;
		// std::cout << "new bounds are " << newBounds << std::endl;
		// std::cout << "reset update " << this << std::endl;
		getSimulationEngine()->getSpatialDatabase()->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}
	if ((followLeader /*follow the leader*/ && id() > 0) || (pursueEvade && id() > 1)) {
		AgentGoalInfo leaderPosition;
		leaderPosition.goalType = AgentGoalTypeEnum::GOAL_TYPE_SEEK_STATIC_TARGET;
		leaderPosition.targetLocation = pursue;
		leaderPosition.targetIsRandom = false;
		leaderPosition.timeDuration = 1000;
		leaderPosition.desiredSpeed = 1.3f;
		goals.push_back(leaderPosition);
	}
	else {
		// no special cases
		for (unsigned i = 0; i < initialConditions.goals.size(); i++) {
			goals.push_back(initialConditions.goals[i]);
		}
	}
	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i< goals.size(); i++) {
		if (goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
				goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			if (goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
				_currentGoal.targetLocation = _goal.targetLocation;
			}
			else
			{
				// std::cout << "goal:" << i << " position:" << goals[i].targetLocation << " random:" << goals[i].targetIsRandom << std::endl;
				_goalQueue.push(goals[i]);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}

	// runLongTermPlanning(_goalQueue.front().targetLocation, dont_plan);

	// pursue and evade stuff
	// if (id() == 0) {
	// 	pursue.x = position().x;
	// 	pursue.z = position().z;
	// }
	// else if (id() == 1) {
	// 	evade.x = position().x;
	// 	evade.z = position().z;
	// }
	// end of pursue and evade stuff

	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	Util::Vector goalDirection;
	if ( !_midTermPath.empty() )
	{
		this->updateLocalTarget();
		goalDirection = normalize( this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize( _goalQueue.front().targetLocation - position());
	}

	// fGoal = _goalQueue.front();
	// while (!_goalQueue.empty())
	// 	_goalQueue.pop();
	// // plane ingress
	// if (fGoal.targetLocation.z > 0) {
	// 	// left side
	// 	AgentGoalInfo door;
	// 	door.targetLocation = Util::Point(5.0f, 0, 37.5f);
	// 	door.goalType = AgentGoalTypeEnum::GOAL_TYPE_SEEK_STATIC_TARGET;
	// 	AgentGoalInfo middle;
	// 	middle.targetLocation = Util::Point(-2.88f, 0, 37.5f);
	// 	middle.goalType = AgentGoalTypeEnum::GOAL_TYPE_SEEK_STATIC_TARGET;
	// 	AgentGoalInfo topOfGoal;
	// 	topOfGoal.targetLocation = Util::Point(-2.88f, 0, fGoal.targetLocation.z - 0.265f);
	// 	_goalQueue.push(door);
	// 	_goalQueue.push(middle);
	// 	_goalQueue.push(topOfGoal);
	// 	_goalQueue.push(fGoal);
	// }
	// else {
	// 	// right side
	// 	AgentGoalInfo door;
	// 	door.targetLocation = Util::Point(5.0f, 0, -37.5f);
	// 	door.goalType = AgentGoalTypeEnum::GOAL_TYPE_SEEK_STATIC_TARGET;
	// 	AgentGoalInfo middle;
	// 	middle.targetLocation = Util::Point(-2.88f, 0, -37.5f);
	// 	middle.goalType = AgentGoalTypeEnum::GOAL_TYPE_SEEK_STATIC_TARGET;
	// 	AgentGoalInfo topOfGoal;
	// 	topOfGoal.targetLocation = Util::Point(-2.88f, 0, fGoal.targetLocation.z + 0.265f);
	// 	_goalQueue.push(door);
	// 	_goalQueue.push(middle);
	// 	_goalQueue.push(topOfGoal);
	// 	_goalQueue.push(fGoal);
	// }
	_prefVelocity =
			(
				(
					(
						Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
						PERFERED_SPEED
					)
					- velocity()
				)
				/
				_SocialForcesParams.sf_acceleration
			)
			*
			MASS;

	// _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif


	// std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
	// _gEngine->addAgent(this, rvoModule);
	assert(_forward.length() != 0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void SocialForcesAgent::calcNextStep(float dt) {
}

//min distance from the line formed by l1-l2 and the point p
std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
	  return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
  const float t = dot(p - l1, l2 - l1) / lSq;
  if (t < 0.0)
  {
	  return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
  }
  else if (t > 1.0)
  {
	  return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
  }
  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}


Util::Vector SocialForcesAgent::calcProximityForce(float dt)
{
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
			_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
			_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
			_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
			_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
			dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface * tmp_agent;
	SteerLib::ObstacleInterface * tmp_ob;
	Util::Vector away = Util::Vector(0,0,0);
	Util::Vector away_obs = Util::Vector(0,0,0);

	// float dst = position().vector().lengthSquared() + 40;
	// // if (dst < 40)
	// // 	dst = 40;
	// if (id() == 2 && position().x < 2.3f) {
	// 	away += Util::Vector(0, 0, 1)/dst;
	// }
	// if (id() == 1) {
	// 	float turnpoint = 1.32f;
	// 	if (position().x > turnpoint) {
	// 		away += Util::Vector(0, 0, -1)/4.2f/dst;
	// 		away += 0.006f * Util::Vector(1, 0, 0);
	// 	}
	// 	if (position().x > 0.7 && position().x <= turnpoint) {
	// 		away += Util::Vector(0, 0, 1)/25.5f;
	// 		away += 0.01f * Util::Vector(1, 0, 0);
	// 	}
	// 	else {
	// 		// if (elapsed > 36 && elapsed < 39){
	// 		// 	away += Util::Vector(0, 0, 1)/dst;
	// 		// }
	// 	}


	/* doorway two way */
	// float turnpoint = 1.5f;
	// if (id() == 0 && position().x < 0.3) {
	// 	away += Util::Vector(0, 0, -1)/410;
	// }
	// else if (id() == 1 && position().x > turnpoint) {
	// 	away += Util::Vector(0, 0, 1)/250;
	// 	away += -0.013 * forward();
	// }
	// else if (id() == 1 && position().x > 0.1f && position().x <= turnpoint) {
	// 	away += Util::Vector(0, 0, -1)/37;
	// 	away += -0.02 * forward();
	// }


	// }


	// if (id() < 100) {
	// 	away.z += 1.0f / 45.0f;
	// }
	// else if (id() < 200) {
	// 	away.z -= 1.0f / 45.0f;
	// }
	// away += -0.001f * forward();

	// float beforeSquare = 1.0f / 60.0f;
	// float inSquare = 1.0f / 30.0f;
	// float afterSquare = 1.0f / 30.0f;
	// if (id() < 100) {

	// 	if (position().x < -6 && position().x > -10		&& position().z < 9) {
	// 		away.z += (10 - position().z) * inSquare;
	// 	}
	// 	else if (position().x < -10){
	// 		away.z += beforeSquare;
	// 	}
	// 	else if (position().x > 6 && position().x < 11) {
	// 		away.z -= afterSquare;
	// 	}
	// }
	// else if (id() < 200) {
	// 	if (position().x > 6 && position().x < 10 		&& position().z > -9) {
	// 		away.z -= std::abs(-10 - position().z) * inSquare;
	// 	}
	// 	else if (position().x > 10) {
	// 		away.z -= beforeSquare;
	// 	}
	// 	else if (position().x < -6 && position().x > -11) {
	// 		away.z += afterSquare;
	// 	}
	// }
	// else if (id() < 300) {
	// 	if (position().z > 6 && position().z < 10 		&& position().x < 9) {
	// 		away.x += (10 - position().x) * inSquare;
	// 	}
	// 	else if (position().z > 10) {
	// 		away.x += beforeSquare;
	// 	}
	// 	else if (position().z < -6 && position().z > -11) {
	// 		away.x -= beforeSquare;
	// 	}
	// }
	// else if (id() < 400) {
	// 	if (position().z < -6 && position().z > -10 	&& position().x > -9) {
	// 		away.x -= std::abs(-10 - position().x) * inSquare;
	// 	}
	// 	else if (position().z < -10) {
	// 		away.x -= beforeSquare;
	// 	}
	// 	else if (position().z > 6 && position().z < 11) {
	// 		away.x += beforeSquare;
	// 	}
	// }

	if (position().z < -4.0f && id()) {
		away.z += 1.0f / 40.0f;
	}
	else if (position().z > 4.0f && id()) {
		away.z -= 1.0f / 40.0f;
	}

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ( (*neighbour)->isAgent() )
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);

			// direction away from other agent
			Util::Vector away_tmp = normalize(position() - tmp_agent->position());
			// std::cout << "away_agent_tmp vec" << away_tmp << std::endl;
			// Scale force
			// std::cout << "the exp of agent distance is " << exp((radius() + tmp_agent->radius()) -
				//	(position() - tmp_agent->position()).length()) << std::endl;

			if (tmp_agent->id() == 0 && tmp_agent->position().x > position().x + 1.0f) {
				away.x -= 1.0f / 90.0f;
				away.z -= 1.0f / 40.0f;
			}

			// away = away + (away_tmp * ( radius() / ((position() - tmp_agent->position()).length() * B) ));
			// if (elapsed > 400*dt && elapsed < 600*dt)
			// 	away = away + cross(up, forward())/50;

			/* double squeeze */
			// if ((normalize(velocity()) + normalize(tmp_agent->velocity())).lengthSquared() < 0.3f) {
			// 	// if the distance between this and tmp_agent is going to decrease in the next timestep
			// 	if ( (position() - tmp_agent->position()).lengthSquared() > (position() + velocity()*dt - (tmp_agent->position() + tmp_agent->velocity()*dt) ).lengthSquared() ) {
			// 		// agents are_heading towards each other and are getting closer
			// 		if (id() % 2 == 0) {
			// 			if (id() == 0){
			// 				away += Util::Vector(0, 0, 1)/168;
			// 				away += -0.006f * forward();
			// 			}
			// 			else{
			// 				away += Util::Vector(0, 0, 1)/168;
			// 				away += -0.004f * forward();
			// 			}
			// 		}
			// 		else {
			// 			if (id() == 1){
			// 				away += Util::Vector(0, 0, -1)/168;
			// 				away += -0.006f * forward();
			// 			}
			// 			else{
			// 				away += Util::Vector(0, 0, -1)/168;
			// 				away += -0.004f * forward();
			// 			}
			// 		}
			// 	}
			// }

			// if (id() == 0 && tmp_agent->id() == 2 && position().x < 2.3f) {
			// 	away += normalize(tmp_agent->position() - position())/100;
			// 	// if (elapsed < 20) {
			// 		away -= 0.001f * forward();
			// 		away += 0.6f * Util::Vector(0, 0, 1)/dst;
			// 	// }
			// }
			// if (id() == 1 && tmp_agent->id() == 3) {
			// 	away += normalize(tmp_agent->position() - position())/100;
			// 	// if (elapsed < 30) {
			// 		away -= 0.01f * forward();
			// 		away += Util::Vector(0, 0, 1)/dst;
			// 	// }
			// }
			// if ( (position() - tmp_agent->position()).length() < 1.5f ) {



			// away = away + (away_tmp * ( radius() / ((position() - tmp_agent->position()).length() * B) ));
			// if (elapsed > 400*dt && elapsed < 600*dt)
			// 	away = away + cross(up, forward())/50;
			// if ((forward() + tmp_agent->forward()).lengthSquared() < 0.2f) {
			// 	// if the distance between this and tmp_agent is going to decrease in the next timestep && if the z coordinates align
			// 	if ( (position() - tmp_agent->position()).lengthSquared() 
			// 		> (position() + velocity()*dt - (tmp_agent->position() + tmp_agent->velocity()*dt) ).lengthSquared()
			// 		&& 	(position().z - tmp_agent->position().z) < 0.6f) {
			// 		// agents are_heading towards each other and are getting closer
			// 		// check whether it's easier to dodge up or down, this might be less efficient than grouping
			// 		// if (position().z >= tmp_agent->position().z)
			// 		// 	away += Util::Vector(0, 0, 1) / 35;
			// 		// else
			// 		// 	away -= Util::Vector(0, 0, 1) / 35;
			// 		if ()
			// 		away -= 0.034f * forward();
			// 	}
			// }

			// hallway-two-way
			// if ((forward() + tmp_agent->forward()).lengthSquared() < 0.2f) {
			// 	if ((position() - tmp_agent->position()).lengthSquared() 
			// 		> (position() + forward()*dt - tmp_agent->position() - tmp_agent->forward()*dt).lengthSquared()) {
			// 		if (id() < 100) {
			// 			away.z += 1.0f / 70.0f;
			// 		}
			// 		else if (id() < 200) {
			// 			away.z -= 1.0f / 70.0f;
			// 		}
			// 	}
			// }

			// follow the leader
			// follow anyone that is in front with a following force
			// the following force is proportional to the z-distance (a.k.a how inline of me they are)
			// if ((forward() - tmp_agent->forward()).lengthSquared() < 0.2f 
			// 	&& (forward() - tmp_agent->forward()).lengthSquared() > 0.15f
			// 	&& position().x < tmp_agent->position().x + tmp_agent->velocity().x*3*dt) {
			// 	float cing = 1.2f*log(1 + std::abs(position().z - tmp_agent->position().z));
			// 	if (cing < 0)
			// 		cing = 0;
			// 	if (cing > 0.052f)
			// 		cing = 0.052f;
			// 	if (position().z <= tmp_agent->position().z)
			// 		cing *= -1;
			// 	away.z += cing;
			// }













				away = away +
						(
							away_tmp
							*
							(
								_SocialForcesParams.sf_agent_a
								*
								exp(
									(
										(
											(
												this->radius()
												+
												tmp_agent->radius()
											)
											-
											(
												this->position()
												-
												tmp_agent->position()
											).length()
										)
										/
										_SocialForcesParams.sf_agent_b
									)
								)


							)
							*
							dt
					);
			/*
			std::cout << "agent " << this->id() << " away this far " << away <<
					" distance " << exp(
							(
								(
									(
										radius()
										+
										tmp_agent->radius()
									)
									-
									(
										position()
										-
										tmp_agent->position()
									).length()
								)
								/
								_SocialForcesParams.sf_agent_b
							)
						) << std::endl;
						*/
		}
		else
		{
			// It is an obstacle
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			CircleObstacle * obs_cir = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if ( obs_cir != NULL && USE_CIRCLES)
			{
				// std::cout << "Found circle obstacle" << std::endl;
				Util::Vector away_tmp = normalize(position() - obs_cir->position());
				away = away +
						(
							away_tmp
							*
							(
									_SocialForcesParams.sf_wall_a
								*
								exp(
									(
										(
											(
												this->radius()
												+
												obs_cir->radius()
											)
											-
											(
												this->position()
												-
												obs_cir->position()
											).length()
										)
										/
										_SocialForcesParams.sf_wall_b
									)
								)


							)
							*
							dt
						);
			}
			else
			{
				Util::Vector wall_normal = calcWallNormal( tmp_ob );
				std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
					// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance

				Util::Vector away_obs_tmp = normalize(position() - min_stuff.second);
				// std::cout << "away_obs_tmp vec" << away_obs_tmp << std::endl;
				// away_obs = away_obs + ( away_obs_tmp * ( radius() / ((position() - min_stuff.second).length() * B ) ) );
				away_obs = away_obs +
						(
							away_obs_tmp
							*
							(
								_SocialForcesParams.sf_wall_a
								*
								exp(
									(
										(
											(this->radius()) -
											(
												this->position()
												-
												min_stuff.second
											).length()
										)
										/
										_SocialForcesParams.sf_wall_b
									)
								)
							)
							*
							dt
						);
			}
		}

	}
	return away + away_obs;
}

Util::Vector SocialForcesAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
	std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
			(_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
	return calcWallRepulsionForce(dt) + (_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt));
}

Util::Vector SocialForcesAgent::calcAgentRepulsionForce(float dt)
{
	// pursue evade stuff
	if (id() == 0) {
		pursue.x = position().x;
		pursue.z = position().z;
	}
	else if (id() == 1) {
		evade.x = position().x;
		evade.z = position().z;
	}
	// end of pursue evade stuff
	Util::Vector agent_repulsion_force = Util::Vector(0,0,0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
				(this));

	SteerLib::AgentInterface * tmp_agent;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ( (*neighbour)->isAgent() )
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
		}
		else
		{
			continue;
		}
		if (id() != tmp_agent->id()) {
			if (tmp_agent->computePenetration(this->position(), this->radius()) > 0.000001) {
				agent_repulsion_force = agent_repulsion_force +
					( tmp_agent->computePenetration(this->position(), this->radius()) * _SocialForcesParams.sf_agent_body_force * dt) *
					normalize(position() - tmp_agent->position());
				// normalized tangential force
				/*
					agent_repulsion_force = agent_repulsion_force +
						(
							(
								(-1*position()) - tmp_agent->position()
							)
							/
							(
								(-1*position()) - tmp_agent->position()
							).length()
						)*0.2;
						*/
				//TODO this can have some funny behaviour is velocity == 0
				Util::Vector tangent = cross(cross(tmp_agent->position() - position(), velocity()),
						tmp_agent->position() - position());
				tangent = tangent /  tangent.length();
				float  tanget_v_diff = dot(tmp_agent->velocity() - velocity(),  tangent);
				// std::cout << "Velocity diff is " << tanget_v_diff << " tangent is " << tangent <<
				// 	" velocity is " << velocity() << " position is " << position() << 
				// 	". Other:" << " velocity is " << tmp_agent->velocity() << " position is " << tmp_agent->position() <<  std::endl;
				agent_repulsion_force = agent_repulsion_force +
				(_SocialForcesParams.sf_sliding_friction_force * dt *
					(
						tmp_agent->computePenetration(this->position(), this->radius())
					) * tangent * tanget_v_diff

				);
				// printf("collision\n");
			}
			else {
				/* If a head on collision is about to happen, but hasn't happened, apply a force that pushes you to the left */

			}
		}

	}
	if (pursueEvade /* pursue evade */ && id() > 1) {
		// apply a pushing force in the direction of this agent from the evade agent
		Vector forca = position() - evade;
		float mag = 2/forca.lengthSquared();
		if (mag > 10)
			mag = 10;
		if (mag < 0.05)
			mag = 0;
		forca = normalize(forca) * mag;
		agent_repulsion_force = agent_repulsion_force + forca;
	}
	return agent_repulsion_force;
}

Util::Vector SocialForcesAgent::calcWallRepulsionForce(float dt)
{

	Util::Vector wall_repulsion_force = Util::Vector(0,0,0);


	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::ObstacleInterface * tmp_ob;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = _neighbors.begin();  tmp_o != _neighbors.end();  tmp_o++)
	{
		if ( !(*neighbour)->isAgent() )
		{
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
		}
		else
		{
			continue;
		}
		if ( tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001 )
		{
			CircleObstacle * cir_obs = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if ( cir_obs != NULL && USE_CIRCLES )
			{
				// std::cout << "Intersected circle obstacle" << std::endl;
				Util::Vector wall_normal = position() - cir_obs->position();
				// wall distance
				float distance = wall_normal.length() - cir_obs->radius();

				wall_normal = normalize(wall_normal);
				wall_repulsion_force = wall_repulsion_force +
					((
						(
							(
									wall_normal
							)
							*
							(
								radius() +
								_SocialForcesParams.sf_personal_space_threshold -
								(
									distance
								)
							)
						)
						/
						distance
					)* _SocialForcesParams.sf_body_force * dt);

				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
					// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
						// std::endl;
				wall_repulsion_force = wall_repulsion_force +
				(
					dot(forward(),  rightSideInXZPlane(wall_normal))
					*
					rightSideInXZPlane(wall_normal)
					*
					cir_obs->computePenetration(this->position(), this->radius())
				)* _SocialForcesParams.sf_sliding_friction_force * dt;

			}
			else
			{
				Util::Vector wall_normal = calcWallNormal( tmp_ob );
				std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
					// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance
				wall_repulsion_force = wall_repulsion_force +
					((
						(
							(
									wall_normal
							)
							*
							(
								radius() +
								_SocialForcesParams.sf_personal_space_threshold -
								(
									min_stuff.first
								)
							)
						)
						/
						min_stuff.first
					)* _SocialForcesParams.sf_body_force * dt);
				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
					// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
						// std::endl;
				wall_repulsion_force = wall_repulsion_force +
				(
					dot(forward(),  rightSideInXZPlane(wall_normal))
					*
					rightSideInXZPlane(wall_normal)
					*
					tmp_ob->computePenetration(this->position(), this->radius())
				)* _SocialForcesParams.sf_sliding_friction_force * dt;
			}
		}

	}
	// return wall_repulsion_force;
	return Util::Vector(0, 0, 0);
}

std::pair<Util::Point, Util::Point> SocialForcesAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
		// Ended here;
	}
	else if ( normal.z == -1 )
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
	}
	else if ( normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
	}
	else // normal.x == -1
	{ 
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
	}
}

/**
 * Basically What side of the obstacle is the agent on use that as the normal
 * DOES NOT SUPPORT non-axis-aligned boxes
 *
 *
 * 			   \		   /
 * 				\		  /
 * 				 \	 a	 /
 *				  \		/
 * 					 _
 * 			a		| |       a
 * 					 -
 * 				  /     \
 * 				 /   a   \
 * 				/	      \
 * 			   /	       \
 *
 *
 */
Util::Vector SocialForcesAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( position().x > box.xmax )
	{
		if ( position().z > box.zmax)
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if ( position().x < box.xmin )
	{
		if ( position().z > box.zmax )
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if ( position().z > box.zmax )
		{
			return Util::Vector(0, 0, 1);
		}
		else if ( position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal( obs );
		}
	}

}

/**
 * Treats Obstacles as a circle and calculates normal
 */
Util::Vector SocialForcesAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
			(box.zmax+box.zmin)/2);
	return normalize(position() - obs_centre);
}

/*
void SocialForcesAgent::computeNeighbors()
{
	agentNeighbors_.clear();
	if (_SocialForcesParams.rvo_max_neighbors > 0) {
		// std::cout << "About to segfault" << std::endl;
		dynamic_cast<SocialForcesAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, _SocialForcesParams.rvo_neighbor_distance * _SocialForcesParams.rvo_neighbor_distance);
		// std::cout << "Made it past segfault" << std::endl;
	}
}*/


void SocialForcesAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	elapsed += dt;
	// std::cout << id() << ": update" << std::endl;
	// std::cout << "_SocialForcesParams.rvo_max_speed " << _SocialForcesParams._SocialForcesParams.rvo_max_speed << std::endl;
	Util::AutomaticFunctionProfiler profileThisFunction( &SocialForcesGlobals::gPhaseProfilers->aiProfiler );
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	// The goalQueue.front() contains the current goal
	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	if ((pursueEvade /*pursue and evade*/ && id() > 1) || (followLeader /* follow the leader */ && id() > 0)) {
		goalInfo.targetLocation = pursue;
	}
	Util::Vector goalDirection;
	// std::cout << "midtermpath empty: " << _midTermPath.empty() << std::endl;
	if ( ! _midTermPath.empty() && (!this->hasLineOfSightTo(goalInfo.targetLocation)) )
	{
		if (reachedCurrentWaypoint())
		{
			this->updateMidTermPath();
		}

		this->updateLocalTarget();
		// if ((false /*pursue and evade*/ && id() > 1) || (true /* follow the leader */ && id() > 0)) {
		// 	_currentLocalTarget.x = pursue.x;
		// 	_currentLocalTarget.z = pursue.z;
		// }
		goalDirection = normalize(_currentLocalTarget - position());

	}
	else
	{
		goalDirection = normalize(goalInfo.targetLocation - position());
	}

	// preferred force 
	// _prefVelocity = goalDirection * PERFERED_SPEED;
	Util::Vector prefForce;
	if (expandingSpiral) { /*Expanding spiral */
		// centerSeeking = Vector(position.x, 0, position.z);
		// tangent = Vector(-position.z, 0, position.x);
		// prefForce = centerSeeking - tangent		
		// next two lines are this ^ in a shorter way
		prefForce.x = position().x/64 + position().z;
		prefForce.z = position().z/64 - position().x;
		prefForce = (((prefForce * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration/dt));
	}
	else
		prefForce = (((goalDirection * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration/dt)); //assumption here
	
	prefForce = prefForce + velocity();
	// _velocity = prefForce;

	// repulsion force
	Util::Vector repulsionForce = calcRepulsionForce(dt);
	if ( repulsionForce.x != repulsionForce.x)
	{
		std::cout << "Found some nan" << std::endl;
		repulsionForce = velocity();
		// throw GenericException("SocialForces numerical issue");
	}
	Util::Vector proximityForce = calcProximityForce(dt);
 // #define _DEBUG_ 1
#ifdef _DEBUG_
	std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
	std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
	std::cout << "agent" << id() << " pref force " << prefForce << std::endl;
#endif
	// _velocity = _newVelocity;
	int alpha=1;
	if ( repulsionForce.length() > 0.0)
	{
		alpha=0;
	}

	_velocity = (prefForce) + repulsionForce + proximityForce;
	// _velocity = (prefForce);
	// _velocity = velocity() + repulsionForce + proximityForce;

	_velocity = clamp(velocity(), _SocialForcesParams.sf_max_speed);
	_velocity.y=0.0f;
#ifdef _DEBUG_
	std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	 * Or when the agent is removed for example its true location will not reflect its location in the grid database.
	 * Not only that but this error will appear random depending on how well the agent lines up with the grid database
	 * boundaries when removed.
	 */
	// std::cout << "Updating agent" << this->id() << " at " << this->position() << std::endl;
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);

	/*
	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius()*WAYPOINT_THRESHOLD_MULTIPLIER)
	{
		_waypoints.erase(_waypoints.begin());
	}
	*/
	/*
	 * Now do the conversion from SocialForcesAgent into the SteerSuite coordinates
	 */
	// _velocity.y = 0.0f;

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
					Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
							goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() != 0)
		{
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else
		{
			if ((pursueEvade /*pursue and evade*/ && id() > 1 && pursueIsLive) || (followLeader /*follow the leader*/ && id() > 0 && pursueIsLive)) {
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = pursue;
				_goalQueue.push(_goal);
			}
			else {
				// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
				disable();
				return;
			}
		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if ( velocity().lengthSquared() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	// _position = _position + (_velocity * dt);
}


void SocialForcesAgent::draw() {
#ifdef ENABLE_GUI
	AgentInterface::draw();
	// if the agent is selected, do some annotations just for demonstration

#ifdef DRAW_COLLISIONS
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
			_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			Util::DrawLib::drawStar(
					this->position()
					+
					(
						(
							dynamic_cast<AgentInterface*>(*neighbor)->position()
							-
							this->position()
						)
					/2), Util::Vector(1,0,0), 0.8f, gRed);
			// Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}
#endif
#ifdef DRAW_HISTORIES
	__oldPositions.push_back(position());
	int points = 0;
	float mostPoints = 100.0f;
	while ( __oldPositions.size() > mostPoints )
	{
		__oldPositions.pop_front();
	}
	for (int q = __oldPositions.size()-1 ; q > 0 && __oldPositions.size() > 1; q--)
	{
		DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q-1),gBlack, q/(float)__oldPositions.size());
	}

#endif

#ifdef DRAW_ANNOTATIONS

	for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	{
		if ( _gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
		else
		{
			//DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i=0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
	}

	for (int i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
	{
		if ( _gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gMagenta);
		}
		else
		{
			// DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
		}
	}

	DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
	DrawLib::drawStar(this->_currentLocalTarget+Util::Vector(0,0.001,0), Util::Vector(1,0,0), 0.24f, gGray10);

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();
	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
		Util::Vector normal = calcWallNormal( *tmp_o );
		std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
		Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				(line.first.z+line.second.z)/2);
		DrawLib::drawLine(midpoint, midpoint+normal, gGreen);
		// Draw the closes point as well
		std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
		DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}