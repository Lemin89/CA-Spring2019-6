//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//

#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"

#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// Set to false for Euclidean distance
#define USE_MANHATTAN_DISTANCE false
#define PRINT_RESULTS false

// choose algorithm
#define A_STAR 1
#define ARA_STAR 2
#define AD_STAR 3
#define USE_ALGORITHM A_STAR

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());

		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
			}
		}
		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}

	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	/*bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		//TODO
		std::cout<<"\nIn A*"<<std::endl;
		return false;
	}*/

	int AStarPlanner::getIndexFromPoint(Util::Point p)
	{
		return gSpatialDatabase->getCellIndexFromLocation(p);
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{	
		std::vector<Util::Point> ClosedSet;
		std::vector<Util::Point> OpenSet;
		std::vector<Util::Point> InConsistSet;
		std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator> NodeMap;
		std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator> CameFromNodeMap;

		gSpatialDatabase = _gSpatialDatabase;
		start = getPointFromGridIndex(getIndexFromPoint(start));
		goal = getPointFromGridIndex(getIndexFromPoint(goal));

		if (USE_ALGORITHM == A_STAR) {
			agent_path = AStarPath(start, goal, _gSpatialDatabase, 1, OpenSet, ClosedSet, InConsistSet, NodeMap, CameFromNodeMap);
		}
		else if (USE_ALGORITHM == ARA_STAR) {
			agent_path = ARAStarPath(start, goal, _gSpatialDatabase, OpenSet, ClosedSet, InConsistSet, NodeMap, CameFromNodeMap);
		}
		//agent_path = ARAStarPath(start, goal, _gSpatialDatabase,OpenSet, ClosedSet,InConsistSet,NodeMap, CameFromNodeMap);
		
		if (agent_path.size() > 0) {
			return true;
		}
		else {
			return false;
		}
	}
	
	std::vector<Util::Point> AStarPlanner::ARAStarPath(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& InConsistSet, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap) {
		std::vector<Util::Point> path;
		float initial_weight = 12;
		float decrease_rate = 1;
		float current_weight = initial_weight;

		//insert start to open set
		SteerLib::AStarPlannerNode StartNode(start, 0, Heuristic(start, goal, initial_weight), nullptr);
		NodeMap.emplace(start, StartNode);
		OpenSet.push_back(StartNode.point);

		//update all sets and return suboptimal path
		path = ImprovePath(start, goal, _gSpatialDatabase, initial_weight, OpenSet, ClosedSet, InConsistSet, NodeMap, CameFromNodeMap);
		
		//publish suboptimal path?

		while (current_weight > 1) {
			current_weight = current_weight - decrease_rate;
			for (int i = 0; i < InConsistSet.size(); i++) {
				OpenSet.push_back(InConsistSet.at(i));
			}
			InConsistSet.clear();

			//update f values in open set
			for (int i = 0; i< OpenSet.size(); i++)
			{
				NodeMap.at(OpenSet[i]).f = NodeMap.at(OpenSet[i]).g + Heuristic(OpenSet[i], goal, current_weight);
			}
			ClosedSet.clear();
			//update all sets and return suboptimal path
			path.clear();

			//update all sets and return suboptimal path
			path = ImprovePath(start, goal, _gSpatialDatabase, current_weight, OpenSet, ClosedSet, InConsistSet, NodeMap, CameFromNodeMap);

			//publish suboptimal path?
		}

		return path;
	}

	//return type change from void to set
	//update all sets and return suboptimal path
	std::vector<Util::Point> AStarPlanner::ImprovePath(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase,float WEIGHT, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& InConsistSet, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap) {
		int lowestFIndex = 0;
		double lowestFScore = 0;
		double GScore;
		std::vector<Util::Point> path;
		AStarPlannerNode startNode(start, 0, Heuristic(start, goal, WEIGHT), nullptr);
		AStarPlannerNode goalNode(goal, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), nullptr);
		// if (OpenSet.empty()) {
		// 	OpenSet.push_back(startNode.point);
		// }
		// if (NodeMap.empty()) {
		// 	NodeMap.emplace(start, startNode);
		// }
		lowestFIndex = 0;
		lowestFScore = NodeMap.at(OpenSet[0]).f;
		for (unsigned i = 0; i < OpenSet.size(); i++) {
			if (NodeMap.at(OpenSet[i]).f < lowestFScore) {
				lowestFScore = NodeMap.at(OpenSet[i]).f;
				GScore = NodeMap.at(OpenSet[i]).g;
				lowestFIndex = i;
			}
			// std::cout << OpenSet[i] << ", ";
		}
		// std::cout << std::endl << lowestFIndex << ", " << lowestFScore << std::endl;
		while (goalNode.f > lowestFScore) {
			AStarPlannerNode CurrentNode = NodeMap.at(OpenSet[lowestFIndex]);
			OpenSet.erase(OpenSet.begin() + lowestFIndex);
			ClosedSet.push_back(OpenSet[lowestFIndex]);
			NeighborNodes(CurrentNode.point, goal, NodeMap, ClosedSet, OpenSet, InConsistSet, CameFromNodeMap, WEIGHT);
			// Recalculate the min
			lowestFIndex = 0;
			lowestFScore = NodeMap.at(OpenSet[0]).f;
			for (unsigned i = 0; i < OpenSet.size(); i++) {
				if (NodeMap.at(OpenSet[i]).f < lowestFScore) {
					lowestFScore = NodeMap.at(OpenSet[i]).f;
					GScore = NodeMap.at(OpenSet[i]).g;
					lowestFIndex = i;
				}
			}
		}
		AStarPlannerNode node = NodeMap.at(goal);
		path.push_back(goal);
		while (node.point != start) {
			node = CameFromNodeMap.at(node);
			path.push_back(node.point);
		}
		path.push_back(start);
		std::reverse(path.begin(), path.end());
		return path;

	}

	std::vector<Util::Point>AStarPlanner::AStarPath(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase,float WEIGHT, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& InConsistSet, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap) {
		std::vector<Util::Point> path;
		double lowestFScore = 0;
		int lowestFIndex = 0;
		double GScore;
		double CostSoFar;
		double TotalLength;

		SteerLib::AStarPlannerNode StartNode(start, 0, Heuristic(start, goal, WEIGHT), nullptr);
		NodeMap.emplace(start, StartNode);
		OpenSet.push_back(StartNode.point);
		while (!OpenSet.empty())
		{
			//Get Node with the lowest F
			lowestFScore = NodeMap.at(OpenSet[0]).f;
			GScore = NodeMap.at(OpenSet[0]).g;
			lowestFIndex = 0;
			for (int i = 0; i< OpenSet.size(); i++)
			{
				if (NodeMap.at(OpenSet[i]).f < lowestFScore)
				{
					lowestFScore = NodeMap.at(OpenSet[i]).f;
					GScore = NodeMap.at(OpenSet[i]).g;
					lowestFIndex = i;
				}
			}
			SteerLib::AStarPlannerNode CurrentNode = NodeMap.at(OpenSet[lowestFIndex]);
			if (CurrentNode.point == goal)
			{
				TotalLength = CurrentNode.g;
				path.push_back(CurrentNode.point);
				while (CurrentNode.point != start)
				{
					CurrentNode = CameFromNodeMap.at(CurrentNode);
					path.push_back(CurrentNode.point);
				}
				path.push_back(start);
				std::reverse(path.begin(), path.end());
				
				if (PRINT_RESULTS)
				{
					std::cout << "\nLength of Solution Path " << TotalLength << '\n';
					std::cout << "Number of Expanded Nodes " << ClosedSet.size() << '\n';
				}
				//std::cout << "path in weighted a*:" << path << std::endl;
				return path;
			}
			ClosedSet.push_back(OpenSet[lowestFIndex]);
			OpenSet.erase(OpenSet.begin() + lowestFIndex);
			NeighborNodes(CurrentNode.point, goal, NodeMap, ClosedSet, OpenSet, InConsistSet, CameFromNodeMap, WEIGHT);
		}
		path.clear();
		return path;
	}

	std::vector<Util::Point>AStarPlanner::GetNeighborPoints(Util::Point OriginPoint)
	{
		int NodeIndex;
		unsigned int x, z;
		NodeIndex = getIndexFromPoint(OriginPoint);
		std::vector<Util::Point> NeighborPoints;
		Util::Point NeighborPoint;
		gSpatialDatabase->getGridCoordinatesFromIndex(NodeIndex, x, z);
		int XRangeMin, XRangeMax, ZRangeMin, ZRangeMax;
		XRangeMin = MAX(x - 1, 0);
		XRangeMax = MIN(x + 1, gSpatialDatabase->getNumCellsX());
		ZRangeMin = MAX(z - 1, 0);
		ZRangeMax = MIN(z + 1, gSpatialDatabase->getNumCellsZ());
		for (int i = XRangeMin; i <= XRangeMax; i += GRID_STEP)
		{
			for (int j = ZRangeMin; j <= ZRangeMax; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				if (index != NodeIndex)
				{
					NeighborPoint = getPointFromGridIndex(index);
					if (USE_MANHATTAN_DISTANCE)
					{
						if (NeighborPoint.x == OriginPoint.x || NeighborPoint.z == OriginPoint.z)
						{
							NeighborPoints.push_back(NeighborPoint);
						}
					}
					else
					{
						NeighborPoints.push_back(NeighborPoint);
					}
				}
			}
		}
		return NeighborPoints;
	}

	void AStarPlanner::NeighborNodes(Util::Point OriginPoint, Util::Point goal, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& InConsistSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap, float WEIGHT)
	{
		int x;
		int y;
		int z;
		double InitialCost;
		SteerLib::AStarPlannerNode OriginNode = NodeMap.at(OriginPoint);
		InitialCost = std::numeric_limits<double>::infinity();
		std::vector<Util::Point> NeighborPoints;
		NeighborPoints = GetNeighborPoints(OriginPoint);

		for (unsigned i = 0; i < NeighborPoints.size(); i++) {
			switch (USE_ALGORITHM) {
			case A_STAR:
				AddNode(NeighborPoints[i], InitialCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap, WEIGHT);
				break;
			case ARA_STAR:
				AddAraNode(NeighborPoints[i], InitialCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, InConsistSet, CameFromNodeMap, WEIGHT);
				break;
			case AD_STAR:
				//AddADNode(NeighborPoints[i], InitialCost, OriginNode, goal, NodeMap, ClosedSet, OpenSet, CameFromNodeMap, WEIGHT);
				break;
			default:
				std::cerr << "Incorrect algorithm selected. Select ASTAR, ARA_STAR or AD_STAR";
				return;
			}
		}
	}

	void AStarPlanner::AddAraNode(Util::Point CurrentPoint, double cost, SteerLib::AStarPlannerNode FromNode, Util::Point goal, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& InconsSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap, float WEIGHT) {
		int NodeIndex;
		double TentativeScore;
		NodeIndex = gSpatialDatabase->getCellIndexFromLocation(CurrentPoint);
		// std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>::iterator CameFromMapIt;
		if (!canBeTraversed(NodeIndex)) {
			return;
		}
		if (NodeMap.count(CurrentPoint) == 0) { // has not been visited before
			SteerLib::AStarPlannerNode InsertNode(CurrentPoint, cost, cost, &FromNode);
			NodeMap.emplace(CurrentPoint, InsertNode);
		}
		// g(s') > g(s) + c(s, s')
		if (NodeMap.at(CurrentPoint).g > FromNode.g + distanceBetween(FromNode.point, CurrentPoint)) {
			NodeMap.at(CurrentPoint).g = FromNode.g + distanceBetween(FromNode.point, CurrentPoint);
			// if currentPoint is in closed set
			if (std::find(ClosedSet.begin(), ClosedSet.end(), CurrentPoint) != ClosedSet.end()) {
				if (std::find(InconsSet.begin(), InconsSet.end(), CurrentPoint) == InconsSet.end()) {
					InconsSet.push_back(CurrentPoint);
				}
			}
			else {
				if (std::find(OpenSet.begin(), OpenSet.end(), CurrentPoint) == OpenSet.end()) { // if not in open set
					OpenSet.push_back(CurrentPoint);
				}
			}
		}
		if (std::find(ClosedSet.begin(), ClosedSet.end(), CurrentPoint) != ClosedSet.end()) { // if currentPoint is in closed set{
			return;
		}
		TentativeScore = FromNode.g + distanceBetween(FromNode.point, CurrentPoint);
		SteerLib::AStarPlannerNode InsertNode(CurrentPoint, TentativeScore, TentativeScore + Heuristic(CurrentPoint, goal, WEIGHT), &FromNode);
		NodeMap.erase(CurrentPoint);
		NodeMap.emplace(CurrentPoint, InsertNode);
		if (CameFromNodeMap.count(InsertNode) != 0) {
			CameFromNodeMap.erase(InsertNode);
		}
		CameFromNodeMap.emplace(InsertNode, FromNode);
	}

	void AStarPlanner::AddNode(Util::Point CurrentPoint, double cost, SteerLib::AStarPlannerNode FromNode, Util::Point goal, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap, float WEIGHT)
	{
		int NodeIndex;
		float DistanceInBetween;
		double TentativeScore;
		NodeIndex = gSpatialDatabase->getCellIndexFromLocation(CurrentPoint);
		std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>::iterator CameFromMapIt;
		if (!canBeTraversed(NodeIndex))
		{
			return;
		}
		if (NodeMap.count(CurrentPoint) == 0)
		{
			SteerLib::AStarPlannerNode InsertNode(CurrentPoint, cost, cost, &FromNode);
			NodeMap.emplace(CurrentPoint, InsertNode);
		}
		if (std::find(ClosedSet.begin(), ClosedSet.end(), CurrentPoint) != ClosedSet.end())
		{
			return;
		}
		TentativeScore = FromNode.g + distanceBetween(FromNode.point, CurrentPoint);
		if (std::find(OpenSet.begin(), OpenSet.end(), CurrentPoint) == OpenSet.end())
		{
			OpenSet.push_back(CurrentPoint);
		}
		else if (TentativeScore >= NodeMap.at(CurrentPoint).g)
		{
			return;
		}
		SteerLib::AStarPlannerNode InsertNode(CurrentPoint, TentativeScore, TentativeScore + Heuristic(CurrentPoint, goal,WEIGHT), &FromNode);
		NodeMap.erase(CurrentPoint);
		NodeMap.emplace(CurrentPoint, InsertNode);
		if (CameFromNodeMap.count(InsertNode) != 0)
		{
			CameFromNodeMap.erase(InsertNode);
		}
		CameFromNodeMap.emplace(InsertNode, FromNode);
	}

	double AStarPlanner::Manhattan(Util::Point point1, Util::Point point2)
	{
		Util::Vector diff = point1 - point2;
		return abs(diff.x) + abs(diff.y) + abs(diff.z);
	}

	double AStarPlanner::Heuristic(Util::Point point1, Util::Point point2, float WEIGHT)
	{
		if (USE_MANHATTAN_DISTANCE) {
			return WEIGHT*Manhattan(point1, point2);
		}
		else {
			return WEIGHT*(double)distanceBetween(point1, point2);
		}
	}
}