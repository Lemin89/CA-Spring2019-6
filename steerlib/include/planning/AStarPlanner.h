//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/
			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = true);
			
			struct epsilonComparator
			{
				bool operator()(Util::Point a, Util::Point b) const
				{
					if (distanceBetween(a, b) < 0.001) {
						return false;
					}
					if (a.x == b.x)
					{
						return a.z < b.z;
					}
					else
					{
						return a.x < b.x;
					}
				}
			};

			struct NodeComparator
			{
				bool operator()(SteerLib::AStarPlannerNode a, SteerLib::AStarPlannerNode b) const
				{
					if (abs(distanceBetween(a.point, b.point)) < 0.001) {
						return false;
					}
					if (a.point.x == b.point.x)
					{
						return a.point.z < b.point.z;
					}
					else
					{
						return a.point.x < b.point.x;
					}
				}
			};

			std::vector<Util::Point> AStarPath(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, float WEIGHT, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& InConsistSet, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap);
			std::vector<Util::Point> ARAStarPath(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& InConsistSet, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap);
			std::vector<Util::Point> ImprovePath(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, float WEIGHT, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& InConsistSet, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap);
			void AddAraNode(Util::Point CurrentPoint, double cost, SteerLib::AStarPlannerNode FromNode, Util::Point goal, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& InconsSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap, float WEIGHT);

			int getIndexFromPoint(Util::Point p);
			std::vector<Util::Point> GetNeighborPoints(Util::Point OriginPoint);
			void NeighborNodes(Util::Point OriginPoint, Util::Point goal, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::vector<Util::Point>& InConsistSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap, float WEIGHT);
			void AddNode(Util::Point CurrentPoint, double cost, SteerLib::AStarPlannerNode FromNode, Util::Point goal, std::map<Util::Point, SteerLib::AStarPlannerNode, epsilonComparator>& NodeMap, std::vector<Util::Point>& ClosedSet, std::vector<Util::Point>& OpenSet, std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode, NodeComparator>& CameFromNodeMap, float WEIGHT);
			double Manhattan(Util::Point FirstPoint, Util::Point SecondPoint);
			double Heuristic(Util::Point a, Util::Point b, float WEIGHT);
	
		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	};
}


#endif
