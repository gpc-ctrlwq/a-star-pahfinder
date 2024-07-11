#pragma once

#include <vector>
#include <forward_list>
#include <memory>


// A star pathfinding, for a 2D grid


using namespace std;


namespace astar {

	struct Point2D {
		int x;
		int y;

		Point2D();
		void operator=(const Point2D& rhs);
		bool operator==(const Point2D& rhs);
	};


	struct Node {
		Point2D pos;
		Node* parent;
		int f;	  // = g + h
		int g;	  // cost to get to this Node from parent
		int h;	  // estimated cost from this Node to target

		Node();
		Node(const Node& rhs);
		Node(Node&& rhs);
		Node& operator=(const Node& rhs);
		bool operator==(const Node& rhs);
	};


	class Astar {
	private:
		int width;
		int height;
		int p;	  // horizontal / vertical movement cost
		int d;	  // diagonal movement cost
		Point2D start;
		Point2D target;

		// whether a coordinate is passable or not. true is blocked, false is passable.
		// in form of (y,x)
		vector<vector<bool>> matrix;

		int calculateG(Node& n) const;
		int calculateH(Node& n) const;
		int calculateF(Node& n) const;

		// create and init a new Node. dynamically allocated, returned Node needs to be free'd eventually
		Node* createNode(const Point2D pos, Node* parent) const;

		int inline nodeHash(const Node* n);
		int inline nodeHash(const Point2D& pos);

	public:
		Astar();
		Astar(const vector<vector<bool>> matrix);
		Astar(const vector<vector<bool>> matrix, const int perpendicularCost, const int diagonalCost);
		~Astar();

		void setCosts(const int perpendicularCost, const int diagonalCost);
		void setMatrix(const vector<vector<bool>>& matrix);
		void setMatrixAt(const int x, const int y, const bool b);

		// find a path from start to target
		// returns the path on success, else nullptr
		unique_ptr<forward_list<Point2D>> findPath(const Point2D& start, const Point2D& target);
	};
}


