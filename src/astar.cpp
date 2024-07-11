#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include "astar.h"
#include <iostream>


using namespace astar;
using namespace std;


Point2D::Point2D() {
	x = -1;
	y = -1;
}

void Point2D::operator=(const Point2D& rhs) {
	this->x = rhs.x;
	this->y = rhs.y;
}

bool Point2D::operator==(const Point2D & rhs) {
	if (this->x == rhs.x && this->y == rhs.y) {
		return true;
	}
	else {
		return false;
	}
}


Node::Node() {
	parent = nullptr;
	f = -1;
	g = -1;
	h = -1;
}

Node::Node(const Node& rhs) {
	this->pos = rhs.pos;
	this->parent = rhs.parent;
	this->f = rhs.f;
	this->g = rhs.g;
	this->h = rhs.h;
}

Node::Node(Node&& rhs) {
	this->pos = rhs.pos;
	this->parent = rhs.parent;
	this->f = rhs.f;
	this->g = rhs.g;
	this->h = rhs.h;
	rhs.pos.x = -1;
	rhs.pos.y = -1;
	rhs.parent = nullptr;
	rhs.f = 0;
	rhs.g = 0;
	rhs.h = 0;
}

Node& Node::operator=(const Node& rhs) {
	this->pos = rhs.pos;
	this->parent = rhs.parent;
	this->f = rhs.f;
	this->g = rhs.g;
	this->h = rhs.h;
	return *this;
}

bool Node::operator==(const Node& rhs) {
	if (this->pos == rhs.pos) {
		return true;
	}
	else {
		return false;
	}
}

struct nodeCompareFunctor {
	bool operator()(const Node* lhs, const Node* rhs) const {
		return lhs->f < rhs->f;
	}
};


// find g cost for n
// adds cost of moving from parent Node to this Node
// returns (positive) g value on success, 0 if no parent, else -1
int Astar::calculateG(Node& n) const {
	if (n.parent == nullptr) {
		n.g = 0;
		return 0;
	}

	// if Nodes are not next to each other
	if (abs(n.pos.x - n.parent->pos.x) > 1 || abs(n.pos.y - n.parent->pos.y) > 1) {
		return -1;
	}

	// if diagonal from parent
	if (abs(n.pos.x - n.parent->pos.x) == 1 && abs(n.pos.y - n.parent->pos.y) == 1) {
		n.g = n.parent->g + d;
	}
	// perpendicular (horizontal or vertical) from parent
	else {
		n.g = n.parent->g + p;
	}

	return n.g;
}

// estimate the cost to move from n to target
// returns estimate
int Astar::calculateH(Node& n) const {
	// manhatten heuristic
	n.h = abs(n.pos.x - target.x) + abs(n.pos.y - target.y);
	return n.h;
}

int Astar::calculateF(Node & n) const {
	int gscore = calculateG(n);
	n.f = -1;
	if (gscore != -1) {
		n.f = gscore + calculateH(n);
	}
	return n.f;
}

Node* Astar::createNode(const Point2D pos, Node* parent) const {
	Node* n = new Node();
	n->pos = pos;
	n->parent = parent;
	calculateF(*n);
	return n;
}

// Node hashing function for std::map
int inline Astar::nodeHash(const Node* n) {
	return n->pos.y * width + n->pos.x;
}

int inline Astar::nodeHash(const Point2D& n) {
	return n.y * width + n.x;
}

Astar::Astar() {
	p = 10;
	d = 14;
	height = 10;
	width = 10;

	// adjust matrix to default dimensions and set all entries to false
	matrix.resize(height);
	for (int ii = 0; ii < height; ii++) {
		matrix.at(ii).resize(width, false);
	}

}

Astar::Astar(const vector<vector<bool>> matrix) {
	this->matrix = matrix;
	height = this->matrix.size();
	width = this->matrix.at(0).size();
	p = 10;
	d = 14;
}

Astar::Astar(const vector<vector<bool>> matrix, const int perpendicularCost, const int diagonalCost) {
	this->matrix = matrix;
	height = this->matrix.size();
	width = this->matrix.at(0).size();
	p = perpendicularCost;
	d = diagonalCost;
}

Astar::~Astar() {}

void Astar::setCosts(const int perpendicularCost, const int diagonalCost) {
	p = perpendicularCost;
	d = diagonalCost;
}

// copies matrix to this->matrix
void Astar::setMatrix(const vector<vector<bool>>& matrix) {
	this->matrix = matrix;
	height = matrix.size();
	width = matrix.at(0).size();
}

// set position x,y to b
void Astar::setMatrixAt(const int x, const int y, const bool b) {
	matrix.at(y).at(x) = b;
}


unique_ptr<forward_list<Point2D>> Astar::findPath(const Point2D& start, const Point2D& target) {
	Node* current;				// current Node in search
	map<int, Node*> nodeMap;	// every Node in search space
	multiset<Node*, nodeCompareFunctor> openSet;	// Nodes to be evalutated

	this->start = start;
	this->target = target;

	// if already at target
	if (start.x == target.x && start.y == target.y) {
		return nullptr;
	}
	// if target is unaccessable. should also test if matrix is initialized or not
	if (matrix.at(target.y).at(target.x) == true) {
		return nullptr;
	}

	// create and init first Node
	current = createNode(start, nullptr);
	nodeMap.insert(pair<int, Node*>(nodeHash(current), current));
	openSet.insert(current);

	Point2D search;	// next point to be evaluated

	// main loop
	while (!openSet.empty()) {
		current = *openSet.cbegin();
		openSet.erase(openSet.cbegin());

		// add surrounding nodes to open list
		// record their parent and determine their F, G, H, scores
		for (int yy = -1; yy <= 1; yy++) {
			for (int xx = -1; xx <= 1; xx++) {
				search.y = current->pos.y + yy;
				search.x = current->pos.x + xx;

				if (search.x < 0 || search.y < 0 || search.x >= width || search.y >= height) {
					continue;
				}
				if (search == current->pos) {
					continue;
				}
				// if position is blocked, skip it
				if (matrix.at(search.y).at(search.x)) {
					continue;
				}

				// if search point already on open list (ie. node for search point already exists, therefor on nodeList);
				Node* n = createNode(search, current);
				auto match = nodeMap.find(nodeHash(search));
				if (match != nodeMap.cend()) {
					// replace if new node is better than old node
					if (n->f < match->second->f) {
						auto oldNode = match->second;
						nodeMap.at(nodeHash(search)) = n;
						auto temp = openSet.find(match->second);
						if (temp != openSet.cend()) {
							openSet.erase(temp);
						}
						openSet.insert(n);
						delete oldNode;
					}
					else {
						delete n;
					}
				}
				else {
					nodeMap.insert(pair<int, Node*>(nodeHash(search), n));
					openSet.insert(n);
				}

				// if path found
				if (search == target) {
					unique_ptr<forward_list<Point2D>> path(new forward_list<Point2D>);

					// follow parents from target to start
					current = nodeMap.find(nodeHash(search))->second;
					do {
						path->push_front(current->pos);
						current = nodeMap.find(nodeHash(current->parent->pos))->second;
					} while (current->parent != nullptr);
					path->push_front(start);

					// free
					for each (auto e in nodeMap) {
						if (e.second != nullptr) {
							delete e.second;
						}
					}
					return move(path);
				}
			}
		}
	}
	// free
	for each (auto e in nodeMap) {
		if (e.second != nullptr) {
			delete e.second;
		}
	}
	return nullptr;
}
