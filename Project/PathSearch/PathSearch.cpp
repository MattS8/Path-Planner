#include "PathSearch.h"
#include <iostream>

namespace fullsail_ai { namespace algorithms {

	PathSearch::PathSearch()
	{
		// Col, Row
		adjacentTilesEven[0] = std::pair<int, int>(-1, -1);
		adjacentTilesEven[1] = std::pair<int, int>(0, -1);
		adjacentTilesEven[2] = std::pair<int, int>(-1, 0);
		adjacentTilesEven[3] = std::pair<int, int>(1, 0);
		adjacentTilesEven[4] = std::pair<int, int>(-1, 1);
		adjacentTilesEven[5] = std::pair<int, int>(0, -1);

		adjacentTilesOdd[0] = std::pair<int, int> (0, -1);
		adjacentTilesOdd[1] = std::pair<int, int>(1, -1);
		adjacentTilesOdd[2] = std::pair<int, int>(-1, 0);
		adjacentTilesOdd[3] = std::pair<int, int>(1, 0);
		adjacentTilesOdd[4] = std::pair<int, int>(0, 1);
		adjacentTilesOdd[5] = std::pair<int, int>(1, 1);
	}

	PathSearch::~PathSearch()
	{
		ClearContainers();
	}

	PathSearch::SearchNode* PathSearch::GetSearchNode(Tile* tile)
	{
		SearchNode* newNode;
		auto gotNode = nodes.find(tile);
		if (gotNode == nodes.end())
		{
			newNode = new SearchNode();
			newNode->tile = tile;
			nodes.insert(std::pair<Tile*, SearchNode*>(tile, newNode));
		}
		else
		{
			newNode = gotNode->second;
		}

		return newNode;
	}

	void PathSearch::initialize(TileMap* _tileMap)
	{
		ClearContainers();

		tileMap = _tileMap;

		// Create SearchNode graph
		for (int row = 0; row < tileMap->getRowCount(); row++)
		{
			for (int col = 0; col < tileMap->getColumnCount(); col++)
			{
				// Only create nodes for tiles that are traversable
				Tile* tile = tileMap->getTile(row, col);
				if (tile->getWeight() == 0)
					continue;

				SearchNode* newSearchNode = GetSearchNode(tile);
				std::pair<int, int>* offsets = row % 2 == 0
					? adjacentTilesEven
					: adjacentTilesOdd;
				for (int a = 0; a < MAX_ADJACENT_NEIGHBORS; a++)
				{
					Tile* adjacentTile = tileMap->getTile(
						row + offsets[a].second, 
						col + offsets[a].first);
					if (adjacentTile != 0 && adjacentTile->getWeight() > 0)
					{
						SearchNode* adjacentSearchNode = GetSearchNode(adjacentTile);
						newSearchNode->neighbors.push_back(adjacentSearchNode);
					}
				}	
			}
		}
	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
		Tile* startTile = tileMap->getTile(startRow, startColumn);
		Tile* goalTile = tileMap->getTile(goalRow, goalColumn);

		// Ensure start and goal tiles are navigable
		if (startTile == 0 || goalTile == 0
			|| startTile->getWeight() == 0 || goalTile->getWeight() == 0)
			return;

		// Set the goal node
		goalNode = nodes.find(goalTile)->second;

		// Create PlannerNode for start
		SearchNode* startNode = nodes.find(startTile)->second;
		PlannerNode* startPNode = new PlannerNode();
		startPNode->searchNode = startNode;
		startPNode->parent = nullptr;
		startPNode->setNodeCost(goalTile);

		// Push start onto queue
		queue.push(startPNode);
		visited[startNode] = startPNode;

		// Add neighbor PlannerNodes to the queue
		//for (int i = 0; i < startNode->neighbors.size(); ++i)
		//{
		//	PlannerNode* pNode = new PlannerNode();
		//	pNode->searchNode = startNode->neighbors[i];
		//	pNode->parent;
		//	pNode->setNodeCost(goalTile);
		//	queue.push(pNode);
		//}
	}

	void PathSearch::update(long timeslice)
	{
		while (!queue.empty() && timeslice > -1)
		{
			PlannerNode* current = queue.front();
			queue.pop();

			if (current->searchNode == goalNode)
			{
				// Goal Achieved

				return;
			}

			for (int i = 0; i < current->searchNode->neighbors.size(); ++i)
			{
				SearchNode* successor = current->searchNode->neighbors[i];

				if (visited.find(successor) == visited.end())
				{
					PlannerNode* successorNode = new PlannerNode();
					successorNode->searchNode = successor;
					successorNode->parent = current;
					successorNode->setNodeCost(goalNode->tile);

					visited[successor] = successorNode;
					queue.push(successorNode);
				}
			}

			timeslice -= 1;
		}

	}

	void PathSearch::exit()
	{
		ClearQueue();

		for (auto itter = visited.begin(); itter != visited.end(); itter++)
			delete itter->second;
	}

	void PathSearch::shutdown()
	{
		ClearContainers();
	}

	bool PathSearch::isDone() const
	{
		return queue.empty();
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		std::vector<Tile const*> temp;
		return temp;
	}

	void PathSearch::debug_DrawSearchNodeConnections(SearchNode* searchNode)
	{
		if (searchNode == nullptr)
			searchNode = nodes.begin()->second;
		if (searchNode == nullptr)
			return;

		if (visited.find(searchNode) != visited.end())
			return;
		visited.insert(std::pair<SearchNode*, PlannerNode*>(searchNode, nullptr));

		unsigned int color = 0xFFFF0000;

		for (int i = 0; i < searchNode->neighbors.size(); i++)
		{
			searchNode->tile->addLineTo(searchNode->neighbors[i]->tile, color);
			debug_DrawSearchNodeConnections(searchNode->neighbors[i]);
		}
	}

	void PathSearch::debug_PrintSearchNodes()
	{
		for (auto itter = nodes.begin(); itter != nodes.end(); itter++)
		{
			std::cout << "Node: (" << itter->second->tile->getXCoordinate()
				<< ", " << itter->second->tile->getYCoordinate()
				<< ") has " << itter->second->neighbors.size()
				<< " neighbors:";

			for (int i = 0; i < itter->second->neighbors.size(); i++)
			{
				SearchNode* neighbor = itter->second->neighbors[i];
				std::cout << "\n\t(" << neighbor->tile->getXCoordinate()
					<< ", " << neighbor->tile->getYCoordinate() << ")";
			}

			std::cout << "\n";
		}
	}

	void PathSearch::ClearContainers()
	{
		ClearQueue();

		for (auto itter = nodes.begin(); itter != nodes.end(); itter++)
			delete itter->second;

		for (auto itter = visited.begin(); itter != visited.end(); itter++)
			delete itter->second;
	}

	void PathSearch::ClearQueue()
	{
		for (int i = 0; i < queue.size(); i++)
		{
			PlannerNode* pNode = queue.front();
			queue.remove(pNode);

			if (visited.find(pNode->searchNode) == visited.end())
				delete pNode;
		}
	}
}}  // namespace fullsail_ai::algorithms

