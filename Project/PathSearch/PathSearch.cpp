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
		adjacentTilesEven[5] = std::pair<int, int>(0, 1);

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
		bestNode = nullptr;
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

		//debug_DrawSearchNodeConnections();
	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
		queue.clear();
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
		startPNode->setNodeCost(goalNode->tile->getRow(), goalNode->tile->getColumn());

		// Push start onto queue
		queue.push(startPNode);
		visited[startNode] = startPNode;

		// Mark startNode as visited
		MarkTileAsVisited(startNode->tile);
		bestNode = startPNode;
	}

	void PathSearch::update(long timeslice)
	{
		// Search
		// Load state from previous pause
		while (!queue.empty() && timeslice > -1)
		{
			PlannerNode* current = queue.front();
			queue.pop();

			bestNode = current;

			if (current->searchNode == goalNode)
			{
				// Goal Achieved
				queue.clear();
				return;
			}

			int sz = current->searchNode->neighbors.size();
			for (int i = 0; i < current->searchNode->neighbors.size(); ++i)
			{
				SearchNode* successor = current->searchNode->neighbors[i];

				if (visited.find(successor) == visited.end())
				{
					PlannerNode* successorNode = new PlannerNode();
					successorNode->searchNode = successor;
					successorNode->parent = current;
					successorNode->setNodeCost(goalNode->tile->getRow(), goalNode->tile->getColumn());

					visited[successor] = successorNode;
					queue.push(successorNode);
				}
			}

			timeslice -= 1;
		}

		// Draw
		tileMap->resetTileDrawing();
		// Draw Visited
		for (auto itter = visited.begin(); itter != visited.end(); ++itter)
			MarkTileAsVisited(itter->second->searchNode->tile);
		// Draw Neighbors
		for (int i = 0; i < bestNode->searchNode->neighbors.size(); ++i)
			MarkTileAsNeighbor(bestNode->searchNode->neighbors[i]->tile);
		// Draw Open
		std::vector<PlannerNode*> openNodes;
		queue.enumerate(openNodes);
		int grade = 1;
		for (auto itter = openNodes.begin(); itter != openNodes.end(); ++itter)
			MarkTileAsOpen((*itter)->searchNode->tile, queue.size() / grade++);

		debug_DrawLineThroughPath();
	}

	void PathSearch::exit()
	{
		bestNode = nullptr;

		ClearQueue();

		for (auto itter = visited.begin(); itter != visited.end(); itter++)
			delete itter->second;
		visited.clear();
	}

	void PathSearch::shutdown()
	{
		goalNode = nullptr;
		bestNode = nullptr;
		ClearContainers();
	}

	bool PathSearch::isDone() const
	{
		return queue.empty();
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		std::vector<Tile const*> temp;

		for (PlannerNode* curr = bestNode; curr != nullptr; curr = curr->parent)
			temp.push_back(curr->searchNode->tile);

		return temp;
	}

	void PathSearch::MarkTileAsOpen(Tile* tile, int grade)
	{
		unsigned int openColor = max(255 - (30 * grade), 100);
		openColor = openColor << 8;
		openColor |= 0xFF000000;

		tile->setMarker(openColor);
	}

	void PathSearch::MarkTileAsNeighbor(Tile* tile)
	{
		tile->setOutline(COLOR_BEST_NEIGHBOR_OUTLINE);
		//tile->setFill(COLOR_BEST_NEIGHBOR_OUTLINE);
	}

	void PathSearch::MarkTileAsVisited(Tile* tile)
	{
		tile->setFill(COLOR_VISITED);
		tile->setOutline(COLOR_VISITED);
	}

	void PathSearch::debug_DrawLineThroughPath()
	{
		PlannerNode* current = bestNode;

		while (current != nullptr)
		{
			if (current->parent != nullptr)
			{
				current->searchNode->tile->addLineTo(
					current->parent->searchNode->tile,
					0xFFFF0000
				);
			}
			current = current->parent;
		}
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

		nodes.clear();
		visited.clear();
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

