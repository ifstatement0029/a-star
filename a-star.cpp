struct XYStruct {
	int x = -1, y = -1;
};

bool compareXYStructs(XYStruct position1, XYStruct position2) {
	if (position1.x == position2.x && position1.y == position2.y) {
		return true;
	}
	return false;
}

vector<XYStruct> getAStarPath(vector<vector<bool>>& maze, XYStruct start, XYStruct end) {
	vector<XYStruct> path;

	//1.  Initialize the open list
	vector<nodeWithParentStruct> openList, closedList;

	/*2.  Initialize the closed list
	put the starting node on the open
	list(you can leave its f at zero)*/
	nodeWithParentStruct startingNode;
	startingNode.node.position = start;
	openList.push_back(startingNode);

	//3.  while the open list is not empty
	bool stopSearch = false;
	while ((int)openList.size() > 0 && stopSearch == false) {
		
		/*a) find the node with the least f on
		the open list, call it "q"*/
		nodeWithParentStruct q;
		int qIndex = -1;
		for (int openListCnt = 0; openListCnt < (int)openList.size(); ++openListCnt) {
			if (q.node.f == -1 || openList[openListCnt].node.f < q.node.f) {
				q = openList[openListCnt];
				qIndex = openListCnt;
			}
		}
		if (q.node.position.x != -1) {

			//b) pop q off the open list
			openList.erase(openList.begin() + qIndex);

			/*c) generate q's 8 successors and set their
			parents to q*/
			vector<nodeWithParentStruct> successors;
			for (int mazeXCnt = q.node.position.x - 1; mazeXCnt <= q.node.position.x + 1; ++mazeXCnt) {
				for (int mazeYCnt = q.node.position.y - 1; mazeYCnt <= q.node.position.y + 1; ++mazeYCnt) {
					if (mazeXCnt >= 0 && mazeXCnt <= (int)maze.size() - 1 && mazeYCnt >= 0 && mazeYCnt <= (int)maze[mazeXCnt].size() - 1) {
						if (mazeXCnt != q.node.position.x || mazeYCnt != q.node.position.y) {
							nodeWithParentStruct currentSuccessor;
							currentSuccessor.node.position = { mazeXCnt, mazeYCnt };
							currentSuccessor.parent.position = q.node.position;
							successors.push_back(currentSuccessor);
						}
					}
				}
			}

			//d) for each successor
			for (int successorsCnt = 0; successorsCnt < (int)successors.size(); ++successorsCnt) {

				//If successor is walkable
				if (maze[successors[successorsCnt].node.position.x][successors[successorsCnt].node.position.y] == false) {

					/*i) if successor is the goal, stop search
					successor.g = q.g + distance between
					successor and q
					successor.h = distance from goal to
					successor(This can be done using many
						ways, we will discuss three heuristics -
						Manhattan, Diagonaland Euclidean
						Heuristics)

					successor.f = successor.g + successor.h*/
					if (compareXYStructs(successors[successorsCnt].node.position, end) == true) {
						stopSearch = true;

						//Add end node to closed list
						closedList.push_back(successors[successorsCnt]);

					}
					int distanceBetweenSuccessorAndQ = -1;
					if (successors[successorsCnt].node.position.x == q.node.position.x || successors[successorsCnt].node.position.y == q.node.position.y) {
						distanceBetweenSuccessorAndQ = 10;
					}
					else {
						distanceBetweenSuccessorAndQ = 14;
					}
					successors[successorsCnt].node.g = q.node.g + distanceBetweenSuccessorAndQ;
					successors[successorsCnt].node.h = getHeuristic(successors[successorsCnt].node.position, end) * 10;
					successors[successorsCnt].node.f = successors[successorsCnt].node.g + successors[successorsCnt].node.h;

					/*ii) if a node with the same position as
					successor is in the OPEN list which has a
					lower f than successor, skip this successor*/
					bool skipSuccessor = false;
					for (int openListCnt = 0; openListCnt < (int)openList.size(); ++openListCnt) {
						if (compareXYStructs(openList[openListCnt].node.position, successors[successorsCnt].node.position) == true && openList[openListCnt].node.f < successors[successorsCnt].node.f) {
							skipSuccessor = true;
							break;
						}
					}
					if (skipSuccessor == true) {
						continue;
					}

					/*iii) if a node with the same position as
					successor  is in the CLOSED list which has
					a lower f than successor, skip this successor
					otherwise, add  the node to the open list
					end(for loop)*/
					skipSuccessor = false;
					for (int closedListCnt = 0; closedListCnt < (int)closedList.size(); ++closedListCnt) {
						if (compareXYStructs(closedList[closedListCnt].node.position, successors[successorsCnt].node.position) == true && closedList[closedListCnt].node.f < successors[successorsCnt].node.f) {
							skipSuccessor = true;
							break;
						}
					}
					if (skipSuccessor == true) {
						continue;
					}
					else {
						openList.push_back(successors[successorsCnt]);
					}
				}

			}

			/*e) push q on the closed list
			end(while loop)*/
			closedList.push_back(q);

		}
	}

	//Path found
	if (stopSearch == true) {

		//Get end node index from closed list and add node position to path
		int currentNodeIndex = -1;
		for (int closedListCnt = 0; closedListCnt < (int)closedList.size(); ++closedListCnt) {
			if (compareXYStructs(closedList[closedListCnt].node.position, end) == true) {
				currentNodeIndex = closedListCnt;
				break;
			}
		}
		path.push_back(closedList[currentNodeIndex].node.position);

		//Get path from closed list, starting from end node to start node, using parents to trace back
		while (compareXYStructs(closedList[currentNodeIndex].node.position, start) == false) {

			//Get next node using parent and add it to path
			for (int closedListCnt = 0; closedListCnt < (int)closedList.size(); ++closedListCnt) {
				if (compareXYStructs(closedList[closedListCnt].node.position, closedList[currentNodeIndex].parent.position) == true) {
					currentNodeIndex = closedListCnt;
					path.push_back(closedList[currentNodeIndex].node.position);
					break;
				}
			}

		}

		//Reverse path so it starts with start node and ends with end node
		reverse(path.begin(), path.end());

	}

	return path;
}