/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"

#include "random.h"
using namespace std;

/* Function: shortestPath
 * 
 * Finds the shortest path between the locations given by start and end in the
 * specified world.	 The cost of moving from one edge to the next is specified
 * by the given cost function.	The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited.	If no path is found, this function should report an
 * error.
 *
 * In Part Two of this assignment, you will need to add an additional parameter
 * to this function that represents the heuristic to use while performing the
 * search.  Make sure to update both this implementation prototype and the
 * function prototype in Trailblazer.h.
 */
Vector<Loc>
shortestPath(Loc start,
             Loc end,
             Grid<double>& world,
             double costFn(Loc from, Loc to, Grid<double>& world),
             double heuristic(Loc st, Loc end, Grid<double> &world)) {

    TrailblazerPQueue<Loc> pq;
    pq.enqueue(start, heuristic(start, end, world));

    Vector<Loc> res;
    Map<Loc, double> routes;
    routes[start] = 0;
    colorCell(world, start, YELLOW);
    Map<Loc, Loc> parents;

    int dir[] = { -1, 0, 1 };
    bool found = false;
    while (!pq.isEmpty()) {
        Loc currNode = pq.dequeueMin();
        colorCell(world, currNode, GREEN);
        if (currNode == end) {
            found = true;
            break;
        }

        for (int i : dir) {
            for (int j : dir) {
                if (i == 0 && j == 0) continue;

                int nextR = currNode.row + i;
                int nextC = currNode.col + j;
                if (!world.inBounds(nextR, nextC)) continue; // validate next move

                Loc nextNode = makeLoc(nextR, nextC);
                double cost = routes[currNode] + costFn(currNode, nextNode, world);

                if (!routes.containsKey(nextNode)) {
                    routes[nextNode] = cost;
                    parents[nextNode] = currNode;
                    pq.enqueue(nextNode, cost + heuristic(nextNode, end, world));
                    colorCell(world, nextNode, YELLOW);
                } else if (routes[nextNode] > cost) {
                    routes[nextNode] = cost; // update min cost
                    parents[nextNode] = currNode;
                    pq.decreaseKey(nextNode, cost + heuristic(nextNode, end, world));
                }
            }
        }
    }

    if (!found) {
        error("ERROR: Couldn't find the path");
    }

    // reconstruct the path
    Loc currNode = end;
    while (currNode != start) {
        res.add(currNode);
        currNode = parents[currNode];
    }
    res.add(start);
    reverse(res.begin(), res.end());

    return res;
}

// generating the maze using kruskal algo
Set<Edge> createMaze(int numRows, int numCols) {
    Set<Edge> res;
    TrailblazerPQueue<Edge> pq;
    Map<Loc, Set<Loc>> forest;
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            Loc newNode = makeLoc(i, j);
            Set<Loc> ss;
            ss.insert(newNode);
            forest[newNode] = ss;
            if (i+1 < numRows) {
                Loc neighbor = makeLoc(i + 1, j);
                Edge edge = makeEdge(newNode, neighbor);
                pq.enqueue(edge, randomReal(0, 1));
            }
            if (j + 1 < numCols) {
                Loc neighbor = makeLoc(i, j + 1);
                Edge edge = makeEdge(newNode, neighbor);
                pq.enqueue(edge, randomReal(0, 1));
            }
        }
    }

    while (!pq.isEmpty()) {
        Edge curr = pq.dequeueMin();
        if (forest[curr.start] == forest[curr.end]) continue;

        Set<Loc> combinedTree = forest[curr.start] + forest[curr.end];
        for (Loc l : combinedTree)
            forest[l] = combinedTree;
        res.insert(curr);
    }

    return res;
}
