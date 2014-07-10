#include<iostream>
#include<fstream>
#include<cstdio>
#include<cmath>
#include<string>
#include<algorithm>
#include<vector>
#include<climits>
#include<float.h>
#include<string.h>

#include <boost/cstdint.hpp>

#include <ros/ros.h>

#include "common.h"
#include "HungarianAlgo.h"

// We consider a bipartite graph with n nodes on the left and right.

// Global variables used in the floodfill.
// foo[0][i] corresponds to i'th node on left; foo[1][i] for the right.

std::vector<std::vector<int> > out[2];  //adjacency list for each node
std::vector<bool> visited[2];           //reached in floodfill
std::vector<int> back[2];               //way to reach it
std::vector<int> used;                  //whether a left node is used
//right nodes are used if and only if out[1][j].size() == 1.

// Floodfill from a node.
//  x in {0, 1}: left or right side
//  y in {0, ..., n-1}: node on side x
//  prev in {-1, 0, ..., n-1}: node on side 1-x that we came in on
//                             (-1 for unassigned node on left)
// Returns:
//  If it reaches an unassigned right node, the index of this node.
//  Otherwise, -1.
int flood(int x, int y, int prev){
  visited[x][y] = 1;
  back[x][y] = prev;
  if (x == 1 && out[x][y].size() == 0) //reached an unassigned right node!
    return y;

  for(int j = 0; j < out[x][y].size(); j++){
    if (!visited[1-x][out[x][y][j]]){
      int tmp = flood(1-x, out[x][y][j], y);
      if (tmp != -1) //Flood reached the end
        return tmp;
    }
  }
  return -1;
}

// starting at node (x, y), follow the back pointers and reverse each edge.
// Return the last node reached (i.e., the newly assigned left node)
inline int reverse(int x, int y){
  while (true) {
    int prev = back[x][y];
    if (prev == -1)       // Reached the unassigned node on the left
      break;
    out[x][y].push_back(prev);
    VECREMOVE(out[1-x][prev], y);
    x = 1-x; y = prev;
  }
  return y;
}

// Set visited to 0 and flood from unassigned left nodes.
inline void reset_flooding(int n){
  for(int i = 0; i < 2; i++)
    std::fill(visited[i].begin(), visited[i].end(), 0);

  for(int i = 0; i < n; i++)
    if(!used[i])
      flood(0, i, -1);
}

/*
  Add edges in order until k nodes can be matched.

  edges is a sorted vector of (dist, (left, right))

  Returns the index of the last edge added; this edge must appear.
 */
int getMinimalMaxEdgeInPerfectMatching(std::vector<Edge> edges, int n, int k){
  for(int i = 0; i < 2; i++) { //Clear the graph
    out[i].clear();
    out[i].resize(n);
  }
  std::fill(used.begin(), used.end(), 0);
  reset_flooding(n);

  int answer;
  for(answer = 0; answer < edges.size(); answer++){
    std::pair<int, int> e = edges[answer].second;
    out[0][e.first].push_back(e.second);
    //printf("Added edge: %d %d\n", e.first, e.second);
    if(visited[0][e.first] && !visited[1][e.second]) {
      int ans = flood(1, e.second, e.first);
      if (ans != -1){  //We made it to the end!
        if (--k == 0)
          break;
        int start = reverse(1, ans);
        used[start] = 1;
        reset_flooding(n);
      }
    }
  }
  // We must use edges[answer] to push k flow with minimal max edge.
  return answer;
}

std::vector<Edge> mmd_msd2(Test t){
  int n = t.starts.size();
  int m = t.targets.size();
  std::vector<Edge> edges;
  std::vector<Edge> answer;

  ROS_INFO("Create edges");
  // Create edges from all pairwise distances squared
  for(int i = 0; i < N; i++) {
    for(int j = 0; j < N; j++) {
		int dist = 1000;
		if ((i < n) && (j < m)) {
			dist = getdist(t.starts[i], t.targets[j]);
		}
		edges.push_back(std::make_pair(pow(dist,2),std::make_pair(i, j)));
    }
  }
  ROS_INFO("sort'em");
  std::sort(edges.begin(), edges.end());

  // Make the global variables in the floodfill have the right sizes
  for(int i = 0; i < 2; i++) {
    visited[i].resize(N);
    back[i].resize(N);
  }
  used.resize(N);

  // Call getMinimalMaxEdgeInPerfectMatching to get minimum maximal edge in a perfect mathcing.
  
  int choice = getMinimalMaxEdgeInPerfectMatching(edges, N, n);
  ROS_INFO("Get Maximal Edge Value %f",edges[choice].first);
  double max_edge_value = edges[choice].first;

  // Now remove (make very large) all edges that are greater than max_edge_value
  for(int i = 0; i < edges.size(); i++){
    if (edges[i].first <= max_edge_value) 
      cost[edges[i].second.first][edges[i].second.second] = edges[i].first;
    else
      cost[edges[i].second.first][edges[i].second.second] = max_edge_value*N+1;
  }

  ROS_INFO("Run the hungarian algorithm");
  hungarian();

  ROS_INFO("Collect the answers");
  for(int i = 0; i < n; i++){
    //printf("Got: %d %d %lf\n", i, h2[i], getdist(t.starts[i], t.targets[h2[i]]));
    answer.push_back(std::make_pair(getdist(t.starts[i], t.targets[xy[i]]),
                                    std::make_pair(i, xy[i])));
  }

  // answer now contains the correct answer.

  return answer;
}
