/*
 * mmdr.h
 *
 *  Created on: Jun 21, 2014
 *      Author: naor
 */

#ifndef MMDR_H_
#define MMDR_H_

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
#include "HungarianAlgoLarge.h"

/*
std::vector<Edge> mmdr_dyna(Test t){
  int n = t.starts.size();
  std::vector<std::vector<Edge> > dists;
  std::vector<std::vector<Edge> > best;
  std::vector<int> prev;


  std::vector<std::vector<int> > sets;

  dists.resize(n);
  for(int i = 0; i < n; i++){
    dists[i].resize(n);
    for(int j = 0; j < n; j++) {
      dists[i][j] = std::make_pair(-getdist(t.starts[i], t.targets[j]),
				   std::make_pair(i, j));
    }
  }


  sets.resize(n+1);

  for(int i = 0; i < 1 << n; i++){
    int count = 0;
    for(int j = 0; j < n; j++)
      if (i & (1 << j))
        count++;
    sets[count].push_back(i);
  }

  best.resize(1<<n);
  prev.resize(1<<n);

  for(int k = 0; k < n; k++){
    for(int i = 0; i < n; i++){
      for(int j = 0; j < sets[k].size(); j++){
        if (sets[k][j] & (1 << i))
          continue;
        int s2 = sets[k][j] | (1 << i);
        std::vector<Edge> tmp = best[sets[k][j]];
        tmp.insert(std::lower_bound(tmp.begin(), tmp.end(), dists[i][k]),
                   dists[i][k]);
        if (best[s2].size() == 0 ||
            edgeVectorLessThan(best[s2], tmp))
          best[s2] = tmp;
      }
    }
  }

  std::vector<Edge> answer = best[sets[n][0]];

  return answer;
}
 */

std::vector<Edge> mmdr_n5(Test t){
  int n = t.starts.size();
  int m = t.targets.size();
  ROS_INFO("num starts %d and num targets %d", n, m);

  std::vector<Edge> edges;
  std::vector<Edge> answer;

  ROS_INFO("Create edges");
  // Create edges from all pairwise distances
  for(int i = 0; i < N; i++) {
    for(int j = 0; j < N; j++) {
    	int dist = 1000;
    	if ((i < n) && (j < m)) {
    		dist = getdist(t.starts[i], t.targets[j]);
    	}
    	edges.push_back(std::make_pair(dist,std::make_pair(i, j)));
    }
  }
  ROS_INFO("sort'em");
  std::sort(edges.begin(), edges.end());

  long lasti = -1;
  for(int i = 0; i < edges.size(); i++) {
    if (!i || edges[i].first != edges[lasti].first)
      lasti = i;

    newdist[edges[i].second.first][edges[i].second.second] = -lasti;
  }

  ROS_INFO("Run the hungarian algorithm");
  hungarian_large();

  ROS_INFO("Collect the answers");
  for(int i = 0; i < n; i++){
    //printf("Got: %d %d %lf\n", i, h[i], getdist(t.starts[i], t.targets[h[i]]));
    answer.push_back(std::make_pair(getdist(t.starts[i], t.targets[xy_large[i]]),
                                    std::make_pair(i, xy_large[i])));
  }

  // answer now contains the correct answer.

  return answer;
}


#endif /* MMDR_H_ */
