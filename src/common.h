/*
 * common.h
 *
 *  Created on: Jun 21, 2014
 *      Author: naor
 */

#ifndef COMMON_H_
#define COMMON_H_

#define NUM_GOALS 50
#define N NUM_GOALS
#define MAXN N
#define DATA_64WORDS_SIZE (MAXN*MAXN-1)/64+2

typedef uint64_t uint64;

// remove an element from a vector by value.
#define VECREMOVE(vec, v) (vec).erase(  \
              std::remove((vec).begin(), (vec).end(), (v)), (vec).end())


typedef std::pair<double, double> Point;

// Edge looks like (dist, (left node, right node))
typedef std::pair<double, std::pair<int, int> > Edge;

struct Test{
  std::vector<Point> starts;
  std::vector<Point> targets;
};

inline double getdist(const Point &a, const Point &b){
  return hypot(a.first-b.first, a.second-b.second);
}

inline bool edgeVectorLessThan(const std::vector<Edge> &a, const std::vector<Edge> &b) {
  const double ERROR_THRESH = .00005;
  for (int i = 0; i < a.size(); i++) {
    if (std::abs(a[i].first-b[i].first) < ERROR_THRESH) {
      continue;
    }
    return a[i] < b[i];
  }
  return false;
}

#endif /* COMMON_H_ */
