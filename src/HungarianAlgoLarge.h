/*
 * HungarianAlgo.h
 *
 *  Created on: Jun 21, 2014
 *      Author: naor
 */

#ifndef HUNGARIANALGO_H_
#define HUNGARIANALGO_H_

#include "common.h"

// -- Start functions for opeations on 64 bit values --
inline bool is_zero(const uint64 (&value)[DATA_64WORDS_SIZE]){
  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++)
    if (value[i] != 0)
      return false;
  return true;
}

inline bool is_equal(const uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){
  if (valueA[DATA_64WORDS_SIZE-1] != valueB[DATA_64WORDS_SIZE-1]) {
    if (is_zero(valueA) && is_zero(valueB)) {
      return true;
      }
    return false;
  }

  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++) {
    if (valueA[i] != valueB[i])
      return false;
  }
  return true;
}

inline bool is_less_than(const uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){
  bool negA = valueA[DATA_64WORDS_SIZE-1];
  bool negB = valueB[DATA_64WORDS_SIZE-1];
  if (!negA && negB) {
    return false;
  } else if (negA && !negB) {
    return true;
  }

  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++) {
    if (valueA[i] == valueB[i])
      continue;
    return valueA[i] < valueB[i] ^ negA;
  }
  return false;
}

inline void set_max(uint64 (&value)[DATA_64WORDS_SIZE]){
  value[DATA_64WORDS_SIZE-1] = 0;
  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++)
    value[i] = (uint64) -1;
}

inline void set_zero(uint64 (&value)[DATA_64WORDS_SIZE]){
  value[DATA_64WORDS_SIZE-1] = 0;

  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++)
    value[i] = 0;
}

inline void set_bit(uint64 (&value)[DATA_64WORDS_SIZE], long bit){
  set_zero(value);
  uint64 val = ((uint64) 1) << std::abs(bit)%64;
  value[DATA_64WORDS_SIZE-1-std::abs(bit)/64-1] = val;
  if (bit < 0) {
    value[DATA_64WORDS_SIZE-1] = 1;
  }
}

inline void assign_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE])
{
  for (int i = 0; i < DATA_64WORDS_SIZE; i++)
    valueA[i] = valueB[i];
}

void subtract_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]);

inline void add_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){
  //std::cout << "add_value\n";
  if (valueA[DATA_64WORDS_SIZE-1] != valueB[DATA_64WORDS_SIZE-1]) {
    uint64 temp[DATA_64WORDS_SIZE];
    assign_value(temp, valueB);
    temp[DATA_64WORDS_SIZE-1] = !temp[DATA_64WORDS_SIZE-1];
    subtract_value(valueA, temp);
    return;
  }

  int carry = 0;
  for (int i = DATA_64WORDS_SIZE-2; i >= 0; i--) {
    valueA[i] += valueB[i] + carry;
    bool overflow = valueA[i] == valueB[i] && carry == 1;
    if (valueA[i] < valueB[i] || overflow)
      carry = 1;
    else
      carry = 0;
  }
}

inline void subtract_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){
  //std::cout << "subtract_value\n";
  if (is_equal(valueA, valueB)) {
    set_zero(valueA);
    return;
  }

  bool negA = valueA[DATA_64WORDS_SIZE-1];
  bool negB = valueB[DATA_64WORDS_SIZE-1];

  if (negA != negB) {
    uint64 temp[DATA_64WORDS_SIZE];
    assign_value(temp, valueB);
    temp[DATA_64WORDS_SIZE-1] = !temp[DATA_64WORDS_SIZE-1];
    add_value(valueA, temp);
    return;
  }

  if ((!negA && is_less_than(valueA, valueB)) ||
      (negA && is_less_than(valueB, valueA))) {
    uint64 temp[DATA_64WORDS_SIZE];
    assign_value(temp, valueB);
    subtract_value(temp, valueA);
    temp[DATA_64WORDS_SIZE-1] = !temp[DATA_64WORDS_SIZE-1];
    assign_value(valueA, temp);
    return;
  }


  uint64 negValueB[DATA_64WORDS_SIZE];
  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++)
    negValueB[i] = ~valueB[i];
  negValueB[DATA_64WORDS_SIZE-1] = 0;
  uint64 one[DATA_64WORDS_SIZE];
  set_bit(one, 0);
  one[DATA_64WORDS_SIZE-1] = 0;
  add_value(negValueB, one);
  uint signA = valueA[DATA_64WORDS_SIZE-1];
  valueA[DATA_64WORDS_SIZE-1] = 0;
  add_value(valueA, negValueB);
  valueA[DATA_64WORDS_SIZE-1] = signA;
}
// -- End functions for operations on 64 bit values --

// Begin implementation of hungarian algorithm (large data)

long newdist[MAXN][MAXN];

//Code for O(n^3) Hungarian algorithm from http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

//double cost[N][N];          //cost matrix
int n_large, max_match_large;        //n workers and n jobs
uint64 lx_large[N][DATA_64WORDS_SIZE], ly_large[N][DATA_64WORDS_SIZE];        //labels of X and Y parts
int xy_large[N];               //xy_large[x] - vertex that is matched with x,
int yx_large[N];               //yx_large[y] - vertex that is matched with y
bool S_large[N], T_large[N];         //sets S and T in algorithm
uint64 slack_large[N][DATA_64WORDS_SIZE];            //as in the algorithm description
int slackx_large[N];           //slackx[y] such a vertex, that
                         // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
int prev_large[N];             //array for memorizing alternating paths

void init_labels_large()
{
  memset(lx_large, 0, sizeof(lx_large));
  memset(ly_large, 0, sizeof(ly_large));
    for (int x = 0; x < n_large; x++)
      for (int y = 0; y < n_large; y++) {
	uint64 cost[DATA_64WORDS_SIZE];
	set_bit(cost, newdist[x][y]);
	  if (is_less_than(lx_large[x], cost)) {
	    assign_value(lx_large[x], cost);
	  }
	  //lx_large[x] = std::max(lx_large[x], newdist[x][y]);
      }
}

void update_labels_large()
{
  int x, y;
  uint64 delta[DATA_64WORDS_SIZE];
  set_max(delta);             //init delta as infinity
    for (y = 0; y < n_large; y++)            //calculate delta using slack
      if (!T_large[y]) {
	if (is_less_than(slack_large[y], delta)) {
	  assign_value(delta, slack_large[y]);
	  //delta = std::min(delta, slack_large[y]);
	}
      }
    for (x = 0; x < n_large; x++)            //update X labels
      if (S_large[x]) {
	subtract_value(lx_large[x], delta);
	//lx_large[x] -= delta;
      }
    for (y = 0; y < n_large; y++)            //update Y labels
      if (T_large[y]) {
	add_value(ly_large[y], delta);
	//ly_large[y] += delta;
      }
    for (y = 0; y < n_large; y++)            //update slack array
      if (!T_large[y]) {
	subtract_value(slack_large[y], delta);
	//slack_large[y] -= delta;
      }
}

void add_to_tree_large(int x, int prevx)
//x - current vertex,prevx - vertex from X before x in the alternating path,
//so we add edges (prevx, xy_large[x]), (xy_large[x], x)
{
    S_large[x] = true;                    //add x to S
    prev_large[x] = prevx;                //we need this when augmenting
    for (int y = 0; y < n_large; y++) {    //update slacks, because we add new vertex to S
      uint64 temp[DATA_64WORDS_SIZE];
      assign_value(temp, lx_large[x]);
      add_value(temp, ly_large[y]);
      uint64 cost[DATA_64WORDS_SIZE];
      set_bit(cost,newdist[x][y]);
      subtract_value(temp, cost);
      if (is_less_than(temp, slack_large[y]))
      //if (lx_large[x] + ly_large[y] - newdist[x][y] < slack_large[y])
        {
	  assign_value(slack_large[y], temp);
	  //slack_large[y] = lx_large[x] + ly_large[y] - newdist[x][y];
	  slackx_large[y] = x;
        }
    }
}

bool augment_large()                         //main function of the algorithm
{
    if (max_match_large == n_large) return true;        //check wether matching is already perfect
    int x, y, root;                    //just counters and root vertex
    int q[N], wr = 0, rd = 0;          //q - queue for bfs, wr,rd - write and read
                                       //pos in queue
    memset(S_large, false, sizeof(S_large));       //init set S
    memset(T_large, false, sizeof(T_large));       //init set T
    memset(prev_large, -1, sizeof(prev_large));    //init set prev - for the alternating tree
    for (x = 0; x < n_large; x++)            //finding root of the tree
        if (xy_large[x] == -1)
        {
            q[wr++] = root = x;
            prev_large[x] = -2;
            S_large[x] = true;
            break;
        }

    for (y = 0; y < n_large; y++)            //initializing slack array
    {
      //slack_large[y] = lx_large[root] + ly_large[y] - newdist[root][y];
      assign_value(slack_large[y], lx_large[root]);
      add_value(slack_large[y], ly_large[y]);
      uint64 cost[DATA_64WORDS_SIZE];
      set_bit(cost, newdist[root][y]);
      subtract_value(slack_large[y], cost);
        slackx_large[y] = root;
    }

//second part of augment_large() function
    while (true)                                                        //main cycle
    {
        while (rd < wr)                                                 //building tree with bfs cycle
        {
            x = q[rd++];                                                //current vertex from X part
            for (y = 0; y < n_large; y++) {                                     //iterate through all edges in equality graph
	      uint64 temp[DATA_64WORDS_SIZE];
	      assign_value(temp, lx_large[x]);
	      add_value(temp, ly_large[y]);
	      uint64 cost[DATA_64WORDS_SIZE];
	      set_bit(cost, newdist[x][y]);
	      if (is_equal(cost, temp) && !T_large[y])
	      //if (newdist[x][y] == lx_large[x] + ly_large[y] &&  !T_large[y])
                {
                    if (yx_large[y] == -1) break;                             //an exposed vertex in Y found, so
                                                                        //augmenting path exists!
                    T_large[y] = true;                                        //else just add y to T,
                    q[wr++] = yx_large[y];                                    //add vertex yx_large[y], which is matched
                                                                        //with y, to the queue
                    add_to_tree_large(yx_large[y], x);                              //add edges (x,y) and (y,yx_large[y]) to the tree
                }
	    }
            if (y < n_large) break;                                           //augmenting path found!
        }
        if (y < n_large) break;                                               //augmenting path found!

        update_labels_large();                                                //augmenting path not found, so improve labeling
        wr = rd = 0;
        for (y = 0; y < n_large; y++)
        //in this cycle we add edges that were added to the equality graph as a
        //result of improving the labeling, we add edge (slackx[y], y) to the tree if
        //and only if !T[y] &&  slack[y] == 0, also with this edge we add another one
        //(y, yx_large[y]) or augment the matching, if y was exposed
	  if (!T_large[y] &&  is_zero(slack_large[y]))
            //if (!T_large[y] &&  slack_large[y] == 0)
            {
                if (yx_large[y] == -1)                                        //exposed vertex in Y found - augmenting path exists!
                {
                    x = slackx_large[y];
                    break;
                }
                else
                {
                    T_large[y] = true;                                        //else just add y to T,
                    if (!S_large[yx_large[y]])
                    {
                        q[wr++] = yx_large[y];                                //add vertex yx_large[y], which is matched with
                                                                        //y, to the queue
                        add_to_tree_large(yx_large[y], slackx_large[y]);                  //and add edges (x,y) and (y,
                                                                        //yx_large[y]) to the tree
                    }
                }
            }
        if (y < n_large) break;                                               //augmenting path found!
    }

    if (y < n_large)                                                          //we found augmenting path!
    {
        max_match_large++;                                                    //increment matching
        //in this cycle we inverse edges along augmenting path
        for (int cx = x, cy = y, ty; cx != -2; cx = prev_large[cx], cy = ty)
        {
            ty = xy_large[cx];
            yx_large[cy] = cx;
            xy_large[cx] = cy;
        }
	return false;
        //augment_large();                                                      //recall function, go to step 1 of the algorithm
    }
    return true;
}//end of augment_large() function

void hungarian_large()
{
  //int ret = 0;                      //weight of the optimal matching
    n_large = N;
    max_match_large = 0;                    //number of vertices in current matching
    memset(xy_large, -1, sizeof(xy_large));
    memset(yx_large, -1, sizeof(yx_large));

    init_labels_large();                    //step 0
    while (!augment_large()){;}                        //steps 1-3
    //for (int x = 0; x < n; x++)       //forming answer there
    //    ret += newdist[x][xy_large[x]];
    //return ret;
}



#endif /* HUNGARIANALGO_H_ */
