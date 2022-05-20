#pragma once
#ifndef __A__STAR__SEARCH
#define __A__STAR__SEARCH

#include <unordered_map>
#include <functional>
#include <algorithm>
#include <queue>
#include <FootstepPlannerLJH/parameters.h>
#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}
// // it's not that convicing, need to rethink!
// // reason why not using multimap:the actual type of value is pair<const keytype, datatype>,
// // so the keytype is not modifiable, but in a_star_search we may need to update the key(cost priority),
// // in this case, we use priority_queue and pair as datatype, the key and data in pair are both modifiable. 

// we eliminate the check for a node being in the frontier with a higher cost.
// By not checking, we end up with duplicate elements in the frontier with several costs.
// That's because we can make the code simpler and faster by using a PriorityQueue that 
// not support the "Decrease Key" operation.
// otherwise for the same value you need to delete the key1-value1 first and insert
// another key2-value1. use map instead.

_FOOTSTEP_PLANNER_BEGIN

// define (wrapper of) the PriorityQueue used by a_star_search
// it's a small top heap, cause we want to take the node with lowest cost among frontier


// only for the pair type to compare
template<typename PQElement>
struct greater
{
    bool operator()(const PQElement& _Left, const PQElement& _Right) const
    {
        return (_Left.first>_Right.first);
    }
};


template<typename T,typename priority_t>
class PriorityQueue
{
public:
    typedef std::pair<priority_t,T> PQElement;
    std::priority_queue<PQElement,std::vector<PQElement>,greater<PQElement> > elements; 
    inline bool empty() const {return elements.empty();};
    inline void put(T item, priority_t priority){elements.emplace(priority,item);};
    
    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};


template<typename Graph>
void a_star_search(
    Graph graph,
    typename Graph::Location start,
    typename Graph::Location goal ,
    std::function<typename Graph::cost_t(typename Graph::Location a,
                                         typename Graph::Location b)> heuristic,
    std::unordered_map<typename Graph::Location,typename Graph::Location>& came_from,
    std::unordered_map<typename Graph::Location,typename Graph::cost_t>& cost_so_far)                            
{
    typedef typename Graph::Location Location;
    typedef typename Graph::cost_t cost_t;
    PriorityQueue<Location,cost_t> frontier;
    std::vector<Location> neighbors;
    frontier.put(start,cost_t(0));

    came_from[start] = start;
    cost_so_far[start] = cost_t(0) + heuristic(start, goal);

    while(!frontier.empty())
    {
        Location current = frontier.get();

        if(current == goal) {break;}

        graph.getneighbors(current, neighbors);
        for(Location next : neighbors)
        {
            cost_t new_cost = cost_so_far[current] + graph.edgecost(current, next);
            if(cost_so_far.find(next) == cost_so_far.end() 
                || new_cost < cost_so_far[next] )
            {
                cost_so_far[next] = new_cost;
                cost_t priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
}

template<typename Location>
std::vector<Location> reconstruct_path(
    Location start,Location goal,
    std::unordered_map<Location,Location> came_from)
{
    std::vector<Location> path;
    // start point of current need to modified if no compelete path were found,
    // in this way start with the lowest-cost node in the queue!
    Location current = goal;
    while(current!=start) 
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
    return path;
}



_FOOTSTEP_PLANNER_END

#endif