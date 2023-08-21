// Copyright 2022 Johnson & Johnson
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_TRANSITIONS_HPP_
#define NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_TRANSITIONS_HPP_

#include <climits>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;

namespace nexus::lifecycle_manager {

/* \brief This class will generate a graph with all the nodes and edges
 * related to the ROS 2 lifecycle managed nodes.
 * It will calculate the transition from a given start vertex to an end vertex.
 */
class LifecycleTransitions
{
  int NO_PARENT = -1;

public:
  LifecycleTransitions()
  {
    // https://design.ros2.org/articles/node_lifecycle.html
    this->_adjacencyMatrix =
    { { 0, 0, 0, 0, 1},
      { 0, 0, 1, 0, 1},
      { 0, 1, 0, 1, 1},
      { 0, 0, 1, 0, 1},
      { 1, 0, 0, 0, 0}};
  }

  // \brief Return the transitions from vertex A to vertex B
  std::vector<int> dijkstra(int startVertex, int endVertex)
  {
    int nVertices = this->_adjacencyMatrix[0].size();

    // shortestDistances[i] will hold the
    // shortest distance from src to i
    std::vector<int> shortestDistances(nVertices);

    // added[i] will true if vertex i is
    // included / in shortest path tree
    // or shortest distance from src to
    // i is finalized
    std::vector<bool> added(nVertices);

    // Initialize all distances as
    // INFINITE and added[] as false
    for (int vertexIndex = 0; vertexIndex < nVertices;
      vertexIndex++)
    {
      shortestDistances[vertexIndex] = INT_MAX;
      added[vertexIndex] = false;
    }

    // Distance of source vertex from
    // itself is always 0
    shortestDistances[startVertex] = 0;

    // Parent array to store shortest
    // path tree
    std::vector<int> parents(nVertices);

    // The starting vertex does not
    // have a parent
    parents[startVertex] = NO_PARENT;

    // Find shortest path for all
    // vertices
    for (int i = 1; i < nVertices; i++)
    {

      // Pick the minimum distance vertex
      // from the set of vertices not yet
      // processed. nearestVertex is
      // always equal to startNode in
      // first iteration.
      int nearestVertex = -1;
      int shortestDistance = INT_MAX;
      for (int vertexIndex = 0; vertexIndex < nVertices;
        vertexIndex++)
      {
        if (!added[vertexIndex]
          && shortestDistances[vertexIndex]
          < shortestDistance)
        {
          nearestVertex = vertexIndex;
          shortestDistance =
            shortestDistances[vertexIndex];
        }
      }

      // Mark the picked vertex as
      // processed
      added[nearestVertex] = true;

      // Update dist value of the
      // adjacent vertices of the
      // picked vertex.
      for (int vertexIndex = 0; vertexIndex < nVertices;
        vertexIndex++)
      {
        int edgeDistance =
          this->_adjacencyMatrix[nearestVertex]
          [vertexIndex];

        if (edgeDistance > 0
          && ((shortestDistance + edgeDistance)
          < shortestDistances[vertexIndex]))
        {
          parents[vertexIndex] = nearestVertex;
          shortestDistances[vertexIndex] =
            shortestDistance + edgeDistance;
        }
      }
    }
    std::vector<int> transitionNodes;
    // Calculate path from start vertex to end vertex
    getPath(endVertex, parents, transitionNodes);

    // Once we have the state, we need to get the transitions
    std::vector<int> solution;
    if (transitionNodes.size() > 1)
    {
      for (unsigned long i = 0; i < transitionNodes.size() - 1; i++)
      {
        if (transitionNodes[i] == State::PRIMARY_STATE_UNCONFIGURED)
        {
          switch (transitionNodes[i + 1])
          {
            case State::PRIMARY_STATE_INACTIVE:
              solution.push_back(Transition::TRANSITION_CONFIGURE);
              break;
            case State::PRIMARY_STATE_FINALIZED:
              solution.push_back(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
              break;
          }
        }
        if (transitionNodes[i] == State::PRIMARY_STATE_INACTIVE)
        {
          switch (transitionNodes[i + 1])
          {
            case State::PRIMARY_STATE_UNCONFIGURED:
              solution.push_back(Transition::TRANSITION_CLEANUP);
              break;
            case State::PRIMARY_STATE_FINALIZED:
              solution.push_back(Transition::TRANSITION_INACTIVE_SHUTDOWN);
              break;
            case State::PRIMARY_STATE_ACTIVE:
              solution.push_back(Transition::TRANSITION_ACTIVATE);
              break;
          }
        }
        if (transitionNodes[i] == State::PRIMARY_STATE_ACTIVE)
        {
          switch (transitionNodes[i + 1])
          {
            case State::PRIMARY_STATE_INACTIVE:
              solution.push_back(Transition::TRANSITION_DEACTIVATE);
              break;
            case State::PRIMARY_STATE_FINALIZED:
              solution.push_back(Transition::TRANSITION_ACTIVE_SHUTDOWN);
              break;
          }
        }
      }
    }

    return solution;
  }

  // \brief Get the path
  void getPath(int currentVertex, std::vector<int> parents,
    std::vector<int>& solution)
  {
    if (currentVertex == NO_PARENT)
    {
      return;
    }
    getPath(parents[currentVertex], parents, solution);
    solution.push_back(currentVertex);
  }

  // \brief Check if the node is already at the goal state
  bool atGoalState(int state, int transition)
  {
    bool result = false;
    if (state == State::PRIMARY_STATE_UNCONFIGURED && (
        transition == Transition::TRANSITION_CLEANUP))
    {
      result = true;
    }
    else if (state == State::PRIMARY_STATE_INACTIVE && (
        transition == Transition::TRANSITION_CONFIGURE ||
        transition == Transition::TRANSITION_DEACTIVATE))
    {
      result = true;
    }
    else if (state == State::PRIMARY_STATE_ACTIVE && (
        transition == Transition::TRANSITION_ACTIVATE))
    {
      result = true;
    }
    else if (state == State::PRIMARY_STATE_FINALIZED && (
        transition == Transition::TRANSITION_UNCONFIGURED_SHUTDOWN ||
        transition == Transition::TRANSITION_INACTIVE_SHUTDOWN ||
        transition == Transition::TRANSITION_ACTIVE_SHUTDOWN))
    {
      result = true;
    }

    return result;
  }

private:
  std::vector<std::vector<int>> _adjacencyMatrix;
};
}

#endif
