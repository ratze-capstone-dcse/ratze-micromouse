/*
 * Copyright 2024 Pedro Fontoura Zawadniak
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "micromouse/vector2int.hpp"
#include "micromouse/direction.hpp"
#include "micromouse/maze.hpp"
#include <queue>
#include <random>
#include <unordered_map>
#include <fstream>
#include <array>
#include <algorithm>

// The states that a robot can be in.
struct State
{
    Vector2Int position;
    Direction orientation;
};

bool operator==(const State& lhs, const State& rhs);

enum class StateTransition
{
	forward, turnLeft, turnRight, invalid
};

StateTransition stateFromTo(State from, State to);

// Q-Learning Solver for micromouse maze navigation
class QLearningMazeRunner
{
public:
    QLearningMazeRunner();
    void updateQValue(State currentState, StateTransition action, State nextState, double reward, bool isGoalReached);
    StateTransition chooseAction(State currentState, const IMaze& maze, bool explorationMode = true);
    void saveQTable(const std::string& filename);
    void loadQTable(const std::string& filename);
    void resetEpisode();
    double getQValue(State state, StateTransition action) const;
    
    // Episode management
    void startNewEpisode();
    bool isExplorationPhase() const { return _explorationPhase; }
    void setExplorationPhase(bool exploration) { _explorationPhase = exploration; }
    int getEpisodeCount() const { return _episodeCount; }
    
    // Reward calculation (made public for QPathManager)
    double calculateReward(State currentState, StateTransition action, State nextState, bool isGoalReached, bool hitWall) const;
    
    // State tracking
    void markStateVisited(State state);
    int getVisitCount(State state) const;
    
private:
    static constexpr double LEARNING_RATE = 0.1;
    static constexpr double DISCOUNT_FACTOR = 0.9;
    static constexpr double EPSILON = 0.1; // for epsilon-greedy exploration
    static constexpr double MIN_EPSILON = 0.01;
    static constexpr double EPSILON_DECAY = 0.995;
    static constexpr int MAX_EXPLORATION_EPISODES = 50;
    
    // Q-table: state -> action -> q-value
    std::unordered_map<uint64_t, std::array<double, 4>> _qTable;
    
    // Visited states tracking for current episode
    std::unordered_map<uint64_t, int> _visitedStates; // state -> visit count
    
    std::random_device _rd;
    std::mt19937 _gen;
    std::uniform_real_distribution<> _dis;
    
    double _currentEpsilon;
    int _episodeCount;
    bool _explorationPhase;
    
    uint64_t stateToKey(const State& state) const;
    State keyToState(uint64_t key) const;
    int actionToIndex(StateTransition action) const;
    StateTransition indexToAction(int index) const;
    double getMaxQValue(State state) const;
    StateTransition getBestAction(State state, const IMaze& maze) const;
    bool isValidAction(State state, StateTransition action, const IMaze& maze) const;
    std::vector<StateTransition> getValidActions(State state, const IMaze& maze) const;
};

class FloodFillAdvSolver
{
public:
    FloodFillAdvSolver(const IMaze& maze);
    void addGoal(Vector2Int position);
    void setGoalToHome();
    void setGoalToMazeGoal();
    void resetGoals();
    bool isSeed(State s) const;
    int cost(State s) const;
    void calculateCosts();
    void calculateCostsOptimized(State seedState);
    State bestMovement(State current) const;

private:
    const IMaze& _maze;
    bool _seeds[16][16][4];
    int _costs[16][16][4];
    std::queue<State> q;
    void setCost(State s, int cost);
    void enqueueBackwardNeighborsIfNotSeed(State s);
    std::vector<State> backwardNeighbors(State s) const;
    std::vector<State> forwardNeighbors(State s) const;
};

class QPathManager
{
public:
    QPathManager(const IMaze& maze);
    StateTransition nextMovement();
    void updatedWalls();
    void onGoalReached();
    void onWallHit();
    State getRobotState() const;
    bool isInExplorationPhase() const;
    int getCurrentEpisode() const;
    void saveProgress(const std::string& filename);
    void loadProgress(const std::string& filename);

private:
    QLearningMazeRunner _qLearner;
    const IMaze& _maze;
    State _robotState;
    State _previousState;
    StateTransition _lastAction;
    bool _waitingForActionResult;
    bool _goalReached;
    bool _wallHit;
    
    bool isGoal(State s) const;
    bool isHome(State s) const;
    State executeAction(State currentState, StateTransition action) const;
};

class PathManager
{
public:
    PathManager(const IMaze& maze);
    StateTransition nextMovement();
    void updatedWalls();
    State getRobotState() const;

private:
    FloodFillAdvSolver _solver;
    const IMaze& _maze;
    bool _goingToCenter;
    State _robotState;
    bool isGoal(State s) const;
    bool isHome(State s) const;
};

#endif
