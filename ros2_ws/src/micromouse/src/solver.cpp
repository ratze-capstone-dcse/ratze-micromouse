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

#include "micromouse/solver.hpp"
#include <cstring>
#include <limits>


// Q-Learning Implementation
QLearningMazeRunner::QLearningMazeRunner() 
    : _gen(_rd()), _dis(0.0, 1.0), _currentEpsilon(EPSILON), _episodeCount(0), _explorationPhase(true)
{
}

uint64_t QLearningMazeRunner::stateToKey(const State& state) const
{
    return (static_cast<uint64_t>(state.position.x) << 24) | 
           (static_cast<uint64_t>(state.position.y) << 16) |
           (static_cast<uint64_t>(state.orientation) << 8);
}

State QLearningMazeRunner::keyToState(uint64_t key) const
{
    State state;
    state.position.x = (key >> 24) & 0xFF;
    state.position.y = (key >> 16) & 0xFF;
    state.orientation = static_cast<Direction>((key >> 8) & 0xFF);
    return state;
}

int QLearningMazeRunner::actionToIndex(StateTransition action) const
{
    switch (action) {
        case StateTransition::forward: return 0;
        case StateTransition::turnLeft: return 1;
        case StateTransition::turnRight: return 2;
        default: return 3; // invalid
    }
}

StateTransition QLearningMazeRunner::indexToAction(int index) const
{
    switch (index) {
        case 0: return StateTransition::forward;
        case 1: return StateTransition::turnLeft;
        case 2: return StateTransition::turnRight;
        default: return StateTransition::invalid;
    }
}

double QLearningMazeRunner::getQValue(State state, StateTransition action) const
{
    uint64_t key = stateToKey(state);
    auto it = _qTable.find(key);
    if (it != _qTable.end()) {
        return it->second[actionToIndex(action)];
    }
    return 0.0; // Default Q-value for unvisited state-action pairs
}

double QLearningMazeRunner::getMaxQValue(State state) const
{
    uint64_t key = stateToKey(state);
    auto it = _qTable.find(key);
    if (it != _qTable.end()) {
        double maxQ = *std::max_element(it->second.begin(), it->second.end());
        return maxQ;
    }
    return 0.0;
}

bool QLearningMazeRunner::isValidAction(State state, StateTransition action, const IMaze& maze) const
{
    if (action == StateTransition::invalid) return false;
    
    // For turns, always valid (robot can turn in place)
    if (action == StateTransition::turnLeft || action == StateTransition::turnRight) {
        return true;
    }
    
    // For forward movement, check if there's a wall
    if (action == StateTransition::forward) {
        return !maze.hasWall(state.position, state.orientation);
    }
    
    return false;
}

std::vector<StateTransition> QLearningMazeRunner::getValidActions(State state, const IMaze& maze) const
{
    std::vector<StateTransition> validActions;
    
    if (isValidAction(state, StateTransition::forward, maze)) {
        validActions.push_back(StateTransition::forward);
    }
    validActions.push_back(StateTransition::turnLeft);
    validActions.push_back(StateTransition::turnRight);
    
    return validActions;
}

StateTransition QLearningMazeRunner::getBestAction(State state, const IMaze& maze) const
{
    std::vector<StateTransition> validActions = getValidActions(state, maze);
    if (validActions.empty()) return StateTransition::invalid;
    
    StateTransition bestAction = validActions[0];
    double bestQ = getQValue(state, bestAction);
    
    for (const auto& action : validActions) {
        double q = getQValue(state, action);
        if (q > bestQ) {
            bestQ = q;
            bestAction = action;
        }
    }
    
    return bestAction;
}

double QLearningMazeRunner::calculateReward(State currentState, StateTransition action, State nextState, bool isGoalReached, bool hitWall) const
{
    (void)currentState; // Suppress unused parameter warning
    (void)action;       // Suppress unused parameter warning  
    (void)nextState;    // Suppress unused parameter warning
    
    double reward = -1.0; // Small negative reward for each step
    
    if (isGoalReached) {
        reward = 100.0; // Large positive reward for reaching goal
    } else if (hitWall) {
        reward = -50.0; // Large negative reward for hitting wall
    }
    
    return reward;
}

StateTransition QLearningMazeRunner::chooseAction(State currentState, const IMaze& maze, bool explorationMode)
{
    std::vector<StateTransition> validActions = getValidActions(currentState, maze);
    if (validActions.empty()) return StateTransition::invalid;
    
    // Epsilon-greedy action selection
    if (explorationMode && _dis(_gen) < _currentEpsilon) {
        // Explore: choose random valid action
        int randomIndex = static_cast<int>(_dis(_gen) * validActions.size());
        return validActions[randomIndex];
    } else {
        // Exploit: choose best action based on Q-values
        return getBestAction(currentState, maze);
    }
}

void QLearningMazeRunner::updateQValue(State currentState, StateTransition action, State nextState, double reward, bool isGoalReached)
{
    uint64_t currentKey = stateToKey(currentState);
    
    // Initialize Q-table entry if it doesn't exist
    if (_qTable.find(currentKey) == _qTable.end()) {
        _qTable[currentKey] = {0.0, 0.0, 0.0, 0.0};
    }
    
    double currentQ = getQValue(currentState, action);
    double maxNextQ = isGoalReached ? 0.0 : getMaxQValue(nextState);
    
    // Q-learning update rule: Q(s,a) = Q(s,a) + α[r + γ*max(Q(s',a')) - Q(s,a)]
    double newQ = currentQ + LEARNING_RATE * (reward + DISCOUNT_FACTOR * maxNextQ - currentQ);
    
    _qTable[currentKey][actionToIndex(action)] = newQ;
}

void QLearningMazeRunner::startNewEpisode()
{
    _episodeCount++;
    
    // Decay epsilon
    if (_currentEpsilon > MIN_EPSILON) {
        _currentEpsilon *= EPSILON_DECAY;
    }
    
    // Switch to exploitation phase after enough exploration
    if (_episodeCount >= MAX_EXPLORATION_EPISODES) {
        _explorationPhase = false;
    }
}

void QLearningMazeRunner::resetEpisode()
{
    // Reset any episode-specific state if needed
}

void QLearningMazeRunner::saveQTable(const std::string& filename)
{
    std::ofstream file(filename, std::ios::binary);
    if (file.is_open()) {
        size_t tableSize = _qTable.size();
        file.write(reinterpret_cast<const char*>(&tableSize), sizeof(tableSize));
        
        for (const auto& entry : _qTable) {
            file.write(reinterpret_cast<const char*>(&entry.first), sizeof(entry.first));
            file.write(reinterpret_cast<const char*>(entry.second.data()), sizeof(double) * 4);
        }
        
        file.write(reinterpret_cast<const char*>(&_episodeCount), sizeof(_episodeCount));
        file.write(reinterpret_cast<const char*>(&_currentEpsilon), sizeof(_currentEpsilon));
        file.close();
    }
}

void QLearningMazeRunner::loadQTable(const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (file.is_open()) {
        _qTable.clear();
        
        size_t tableSize;
        file.read(reinterpret_cast<char*>(&tableSize), sizeof(tableSize));
        
        for (size_t i = 0; i < tableSize; ++i) {
            uint64_t key;
            std::array<double, 4> values;
            file.read(reinterpret_cast<char*>(&key), sizeof(key));
            file.read(reinterpret_cast<char*>(values.data()), sizeof(double) * 4);
            _qTable[key] = values;
        }
        
        file.read(reinterpret_cast<char*>(&_episodeCount), sizeof(_episodeCount));
        file.read(reinterpret_cast<char*>(&_currentEpsilon), sizeof(_currentEpsilon));
        file.close();
    }
}

// QPathManager Implementation
QPathManager::QPathManager(const IMaze& maze) 
    : _maze(maze), _waitingForActionResult(false), _goalReached(false), _wallHit(false)
{
    _robotState.position = {0, 0};
    _robotState.orientation = Direction::up;
    _qLearner.loadQTable("qtable.dat"); // Try to load previous learning
}

StateTransition QPathManager::nextMovement()
{
    // If we were waiting for action result, update Q-value
    if (_waitingForActionResult) {
        double reward = _qLearner.calculateReward(_previousState, _lastAction, _robotState, _goalReached, _wallHit);
        _qLearner.updateQValue(_previousState, _lastAction, _robotState, reward, _goalReached);
        
        if (_goalReached) {
            _qLearner.startNewEpisode();
            onGoalReached();
        }
        
        _waitingForActionResult = false;
        _goalReached = false;
        _wallHit = false;
    }
    
    // Choose next action
    _previousState = _robotState;
    _lastAction = _qLearner.chooseAction(_robotState, _maze, _qLearner.isExplorationPhase());
    
    // Update robot state based on action (prediction)
    if (_lastAction != StateTransition::invalid) {
        _robotState = executeAction(_robotState, _lastAction);
        _waitingForActionResult = true;
    }
    
    return _lastAction;
}

State QPathManager::executeAction(State currentState, StateTransition action) const
{
    State nextState = currentState;
    
    switch (action) {
        case StateTransition::forward:
            switch (currentState.orientation) {
                case Direction::up:
                    nextState.position.y++;
                    break;
                case Direction::down:
                    nextState.position.y--;
                    break;
                case Direction::left:
                    nextState.position.x--;
                    break;
                case Direction::right:
                    nextState.position.x++;
                    break;
            }
            break;
        case StateTransition::turnLeft:
            switch (currentState.orientation) {
                case Direction::up:
                    nextState.orientation = Direction::left;
                    break;
                case Direction::left:
                    nextState.orientation = Direction::down;
                    break;
                case Direction::down:
                    nextState.orientation = Direction::right;
                    break;
                case Direction::right:
                    nextState.orientation = Direction::up;
                    break;
            }
            break;
        case StateTransition::turnRight:
            switch (currentState.orientation) {
                case Direction::up:
                    nextState.orientation = Direction::right;
                    break;
                case Direction::right:
                    nextState.orientation = Direction::down;
                    break;
                case Direction::down:
                    nextState.orientation = Direction::left;
                    break;
                case Direction::left:
                    nextState.orientation = Direction::up;
                    break;
            }
            break;
        default:
            break;
    }
    
    return nextState;
}

bool QPathManager::isGoal(State s) const
{
    // Goal positions (center of maze)
    return (s.position.x >= 7 && s.position.x <= 8 && s.position.y >= 7 && s.position.y <= 8);
}

bool QPathManager::isHome(State s) const
{
    return s.position.x == 0 && s.position.y == 0;
}

void QPathManager::onGoalReached()
{
    _goalReached = true;
}

void QPathManager::onWallHit()
{
    _wallHit = true;
}

void QPathManager::updatedWalls()
{
    // Walls have been updated, Q-learner will adapt based on new observations
}

State QPathManager::getRobotState() const
{
    return _robotState;
}

bool QPathManager::isInExplorationPhase() const
{
    return _qLearner.isExplorationPhase();
}

int QPathManager::getCurrentEpisode() const
{
    return _qLearner.getEpisodeCount();
}

void QPathManager::saveProgress(const std::string& filename)
{
    _qLearner.saveQTable(filename);
}

void QPathManager::loadProgress(const std::string& filename)
{
    _qLearner.loadQTable(filename);
}



