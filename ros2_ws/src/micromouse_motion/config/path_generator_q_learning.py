import numpy as np
import yaml
import os

class MazeQLearning:
    def __init__(self, maze, cell_size=0.6):
        """
        Initialize Q-learning maze solver.
        
        Args:
            maze: 2D array where 0=free, 1=wall
            cell_size: size of each cell in meters (default 0.6m = 60cm)
        """
        self.maze = np.array(maze)
        self.rows, self.cols = self.maze.shape
        self.cell_size = cell_size
        
        # Q-learning parameters
        self.alpha = 0.1  # Learning rate
        self.gamma = 0.9  # Discount factor
        self.epsilon = 0.1  # Exploration rate
        
        # Actions: 0=up, 1=right, 2=down, 3=left
        self.actions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        
        # Initialize Q-table: (state_row, state_col, action)
        self.q_table = np.zeros((self.rows, self.cols, 4))
        
    def is_valid_state(self, row, col):
        """Check if state is valid (within bounds and not a wall)"""
        return (0 <= row < self.rows and 
                0 <= col < self.cols and 
                self.maze[row, col] == 0)
    
    def get_reward(self, row, col, goal):
        """Get reward for reaching a state"""
        if (row, col) == goal:
            return 100  # Large positive reward for reaching goal
        elif not self.is_valid_state(row, col):
            return -100  # Large negative reward for hitting wall
        else:
            return -1  # Small negative reward to encourage shorter paths
    
    def choose_action(self, state):
        """Choose action using epsilon-greedy policy"""
        if np.random.random() < self.epsilon:
            return np.random.randint(4)  # Explore
        else:
            return np.argmax(self.q_table[state[0], state[1], :])  # Exploit
    
    def train(self, start, goal, episodes=1000):
        """Train Q-learning agent"""
        print(f"Training Q-learning agent for {episodes} episodes...")
        
        for episode in range(episodes):
            state = start
            steps = 0
            max_steps = self.rows * self.cols * 2
            
            while state != goal and steps < max_steps:
                # Choose action
                action = self.choose_action(state)
                
                # Take action
                next_row = state[0] + self.actions[action][0]
                next_col = state[1] + self.actions[action][1]
                next_state = (next_row, next_col)
                
                # Get reward
                reward = self.get_reward(next_row, next_col, goal)
                
                # Update Q-value
                if self.is_valid_state(next_row, next_col):
                    max_next_q = np.max(self.q_table[next_row, next_col, :])
                    self.q_table[state[0], state[1], action] += \
                        self.alpha * (reward + self.gamma * max_next_q - 
                                     self.q_table[state[0], state[1], action])
                    state = next_state
                else:
                    # Invalid move, stay in place and apply penalty
                    self.q_table[state[0], state[1], action] += \
                        self.alpha * (reward - self.q_table[state[0], state[1], action])
                
                steps += 1
            
            if (episode + 1) % 100 == 0:
                print(f"Episode {episode + 1}/{episodes} completed")
        
        print("Training completed!")
    
    def get_optimal_path(self, start, goal):
        """Extract optimal path from learned Q-table"""
        path = [start]
        state = start
        visited = set([start])
        max_steps = self.rows * self.cols
        
        while state != goal and len(path) < max_steps:
            # Choose best action (no exploration)
            action = np.argmax(self.q_table[state[0], state[1], :])
            
            # Take action
            next_row = state[0] + self.actions[action][0]
            next_col = state[1] + self.actions[action][1]
            next_state = (next_row, next_col)
            
            # Check if valid and not revisiting
            if self.is_valid_state(next_row, next_col):
                if next_state in visited:
                    # Try second best action to avoid loops
                    q_values = self.q_table[state[0], state[1], :].copy()
                    q_values[action] = -np.inf
                    action = np.argmax(q_values)
                    next_row = state[0] + self.actions[action][0]
                    next_col = state[1] + self.actions[action][1]
                    next_state = (next_row, next_col)
                    
                    if not self.is_valid_state(next_row, next_col) or next_state in visited:
                        break
                
                state = next_state
                path.append(state)
                visited.add(state)
            else:
                break
        
        return path
    
    def path_to_waypoints(self, path):
        """Convert cell path to waypoints in meters (center of cells)"""
        waypoints = []
        for row, col in path:
            # Calculate center position of cell in meters
            x = col * self.cell_size + self.cell_size / 2
            y = row * self.cell_size + self.cell_size / 2
            waypoints.append([x, y])
        return waypoints
    
    def save_waypoints(self, waypoints, filename='waypoints.yaml', waypoint_name='waypoints'):
        """Save waypoints to YAML file"""
        filepath = os.path.join(os.path.dirname(__file__), filename)
        
        # Load existing data if file exists
        existing_data = {}
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as f:
                    existing_data = yaml.safe_load(f) or {}
            except Exception as e:
                print(f"Warning: Could not load existing file: {e}")
        
        # Add or update the waypoint set
        existing_data[waypoint_name] = waypoints
        
        # Save to file with proper formatting
        with open(filepath, 'w') as f:
            for key, value in existing_data.items():
                f.write(f"{key}:\n")
                for wp in value:
                    f.write(f"  - [{wp[0]:.3f}, {wp[1]:.3f}]\n")
                f.write("\n")
        
        print(f"Waypoints saved to {filepath} under key '{waypoint_name}'")
        print(f"Total waypoints: {len(waypoints)}")


def main():
    # Define a 3x3 meter maze (5x5 cells at 0.6m = 3.0m total)
    # 0=free, 1=wall
    maze = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    
    # Define start and goal positions (row, col)
    start = (0, 0)  # Top-left
    goal = (4, 4)   # Bottom-right
    
    # Create Q-learning solver
    solver = MazeQLearning(maze, cell_size=0.6)
    
    # Train the agent
    solver.train(start, goal, episodes=1000)
    
    # Get optimal path
    path = solver.get_optimal_path(start, goal)
    print(f"\nOptimal path (cells): {path}")
    
    # Convert to waypoints
    waypoints = solver.path_to_waypoints(path)
    print(f"\nWaypoints (meters):")
    for i, wp in enumerate(waypoints):
        print(f"  {i}: [{wp[0]:.2f}, {wp[1]:.2f}]")
    
    # Save to YAML file with custom waypoint name
    solver.save_waypoints(waypoints, filename='waypoints.yaml', waypoint_name='waypoints_qlearning')
    
    # Visualize maze and path
    print("\nMaze visualization (S=start, G=goal, *=path, #=wall, .=free):")
    for i in range(len(maze)):
        row_str = ""
        for j in range(len(maze[0])):
            if (i, j) == start:
                row_str += "S "
            elif (i, j) == goal:
                row_str += "G "
            elif (i, j) in path:
                row_str += "* "
            elif maze[i][j] == 1:
                row_str += "# "
            else:
                row_str += ". "
        print(row_str)


if __name__ == "__main__":
    main()
