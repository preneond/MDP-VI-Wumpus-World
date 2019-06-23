## MDP-VI-Wumpus-World
MDP Value Iteration applied in Wumpus World to help robot Emil find its way to the pot(s) of gold in a world filled with deadly pits.

### Environment
Robot Emil moves in grid world with 4 possible actions that have stochastic effects. Gold, pits and wumpus are terminal states. Reaching gold gives agent large positive utility while falling into a pit or encountering Wumpus gives agent large negative utility. Agent also gets small penalty for each move.

Robot can execute following actions with stochastic effects (class Action):
- NORTH – Actual effect: 80% NORTH, 10% EAST, 10% WEST
- SOUTH – Actual effect: 80% SOUTH, 10% EAST, 10% WEST
- EAST – Actual effect: 80% EAST, 10% NORTH, 10% SOUTH
- WEST – Actual effect: 80% WEST, 10% NORTH, 10% SOUTH

If the robot executes a move action that would end up in an obstacle, it bounces back to its current-position.
The environment is a matrix MxN, where the first index represents columns (x-coordinate) and the second index represents rows (y-coordinate). The columns (rows) are indexed starting from 0, i.e. we have columns (rows) 0,1,…,M-1 (N-1).
Each cell can contain (class CellContent):
- EMPTY
- OBSTACLE
- GOLD
- PIT
The simulation finishes if the agent reaches the gold (only in Task 1), falls into a pit, or after h steps.

### Rewards
- -1 at each move action
- -100 + (-1) for action that results in a pit
- -100 + (-1) when a Wumpus moves to the same cell as the agent
- 100 + (-1) for action that reaches the gold
