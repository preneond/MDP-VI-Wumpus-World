package wumpus.agent;

import wumpus.world.*;
import wumpus.world.WorldState;


import javax.vecmath.Point2i;
import java.util.ArrayList;
import java.util.List;


public class Agent implements WorldModel.Agent {

    private Action[][] policy;
    private int numRows = 0;
    private int stepCounter = 0;
    private int numCols = 0;
    private float gamma = 0.99f;
    private float minErr = 0.001f;

    /**
     * This method is called when the simulation engine requests the next action.
     * You are given a position of the robot and the map of the environment.
     * <p>
     * The top-left corner has coordinates (0,0).
     * <p>
     * You can check whether there is an obstacle on a particular cell of the map
     * by querying map[x][y] == wumpus.world.CellContent.OBSTACLE.
     * <p>
     * There is one gold on the map. You can query whether a position contains gold by
     * querying map[x][y] == wumpus.world.CellContent.GOLD.
     * <p>
     * Further, there are several pits on the map. You can query whether a position contains pit by
     * querying map[x][y] == wumpus.world.CellContent.PIT.
     *
     * @return action to perform in the next step
     */

    public Action nextStep(WorldState state) {
        //---- value iteration ----
        if (policy == null) {
            DebugVis.initVis();
            policy = computePolicy(state.getMap());
        }

//        System.out.println("# Step:" + stepCounter);
        stepCounter++;
        return policy[state.getAgent().x][state.getAgent().y];
    }

    /**
     * Compute an optimal policy for the agent.
     *
     * @param map map of the environment
     * @return an array that contains for each cell of the environment one action,
     * i.e. one of: Action.NORTH, Action.SOUTH, Action.EAST, Action.WEST.
     */
    private Action[][] computePolicy(CellContent[][] map) {
        // the size of the map can be obtained as
        numRows = map.length; // the number of columns in the x-dimension,
        numCols = map[0].length; // the number of rows in the y-dimension
        Point2i[] wumpuses = new Point2i[0];

        /**** YOUR CODE HERE *****/

        float[][] stateValues = valueIteration(wumpuses, map);

        /* You can visualize a value for each state like this... (Press "s" in the visualization window) */
        DebugVis.setStateValues(stateValues);

//		float[][][] stateActionValues = computeStateActionValues(rows, cols);
//		/* You can visualize a value for each action at each state like this... (Press "a" in the visualization window) */
//		DebugVis.setStateActionValues(stateActionValues);

        Action[][] policy = getPolicyFromStateValues(stateValues, wumpuses, map);
        /* You can visualize a policy (optimal action at each state) like this... (Press "p" in the visualization window) */
        DebugVis.setPolicy(policy);

        return policy;
    }

    private float[][] valueIteration(Point2i[] wumpuses, CellContent[][] map) {
        float[][] stateValues = new float[numRows][numCols];
        float[][] prevValues = new float[numRows][numCols];
        Point2i position;
        WorldState state;
        int iter = 0;
        do {
//            System.out.println("# Iteration :" + iter);
            for (int i = 0; i < numRows; i++) {
                prevValues[i] = stateValues[i].clone();
            }

            for (int i = 0; i < numRows; i++) {
                for (int j = 0; j < numCols; j++) {
                    position = new Point2i(j, i);
                    state = new WorldState(position, wumpuses, map);
                    // Vi+1(s) = max_a sum_s' T(s,a,s')[R(s,a,s')+Vi(s')

                    // maximize over actions
                    float value = -Integer.MAX_VALUE;
                    for (Action action : Action.values()) {
//                        System.out.println(position + ": " + action);

                        // summing over s'
                        float curr_value = 0;
                        for (Transition transition : WorldModel.getTransitions(state, action)) {
                            int succX = transition.successorState.getY();
                            int succY = transition.successorState.getX();
                            curr_value += transition.probability * (transition.reward + gamma * prevValues[succX][succY]);

//                            System.out.println(position + ": There is a transition to "
//                                    + transition.successorState + " with probability "
//                                    + transition.probability + " and reward "
//                                    + transition.reward + ".");
                        }
                        value = Math.max(value, curr_value);
                    }

                    stateValues[i][j] = value;
                }
            }
            iter++;
//            if (iter > 500) break;
        } while (!isConverged(prevValues, stateValues, minErr));

        return stateValues;
    }

    private boolean isConverged(float[][] V, float[][] V_prev, float minErr) {
        boolean converged = true;
//        System.out.println("=================================");
        for (int i = 0; i < numRows; i++) {
//            System.out.print("|| ");
            for (int j = 0; j < numCols; j++) {
                float dev = Math.abs(V[i][j] - V_prev[i][j]);
//                System.out.printf("%.2f ", V[i][j]);//Math.abs(V[i][j] - V_prev[i][j]));
//                System.out.printf("%.2f ", Math.abs(V[i][j] - V_prev[i][j]));

                if (dev > minErr) {
                    converged = false;
//                    return false;
                }
            }
//            System.out.print("||\n");

        }
//        System.out.println("=================================");
        return converged;
    }


    private Action[][] getPolicyFromStateValues(float[][] stateValues, Point2i[] wumpuses, CellContent[][] map) {
        Action[][] policy = new Action[numRows][numCols];
        WorldState state;
        Point2i position;

        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numCols; j++) {
                position = new Point2i(i, j);
                state = new WorldState(position, wumpuses, map);
                // Pi(s) = arg max_a sum_s' Tk(s,a,s')[Rk(s,a,s')+ gamma*Vk(s')

                // maximize over actions
                float value = -Integer.MAX_VALUE;
                Action action = Action.EAST;
                for (Action a : Action.values()) {
                    float curr_value = 0;
                    for (Transition transition : WorldModel.getTransitions(state, a)) {
                        int succX = transition.successorState.getY();
                        int succY = transition.successorState.getX();
                        curr_value += transition.probability * (transition.reward + stateValues[succX][succY]);
                    }
                    if (curr_value > value) {
                        value = curr_value;
                        action = a;
                    }
                }

                policy[i][j] = action;
            }
        }
        return policy;
    }
}
