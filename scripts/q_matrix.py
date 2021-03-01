#!/usr/bin/env python3
""" This script publishes ROS messages based on the q_matrix algorithm. """
import rospy
import random
from q_learning_project.msg import RobotMoveDBToBlock, QLearningReward, QMatrix, QMatrixRow

from std_msgs.msg import Header


class Q_Matrix:



    def __init__(self):
        rospy.init_node('q_matrix')    # initialize our ROS node

        self.initialized = False
        self.matrix_converged = False
        self.reward_found = False



        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size = 10)
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size = 10)
        self.reward_sub = rospy.Subscriber("/q_learning/reward", QLearningReward, self.process_reward)

        # Initializing the a_t values:
        self.a_t = []
        for i in range (0, 9):
            color = "red"
            if (int(i/3) == 1): color = "green"
            elif (int(i/3) == 2): color = "blue"
            self.a_t.append((color, i%3 + 1))

        # Initializing the s_t values:
        self.s_t = []
        for i in range (0,4):
            for j in range (0,4):
                for k in range(0,4):
                    self.s_t.append ((k, j, i))
        

        # The empty action matrix
        self.action_matrix = [[-1] * 64] * 64
        # The empty q matrix
        self.q_matrix = [[-1.0] * 9] * 64

        # Set all valid actions to zero.
        self.fill_q_matrix()


        # Helper variables.
        self.current_state = 0
        self.last_reward = 0
        self.t = 0

        # To be our final, optimal sequence.
        self.best_actions = [-1] * 3

        self.initialized = True

        # The good stuff.
        self.converge_q_matrix()


    def process_reward(self, data):
        if (self.matrix_converged):
            return
        self.last_reward = data.reward
        self.reward_found = True
        print ("REWARD FOUND!")
        return



    def fill_q_matrix(self):
        # Set any valid actions from each state to zero.
        # Reminder, states are of form (redpos, greenpos, bluepos)
        # Actions are of form (color, block)
        for s in range (0, 64):
            red_pos = self.s_t[s][0]
            green_pos = self.s_t[s][1]
            blue_pos = self.s_t[s][2]

            self.q_matrix[s] = [-1.0] * 9

            # An action is invalid if a dumbell is already placed or moved to an already occupied block.
            # A state is invalid if any two dumbells are in the same place (aside from origin.)

            # Valid state check.
            if (red_pos == green_pos or red_pos == blue_pos):
                if (red_pos != 0):
                    continue
            if (green_pos == blue_pos):
                if (green_pos != 0):
                    continue

            # Valid action check.
            for a in range (0, 3):
                # The reds
                if (red_pos != 0):
                    continue
                new_pos = self.a_t[a][1]
                if (new_pos != green_pos and new_pos != blue_pos):
                    self.q_matrix[s][a] = 0.0
            
            for a in range (3, 6):
                # The greens
                if (green_pos != 0):
                    continue
                new_pos = self.a_t[a][1]
                if (new_pos != red_pos and new_pos != blue_pos):
                    self.q_matrix[s][a] = 0.0

            for a in range (6, 9):
                # The blues
                if (blue_pos != 0):
                    continue
                new_pos = self.a_t[a][1]
                if (new_pos != red_pos and new_pos != green_pos):
                    self.q_matrix[s][a] = 0.0


        
    def converge_q_matrix(self):
        # The actual reenforcement learning
        alpha = 1
        gamma = 0.3
        t = 0
        r = rospy.Rate(1)
        r.sleep()

        convergence_counter = 30
        c = 0


        # More rigorous convergence to be added later.
        while (t <= 665 and (c < convergence_counter or t%3!=0)):
            self.reward_found = False
            state = self.current_state
            # While we can assume the state is always valid, it doesn't hurt to check.
            if (not self.is_valid_state(self.q_matrix[state])):
                print ("INVALID STATE REACHED")
                break

            # Pick a random valid action
            action = random.randint(0, 8)
            while (self.q_matrix[state][action] == -1):
                action = random.randint(0,8)

            # Perform the action
            p_action = RobotMoveDBToBlock()
            p_action.robot_db = self.a_t[action][0]
            p_action.block_id = self.a_t[action][1]
            print (p_action.robot_db, p_action.block_id)
            self.action_pub.publish(p_action)

            # Values for updating q index value
            new_state = self.determine_new_state(state, action)
            maxVal = self.get_max_reward(new_state)

            # Wait for reward
            while (not self.reward_found):
                r.sleep()
    
            temp = self.q_matrix[state][action]
            self.q_matrix[state][action] = self.last_reward + gamma * maxVal
            
            # Publish the new matrix.
            self.publish_matrix()


            # Have we converged yet? (Has no significant update happened recently?)
            if (c >= convergence_counter):
                pass
            elif (abs(temp - self.q_matrix[state][action]) <= 0.05):
                c += 1
            else:
                c = 0

            # Update the abstract state.
            self.current_state = new_state
            
            t += 1
            print(t)
            print("Convergence counter is: " + str(c))
            print ("New state: ", new_state)
            print (self.s_t[new_state])
            print ("----------------------------")

            # Reset the world.
            if (t%3 == 0):
                self.current_state = 0

            #rospy.sleep(5)

        print (self.q_matrix)



        print ("The best action sequence is: ")
        next_state = 0
        for i in range (0, 3):
            action = self.get_best_action(next_state)
            next_state = self.determine_new_state(next_state, action)
            print (self.s_t[next_state])
            self.best_actions[i] = action


        self.matrix_converged = True

        return






    def is_valid_state(self, ls):
        for l in ls:
            if (l != -1):
                return True
        return False



    def determine_new_state(self, state, action):
        # Assumes a valid action was taken.
        red_pos = self.s_t[state][0]
        green_pos = self.s_t[state][1]
        blue_pos = self.s_t[state][2]

        a = self.a_t[action]

        # Which dumbell is moved?
        if (a[0] == "red"):
            red_pos = a[1]
        elif (a[0] == "green"):
            green_pos = a[1]
        else:
            blue_pos = a[1]
        ns = (red_pos, green_pos, blue_pos)
        for i in range(0, 64):
            if (ns == self.s_t[i]):
                return i
        print ("COULDNT FIND NEW STATE!")
        return -1


    def get_max_reward(self, state):
        s = self.q_matrix[state]
        val = -1
        for i in range (0, 9):
            val = max(s[i], val)
        return val

    def get_best_action(self, state):
        s = self.q_matrix[state]
        index = 0
        for i in range (0, 9):
            if (s[index] < s[i]):
                index = i
        return index

    def publish_matrix(self):
        # Totally not inefficient way of publishing to the QMatrix topic.
        my_header = Header(stamp=rospy.Time.now(), frame_id="QMatrix")
        qMatrix = QMatrix()
        qMatrix.header = my_header
        qMatrix.q_matrix = []
        for i in range(0, len(self.q_matrix)):
            qrow = QMatrixRow()
            for j in range(0, len(self.q_matrix[i])):
                qrow.q_matrix_row.append(int(self.q_matrix[i][j]))
            qMatrix.q_matrix.append(qrow)
        self.matrix_pub.publish(qMatrix)




    def run(self):        
        r = rospy.Rate(10)
        while (not self.matrix_converged):
            r.sleep()

        # We got the values! Let's publish them when the user is ready.
        while (not rospy.is_shutdown()):
            input ("The matrix has converged!!! Press Enter to publish the best course of action.")
            for i in range (0, 3):
                p_action = RobotMoveDBToBlock()
                p_action.robot_db = self.a_t[self.best_actions[i]][0]
                p_action.block_id = self.a_t[self.best_actions[i]][1]
                print (p_action.robot_db, p_action.block_id)
                self.action_pub.publish(p_action)
            r.sleep()



if (__name__ == '__main__'):
    ROS = Q_Matrix()
    ROS.run();
