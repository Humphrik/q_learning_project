
# Robotics Q-Learning Project Implementation Plan
**Partner Names:** Kenneth Humphries, Kailin Wu

## A 1-2 sentence description of how your team plans to implement each of the following components of this project as well as a 1-2 sentence description of how you will test each component:
   
### Q-learning algorithm
-   Executing the Q-learning algorithm
    

We plan to execute the Q-learning algorithm by creating a Q-Matrix and Action Matrix, and update the values inside the Q-Matrix by running the provided phantom_robot_movement.py. To test that our Q-learning algorithm is correct, we may consider printing certain values from the Q-Matrix to console and checking if they are consistent with expectations, perhaps after only a few movements have been made.

-   Determining when the Q-matrix has converged
    

Our algorithm will track whether or not a significant update was made in the Q-Matrix during each iteration of the loop.The Q-Matrix has converged once it stops updating for 10 loops of the algorithm - this number is chosen arbitrarily and we will increase or decrease the number of loops depending on testing. For example, if the robot’s movement takes an inefficient path to reach its destination because the matrix didn’t converge as we had thought, we will increase this number.

Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward

Once the algorithm has converged, the algorithm will track the current state of the field, and choose the action that has the highest reward value based on the corresponding Q-Matrix entry. This behavior can easily be tested with artificial Q reward values to ensure that the robot always chooses the most rewarding path.

### Robot perception
    

-   Determining the identities and locations of the three colored dumbbells
    

The robot will identify the dumbbell’s location with the LiDAR scan and its color with the camera (like the line following exercise.) This can be tested visually - we will run the algorithm and if the robot inaccurately identifies or fails to identify the color of the dumbbell, we will adjust the threshold values of the colors until the robot performs consistently. The location detection can also be tested visually - if the robot moves to pick up a dumbbell and it does not move to the expected position, we will adjust the algorithm. Testing this behavior can be done by manually moving each dumbbell and checking whether the robot can correctly locate them.

-   Determining the identities and locations of the three numbered blocks
    

Determining the location of the three numbered blocks will also be done either with the LiDAR scan or main camera. Determining the identities of these blocks can be done by analyzing the numbers on the blocks, similar to how the color of the dumbbells is analyzed. Alternatively, the identity of the blocks could be determined by placing dumbbells and random blocks and analyzing the change in reward. Ultimately, testing this behavior will be done by manually shifting the blocks and determining whether the robot can correctly locate them.

  

### Robot manipulation & movement
    

-   Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
    

We will find the height and thickness of the grip area on the dumbbell through testing and hard-coded, consistent picking up and putting down motion. This can be tested by watching the robot’s performance in Gazebo. If the robot fails to consistently pick up and put down the dumbbell with the hard-corded algorithm, we may need to customize this motion to account for the robot’s distance and angle away from the dumbbell before it starts the motion. Alternatively, the robot will have to maneuver into a specified distance and angle away from the robot every before it can use the hard-coded motion.

-   Navigating to the appropriate locations to pick up and put down the dumbbells
    

Navigating to the appropriate locations will be done by comparing the robot’s current position (given via the odometry) to the determined location of the dumbbell or block. Essentially, we will create and test a “drive-to-location” method for the robot, which accepts an x, y, and theta value.

  

## A brief timeline sketching out when you would like to have accomplished each
    

Sunday Feb 21 - Finish writing Q-learning algorithm and initial test before robot is moving

Wednesday, Feb 24- Finish creating a module for identifying and locating the dumbbells and blocks.

Friday Feb 26 - Finish robot arm manipulation; robot can consistently pick up and put down a dumbbell at a given location.

Sunday, Feb 28 - Successfully implement all 3 sections of code together.

Monday, March 1- submit project
