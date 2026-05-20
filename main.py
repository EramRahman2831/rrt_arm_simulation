import pybullet as p
import pybullet_data
import time
import numpy as np
import random

# connect to GUI so we can see the simulation window
p.connect(p.GUI)

# set path so pybullet can find built-in models like plane and robot
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# load a flat ground plane into the world
p.loadURDF("plane.urdf")

# load the kuka robot arm and keep it fixed in place
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
# index of the robot's end effector (last link)
end_effector = 11


# helper function that runs the RRT algorithm
def run_rrt(start, goal):

    # store all nodes in the tree
    nodes = [start]

    # map each node index to its parent index
    parents = {0: None}

    # step size for each move
    step_size = 0.05

    # distance threshold to consider goal reached
    goal_threshold = 0.08

    # run the algorithm for a fixed number of iterations
    for _ in range(500):

        # sample a random point in space
        rand = [
            random.uniform(0.2, 0.8),
            random.uniform(-0.5, 0.5),
            random.uniform(0.2, 0.8)
        ]

        # find index of the closest node to the random point
        nearest_index = min(
            range(len(nodes)),
            key=lambda i: np.linalg.norm(
                np.array(nodes[i]) - np.array(rand)
            )
        )

        # get the nearest node position
        nearest = nodes[nearest_index]

        # compute direction from nearest node to random point
        direction = np.array(rand) - np.array(nearest)

        # get length of direction vector
        norm = np.linalg.norm(direction)

        # skip if direction is zero (avoid division by zero)
        if norm == 0:
            continue

        # normalize direction to unit vector
        direction = direction / norm

        # move a small step toward the random point
        new_node = list(np.array(nearest) + step_size * direction)

        # add new node to the tree
        nodes.append(new_node)

        # get index of new node
        new_index = len(nodes) - 1

        # store its parent
        parents[new_index] = nearest_index

        # check if new node is close to goal
        if np.linalg.norm(np.array(new_node) - np.array(goal)) < goal_threshold:

            # build path by tracing parents back to start
            path = []
            curr_index = new_index

            while curr_index is not None:
                path.append(nodes[curr_index])
                curr_index = parents[curr_index]

            # return path from start to goal
            return path[::-1]

    # return empty list if no path found
    return []


# sliders for user to choose target position
target_x = p.addUserDebugParameter("target x", 0.2, 0.8, 0.5)
target_y = p.addUserDebugParameter("target y", -0.5, 0.5, 0)
target_z = p.addUserDebugParameter("target z", 0.2, 0.8, 0.5)

# button to trigger RRT planning
run_button = p.addUserDebugParameter("run rrt", 1, 0, 0)


# create a visible red sphere to represent the target

# collision shape 
sphere_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)

# visual shape (this is what we actually see)
sphere_visual = p.createVisualShape(
    p.GEOM_SPHERE,
    radius=0.03,
    rgbaColor=[1, 0, 0, 1]  # red color
)

# create the sphere object in the world
target_sphere = p.createMultiBody(
    baseMass=0,  # zero mass means it does not move and stays in place
    baseCollisionShapeIndex=sphere_collision,
    baseVisualShapeIndex=sphere_visual,
    basePosition=[0.5, 0, 0.5]
)


# store previous button value to detect clicks
prev_button = 0


# main simulation loop (runs forever)
while True:

    # read current slider values
    x = p.readUserDebugParameter(target_x)
    y = p.readUserDebugParameter(target_y)
    z = p.readUserDebugParameter(target_z)

    # update the position of the target sphere so user can see it move
    p.resetBasePositionAndOrientation(
        target_sphere,
        [x, y, z],
        [0, 0, 0, 1]
    )

    # read button value
    button = p.readUserDebugParameter(run_button)

    # trigger only when button is pressed (edge detection)
    if button > prev_button:

        print("planning...")

        # get current end-effector position as starting point
        start = list(p.getLinkState(robot, end_effector)[0])

        # goal is the position selected by sliders
        goal = [x, y, z]

        # run the RRT algorithm
        path = run_rrt(start, goal)

        print("path length:", len(path))

        # follow the generated path
        for point in path:

            # compute joint angles needed to reach this point
            joint_angles = p.calculateInverseKinematics(
                robot,
                end_effector,
                point
            )

            # send commands to each joint motor
            for i in range(7):
                p.setJointMotorControl2(
                    robot,
                    i,
                    p.POSITION_CONTROL,
                    targetPosition=joint_angles[i]
                )

            # step simulation multiple times for smooth movement
            for _ in range(10):
                p.stepSimulation()
                time.sleep(1/240)

    # update previous button state
    prev_button = button

    # continue simulation
    p.stepSimulation()
    time.sleep(1/240)