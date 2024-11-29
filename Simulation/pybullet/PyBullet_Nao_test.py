import pybullet as p
import time
import matplotlib.pyplot as plt

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)


startPos = [0,0,0.3]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])


#flags can also use p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
#don't use URDF_USE_SELF_COLLISION, since some connected body parts overlap
nao = p.loadURDF("Simulation/Models/nao.urdf", 
                 startPos,
                 startOrientation,
                 flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                 useFixedBase=True)


left_tibia_id = 16

# Lock all joints except left knee and foot
for i in range(p.getNumJoints(nao)):
    joint_info = p.getJointInfo(nao, i)
    joint_name = joint_info[1].decode('utf-8')
    
    # Print joint names to verify
    print(f"Joint {i}: {joint_name}")
    
    # Set high damping for all joints except left knee and foot
    if joint_name not in ['LKneePitch', 'LAnklePitch', 'LAnkleRoll']:
        p.setJointMotorControl2(nao, 
                              i, 
                              p.POSITION_CONTROL,
                              targetPosition=0,
                              force=1000)  # High force to lock the joint
    else:
        print(f"Leaving {joint_name} (ID: {i}) free to move")
        # Optional: Set lower damping for the free joints
        p.setJointMotorControl2(nao,
                              i,
                              p.POSITION_CONTROL,
                              targetPosition=0,
                              force=100)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
pi=3.1415
shoulderPitch = pi/2
shoulderRoll = 0.1 
p.setJointMotorControl2(nao,56,p.POSITION_CONTROL,targetPosition=shoulderPitch,force=1000)
p.setJointMotorControl2(nao,39,p.POSITION_CONTROL,targetPosition=shoulderPitch,force=1000)
p.setJointMotorControl2(nao,57,p.POSITION_CONTROL,targetPosition=-shoulderRoll,force=1000)
p.setJointMotorControl2(nao,40,p.POSITION_CONTROL,targetPosition=shoulderRoll,force=1000)
p.setGravity(0,0,-9.81)
timeStep=1./240.
p.setTimeStep(timeStep)
target = 0

# Define the vector offset from the tibia link origin to the desired tracking point
# Adjust these values based on your needs
tracking_point_offset = [0.1, 0, 0]  # Example offset in meters

tibia_positions_x = []
tibia_positions_y = []
tracking_point_positions_x = []
tracking_point_positions_y = []

while (target<20):
    # Get tibia position and orientation
    tibia_state = p.getLinkState(nao, left_tibia_id)
    tibia_position = list(tibia_state[0])
    tibia_orientation = tibia_state[1]
    
    # Calculate the tracking point position in world coordinates
    # This uses the rotation matrix to properly transform the offset vector
    tracking_point_world = p.multiplyTransforms(
        tibia_position,      # Position of tibia
        tibia_orientation,   # Orientation of tibia
        tracking_point_offset,  # Offset vector in local coordinates
        [0, 0, 0, 1]        # No additional rotation for the offset
    )[0]  # [0] gets the position, [1] would get the orientation
    
    print(f"Left Tibia Position: {tibia_position}")
    print(f"Tracking Point Position: {tracking_point_world}")
    
    tibia_positions_x.append(tibia_position[0])
    tibia_positions_y.append(tibia_position[1])
    tracking_point_positions_x.append(tracking_point_world[0])
    tracking_point_positions_y.append(tracking_point_world[1])

    # Visualize both points
    p.addUserDebugPoints([tibia_position], [[1, 0, 0]])  # Red for tibia
    p.addUserDebugPoints([tracking_point_world], [[0, 1, 0]])  # Green for tracking point
    
    tibia_position[1] -= 0.05
    
    p.addUserDebugPoints([tibia_position], [tibia_position])
    p.setJointMotorControl2(nao, left_tibia_id, p.VELOCITY_CONTROL, targetVelocity=0.3, force=50)
    p.stepSimulation()
    
    target+=timeStep
    time.sleep(timeStep)

# Plot both trajectories
plt.plot(tibia_positions_x, tibia_positions_y, 'r-', label='Tibia Origin')
plt.plot(tracking_point_positions_x, tracking_point_positions_y, 'g-', label='Tracking Point')
plt.legend()
plt.show()

# Save the plot with both trajectories
plt.savefig('trajectory_plot.png')

