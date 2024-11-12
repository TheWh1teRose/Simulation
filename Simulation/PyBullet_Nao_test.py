import pybullet as p
import time
import matplotlib.pyplot as plt

p.connect(p.GUI)
p.loadURDF("Simulation/Models/Plane/plane.urdf")
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)


startPos = [0,0,1]
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

tibia_positions_x = []
tibia_positions_y = []

while (target<20):
	# Get tibia position and orientation
	tibia_state = p.getLinkState(nao, left_tibia_id)
	tibia_position = tibia_state[0]  # World position coordinates (x, y, z)
	tibia_orientation = tibia_state[1]  # World orientation (quaternion)
	
	print(f"Left Tibia Position: {tibia_position}")
	tibia_positions_x.append(tibia_position[0])
	tibia_positions_y.append(tibia_position[1])
	
	p.setJointMotorControl2(nao, left_tibia_id, p.VELOCITY_CONTROL, targetVelocity=0.1, force=50)

	p.stepSimulation()
	target+=timeStep
	
	time.sleep(timeStep)
     

plt.plot(tibia_positions_x, tibia_positions_y)
plt.show()

# Save the plot to a file (in case showing doesn't work)
plt.savefig('trajectory_plot.png')

