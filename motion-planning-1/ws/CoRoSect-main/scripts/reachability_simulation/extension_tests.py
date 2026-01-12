import numpy as np
import time
import pybullet as p
import pybullet_data
import json

from sphere_generator import Sphere

threshold = 0.01  # threshold for reaching a point
iterations = 1  # number of random starting positions to attempt
grid_resolution = 30  # number of points linearly-spaced in each dimension
rot_resolution = 5
timeout = 0.5  # allocated time to attempt a reach
realtime = False  # debug mode
reachability = False  # Reachability tests whether centre of sphere can be reached.
# If False, manipulability is tested instead, reaching for key-points around a sphere.
debug = True

lengths = [0.5]
heights = [0.0]

est_time = (np.power(grid_resolution, 3) * iterations * len(lengths) * len(heights) * timeout) / 3600
if not reachability:
    est_time *= np.power(rot_resolution - 1, 3)
print('EST. Time:', est_time)

for height in heights:
    for length in lengths:
        if debug:
            sim_id = p.connect(p.GUI)
        else:
            sim_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        if realtime:
            p.setRealTimeSimulation(True)

        prismatic = 'fixed'
        joint = 'fixed'
        revolute = 'fixed'
        if length > 0.0:
            prismatic = 'prismatic'
            joint = 'continuous'
            revolute = 'revolute'

        add_rail = ['  <!-- joint between link_7 and rail -->\n',
                    '  <joint name="lbr_iiwa_joint_rail" type="fixed">\n',
                    '    <parent link="lbr_iiwa_link_7"/>\n',
                    '    <child link="extension_rail"/>\n',
                    '    <origin rpy="0 0 0" xyz="' + str(length / 2) + ' 0 0.05"/>\n',
                    '  </joint>\n',
                    '  <link name="extension_rail">\n',
                    '    <inertial>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0.02"/>\n',
                    '      <mass value="' + str(length / 10) + '"/>\n',
                    '      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>\n',
                    '    </inertial>\n',
                    '    <visual>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <box size="' + str(length) + ' 0.025 0.025"/>\n',
                    '      </geometry>\n',
                    '      <material name="Grey"/>\n',
                    '    </visual>\n',
                    '    <collision>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '          <box size="' + str(length) + ' 0.025 0.025"/>\n',
                    '        </geometry>\n',
                    '    </collision>\n',
                    '  </link>\n',

                    '  <!-- joint between rail and end_effector_base -->\n',
                    '  <joint name="lbr_iiwa_joint_end_base" type="' + prismatic + '">\n',
                    '    <parent link="extension_rail"/>\n',
                    '    <child link="end_effector_base"/>\n',
                    '    <origin rpy="0 0 0" xyz="' + str(-length / 2) + ' 0 0.01"/>\n',
                    '    <axis xyz="1 0 0"/>\n',
                    '    <limit lower="0.0" upper="' + str(length) + '" effort="30" velocity="0.1"/>\n',
                    '  </joint>\n',
                    '  <link name="end_effector_base">\n',
                    '    <inertial>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0.02"/>\n',
                    '      <mass value="0.01"/>\n',
                    '      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>\n',
                    '    </inertial>\n',
                    '    <visual>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <cylinder length="0.01" radius="0.025"/>\n',
                    '      </geometry>\n',
                    '      <material name="Orange"/>\n',
                    '    </visual>\n',
                    '    <collision>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <cylinder length="0.01" radius="0.025"/>\n',
                    '      </geometry>\n',
                    '    </collision>\n',
                    '  </link>\n',

                    '  <!-- joint between end_effector_base and end_rotator_joint-->\n',
                    '  <joint name="lbr_iiwa_end_rotator_joint" type="' + joint + '">\n',
                    '    <parent link="end_effector_base"/>\n',
                    '    <child link="end_rotator_joint"/>\n',
                    '    <origin rpy="0 0 0" xyz="0 0 0.015"/>\n',
                    '    <axis xyz="0 0 1"/>\n',
                    '  </joint>\n',
                    '  <link name="end_rotator_joint">\n',
                    '    <inertial>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0.02"/>\n',
                    '      <mass value="0.0001"/>\n',
                    '      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>\n',
                    '    </inertial>\n',
                    '    <visual>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <cylinder length="0.01" radius="0.025"/>\n',
                    '      </geometry>\n',
                    '      <material name="Gray"/>\n',
                    '    </visual>\n',
                    '    <collision>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <cylinder length="0.01" radius="0.025"/>\n',
                    '      </geometry>\n',
                    '    </collision>\n',
                    '  </link>\n',

                    '  <!-- joint between end_rotator_joint and end_effector_body -->\n',
                    '  <joint name="lbr_iiwa_joint_end_body" type="' + revolute + '">\n',
                    '    <parent link="end_rotator_joint"/>\n',
                    '    <child link="end_effector_body"/>\n',
                    '    <origin rpy="0 0 0" xyz="0 0 0.01"/>\n',
                    '    <axis xyz="0 1 0"/>\n',
                    '    <limit lower="0" upper="1.571"/>\n',
                    '  </joint>\n',
                    '  <link name="end_effector_body">\n',
                    '    <inertial>\n',
                    '      <origin rpy="0 0 1.571" xyz="0 0 0.01"/>\n',
                    '      <mass value="1.251"/>\n',
                    '      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>\n',
                    '    </inertial>\n',
                    '    <visual>\n',
                    '      <origin rpy="0 0 1.571" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <mesh filename="meshes/rg6.stl"/>\n',
                    '      </geometry>\n',
                    '      <material name="Blue"/>\n',
                    '    </visual>\n',
                    '    <collision>\n',
                    '      <origin rpy="0 0 1.571" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <mesh filename="meshes/rg6.stl"/>\n',
                    '      </geometry>\n',
                    '    </collision>\n',
                    '  </link>\n',

                    '  <!-- dummy end -->\n',
                    '  <joint name="dummy_joint" type="fixed">\n',
                    '    <parent link="end_effector_body"/>\n',
                    '    <child link="dummy_link"/>\n',
                    '    <origin rpy="0 0 0" xyz="0 0 0.25"/>\n',
                    '  </joint>\n',
                    '  <link name="dummy_link">\n',
                    '    <visual>\n',
                    '      <origin rpy="0 0 0" xyz="0 0 0"/>\n',
                    '      <geometry>\n',
                    '        <box size="0.025 0.025 0.025"/>\n',
                    '      </geometry>\n',
                    '      <material name="Grey"/>\n',
                    '    </visual>\n',
                    '  </link>\n']

        workspace = ['<?xml version="1.0"?>\n',
                     '<robot name="workspace_table">\n',

                     '<material name="white">\n',
                     '<color rgba="0.5 0.5 0.5 1"/>\n',
                     '</material>\n',

                     '<link name="base_link">\n',
                     '<visual>\n',
                     '<geometry>\n',
                     '<box size="0.6 1.6 ' + str(height) + '"/>\n',
                     '</geometry>\n',
                     '<material name="white"/>\n',
                     '</visual>\n',
                     '<collision>\n',
                     '<geometry>\n',
                     '<box size="1.6 1.6 ' + str(height) + '"/>\n',
                     '</geometry>\n',
                     '</collision>\n',
                     '</link>\n',
                     '</robot>\n']

        # with open('res/kuka_iiwa/original_kuka_model.urdf', 'r') as model_file:
        #     lines = model_file.readlines()[:-1]
        # model_file.close()
        # 
        # lines += add_rail
        # lines += '</robot>\n'
        # 
        # with open('res/kuka_iiwa/CoRoSect_KUKA.urdf', 'w') as new_model_file:
        #     for line in lines:
        #         new_model_file.write(line)
        # new_model_file.close()
        # 
        # with open('res/workspace.urdf', 'w') as mounting:
        #     for line in workspace:
        #         mounting.write(line)
        # mounting.close()

        base_position = np.array([0, 0, height])

        base = p.loadURDF('res/workspace.urdf', [.0, 0.6, height / 2], useFixedBase=True)
        # box = p.loadURDF('res/icf_crate.urdf', [0, -0.75, 0.12], useFixedBase=True)
        # kuka = p.loadURDF('res/kuka_iiwa/CoRoSect_KUKA.urdf',
        #                   base_position,
        #                   p.getQuaternionFromEuler([.0, .0, -np.pi / 2]),
        #                   useFixedBase=True)
        kuka = p.loadURDF('urdf/extended_iiwa.urdf',
                            base_position,
                            p.getQuaternionFromEuler([.0, .0, -np.pi / 2]),
                            useFixedBase=True)
        # icf_crate_pos = [.0, -.7, 0 + 0.13]
        # ifc_crate = p.loadURDF('res/icf_crate.urdf', icf_crate_pos, useFixedBase=True)

        joint_count = p.getNumJoints(kuka)
        print(joint_count)
        if length > 0.0:
            articulate_joints = [0, 1, 2, 3, 4, 5, 6, 9, 10, 11]
        else:
            articulate_joints = [0, 1, 2, 3, 4, 5]
        joint_ranges = np.array([[-2.966, 2.966],  # l1
                                 [-2.094, 2.094],  # l2
                                 [-2.967, 2.967],  # l3
                                 [-2.094, 2.094],  # l4
                                 [-2.967, 2.966],  # l5
                                 [-2.094, 2.094],  # l6
                                 [-3.054, 3.054],  # l7
                                 [0, 0],  # fixed ee
                                 [0, 0],  # fixed rail
                                 [0, 0.33],  # linear drive
                                 [-1.57, 1.57],  # end rotation joint
                                 [-1.57, 0],  # extra DoF
                                 [0, 0],  # ee mount
                                 [0, 0],  # ee body
                                 [0, 0]])  # dummy end-point
        # joint_ranges = np.array([[-2.9670597283903604, 2.9670597283903604],  # l1
        #                          [-3.2288591161895095, 1.1344640137963142],  # l2
        #                          [-2.3911010752322315, 2.8448866807507573],  # l3
        #                          [-3.2288591161895095, 3.2288591161895095],  # l4
        #                          [-2.0943951023931953, 2.0943951023931953],  # l5
        #                          [-6.1086523819801535, 6.1086523819801535]])  # l6

        memory = {'x': None,
                  'y': None,
                  'z': None,
                  'scores': None}

        x = np.linspace(-2, 2, grid_resolution)
        y = np.linspace(-2, 2, grid_resolution)
        z = np.linspace(0, 4, grid_resolution)
        rot_space = np.linspace(0, 2 * np.pi, rot_resolution)
        memory['x'] = x.tolist()
        memory['y'] = y.tolist()
        memory['z'] = z.tolist()

        memory['scores'] = np.zeros([grid_resolution, grid_resolution, grid_resolution])

        initial_state = np.zeros(joint_count)

        if not reachability:
            for i in range(grid_resolution):
                for j in range(grid_resolution):
                    for k in range(grid_resolution):
                        point = np.array([x[i], y[j], z[k]])
                        print('\r[%f, %f, %f]' % (point[0], point[1], point[2]))

                        s = Sphere(centre=point, radius=0.05, semi=True)
                        if np.linalg.norm(base_position - point) < 1.1 + length and point[2] > 0.0:
                            if debug:
                                point_sphere = p.loadURDF("res/sub_target.urdf",
                                                          point,
                                                          p.getQuaternionFromEuler([0., 0., 0.]),
                                                          useFixedBase=True)
                            for rx in range(rot_resolution - 1):
                                for ry in range(rot_resolution - 1):
                                    for rz in range(rot_resolution - 1):
                                        target_orientation = np.array([rot_space[rx], rot_space[ry], rot_space[rz]])
                                        print('\r[%f, %f, %f]' % (target_orientation[0],
                                                                  target_orientation[1],
                                                                  target_orientation[2]), end="")
                                        count = 0
                                        for r in range(iterations):
                                            # Reset arm position
                                            for joint in range(joint_count):
                                                p.resetJointState(kuka, jointIndex=joint, targetValue=initial_state[joint])

                                            reached = False
                                            start_time = time.time()
                                            while not reached:
                                                if not realtime:
                                                    p.stepSimulation()
                                                position = p.getLinkState(kuka, joint_count - 1)[0]
                                                # position = p.getLinkState(kuka, articulate_joints[-1])[0]
                                                target_joint_positions = p.calculateInverseKinematics(kuka,
                                                                                                      joint_count - 1,
                                                                                                      # articulate_joints[-1],
                                                                                                      targetPosition=point.tolist(),
                                                                                                      targetOrientation=p.getQuaternionFromEuler(target_orientation),
                                                                                                      lowerLimits=joint_ranges[
                                                                                                          articulate_joints, 0].tolist(),
                                                                                                      upperLimits=joint_ranges[
                                                                                                          articulate_joints, 1].tolist(),
                                                                                                      jointRanges=(joint_ranges[
                                                                                                                       articulate_joints, 1] -
                                                                                                                   joint_ranges[
                                                                                                                       articulate_joints, 0]).tolist(),
                                                                                                      restPoses=np.zeros(
                                                                                                          len(articulate_joints)).tolist(),
                                                                                                      maxNumIterations=100,
                                                                                                      solver=p.IK_DLS)
                                                # print(target_joint_positions)
                                                p.setJointMotorControlArray(kuka, jointIndices=articulate_joints,
                                                                            controlMode=p.POSITION_CONTROL,
                                                                            targetPositions=target_joint_positions)
                                                if np.linalg.norm(position - point) < threshold:
                                                    reached = True
                                                    count = 1
                                                if time.time() - start_time > timeout:
                                                    reached = True
                                        s.scores.append(count)
                            if debug:
                                p.removeBody(point_sphere)
                        memory['scores'][i, j, k] = s.average_scores()
        else:
            for i in range(grid_resolution):
                for j in range(grid_resolution):
                    for k in range(grid_resolution):
                        point = np.array([x[i], y[j], z[k]])
                        print('\r[%f, %f, %f]' % (point[0], point[1], point[2]), end="")

                        count = 0
                        if np.linalg.norm(base_position - point) < 1.45 + length and point[2] > 0.0: # out of physical range
                            if debug:
                                target = p.loadURDF("res/sub_target.urdf",
                                                    point,
                                                    p.getQuaternionFromEuler([0., 0., 0.]),
                                                    useFixedBase=True)

                            for r in range(iterations):
                                # Reset arm position
                                for joint in range(joint_count):
                                    p.resetJointState(kuka, jointIndex=joint, targetValue=initial_state[joint])

                                reached = False
                                start_time = time.time()
                                while not reached:
                                    if not realtime:
                                        p.stepSimulation()
                                    position = p.getLinkState(kuka, joint_count - 1)[0]
                                    target_joint_positions = p.calculateInverseKinematics(kuka,
                                                                                          joint_count - 1,
                                                                                          targetPosition=point,
                                                                                          lowerLimits=joint_ranges[
                                                                                              articulate_joints, 0].tolist(),
                                                                                          upperLimits=joint_ranges[
                                                                                              articulate_joints, 1].tolist(),
                                                                                          jointRanges=(joint_ranges[
                                                                                                           articulate_joints, 1] -
                                                                                                       joint_ranges[
                                                                                                           articulate_joints, 0]).tolist(),
                                                                                          restPoses=np.zeros(
                                                                                              len(articulate_joints)).tolist(),
                                                                                          maxNumIterations=100,
                                                                                          solver=p.IK_DLS)
                                    p.setJointMotorControlArray(kuka, jointIndices=articulate_joints,
                                                                controlMode=p.POSITION_CONTROL,
                                                                targetPositions=target_joint_positions)
                                    if np.linalg.norm(position - point) < threshold:
                                        reached = True
                                        count += 1
                                    if time.time() - start_time > timeout:
                                        reached = True
                            if debug:
                                p.removeBody(target)
                        memory['scores'][i, j, k] = count / iterations

        memory['scores'] = memory['scores'].tolist()
        # Save data
        with open('./save/FINAL_' + ('Reachability' if reachability else 'Manipulability') + '_' +
                  'Rail_Length-' + str(length) +
                  '_Mounting-Height-' + str(height) +
                  '.json', 'w') as file:
            json.dump(memory, file)
        file.close()

        p.removeBody(kuka)
        p.removeBody(base)
        p.disconnect(sim_id)
