import numpy as np
import json
import pybullet as p
import pybullet_data
# from trajectory_subscriber import DataCollector

# DO NOT CHANGE
kuka_joints = [1, 2, 3, 4, 5, 6, 7]
kuka_links = [1, 2, 3, 4, 5, 6, 7, 8]
flange = 7
drive_joints = [11, 10, 12]  # order is not conserved in publisher
drive_links = [10, 11, 12, 13]  # link 9 has no physical component hence is skipped

robot_model_path = '/home/moveit2/Documents/CS_SMART_M_Group_04/final_setup.urdf'


class SaveVerificationException(Exception):
    """
    Custom Exception for when a .json save file cannot be verified within acceptable tolerance.
    """
    def __init__(self,
                 message="Large error in loaded json, save file may be corrupted."):
        self.message = message
        super().__init__(self.message)


class JointParseException(Exception):
    """
    Custom Exception for when a trajectory point does not contain the expected amount of joints.
    """
    def __init__(self,
                 point,
                 message="Trajectory component does not contain all joint states, skipping trajectory point "):
        self.message = message + str(point)
        super().__init__(self.message)


def get_link_positions(joint_positions, visualize=False, index=0):
    """
    Main function which connects to pybullet, generates a model of the KUKA robot and computes link positions in
    Cartesian space.
    :param visualize: show visual representation in Pybullet rendering. Helps build intuition.
    :param joint_positions: the string of joint positions as received from the MES.

    DATA FORMAT
    The final result is published as a single string delimited by '&' between 12 links of the robot. Each link string then contains
    a trajectory of n poses delimited by '@'. 

    result = 'l_1 & l_2 & ... & l_12'
        where 
            l_i = 'x_0, y_0, z_0, Rx_0, Ry_0, Rz_0 @ x_1, y_1, z_1, Rx_1, Ry_1, Rz_1 @ ... @ x_n, y_n, z_n, Rx_n, Ry_n, Rz_n'
        where 
            n = number of points in the trajectory (determined by motion planning)

    """
    if visualize:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load the robot
    robot_model = p.loadURDF(robot_model_path, [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
    # Parse string received from MES
    full_trajectory_matrix = parse_joint_positions(joint_positions)

    # Compute Cartesian positions of all links at all trajectory points. Physical links are listed at the top.
    link_states_list = transform_trajectory_to_link_states(full_trajectory_matrix, robot_model)
    # print(link_states_list)  # UNCOMMENT TO VIEW IN CONSOLE

    # Parse into strings delimited by '@' of Cartesian poses for each link in the robot.
    cartesian_strings, link_ids = matrix_list_to_cartesian_string(link_states_list)

    ''' 
    Gather data into a single string variable for publishing in the AAS. The '&' symbol is used to delimit each link
    that is being published.
    '''
    result = cartesian_strings[0]
    for string in cartesian_strings[1:]:
        result += '&' + string

    # Save Cartesian positions of all links at all trajectory points.
    cartesian_string_to_file(cartesian_strings, link_ids, file_path='link_states_trajectory_' + str(index) + '.txt')

    if visualize:
        while True:
            continue
    '''
    Assuming the entire trajectory is parsed correctly, the link_states_list is list of np matrices. Each matrix is 
    dimensions 6x12. The format is the [x, y, z, rx, ry, rz] position for each of the 12 links that make the physical
    components of the robot. NOTE: links and joints are not the same! 
    '''
    return result


def cartesian_string_to_file(link_strings, link_ids, file_path='cartesian_link_states.txt'):
    """
    Method to write the Cartesian link positions to a .txt file for convenience. The first line is always the indices of
    the links for which Cartesian poses are listed below, in order. Links as can be referenced at the .urdf file.
    This method will write at most 12 additional lines since there are 12 links in the robot, depending on which ones
    were referenced as relevant. Each line is then an '@' delimited string of Cartesian poses.
    :param link_strings: list of strings containing Cartesian poses for each link in the robot.
    :param link_ids:
    :param file_path: path to .txt.
    """
    with open(file_path, 'w') as file:
        file.writelines(link_ids + '\n')
        for string in link_strings:
            file.writelines(string + '\n')


def matrix_list_to_cartesian_string(link_states_list, relevant_links=None, save=False):
    """
    Method to convert a link states matrix into a string of Cartesian poses for the IMS. Allows you to select which
    links you are interested in tracking through the trajectory. If none are specified, will generate a list of strings
    for each of the links of the robot. NOTE: the relevant_links parameter references the .urdf robot description file.
    Not all the links have physical components and are hence skipped. Make sure you align with this in the HoloLens 2.
    :param link_states_list: a list containing 6x12 matrices containing information about Cartesian position of each
    12 link at every point along the trajectory.
    :param relevant_links: a list of link indices for which to produce a string.
    :param save: writes to a .txt file. See cartesian_string_to_file() for format.
    :return: link_strings is a list of strings where each string is the '@' delimited Cartesian poses for the relevant
    link along a trajectory. link_ids is a list of ids for which link a string belongs to.
    """
    if relevant_links is None:
        relevant_links = np.arange(len(kuka_links) + len(drive_links))
    link_ids = np.array2string(np.array([(kuka_links + drive_links)[i] for i in relevant_links]))
    link_strings = []
    for link in relevant_links:
        string = ''
        for matrix in link_states_list:
            if string == '':
                string += np.array2string(matrix[link, :], separator=',')\
                    .replace('\n', '') \
                    .replace(' ', '') \
                    .replace('[', '') \
                    .replace(']', '')
            else:
                string += '@' + np.array2string(matrix[link, :], separator=',') \
                    .replace('\n', '') \
                    .replace(' ', '') \
                    .replace('[', '') \
                    .replace(']', '')
        link_strings.append(string)
    if save:
        cartesian_string_to_file(link_strings, link_ids)
    return link_strings, link_ids


def verify_integrity(link_states_list, new_link_states_list):
    """
    Method used to make sure the saved and loaded json contain the same data about link states.
    :param link_states_list: calculated link states directly from pybullet.
    :param new_link_states_list: link states loaded from json.
    """
    for matrix, new_matrix in zip(link_states_list, new_link_states_list):
        x = np.sum(matrix - new_matrix)
        if x > 1e-6:
            raise SaveVerificationException()


def write_links_to_file(link_states, file_path='link_states.json'):
    """
    Write the link_states to a json. Enumerated in a dict {trajectory_point: matrix}. Each matrix holds the Cartesian
    coordinates for each of the 12 links of the KUKA lbr iiwa 14 and Linear Drive.
    :param link_states: list of matrices to save to a file.
    :param file_path: path to .json.
    """
    save_dict = {}
    for point, link in enumerate(link_states):
        save_dict[str(point)] = (np.array2string(link, separator=',')
                                 .replace('\n', '')
                                 .replace(' ', ''))
    with open(file_path, 'w') as file:
        json.dump(save_dict, file)
        file.close()
    verify_integrity(link_states, read_links_from_file(file_path))


def read_links_from_file(file_path='link_states.json'):
    """
    Read the json file containing saved data into a list of matrices. Each matrix is dimensions 6x12.
    The format is the [x, y, z, rx, ry, rz] position for each of the 12 links that make the physical components of the
    robot.
    :param file_path: where the .json is saved.
    :return: a list containing 6x12 np matrices containing information about Cartesian position of each 12 link at every
    point along the trajectory.
    """
    with open(file_path, 'r') as file:
        load_dict = json.load(file)
        file.close()
    link_states_list = []
    for string_matrix in load_dict.values():
        link_states = []
        for coordinate in string_matrix[2:-1].split('],['):
            link_states.append(np.fromstring(coordinate, dtype=float, sep=','))
        link_states_list.append(np.array(link_states))
    return link_states_list


def transform_trajectory_to_link_states(trajectory, robot):
    """
    Transform a trajectory to a list of Cartesian positions for each link.
    :param trajectory: a parsed matrix of (n_points x n_joints), as determined from parsing the trajectory.
    :param robot: the pybullet instance of the KUKA robot.
    :return: a list containing 6x12 matrices containing information about Cartesian position of each 12 link at every point
    along the trajectory.
    """
    states_along_trajectory = []
    n_points = trajectory.shape[0]
    for trajectory_point in range(n_points):
        kuka_joint_positions = trajectory[trajectory_point][:7]
        drive_joint_positions = trajectory[trajectory_point][7:]
        for i, j in zip(kuka_joints, kuka_joint_positions):
            p.resetJointState(robot, i, j)
        for i, j in zip(drive_joints, drive_joint_positions):
            p.resetJointState(robot, i, j)
        link_data = []
        get_link_states(kuka_links[0], kuka_links[-1], link_data, robot)
        get_link_states(drive_links[0], drive_links[-1], link_data, robot)
        states_along_trajectory.append(np.array(link_data))
    return states_along_trajectory


def get_link_states(start, end, data, robot_model):
    """
    Recursive method to build the list of link information from the ground up to the last joint.
    :param start: which link to start with.
    :param end: which link to end with.
    :param data: placeholder for link states.
    :param robot_model: pybullet instance of the KUKA robot.
    """
    data.append(p.getLinkState(robot_model, start)[0] + p.getEulerFromQuaternion(p.getLinkState(robot_model, start)[1]))
    if start == end:
        return
    else:
        get_link_states(start + 1, end, data, robot_model)


def parse_joint_positions(joint_positions_string):
    """
    Parse the string of the trajectory from the MES into a matrix.
    :param joint_positions_string: trajectory separated by '@' for each point.
    :return: numpy matrix of (n_points x n_joints).
    """
    joint_trajectory = []
    string_data = joint_positions_string.split('@')
    for position in string_data:
        try:
            position_list = [float(i) for i in position.split(',')]
            if len(position_list) != 10:
                raise JointParseException(position_list)
            else:
                joint_trajectory.append(np.array(position_list))
        except Exception as e:
            print(e)
    return np.array(joint_trajectory)


if __name__ == "__main__":
    # Sample Trajectory of 2 points. 10 joints are being controlled. 0-6 are the KUKA, 7-10 are the Linear Drive.
    # NOTE index 8 is in m not rad (position of linear rail).

    # Wait for a trajectory to be received by the data collector
    '''trajectory_record = DataCollector()
    while trajectory_record.latest_trajectory == "":
        continue
    '''
    # with open('doc.txt', 'r') as file:
    #     trajectories = file.readlines()

    dummy = ["1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,0.1,1.5@-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,0.3,-1.5"]
    for key, value in enumerate(dummy):
        trajectory_link_states = get_link_positions(joint_positions=value, visualize=False, index=key)

