import sys
import os
from opcua import ua, uamethod, Server
import json
from datetime import datetime
import rospy
from corosect_msgs.msg import heartbeat
from corosect_msgs.srv import command_m_robot
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import String

import threading
# TODO Pilot Detections folder should not be empty! Possibly incomplete pull from github missing sub-package.
from test_return import threading_tranding
import trajectory_cartesianState_translator

asset_name = 'AASMRobotVI'

vi_task_id = 0  # initialize value for taskID
m_robot_task_id = 0
vi_status = "Ready"

ROOTDIR = "./corosect/corosect/smart_man_moveit_config/scripts/"
server_ip = "192.168.1.190"
server_port = "4844"

global publish_dir
global clients

lock = threading.Lock()

# class for OPC-Ua Server
class OPCUAServer:

    def __init__(self, name, endpoint, namespace,
                 xml_path):  # used for initalizing the values required for the OPC-UA Server
        # Configuration
        print("---> Initializing")
        self.servername = name
        self.server = Server()
        self.connected_clients = None
        self.server.import_xml(xml_path)
        self.idx = self.server.get_namespace_index(namespace)
        self.server.set_endpoint(endpoint)
        self.server.set_server_name(name)

    def start_server(self):  # to start the server
        # Start server
        print("---> Setup of", self.servername, '...')
        try:
            self.server.start()
            print('---> Success')
        except OSError as e:
            print(e)
        return self

    def link_methods(
            self):  # This method is used for linking all the methods defined in information model (AAS) with the actual implemented method here in this server.
        for function in method_submodel_mapping.keys():
            self.myobject_nodeid = self.get_operation_nodeid(function)  # calling helper method to get node id
            self.myobject = self.server.get_node(self.myobject_nodeid)
            self.server.link_method(self.myobject,
                                    function)  # linking of operation defined in information model and its actual implementation here

    def get_operation_nodeid(self,
                             function_name):  # to reterive node id defined in the infromation model for the given hierarichal address
        self.submodel, self.operation_name = method_submodel_mapping[function_name]
        self.myobject_nodeid = self.server.get_objects_node().get_child(
            ["0:AASROOT", "0:{}".format(asset_name), "0:{}".format(self.submodel), "0:{}".format(self.operation_name),
             "0:Operation"]).nodeid
        return self.myobject_nodeid

    def get_property_node(self, property_name):
        self.submodel = property_submodel_mapping[property_name]
        self.myobject_node = self.server.get_objects_node().get_child(
            ["0:AASROOT", "0:{}".format(asset_name), "0:{}".format(self.submodel), "0:{}".format(property_name),
             "0:Value"])
        return self.myobject_node

    def update_opcua_variables(self, property_name, data):  # this method will be used for setting/updating the values in the address space / information model (AAS) developed by HSEL
        self.myobject_node = self.get_property_node(property_name)
        self.connected_clients = self.server.bserver.clients
        data_publish = ua.DataValue(data)
        data_publish.SourceTimestamp = datetime.utcnow()
        data_publish.ServerTimestamp = datetime.utcnow()
        self.myobject_node.set_value(data_publish)

    # for closing the server
    def close_server(self):
        print("Closing Server", self.servername, "....")
        self.server.stop()

    # change props for status
    def changeProp(filename, key, new_value):
        with lock:
            # change value
            with open(filename) as fp:
                listObj = json.load(fp)
            listObj[key] = new_value

            with open(filename, 'w') as json_file:
                json.dump(listObj, json_file,
                          indent=4,
                          separators=(',', ': '))

    # write properties-VI results to json
    # needed for prepilots. No need for invertapro
    def write_props_to_json(filename, id, path, anomaly, counting, inspection_type, farm, crate, dol, task_id, k):
        listObj = []
        with open(filename) as fp:
            listObj = json.load(fp)
        if k == 0:
            listObj.clear()
        if inspection_type == 0:
            prop = {
                "Timestamp": str(datetime.utcnow()),
                "Task_id": task_id + 1,
                "Farm": vi_configured_farm_type[farm],
                "CrateID": crate,
                "DOL": vi_configured_dol[dol],
                "Image_Id": id,
                "Image_Path": path,
                "Abnormalities": anomaly,
                "Count": counting
            }
        elif inspection_type == 1:
            prop = {
                "Timestamp": str(datetime.utcnow()),
                "Task_id": task_id + 1,
                "Farm": vi_configured_farm_type[farm],
                "CrateID": crate,
                "DOL": vi_configured_dol[dol],
                "Image_Id": id,
                "Image_Path": path,
                "Abnormalities": anomaly
            }
        listObj.append(prop)
        with open(filename, 'w') as json_file:
            json.dump(listObj, json_file,
                      indent=4,
                      separators=(',', ': '))


# OPC-UA Methods

# Functions for the M-Robot+
@uamethod  # To make MRobot execute a predefined task
def mrobot_execute_task(parent, task_id):
    if task_id is not None:
        print('Task ID:', task_id, 'called on', datetime.now().strftime("%d/%m/%Y %H:%M:%S"))
        rospy.wait_for_service('/real_world/task_manager/execute_task')
        try:
            perform_task = rospy.ServiceProxy('/real_world/task_manager/execute_task', command_m_robot)
            response = perform_task(task_id)
            return response.success
        except rospy.ServiceException as e:
            print('Service call failed: %s'%e)
            return False
        

@uamethod  # Stop execution of current tasks/motions
def mrobot_stop(parent):
    stop_cmd = "stop"
    return True


@uamethod  # Continue task after stop (if possible)
def mrobot_resume(parent):
    resume_cmd = "resume"
    return True


@uamethod  # Return exceptions trace
def mrobot_return_error_stack(parent):
    error_msg = 'MRobot Error --  see KUKA SmartPad'
    return error_msg, True


@uamethod  # Emergency T1 stopincorrect
def mrobot_emergency_stop(parent):
    em_stop_cmd = "emergency_stop"
    return True


@uamethod  # MES requests vision system to start visual inspection of a crate
def vi_start(parent, InspectionType, Farm, CrateID, DOL, Datetime):
    pass


@uamethod  # Immediately stop all curently running tasks
def vi_stop(parent):
    pass


@uamethod  # Return exceptions trace
def vi_return_error_stack(parent):
    pass


@uamethod  # return the recorded historical analysis data
def vi_hist_data(parent, CrateID, InspectionType, StartDate, EndDate, RecentRange):
    pass

#-------------VI ROS Subscriber-------------
#split the published string into property and value using @
def parsing_prop(vi_prop_val):
    vi_prop_val=vi_prop_val.split("@")
    vi_prop=vi_prop_val[0]
    vi_val=vi_prop_val[1]
    return vi_prop,vi_val

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "status : %s", data.data)

    #split the published string into property and value
    ua_prop=parsing_prop(data.data)
    
    #change opcua poperty
    if ua_prop[1]=="VIResults":
        vi_results=json.load(open('/home/moveit2/corosect_ws/src/corosect/corosect_vi/scripts/AAS_VI_ICF/output_results/final.json'))
        myserver.update_opcua_variables("VIResults",json.dumps(vi_results))

    else:
        myserver.update_opcua_variables(ua_prop[0],ua_prop[1])


class DataCollector:

    def __init__(self) -> None:
        rospy.init_node('heartbeat_subscriber')
        self.movement_status = 0
        self.status = 0
        self.task_id = 0
        self.task_status = 0
        self.workspace_free = 0
        self.latest_trajectory = ''

        rospy.Subscriber('/real_world/heartbeat', heartbeat, self.heartbeat_callback)
        rospy.Subscriber('/real_world/arm_controller/follow_joint_trajectory/goal',
                          FollowJointTrajectoryActionGoal,
                          self.arm_controller_goal_callback)
        rospy.Subscriber("/real_world/vi", String, callback)
        self.p = rospy.Publisher('/real_world/joint_states_string', String, queue_size=1)

    def heartbeat_callback(self, data):
        self.movement_status = data.movement_status
        self.status = data.status
        self.task_id = data.task_id
        self.task_status = data.task_status
        self.workspace_free = data.workspace_free

    def arm_controller_goal_callback(self, msg):
        buffer = ''
        points = msg.goal.trajectory.points
        for i in range(len(points)):
            point = points[i]
            buffer += ','.join([f'{joint_position:.8f}' for joint_position in point.positions])
            if i != len(points) - 1:
                buffer += '@'
        link_trajectory = trajectory_cartesianState_translator.get_link_positions(joint_positions=buffer, visualize=False, index=0)
        self.latest_trajectory = link_trajectory
        self.p.publish(buffer)


if __name__ == "__main__":

    D = DataCollector()
    trend_thread = threading_tranding()

    # All the configuration messages
    inspections_configured = {
        0: "overall",
        1: "abnormalities"
    }

    mrobot_task_configured = {
        2: "Invertapro Take Sample",
        4: "Invertapro Replace Inlay",
        6: "Invertapro Prepare Crate",
        7: "Invertapro Remove Tipped Inlay",
        8: "Invertapro Crate Inspection",
        9: "Invertapro Remove Inlay and Stack",
        12: "ICF VI",
        13: "ICF Dispose Objects",
        14: "ICF Replenish Objects",
        21: "Nasekomo Return Home Task",
        22: "Nasekomo Take Sample",
        23: "Nasekomo Tent Prep Task",
        24: "Nasekomo OD Management",
        25: "Nasekomo Open-Day Task",
        99: "Homing Task",
        10001: "Certh Learning from Human Demonstration"
    }

    mrobot_movement_status_configured = {
        0: "Not Moving",
        1: "Moving",
    }

    vi_configured_inspection_type = {0: "overall", 1: "abnormalities"}

    vi_configured_farm_type = {0: "Nasekomo", 1: "ICF", 2: "Entocycle", 3: "Invertapro"}

    vi_configured_dol = {0: "eggs", 1: "larvae", 2: "pupae", 3: "adult"}

    status_configured = {-1: 'Not Initialized', 0: 'Available', 1: 'Task in process', 2: 'Error'}

    task_status_configured = {-1: 'No task configured'}

    property_submodel_mapping = {

        "VIResults": "OperationalData",

        "MRobotMovementStatus": "AssetConditionMonitoring",
        "MRobotStatus": "AssetConditionMonitoring",
        "MRobotObjectHeld": "AssetConditionMonitoring",
        "MRobotTaskStatus": "AssetConditionMonitoring",
        "MRobotTaskID": "AssetConditionMonitoring",
        "MRobotTrajectory": "AssetConditionMonitoring",
        "VITaskStatus": "AssetConditionMonitoring",
        "VITaskID": "AssetConditionMonitoring",
        "VIStatus": "AssetConditionMonitoring",
        "CobotDRobotWorkspaceFree": "AssetConditionMonitoring",

        "MRobotTaskConfigured": "TechnicalData",
        "MRobotMovementStatusConfigured": "TechnicalData",
        "VIConfiguredInspectionType": "TechnicalData",
        "VIConfiguredFarmType": "TechnicalData",
        "VIConfiguredDOL": "TechnicalData",
        "StatusConfigured": "TechnicalData",
        "TaskStatusConfigured": "TechnicalData",

        "ManufacturerName": "Nameplate",
        "ManufacturerProductDesignation": "Nameplate",
        "CountryCode": "Nameplate",
        "Street": "Nameplate",
        "Zip": "Nameplate",
        "CityTown": "Nameplate",
        "StateCounty": "Nameplate",
        "ManufacturerProductFamily": "Nameplate",
        "YearOfConstruction": "Nameplate",
        "SerialNumber": "1",
        "ClassificationSystem": "Nameplate",
        "DateOfManufacture": "Nameplate",
        "ProductCountryOfOrigin": "Nameplate",
        "QrCode": "Nameplate",
        "ProductIdentifier": "Nameplate"

    }  # for defining all the properties and their hierarchy in the infromation model Eg: PropertyName as per AAS Excel Sheet : SubmodelName as per AAS Excel Sheet

    method_submodel_mapping = {

        mrobot_execute_task: ("OperationalCapability", "MRobotExecuteTask"),
        mrobot_stop: ("OperationalCapability", "MRobotStop"),
        mrobot_resume: ("OperationalCapability", "MRobotResume"),
        mrobot_return_error_stack: ("OperationalCapability", "MRobotReturnStackError"),
        mrobot_emergency_stop: ("OperationalCapability", "MRobotEmergencyStop"),
        vi_start: ("OperationalCapability", "VIStart"),
        vi_stop: ("OperationalCapability", "VIStop"),
        vi_return_error_stack: ("OperationalCapability", "VIReturnStackError"),
        vi_hist_data: ("OperationalCapability", "VIHistData"),

    }  # for defining all the operations and their hierarchy in the infromation model Eg: actual funtion name: submodel_name and operation name as per AAS excel Sheet

    try:
        server_name = "MRobot OPCUA Server"  # name of the opc-ua server
        endpoint_address = "opc.tcp://" + server_ip + ":" + server_port  # End point of the server you wish to create.
        xml_path = './corosect/corosect/smart_man_moveit_config/scripts/AAS_MRobot_VI_v2Rev3.xml'  # PLEASE CHANGE THE PATH TO THE INFORMATION MODEL (AAS) DELIVERED BY THE HSEL
        namespace = "http://admin-shell.io/i4aas/instance/"  # NEED NOT BE CHANGED: EASIER FOR IMS TO ACESS IF KEPT COMMON
        myserver = OPCUAServer(server_name, endpoint_address, namespace, xml_path)
        myserver.start_server()  # FOR STARTING THE SERVER
        myserver.link_methods()  # calling link methods to link all the methods defined here with the operations defined in Information Model # DO NOT REMOVE THIS

        # updating the OPC-UA variables

        # update technical data. These are configuration messages --> No need of while loop.
        myserver.update_opcua_variables('StatusConfigured', json.dumps(status_configured))
        myserver.update_opcua_variables('MRobotMovementStatusConfigured', json.dumps(mrobot_movement_status_configured))
        myserver.update_opcua_variables('MRobotTaskConfigured', json.dumps(mrobot_task_configured))
        myserver.update_opcua_variables('VIConfiguredInspectionType', json.dumps(inspections_configured))
        myserver.update_opcua_variables('VIConfiguredDOL', json.dumps(vi_configured_dol))
        myserver.update_opcua_variables('VIConfiguredFarmType', json.dumps(vi_configured_farm_type))
        myserver.update_opcua_variables('TaskStatusConfigured', json.dumps(task_status_configured))

        def change_nodes():
            connections = 0
            unique_clients = []

            while True:
                try:
                    n_clients = len(myserver.connected_clients)
                    if connections < n_clients:
                        new = None
                        for client in myserver.connected_clients:
                            if client not in unique_clients:
                                new = client
                        unique_clients = [c for c in myserver.connected_clients]
                        connections = n_clients
                        print('---> Client Connected:', new, 'on', datetime.now().strftime("%d/%m/%Y %H:%M:%S"))

                    elif connections > n_clients:
                        lost = None
                        for client in unique_clients:
                            if client not in myserver.connected_clients:
                                lost = client
                        unique_clients = [c for c in myserver.connected_clients]
                        connections = n_clients
                        print('---> Client Disconnected:', lost, 'on', datetime.now().strftime("%d/%m/%Y %H:%M:%S"))

                except Exception as connection_error:
                    print('Connection Error:', connection_error)

                try:
                     # update operational data

                    # update monitoring data
                    myserver.update_opcua_variables('MRobotStatus', D.status)
                    myserver.update_opcua_variables('MRobotMovementStatus', D.movement_status)
                    myserver.update_opcua_variables('MRobotTaskID', D.task_id)
                    myserver.update_opcua_variables('MRobotTaskStatus', D.task_status)
                    myserver.update_opcua_variables('CobotDRobotWorkspaceFree', D.workspace_free)
                    myserver.update_opcua_variables('MRobotTrajectory', D.latest_trajectory),     
                except Exception as update_error:
                    print('Update Error:', update_error)

        change_nodes()
    except Exception as setup_error:
        print("Unexpected error:", setup_error)
