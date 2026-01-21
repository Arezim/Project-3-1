#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState


# --- Simple SDF: a small red ball ---
BALL_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <sphere><radius>0.20</radius></sphere>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <sphere><radius>0.20</radius></sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def make_pose(x, y, z):
    p = Pose()
    p.position = Point(x, y, z)
    p.orientation = Quaternion(0, 0, 0, 1)
    return p


def build_ball_sdf(model_name):
    return BALL_SDF_TEMPLATE.format(model_name=model_name)


def main():
    rospy.init_node("spawn_move_delete_and_clear_octomap")

    # --- Params you may want to tweak ---
    model_name = rospy.get_param("~model_name", "moving_ball")
    reference_frame = rospy.get_param("~reference_frame", "world")

    spawn_xyz = rospy.get_param("~spawn_xyz", [-0.371791, 0.191214, 1.229632])   # start position
    robot_xyz = rospy.get_param("~robot_xyz", [-0.909682, -1.335030, 0.873373])   # where to move toward
    robot_model_name = rospy.get_param("~robot_model_name", "")  # optional Gazebo model name
    speed_mps = float(rospy.get_param("~speed_mps", 0.15))        # movement speed
    lifetime_s = float(rospy.get_param("~lifetime_s", 10.0))      # delete after this

    octomap_clear_period_s = float(rospy.get_param("~octomap_clear_period_s", 2.0))

    # --- Wait for Gazebo services ---
    rospy.loginfo("Waiting for Gazebo services...")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/get_model_state")

    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    get_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    # --- Octomap clear service (optional; only used if available) ---
    clear_octomap = None
    try:
        rospy.wait_for_service("/clear_octomap", timeout=5.0)
        clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        rospy.loginfo("Found /clear_octomap service. Will clear every %.1fs.", octomap_clear_period_s)
    except rospy.ROSException:
        rospy.logwarn("No /clear_octomap service found. (Continuing without octomap clearing.)")

    # --- Spawn model (delete if it already exists) ---
    try:
        delete_srv(model_name)
    except Exception:
        pass  # ok if it didn't exist

    rospy.loginfo("Spawning model '%s'...", model_name)
    spawn_pose = make_pose(spawn_xyz[0], spawn_xyz[1], spawn_xyz[2])
    spawn_xml = build_ball_sdf(model_name)
    resp = spawn_srv(model_name, spawn_xml, "/", spawn_pose, reference_frame)
    if not resp.success and reference_frame:
        rospy.logwarn(
            "spawn_sdf_model failed in frame '%s': %s. Retrying in world frame.",
            reference_frame,
            resp.status_message,
        )
        resp = spawn_srv(model_name, spawn_xml, "/", spawn_pose, "")
    if not resp.success:
        rospy.logerr("spawn_sdf_model failed: %s", resp.status_message)
        return
    rospy.loginfo("spawn_sdf_model success: %s", resp.status_message)
    try:
        state = get_state_srv(model_name, "")
        rospy.loginfo(
            "Model '%s' pose after spawn: (%.3f, %.3f, %.3f)",
            model_name,
            state.pose.position.x,
            state.pose.position.y,
            state.pose.position.z,
        )
    except rospy.ServiceException as e:
        rospy.logwarn("get_model_state failed: %s", str(e))

    # --- Timers ---
    start_time = rospy.Time.now()
    last_clear_time = rospy.Time(0)

    rate = rospy.Rate(30)  # movement update rate
    rospy.loginfo("Moving '%s' toward robot for %.1fs, then deleting.", model_name, lifetime_s)

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start_time).to_sec()

        # Periodic octomap clear (every N seconds)
        if clear_octomap is not None:
            if (rospy.Time.now() - last_clear_time).to_sec() >= octomap_clear_period_s:
                try:
                    clear_octomap()
                    last_clear_time = rospy.Time.now()
                except rospy.ServiceException as e:
                    rospy.logwarn("clear_octomap failed: %s", str(e))

        # Delete after lifetime
        if t >= lifetime_s:
            rospy.loginfo("Deleting model '%s'...", model_name)
            try:
                delete_srv(model_name)
            except rospy.ServiceException as e:
                rospy.logwarn("delete_model failed: %s", str(e))
            break

        # Move toward target
        if robot_model_name:
            try:
                robot_state = get_state_srv(robot_model_name, "")
                robot_xyz = [
                    robot_state.pose.position.x,
                    robot_state.pose.position.y,
                    robot_state.pose.position.z,
                ]
            except rospy.ServiceException as e:
                rospy.logwarn("get_model_state(%s) failed: %s", robot_model_name, str(e))

        # Get current position estimate (we integrate ourselves for simplicity)
        # Compute a straight-line step toward robot_xyz
        # Current position = spawn + speed * t toward target (capped at target)
        dx = robot_xyz[0] - spawn_xyz[0]
        dy = robot_xyz[1] - spawn_xyz[1]
        dz = robot_xyz[2] - spawn_xyz[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz) + 1e-9

        travel = min(speed_mps * t, dist)
        x = spawn_xyz[0] + (dx/dist) * travel
        y = spawn_xyz[1] + (dy/dist) * travel
        z = spawn_xyz[2] + (dz/dist) * travel

        state = ModelState()
        state.model_name = model_name
        state.reference_frame = reference_frame
        state.pose = make_pose(x, y, z)
        state.twist = Twist()  # keep twist zero; we "teleport" smoothly

        try:
            set_state_srv(state)
        except rospy.ServiceException as e:
            rospy.logwarn("set_model_state failed: %s", str(e))

        rate.sleep()


if __name__ == "__main__":
    main()
