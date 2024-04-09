import rospy
from geometry_msgs.msg import PointStamped
from dynamic_reconfigure.client import Client
import uuid

def update_prohibition_area(x, y):
    new_area = [
        [x - 1, y],
        [x + 1, y]
    ]

    global_param_name = '/move_base/global_costmap/costmap_prohibition_layer/prohibition_areas'
    global_areas = rospy.get_param(global_param_name, [])
    rospy.loginfo("Current global prohibition areas: %s", global_areas)

    global_areas.append(new_area)

    try:
        rospy.set_param(global_param_name, global_areas)
        rospy.loginfo("Successfully updated global prohibition areas: %s", global_areas)
    except Exception as e:
        rospy.logerr("Failed to update global prohibition areas: %s", e)

    local_param_name = '/move_base/local_costmap/costmap_prohibition_layer/prohibition_areas'
    local_areas = rospy.get_param(local_param_name, [])
    rospy.loginfo("Current local prohibition areas: %s", local_areas)

    local_areas.append(new_area)

    try:
        rospy.set_param(local_param_name, local_areas)
        rospy.loginfo("Successfully updated local prohibition areas: %s", local_areas)
    except Exception as e:
        rospy.logerr("Failed to update local prohibition areas: %s", e)

    reload_params()

def reload_params():
    # 生成一个唯一的触发器值
    trigger_value = str(uuid.uuid4())

    # 更新全局和局部代价地图的触发器参数
    move_base_client = Client("/move_base/global_costmap/costmap_prohibition_layer")
    move_base_client.update_configuration({"prohibition_areas_reload_trigger": trigger_value})

    move_base_client = Client("/move_base/local_costmap/costmap_prohibition_layer")
    move_base_client.update_configuration({"prohibition_areas_reload_trigger": trigger_value})

    rospy.loginfo("Successfully triggered move_base to reload prohibition area parameters with trigger value: %s", trigger_value)

def cone_position_callback(msg):
    x, y = msg.point.x, msg.point.y
    rospy.loginfo("Received cone position: x = %s, y = %s", x, y)
    update_prohibition_area(x, y)

def listener():
    rospy.init_node('cone_virtual_wall_generator', anonymous=True)
    rospy.Subscriber("/gazebo/cone_position", PointStamped, cone_position_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
