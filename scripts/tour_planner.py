import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import habitat
import habitat_sim.bindings as hsim
import magnum as mn
import csv

rospy.init_node("get_points",anonymous=False)

class tour_planner:
	selection_done = False
	_sensor_rate = 50  # hz
	_r = rospy.Rate(_sensor_rate)
	def __init__(self):
		self._pub_plan = rospy.Publisher("~global_plan", Path, queue_size=0)
		self.selected_points = []
		self.final_plan = []

	def generate_plan(self):
		self.final_plan = self.selected_points
		with open('../tour_planning/my_src/handpicked_points.csv', newline='') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				self.final_plan.append(list(map(float,row)))

		self.final_plan = np.asarray(self.final_plan)*0.025
		print(self.final_plan)
		self.publish_plan()

	def publish_plan(self):
		msg = Path()
		msg.header.frame_id = "/map"
		msg.header.stamp = rospy.Time.now()
		for wp in self.final_plan:
			pose = PoseStamped()
			pose.pose.position.x = wp[0]
			pose.pose.position.y = wp[1]
			pose.pose.position.z = 0.0
			pose.pose.orientation.x = 0.0
			pose.pose.orientation.y = 0.0
			pose.pose.orientation.z = 0.0
			pose.pose.orientation.w = 1.0
			msg.poses.append(pose)
		rospy.loginfo("Publishing Plan...")
		self._pub_plan.publish(msg)
		self._r.sleep()


def callback(point, tour_plan):
    if (tour_plan.selection_done == True):
    	print("Already selected points, going to generate plan now!")
    	tour_plan.generate_plan()
    	return
    tour_plan.selected_points.append([point.point.x, point.point.y, 0.0])
    tour_plan.selection_done = bool(input("Want to finish selection?"))


def main():
	tour_plan = tour_planner()
	
	rospy.Subscriber("/clicked_point", PointStamped,callback, tour_plan,queue_size=1)
	rospy.spin()
    # # define a list capturing how long it took
    # # to update agent orientation for past 3 instances
    # # TODO modify dt_list to depend on r1
    # dt_list = [0.009, 0.009, 0.009]
	while not rospy.is_shutdown():
		tour_plan.generate_plan()
    #     start_time = time.time()
    #     # cv2.imshow("bc_sensor", my_env.observations['bc_sensor'])
    #     # cv2.waitKey(100)
    #     # time.sleep(0.1)
    #     my_env.update_orientation()

    #     dt_list.insert(0, time.time() - start_time)
    #     dt_list.pop()
    #     my_env.set_dt(sum(dt_list) / len(dt_list))


if __name__ == "__main__":
    main()