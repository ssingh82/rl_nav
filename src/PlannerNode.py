#!/usr/bin/python
import roslib; roslib.load_manifest('rl_nav')
import rospy

from pathplanner import *
from numpy import *
from tf.transformations import *
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32MultiArray, Empty, String, Float32
from visualization_msgs.msg import Marker
from rl_nav.srv import ExpectedPath, ExpectedPathResponse, QPrediction, QPredictionResponse
from gazebo_msgs.msg import ModelStates
import tf
import datetime
import time
import cPickle
import rospkg

class PLanner2D(object):

	def __init__(self):

		rospack = rospkg.RosPack()
		rl_nav = rospack.get_path('rl_nav')
		self.QRegressor = cPickle.load(open(rl_nav+'/qRegressor.pkl','rb'))

		self.state = 0
		self.delay = 10
		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=100)
		#self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
		self.status_pub = rospy.Publisher('/planner/status',String,queue_size=100)
		self.inp_pub = rospy.Publisher('/planner/global/path', Float32MultiArray,queue_size=100)
		self.safe_path_pub = rospy.Publisher("/rl/safe_action", Marker, queue_size=1);
		self.unsafe_path_pub = rospy.Publisher("/rl/unsafe_action", Marker, queue_size=1);
		self.global_path_pub = rospy.Publisher("/planner/global_path", Marker, queue_size=1);
		self.lookahead_path_pub = rospy.Publisher("/planner/lookahead_path", Marker, queue_size=1);
		self.waypoint_pub = rospy.Publisher('/move_base_simple/waypoint',PoseStamped,queue_size=100)

		self.pathSRV = rospy.Service('/planner/global/expected_path', ExpectedPath, self.sendLookahead)
		self.qSRV = rospy.Service('/planner/qValue', QPrediction, self.sendQValue)
		self.VEL_SCALE = 1.0
		self.map = rospy.get_param('~map',-1)
		if(self.map==1):
			self.points = [4.0,7.0,0.0,14.0]
			self.goal = [6.0, 2.0,-tan(pi/4.0),28.0] #map 1
		elif(self.map==2):
			self.points = [2.5,5.5,tan(pi/4.0),14.0]
			self.goal = [7.0,6.5,0.0,28.0] #map 2
		elif(self.map==3):
			self.points = [4.0,4.0,tan(pi/4.0),14.0]
			self.goal = [6.0,6.0,-tan(pi/4.0),28.0] #map 3
		else:
			self.goal = []
			self.points = []
		
		self.robotState = Pose()
		self.robotPTAMPose = Pose()
		self.robotPTAMWorldPose = Pose()
		self.goalPose = None
		self.waypointPose = None

		self.rlSafePath = Marker()
		self.rlSafePath.id=0
		self.rlSafePath.lifetime=rospy.Duration(1)
		self.rlSafePath.header.frame_id = "/world"
		self.rlSafePath.header.stamp = rospy.Time.now()
		self.rlSafePath.ns = "pointcloud_publisher"
		self.rlSafePath.action = Marker.ADD
		self.rlSafePath.type = Marker.LINE_STRIP
		self.rlSafePath.color.g=1.0
		self.rlSafePath.color.a=1.0
		self.rlSafePath.scale.x=0.01
		
		self.rlUnsafePath = Marker()
		self.rlUnsafePath.id=0
		self.rlUnsafePath.lifetime=rospy.Duration(1)
		self.rlUnsafePath.header.frame_id = "/world"
		self.rlUnsafePath.header.stamp = rospy.Time.now()
		self.rlUnsafePath.ns = "pointcloud_publisher"
		self.rlUnsafePath.action = Marker.ADD
		self.rlUnsafePath.type = Marker.LINE_STRIP
		self.rlUnsafePath.color.r=1.0
		self.rlUnsafePath.color.a=1.0
		self.rlUnsafePath.scale.x=0.01

		self.GlobalPath = Marker()
		self.GlobalPath.id=0
		self.GlobalPath.lifetime=rospy.Duration(1)
		self.GlobalPath.header.frame_id = "/world"
		self.GlobalPath.header.stamp = rospy.Time.now()
		self.GlobalPath.ns = "pointcloud_publisher"
		self.GlobalPath.action = Marker.ADD
		self.GlobalPath.type = Marker.LINE_STRIP
		self.GlobalPath.color.g=1.0
		self.GlobalPath.color.a=1.0
		self.GlobalPath.scale.x=0.01

		self.lookaheadPath = Marker()
		self.lookaheadPath.id=0
		self.lookaheadPath.lifetime=rospy.Duration(1)
		self.lookaheadPath.header.frame_id = "/world"
		self.lookaheadPath.header.stamp = rospy.Time.now()
		self.lookaheadPath.ns = "pointcloud_publisher"
		self.lookaheadPath.action = Marker.ADD
		self.lookaheadPath.type = Marker.LINE_STRIP
		self.lookaheadPath.color.r=1.0
		self.lookaheadPath.color.g=1.0
		self.lookaheadPath.color.a=1.0
		self.lookaheadPath.scale.x=0.1
		

		self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.receiveGoal)
		self.waypoint_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.receiveWaypoint)
		self.ptam_pose_sub = rospy.Subscriber("/vslam/pose_world", PoseWithCovarianceStamped, self.receivePTAMPose)
		self.pose_sub = rospy.Subscriber("/my_pose", PoseStamped, self.receivePose)
		self.inp_sub = rospy.Subscriber('/planner/input', Float32MultiArray, self.receiveInput)
		self.inp_mul_sub = rospy.Subscriber('/planner/input/global', Empty, self.receiveInputGlobal)
		self.reset_sub = rospy.Subscriber('/planner/reset', Empty, self.receiveBreak)
		self.safe_traj_sub = rospy.Subscriber('/rl/safe_trajectories', Float32MultiArray, self.receiveSafeTrajs)
		self.unsafe_traj_sub = rospy.Subscriber('/rl/unsafe_trajectories', Float32MultiArray, self.receiveUnsafeTrajs)
		# self.PTAM_pose_sub = rospy.Subscriber("/vslam/pose", PoseWithCovarianceStamped, self.receivePTAMPose)
		gazeboModelStates_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.receiveGazeboModelStates)
		
		self.to = 0.0
		self.tf = 28.0

	def trajjer(self, data):
		traj_list=[]
		num_trajs = int(data[-1])
		data = data[:-1]
		if num_trajs==0:
			return traj_list
		size_trajs = int(len(data)/float(num_trajs))
		for i in range(num_trajs):
			inp = data[i*size_trajs:(i+1)*size_trajs]
			to = inp[-2]
			tf = inp[-1]
			coeffs = get_coeffs(inp[:-3],to,tf)
			
			t=to
			traj=[]
			while t<tf:
				point0,kt0 = getPos(coeffs,to,tf,t,inp[1])
				point0.x ,point0.y = point0.z, inp[-3]*point0.x
				point0.z = 0
				traj.append(point0)
				t = t + 0.1
			traj_list = traj_list + traj
			traj.reverse()
			traj_list = traj_list + traj
		return traj_list

	def receiveUnsafeTrajs(self, trajectories):
		self.rlUnsafePath.pose = self.robotPTAMPose
		self.rlUnsafePath.points = self.trajjer(trajectories.data)
		self.rlUnsafePath.header.stamp = rospy.Time.now()
		self.unsafe_path_pub.publish(self.rlUnsafePath)
				

	def receiveSafeTrajs(self, trajectories):
		self.rlSafePath.pose = self.robotPTAMPose
		self.rlSafePath.points = self.trajjer(trajectories.data)
		self.rlSafePath.header.stamp = rospy.Time.now()
		self.safe_path_pub.publish(self.rlSafePath)

	def receivePTAMPose(self, pose):
		self.robotPTAMWorldPose = pose.pose.pose

		# self.rlSafePath.header.stamp = rospy.Time.now()
		# self.safe_path_pub.publish(self.rlSafePath)

		# self.rlUnsafePath.header.stamp = rospy.Time.now()
		# self.unsafe_path_pub.publish(self.rlUnsafePath)

		self.lookahead_path_pub.publish(self.lookaheadPath)

	def receivePose(self, pose):
		self.robotPTAMPose = pose.pose

	def receiveGoal(self, goalPose):
		if self.state==0 and self.points!=[]:
			self.goalPose = goalPose
			distance1 = linalg.norm(array([self.robotPTAMPose.position.x, self.robotPTAMPose.position.y]) 
								  - array([self.points[0], self.points[1]]))
			distance2 = linalg.norm(array([goalPose.pose.position.x, goalPose.pose.position.y]) 
								  - array([self.points[0], self.points[1]]))
			self.goal =[goalPose.pose.position.x, goalPose.pose.position.y, 
						 tan(euler_from_quaternion([ goalPose.pose.orientation.x,
										goalPose.pose.orientation.y,
										goalPose.pose.orientation.z,
										goalPose.pose.orientation.w])[2]),self.tf]
			
			print 'received goal', self.goal
			self.receiveInputGlobal(Empty())

	def receiveWaypoint(self, waypointPose):
		ps = PoseStamped()
		ps.header = waypointPose.header
		ps.pose = waypointPose.pose.pose
		self.waypoint_pub.publish(ps)
		if self.state==0:
			waypointPose = waypointPose.pose
			self.points = [waypointPose.pose.position.x, waypointPose.pose.position.y, 
						 tan(euler_from_quaternion([ waypointPose.pose.orientation.x,
										waypointPose.pose.orientation.y,
										waypointPose.pose.orientation.z,
										waypointPose.pose.orientation.w])[2]), self.tf/2]
			print 'received waypoint', self.points

	def receiveGazeboModelStates(self,modelStates):
		self.robotState = modelStates.pose[-1]

	def receiveInput(self, inputArray):
		if self.state==1:
			return 0
		inp = inputArray.data
		
		status = String()
		status.data = "BUSY"
		self.status_pub.publish(status)

		self.state = 1
		to = inp[-2]
		tf = inp[-1]
		coeffs = get_coeffs(inp[:-3],to,tf)
		
		self.status_pub.publish(status)
		start_time = datetime.datetime.now()
		t=to
		r = rospy.Rate(self.delay)
		cmd = Twist()
		while t<tf and self.state==1:
		
			dx,dy,dk = getVel(coeffs,to,tf,t)
			cmd.linear.x = self.VEL_SCALE*inp[-3]*sqrt(dx**2 + dy**2)
			cmd.angular.z = self.VEL_SCALE*inp[-3]*dk
			self.vel_pub.publish(cmd)
			r.sleep()
			t = t + 1.0/self.delay
			self.status_pub.publish(status)
		time.sleep(0.3)
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		status.data = "DONE"
		#rospy.Rate(1).sleep()
		self.state = 0
		self.status_pub.publish(status)

	def getQuadrant(self,angle):
		if 0<=angle<pi/2.0:
			return 1
		if pi/2.0<=angle<pi:
			return 1

	def receiveInputGlobal(self, empty):
		# pose = [self.robotState.position.x, self.robotState.position.y, 
		# 		tan(euler_from_quaternion([ self.robotState.orientation.x,
		# 								self.robotState.orientation.y,
		# 								self.robotState.orientation.z,
		# 								self.robotState.orientation.w])[2]),0]
		pose = [self.robotPTAMPose.position.x, self.robotPTAMPose.position.y, 
						tan(euler_from_quaternion([ self.robotPTAMPose.orientation.x,
												self.robotPTAMPose.orientation.y,
												self.robotPTAMPose.orientation.z,
												self.robotPTAMPose.orientation.w])[2]),0]

		distance = linalg.norm(array([self.robotPTAMPose.position.x, self.robotPTAMPose.position.y]) 
							  - array([self.points[0], self.points[1]]))
		# distance2 = linalg.norm(array([self.goal[0], self.goal[1]]) 
		# 					  - array([self.points[0], self.points[1]]))
		# self.points[-1] = 1.2*self.tf*distance1/(distance1+distance2)

		goal = self.goal[:]
		my_points = self.points[:]
		# if(self.map==1 or self.map==2 or self.map==3):
		# 	if pose[0]>=4:
		# 		points_done = 1
		# 		goal[-1]=14.0
		# 		my_points = []
		# 	else:
		# 		if pose[1]>=4:
		# 			my_points[-1]=7.0
		# 			goal[-1]=21.0
		# ps = PoseStamped()
		# ps.header.stamp = rospy.Time.now()
		# ps.header.frame_id="world"
		# if my_points==[]:
		# 	ps.pose.position.x = goal[0]
		# 	ps.pose.position.y = goal[1]
		# 	q = quaternion_from_euler(0,0,arctan(goal[2]))
		# else:
		# 	ps.pose.position.x = my_points[0]
		# 	ps.pose.position.y = my_points[1]
		# 	q = quaternion_from_euler(0,0,arctan(my_points[2]))
		# ps.pose.orientation.w = q[3]
		# ps.pose.orientation.x = q[0]
		# ps.pose.orientation.y = q[1]
		# ps.pose.orientation.z = q[2]
		# self.waypoint_pub.publish(ps)
		# elif self.map==4:
		# 	if pose[0] < -1.5:
		# 		goal = goal[0:1]
		# 		my_points = my_points[0:1]
		# 		points_done = 0
		# 		print 'if'
		# 	elif pose[0] >=-1.5 and pose[0] < 1.5:
		# 		goal = goal[-1:]
		# 		my_points = my_points[-1:]
		# 		points_done = 0
		# 		print 'elif1'
		# 	elif pose[0] >= 1.5:
		# 		goal = goal[-1:]
		# 		goal[0][-1]=14.0
		# 		my_points = my_points[-1:]
		# 		points_done = 1
		# 		print 'elif2'
		# print goal 
		# print my_points
		points = [pose] + my_points+ goal
		# 	if pose[1]<=4:
		# 		points_done = 3
		# 	else:
		# 		points_done = 2
		# else:
		# 	if pose[1]>=4:
		# 		points_done = 1
		# 	else:
		# 		points_done = 0
		
		
		status = String()
		inp = [pose[0],pose[1],pose[2],
				goal[0],goal[1],goal[2],
			   0.0,0.0,0.0,0.0,0.0,0.0]
		# print inp
		# print points[1:-1]
		status.data = "BUSY"
		self.status_pub.publish(status)	
		
		to = self.to
		tf = goal[-1]
		# if points[1:-1]==[]:
		# 	coeffs = get_coeffs(inp,to,tf)
		# else:
		# coeffs = get_coeffs_points(inp,to,tf,self.points)
		coeffs = get_coeffs(inp,to,tf,my_points)
		self.status_pub.publish(status)
		t=to

		self.status_pub.publish(status)
		self.state = 2
		r = rospy.Rate(self.delay)
		cmd = Twist()
		self.GlobalPath.points = []
		while t<tf:
			point1,kt1 = getPos(coeffs,to,tf,t,pose[1])
			point1.x, point1.y, point1.z = point1.z, point1.x, 0.0
			self.GlobalPath.points.append(point1)
			t = t + 0.1
		t=to
		self.global_path_pub.publish(self.GlobalPath)
		while t<tf and self.state==2:
			self.global_path_pub.publish(self.GlobalPath)
			dx,dy,dk = getVel(coeffs,to,tf,t)
			point1,kt1 = getPos(coeffs,to,tf,t,pose[1])
			point2,kt2 = getPos(coeffs,to,tf,min(t+1,tf),pose[1])
			message = Float32MultiArray()
			message.data= [ 0.0,0.0,0.0,
							point2.z - point1.z, point2.x - point1.x, tan(arctan(kt2) - arctan(kt1)),
				   			0.0,0.0,0.0,0.0,0.0,0.0,0.0]
			
			#print self.robotPTAMPose.position.x, self.robotPTAMPose.position.y
			self.status_pub.publish(status)
			yaw = euler_from_quaternion([ self.robotState.orientation.x,
										self.robotState.orientation.y,
										self.robotState.orientation.z,
										self.robotState.orientation.w])[2]
			if fabs(yaw) < pi/2.0:
				cmd.linear.x = sign(dx) * sqrt(dx**2 + dy**2)
			elif fabs(yaw) > pi/2.0:
				cmd.linear.x = -sign(dx) * sqrt(dx**2 + dy**2)
			else:
				cmd.linear.x = sign(yaw)*sign(dy) * sqrt(dx**2 + dy**2)
			cmd.angular.z = self.VEL_SCALE*dk
			cmd.linear.x = self.VEL_SCALE*cmd.linear.x
			message.data[-1] = sign(cmd.linear.x)
			# T = 10*t
			# if(int(T)==T):
			self.inp_pub.publish(message)
			lookahead = [ point1.z, point1.x, kt1,
						  point2.z, point2.x, kt2,
				   			0.0,0.0,0.0,0.0,0.0,0.0,0.0]
			#traj = array(self.trajjer(lookahead+[0,1,1]))
			# traj = traj.T
			# traj[0], traj[1] = traj[1], traj[0]
			# traj = traj.T
			#self.lookaheadPath.points = traj.tolist()
			#self.lookaheadPath.pose = self.robotPTAMPose
			self.vel_pub.publish(cmd)
			r.sleep()
			t = t + 1.0/self.delay
			self.status_pub.publish(status)
				
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		status.data = "DONE1"
		rospy.Rate(2).sleep()
		self.status_pub.publish(status)
		self.state = 0

	def sendLookahead(self, req):
		pose = [self.robotPTAMPose.position.x, self.robotPTAMPose.position.y, 
				tan(euler_from_quaternion([ self.robotPTAMPose.orientation.x,
										self.robotPTAMPose.orientation.y,
										self.robotPTAMPose.orientation.z,
										self.robotPTAMPose.orientation.w])[2]),0]

		# if pose[0]>4:
		# 	points_done = 1
		# else:
		# 	points_done = 0
		goal = self.goal[:]
		my_points = self.points[:]
		# points = [[self.robotPTAMPose.position.x, self.robotPTAMPose.position.y, 
		# 		tan(euler_from_quaternion([ self.robotPTAMPose.orientation.x,
		# 								self.robotPTAMPose.orientation.y,
		# 								self.robotPTAMPose.orientation.z,
		# 								self.robotPTAMPose.orientation.w])[2]),0]] + self.points[points_done:]
		# inp = [points[0][0],points[0][1],points[0][2],
		# 		points[-1][0],points[-1][1],points[-1][2],
		# 	   0.0,0.0,0.0,0.0,0.0,0.0]

		inp = [pose[0],pose[1],pose[2],
				goal[0],goal[1],goal[2],
			   0.0,0.0,0.0,0.0,0.0,0.0]
		
		to = self.to
		tf = goal[-1]
		#coeffs = get_coeffs_points(inp,to,tf,self.points)
		coeffs = get_coeffs(inp,to,tf,self.points)

		t=to

		
		point0,kt0 = getPos(coeffs,to,tf,t,pose[1])
		point1,kt1 = getPos(coeffs,to,tf,t+1,pose[1])
		point2,kt2 = getPos(coeffs,to,tf,t+2,pose[1])
		message = Float32MultiArray()
		message.data = [ 0.0,0.0,0.0,
						point1.z - point0.z, point1.x - point0.x, tan(arctan(kt1) - arctan(kt0)),
						0.0,0.0,0.0,0.0,0.0,0.0] + \
						[ 0.0,0.0,0.0,
						point2.z - point0.z, point2.x - point0.x, tan(arctan(kt2) - arctan(kt0)),
						0.0,0.0,0.0,0.0,0.0,0.0] + \
						[ 0.0,0.0,0.0,
						point2.z - point1.z, point2.x - point1.x, tan(arctan(kt2) - arctan(kt1)),
						0.0,0.0,0.0,0.0,0.0,0.0]
		return ExpectedPathResponse(message)
				

	def receiveBreak(self, empty):
		self.state = 0
		# self.goalPose = None
		# self.waypointPose = None
		# self.goal = []
		# self.points = []
		# self.rlUnsafePath.points = []
		# self.rlSafePath.points = []
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())

	def sendQValue(self, qPredictor):
		return QPredictionResponse(Float32(self.QRegressor.predict([qPredictor.qInput.data])))


rospy.init_node('PlannerNode')
PLanner2D()
rospy.spin()
