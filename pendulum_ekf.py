
import numpy as np
from periodic import PeriodicProcess
import uorb
import pendulum

class Estimator(PeriodicProcess):

    def __init__(self, period, uorb_manager):
        super(Estimator, self).__init__(period)
        self.uorb_manager = uorb_manager
        self.P0 = np.eye(2)
        self.curr_time = 0
	self.c = np.array([[0],[0],[0]])

    def initialize(self, t, x0=np.array([[0],[0]]), P0=np.eye(2)):
        super(Estimator, self).initialize(t)
        # initialize position publication
        self.pos = uorb.Publication(
            self.uorb_manager,
            'vehicle_global_position',
            uorb.Topic_vehicle_global_position(*((0,)*11)))
        
        # initialize attitude publication
        self.att = uorb.Publication(
            self.uorb_manager,
            'vehicle_attitude',
            uorb.Topic_vehicle_attitude(*((0,)*16)))
        
        # intialize sensor subscription
        self.sensor = uorb.Subscription(
            self.uorb_manager,
            'sensor_combined')

        # intialize actuator output subscription
        self.actuators = uorb.Subscription(
            self.uorb_manager,
            'actuator_outputs')
        
        # initialize gps subscription
        #TODO once gps is set up
        #self.gps = uorb.Subscription(
        #    self.uorb_manager,
        #    'gps')
        
        # TODO, this is a hack, we make the estimator
        # just give the sim state for now
        self.sim = uorb.Subscription(
            self.uorb_manager,
            'sim_state')
        
	self.m = 1
	self.l = 1
	self.g = 9.8

        self.x0 = x0
        self.x = x0
        
        self.P0 = P0
        self.P = P0
        
        self.u = 0
        self.y = np.array([[0],[0],[0]])
        
        self.Q = 0.01 * np.eye(2) # TODO fix this
        self.R = 0.01 * np.eye(3) # TODO fix this
        
        
    def predict(self, newTime):
        "integrate a time step into the future"
	dt = newTime - self.curr_time
        # TODO probably should turn f, a, c into class methods
        xdot = pendulum.compute_f(self.curr_time, self.x, self.u, self.m, self.g, self.l) #no time dependence
        self.a = pendulum.compute_a(
                            self.curr_time, self.x, self.u, self.m, self.g, self.l)
        self.b = pendulum.compute_b(
                            self.curr_time, self.x, self.u, self.m, self.g, self.l)
    
        Pdot = self.a.dot(self.P) + self.P.dot(self.a.transpose()) \
                    + self.Q
        
        self.x += dt * xdot # TODO right now just rectangular integration:
        self.P += dt * Pdot # should make it better
	
	self.curr_time = newTime
    

    def update_state(self): #TODO figure out how this uses t
        "update step for EKF"
        self.c = pendulum.compute_g_accel_h(self.curr_time, self.x, self.u, self.m, self.g, self.l)
        self.ct = self.c.transpose()
        
        self.K = self.P.dot(self.ct.dot(
                    np.linalg.inv(self.c.dot(self.P.dot(self.ct))+self.R)))
        # TODO break the following down so that an individual sensor
        # can update the estimate rather than doing all or nothing

        self.x += self.K.dot(self.y - 
		pendulum.compute_g_accel(self.curr_time, self.x, self.u, 
                self.m, self.g, self.l))
        
        self.P -= self.K.dot(self.c.dot(self.P))
        
    def run(self, t):
        # update gps
        #if self.gps.updated():
	#    self.gps.update()
	#    self.curr_time = self.gps.timestamp
        #   TODO add gps info to measurement vector
        
        # update sensors
        #if self.sensor.updated():
        self.sensor.update()
        #self.curr_time = self.sensor.data.accelerometer_timestamp #probably not the way to do this
        self.y = np.array([[self.sensor.data.accelerometer_m_s2[0]],
				[self.sensor.data.accelerometer_m_s2[1]],
				[self.sensor.data.accelerometer_m_s2[2]]])
            
        # update actuators
        #if self.actuators.updated():
	self.actuators.update()
	#should also check for a reasonable time gap between actuator signals
        self.u = self.actuators.data.output[0] # we only use the first actuator for pendulum

	#update and propagate state estimate
        self.update_state()
	self.predict(self.sensor.data.gyro1_rad_s)

	# publish new estimated position
        self.pos.data.timestamp = t
        self.pos.data.lat = 0 
        self.pos.data.lon = 0
        self.pos.data.alt = 0
        self.pos.data.vel_n = 0
        self.pos.data.vel_e = 0
        self.pos.data.vel_d = 0
        self.pos.data.yaw = 0
        self.pos.publish()
        
        # publish new estimated attitude
        self.att.data.timestamp = t
        self.att.data.roll = 0
        self.att.data.pitch = self.x[0]
        self.att.data.yaw = 0
        self.att.data.rollspeed = 0
        self.att.data.pitchspeed = self.x[1]
        self.att.data.yawspeed = 0
        self.att.publish()
