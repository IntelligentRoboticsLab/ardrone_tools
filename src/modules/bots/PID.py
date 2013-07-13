class PID_controller():

    def __init__( self, start_time, proportional = 0.1, integral = 0.1, derivative = 0.1 ):
    
        self.gain_prop  = proportional
    	self.gain_inter = integral
	self.gain_deriv = derivative 
    	self.times  = [start_time]
    	self.errors = [0]

    def add_error( self, time, process_value, setpoint ):
	self.times.append(  time )
	self.errors.append( setpoint - process_value )

    def controller( self, time, process_value, setpoint ):
	self.add_error( time, process_value, setpoint )
	return self.manipulate_value()

    def manipulate_value(self):
	prop  = self.errors[-1] * self.gain_prop
	inter = self.intergral()
	deriv = self.derivative()
	return prop+inter+deriv

    def intergral(self):
	#for loops zijn sloom
	length = len(self.errors)
	area   = 0
	i      = 1
	while i < length:
	    delta  =  self.times[i]    - self.times[i-1]
	    value  = (self.errors[i-1] + (self.errors[i] - self.errors[i-1])/2)
	    area  += value*delta
	    i     += 1
	return self.gain_inter * area
    
    def derivative(self):
	return self.gain_deriv*(self.errors[-1] - self.errors[-2])*(self.times[-1]-self.times[-2])
