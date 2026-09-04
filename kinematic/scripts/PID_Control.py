# !/usr/bin/env python3 

class PIDController:
    def __init__(self, Kp, Ki, Kd, MaxValue, MinValue, T = 0.02,  MaxValue_Integer = 5, MinValue_integer = -5, SamplingTime = 0.01, limit_EN = True):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = T
        self.limMin = MinValue
        self.limMax = MaxValue
        self.limMin_Int = MinValue_integer
        self.limMax_int = MaxValue_Integer
        self.TimeSam = SamplingTime
        self.integrator = 0
        self.prevError = 0
        self.differentiator = 0
        self.prevMeasurement = 0
        self.outVal = 0
        self.limit_EN = True

    def PID_Calc(self, setPoint, measurement) :
        error = setPoint - measurement
        propotional = self.Kp * error
        self.integrator = self.integrator + 0.5 * self.Ki * self.TimeSam * (error + self.prevError)
        
        if self.integrator > self.limMax_int:
            self.integrator = self.limMax_int
        elif self.integrator < self.limMin_Int:
            self.integrator = self.limMin_Int
        
        self.differentiator = -(2.0 * self.Kd * (measurement - self.prevMeasurement) 
                              +(2.0 * self.tau - self.TimeSam) * self.differentiator) \
                              /(2.0 * self.tau + self.TimeSam)
        
        self.outVal = propotional + self.integrator + self.differentiator

        if self.limit_EN == 1:
            if self.outVal > self.limMax :
                self.outVal = self.limMax
            elif self.outVal < self.limMin :
                self.outVal = self.limMin
        else :
            pass

        self.prevError = error
        self.prevMeasurement = measurement

        return self.outVal
