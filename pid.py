
import numpy as np

TS = 0.1 # Sampling time

class Derivator:
    def __init__(self):
        self.x = [0]
        self.dx = [0]
        self.T = TS 
        self.t = [0]
    
    def diff(self,val):
        """
            Compute (x_k - x_{k-1})/T and updates the memory 
        """
        dif = (val - self.x[-1]) / self.T
        self.x.append(val) # memory
        self.dx.append(dif) # memory
        self.time_update()
        return self.dx[-1]
    
    def time_update(self):
        """ time vector"""
        self.t.append(self.t[-1]+self.T)
    
    
    def __call__(self,val):
        """ Call like diff(error) """
        return self.diff(val)

class Integrator:
    def __init__(self):
        self.x = [0]
        self.ix = [0]
        self.T = TS 
        self.t = [0]       
    
    def integ(self,val):
        """
            Compute (x_k - x_{k-1})/T and updates the memory 
        """  
        integral = np.sum(self.T * np.array(self.x)) 
        self.x.append(val) # memory              
        self.ix.append(integral) # memory
        self.time_update()
        return self.ix[-1] 
    
    def time_update(self):
        """ time vector"""
        self.t.append(self.t[-1]+self.T)
    
    def __call__(self,val):
        """ Call like integ(error) """
        return self.integ(val)    
    

class PID:

    def __init__(self, k_p=k_p,k_i=k_i,k_d=k_d):
        
        # Ziegler Nichols method 
        # Check here https://en.wikipedia.org/wiki/Ziegler–Nichols_method
        if k_d == 0:
            self.k_p = 0.45*K_u
            self.k_i = 0.54*K_u/T_u
            self.k_d = k_d
        else: 
            self.k_p = k_p
            self.k_i = k_i
            self.k_d = k_d
        
#         self.k_p = 0.3*K_u
#         self.k_i = 1.2*K_u/T_u
#         self.k_d = 3*K_u*T_u/40
        
        self.T = TS # Sampling time
        self.t = [0]
        
        self.u_p = [0]# Proportional term
        self.u_i = [0]# Integral term 
        self.u_d = [0] # Derivative term
        
        self.control = [0]# Control memory
        
        self.integ = Integrator()
        self.diff = Derivator()
        
        
    def apply_control(self,error):
        
        P = self.k_p * error
        self.u_p.append(P)
        I = self.k_i * self.integ(error)
        self.u_i.append(I)
        D = self.k_d * self.diff(error)
        self.u_d.append(D)
                             
        u_f = self.u_p[-1]+self.u_i[-1]+self.u_d[-1]
        self.time_update()
        self.control.append(u_f)
        return u_f
                        
    def time_update(self):
        """ time vector"""
        self.t.append(self.t[-1]+self.T)                        
                             
    def __call__(self,error):
        """ Callable """        
        return self.apply_control(error)
                             

U_MAX = 10

class PIDlim:

    def __init__(self, k_p=k_p,k_i=k_i,k_d=k_d,u_max=U_MAX):
        
        # Ziegler Nichols method 
        # Check here https://en.wikipedia.org/wiki/Ziegler–Nichols_method
        if k_d == 0:
            self.k_p = 0.45*K_u
            self.k_i = 0.54*K_u/T_u
            self.k_d = k_d
        else: 
            self.k_p = k_p
            self.k_i = k_i
            self.k_d = k_d
        
#         self.k_p = 0.3*K_u
#         self.k_i = 1.2*K_u/T_u
#         self.k_d = 3*K_u*T_u/40
        
        self.T = TS # Sampling time
        self.t = [0]
        
        self.u_p = [0]# Proportional term
        self.u_i = [0]# Integral term 
        self.u_d = [0] # Derivative term
        
        self.u_max = u_max
        self.u_min = -u_max
        
        self.control = [0]# Control memory
        self.control_bnd = [0]
        
        self.integ = Integrator()
        self.diff = Derivator()
        
        
    def apply_control(self,error):
        
        P = self.k_p * error
        self.u_p.append(P)
        I = self.k_i * self.integ(error)
        self.u_i.append(I)
        D = self.k_d * self.diff(error)
        self.u_d.append(D)
                             
        u_f = self.u_p[-1]+self.u_i[-1]+self.u_d[-1]
        self.control.append(u_f)
        
        # Bound control 
        u_f = max(self.u_min,min(u_f,self.u_max))
        self.control_bnd.append(u_f)
        
        self.time_update()

        return u_f
                        
    def time_update(self):
        """ time vector"""
        self.t.append(self.t[-1]+self.T)                        
                             
    def __call__(self,error):
        """ Callable """        
        return self.apply_control(error)
                             

class PIDantiwindup:

    def __init__(self, k_p=k_p,k_i=k_i,k_d=k_d,u_max=U_MAX):
        
        # Ziegler Nichols method 
        # Check here https://en.wikipedia.org/wiki/Ziegler–Nichols_method
        if k_d == 0:
            self.k_p = 0.45*K_u
            self.k_i = 0.54*K_u/T_u
            self.k_d = k_d
        else: 
            self.k_p = k_p
            self.k_i = k_i
            self.k_d = k_d
        
#         self.k_p = 0.3*K_u
#         self.k_i = 1.2*K_u/T_u
#         self.k_d = 3*K_u*T_u/40
        
        self.T = TS # Sampling time
        self.t = [0]
        
        self.u_p = [0]# Proportional term
        self.u_i = [0]# Integral term 
        self.u_d = [0] # Derivative term
        
        self.u_max = u_max
        self.u_min = -u_max
        
        self.control = [0]# Control memory
        self.control_bnd = [0]
        
        self.T_t = 1 # Time constant for integration reset 
        
        self.integ = Integrator()
        self.diff = Derivator()
        
        
    def apply_control(self,error):
        
        P = self.k_p * error
        self.u_p.append(P)
        
        wind_reset = (self.control_bnd[-1] - self.control[-1])/self.T_t
        
        I = self.integ(self.k_i * error + wind_reset) # Anti windup mechanism
        
        self.u_i.append(I)
        D = self.k_d * self.diff(error)
        self.u_d.append(D)
                             
        u_f = self.u_p[-1]+self.u_i[-1]+self.u_d[-1]
        self.control.append(u_f)
        
        # Bound control 
        u_f = max(self.u_min,min(u_f,self.u_max))
        self.control_bnd.append(u_f)
        
        self.time_update()

        return u_f
                        
    def time_update(self):
        """ time vector"""
        self.t.append(self.t[-1]+self.T)                        
                             
    def __call__(self,error):
        """ Callable """        
        return self.apply_control(error)
