import numpy as np
import matplotlib.pylab as plt


class OriginalFormulation(object):

    def __init__(self, K=50., D=None):
        self.K = K
        if D is None: 
            D = 2.0 * np.sqrt(self.K)    
        self.D = D
            
    def acceleration(self, x, dx, start, goal, tau, f, s):
        """
        Compute the acceleration value of the transformation system
        at the phase s.

        Return: 
            ddx float: the value of the acceleration variable at the 
                       phase s
        """        
        #------------------------------------------------------------
        # Place your code here
        return 
        #------------------------------------------------------------
  
    def fs(self, x, dx, ddx, start, goal, tau, s):
        """
        Compute the nonlinear term f of the transformation system.

        Return: 
            f float: the value of the nonlinear term at the phase s
        """
        #------------------------------------------------------------
        # Place your code here
        return 
        #------------------------------------------------------------
        
class ImprovedFormulation(object):
    
    def __init__(self, K=50., D=None):
        self.K = K
        if D is None: 
            D = self.K/4.
        self.D = D
    
    def acceleration(self, x, dx, start, goal, tau, f, s):
        """
        Compute the acceleration value of the transformation system
        at the phase s.

        Return: 
            ddx float: the value of the acceleration variable at the 
                       phase s
        """                
        #------------------------------------------------------------
        # Place your code here
        return 
        #------------------------------------------------------------
    
    def fs(self, x, dx, ddx, start, goal, tau, s):        
        """
        Compute the nonlinear term f of the transformation system.

        Return: 
            f float: the value of the nonlinear term at the phase s
        """
        #------------------------------------------------------------
        # Place your code here
        return 
        #------------------------------------------------------------




class DMPs_discrete(object):

    def __init__(self, dims, bfs, dt=.01, tau=1., alpha=14, enable_improved=False, **kwargs):
        '''
        dims int: number of dynamic motor primitives
        bfs int: number of basis functions per DMP
        dt float: timestep for simulation
        tau float: scales the timestep
                   increase tau to make the system execute faster
        alpha float: canonical system parameter
        '''
        
        self.dmps = dims 
        self.bfs  = bfs 
        self.dt   = dt
        self.tau  = tau
        self.alpha = alpha/2.0
        self.alpha_x = alpha

        self.prep_centers_and_variances()
        
        if enable_improved is False:
            self.formulation = OriginalFormulation()
        else:
            self.formulation = ImprovedFormulation()
        
        return


    def prep_centers_and_variances(self):
        '''
        Set the centre of the Gaussian basis functions be spaced evenly 
        throughout run time.
        '''
        self.c = np.zeros(self.bfs)
        self.h = np.zeros(self.bfs)

        t = np.linspace(0,1,self.bfs) *0.5

        # From DMP matlab code
        self.c = np.exp(-self.alpha_x*t)
        self.D = (np.diff(self.c)*0.55)**2
        self.D = np.append(self.D, self.D[-1])
        self.D = 1/self.D
        self.h = np.ones(self.bfs)*0.5

            
    def gen_psi(self, x):
        '''
        Generates the activity of the basis functions for a given state of the 
        canonical system.
        
        x float: the current state of the canonical system
        '''
        if isinstance(x, np.ndarray):
            x = x[:,None]
        return np.exp(-self.h * (x - self.c)**2 * self.D)        

    
    def gen_phase(self, n_steps, tau=None):
        """
        Generate phase for open loop movements.

        n_steps int: number of steps
        """
        if tau is None: tau = self.tau
        return np.exp(-self.alpha/tau * np.linspace(0, 1, n_steps))
        
        
    def learn(self, y_des):
        """
        Encode a set of weights from the input trajectories.

        y_des list/array: the desired trajectories of each DMP
                          should be shaped [dmps, run_time]
        """
        # Set variables
        n_samples, dims, n_steps = np.shape(y_des)
        self.n_steps = n_steps
        assert dims==self.dmps, "wrong dimensions"
        
        # Get start and goal
        self.y0   = np.mean(y_des[:,:,0], axis=0)
        self.goal = np.mean(y_des[:,:,-1], axis=0)

        # Calculate yd_des, ydd_des
        yd_des = np.diff(y_des) / self.dt
        yd_des = np.concatenate((np.zeros((n_samples, self.dmps, 1)), yd_des), axis=2)

        ydd_des = np.diff(yd_des) / self.dt
        ydd_des = np.concatenate((np.zeros((n_samples, self.dmps, 1)), ydd_des), axis=2)
        
        # Get a canonical system
        x_track = self.gen_phase(n_steps)
        
        #------------------------------------------------------------
        # Place your code here
        
        # Calculate f
        # f_des =


        
        
        # Calculate weights
        # psi_track = 

        # x_track   = 
        # psi_track = 
        # f_des     = 
        # f_des     = 

        #self.w = 

                
        #------------------------------------------------------------

        # set up tracking vectors
        y_track   = np.zeros((self.dmps, n_steps)) 
        yd_track  = np.zeros((self.dmps, n_steps)) 
        ydd_track = np.zeros((self.dmps, n_steps)) 
        
        y   = self.y0.copy()
        yd  = np.zeros(self.dmps)   
        ydd = np.zeros(self.dmps)  
        
        #------------------------------------------------------------
        # Place your code here
        #x_track   = 
        #psi_track = 
        
        #f =
        

        
        # Recover the demonstration using the learned weights (for confirmation)



            # record timestep
            #y_track[:,t] = 
            #yd_track[:,t] = 
            #ydd_track[:,t] = 
        #------------------------------------------------------------
                
        return y_track, yd_track, ydd_track

    
    def plan(self, y0=None, goal=None, **kwargs):
        '''
        Run the DMP system within a specific period.

        y0   list/array: start position
        goal list/array: goal position
        tau  float:      scales the timestep
                         increase tau to make the system execute faster
        '''

        if y0 is None: y0 = self.y0
        if goal is None: goal = self.goal
        n_steps = int(self.n_steps/self.tau)
        
        # set up tracking vectors
        y_track   = np.zeros((self.dmps, n_steps)) 
        yd_track  = np.zeros((self.dmps, n_steps)) 
        ydd_track = np.zeros((self.dmps, n_steps)) 
        x_track   = self.gen_phase(n_steps, self.tau)

        #------------------------------------------------------------
        # Place your code here
        y   = 
        yd  = 
        ydd = 
        





        
            # record timestep
            #y_track[:,t] = 
            #yd_track[:,t] = 
            #ydd_track[:,t] = 
        #------------------------------------------------------------
            
        return y_track, yd_track, ydd_track

    
    def plot_traj(self, trajs_demo, trajs_gen, axis_num=0):
        """Plot trajectories over an axis """
        
        fig = plt.figure()
        plt.title('Trajectory (X) - Demo (Td) and generated (Tg)')
        for i in range(len(trajs_demo)):
            plt.plot(trajs_demo[i,axis_num,:], 'r--', label='Td')
        for i in range(len(trajs_gen)):
            plt.plot(trajs_gen[i,axis_num,:],'g-', label='Tg')

        plt.legend()
        plt.show()
        
    def plot_basis(self):
        """Plot basis functions """
        fig = plt.figure()

        x = self.gen_phase(200)

        for idx in range(len(self.c)):
            psi = self.gen_psi(x)
            plt.plot(x, psi)

        plt.show()

    def plot_f(self, f, f_des=None):
        """Plot nonlinear functions """
        
        fig = plt.figure()
        plt.plot(f)
        if f_des is not None:
            plt.plot(f_des, '--')
        plt.show()

    def plot_canonical_sys(self):
        """Plot the phase change in the canonical system """
        x = self.gen_phase(200)
        
        fig = plt.figure()
        plt.plot(x)
        plt.show()



