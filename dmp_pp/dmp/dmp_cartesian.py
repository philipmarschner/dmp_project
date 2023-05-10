'''
Copyright (C) 2020 Michele Ginesi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
import scipy.integrate
import scipy.interpolate
import scipy.linalg
import copy
import pdb

from dmp_pp.dmp.cs import CanonicalSystem
from dmp_pp.dmp.exponential_integration import exp_eul_step
from dmp_pp.dmp.exponential_integration import phi1
from dmp_pp.dmp.rotation_matrix import roto_dilatation
from dmp_pp.dmp.derivative_matrices import compute_D1, compute_D2

class DMPs_cartesian(object):
    '''
    Implementation of discrete Dynamic Movement Primitives in cartesian space,
    as described in
    [1] Park, D. H., Hoffmann, H., Pastor, P., & Schaal, S. (2008, December).
        Movement reproduction and obstacle avoidance with Dynamic movement
        primitives and potential fields.
        In Humanoid Robots, 2008. Humanoids 2008. 8th IEEE-RAS International
        Conference on (pp. 91-98). IEEE.
    [2] Hoffmann, H., Pastor, P., Park, D. H., & Schaal, S. (2009, May).
        Biologically-inspired dynamical systems for movement generation:
        automatic real-time goal adaptation and obstacle avoidance.
        In Robotics and Automation, 2009. ICRA'09. IEEE International
        Conference on (pp. 2587-2592). IEEE.
    '''

    def __init__(self,
        n_dmps = 3, n_bfs = 50, dt = 0.01, x_0 = None, x_goal = None, T = 1.0,
        K = 1050, D = None, w = None, tol = 0.01, alpha_s = 4.0,
        rescale = None, basis = 'gaussian', **kwargs):
        '''
        n_dmps int   : number of dynamic movement primitives (i.e. dimensions)
        n_bfs int    : number of basis functions per DMP (actually, they will
                       be one more)
        dt float     : timestep for simulation
        x_0 array     : initial state of DMPs
        x_goal array   : x_goal state of DMPs
        T float      : final time
        K float      : elastic parameter in the dynamical system
        D float      : damping parameter in the dynamical system
        w array      : associated weights
        tol float    : tolerance
        alpha_s float: constant of the Canonical System
        rescale      : tell which affine transformation use in the Cartesian
                       component to be make affine invariant, possible values
                       are:
                         None: no scalability
                         'rotodilatation': use rotodilatation
                         'diagonal': use a diagonal matrix ("old" DMP 
                            formulation)
        basis string : type of basis functions
        '''

        # Tolerance for the accuracy of the movement: the trajectory will stop
        # when || x - g || <= tol
        self.tol = copy.deepcopy(tol)
        self.n_dmps = copy.deepcopy(n_dmps)
        self.n_bfs = copy.deepcopy(n_bfs)

        # Default values give as in [2]
        self.K = copy.deepcopy(K)
        if D is None:
            D = 2 * np.sqrt(self.K)
        self.D = copy.deepcopy(D)

        # Set up the CS
        self.cs = CanonicalSystem(dt = dt, run_time = T, alpha_s = alpha_s)

        # Create the matrix of the linear component of the problem
        self.compute_linear_part()

        # Set up the DMP system
        if x_0 is None:
            x_0 = np.zeros(self.n_dmps)
        if x_goal is None:
            x_goal = np.ones(self.n_dmps)
        self.x_0 = copy.deepcopy(x_0)
        self.x_goal = copy.deepcopy(x_goal)
        self.rescale = copy.deepcopy(rescale)
        self.basis = copy.deepcopy(basis)
        self.reset_state()
        self.gen_centers()
        self.gen_width()

        # If no weights are give, set them to zero
        if w is None:
            w = np.zeros([self.n_dmps, self.n_bfs + 1])
        self.w = copy.deepcopy(w)

    def compute_linear_part(self):
        '''
        Compute the linear component of the problem.
        '''
        self.linear_part = np.zeros([2 * self.n_dmps, 2 * self.n_dmps])
        self.linear_part\
            [range(0, 2 * self.n_dmps, 2), range(0, 2 * self.n_dmps, 2)] = \
                - self.D
        self.linear_part\
            [range(0, 2 * self.n_dmps, 2), range(1, 2 * self.n_dmps, 2)] = \
                - self.K
        self.linear_part\
            [range(1, 2 * self.n_dmps, 2), range(0, 2 * self.n_dmps, 2)] = 1.

    def gen_centers(self):
        '''
        Set the centres of the basis functions to be spaced evenly throughout
        run time
        '''
        # Desired activations throughout time
        self.c = np.exp(- self.cs.alpha_s * self.cs.run_time *
            ((np.cumsum(np.ones([1, self.n_bfs + 1])) - 1) / self.n_bfs))

    def gen_psi_retrain(self,s,index):
    
        c = np.reshape(self.c[index[0]:index[-1]+1], [len(index), 1])
        w = np.reshape(self.width[index[0]:index[-1]+1], [len(index),1 ])
        if (self.basis == 'gaussian'):
            xi = w * (s - c) * (s - c)
            psi_set = np.exp(- xi)
        else:
            xi = np.abs(w * (s - c))
            if (self.basis == 'mollifier'):
                psi_set = (np.exp(- 1.0 / (1.0 - xi * xi))) * (xi < 1.0)
            elif (self.basis == 'wendland2'):
                psi_set = ((1.0 - xi) ** 2.0) * (xi < 1.0)
            elif (self.basis == 'wendland3'):
                psi_set = ((1.0 - xi) ** 3.0) * (xi < 1.0)
            elif (self.basis == 'wendland4'):
                psi_set = ((1.0 - xi) ** 4.0 * (4.0 * xi + 1.0)) * (xi < 1.0)
            elif (self.basis == 'wendland5'):
                psi_set = ((1.0 - xi) ** 5.0 * (5.0 * xi + 1)) * (xi < 1.0)
            elif (self.basis == 'wendland6'):
                psi_set = ((1.0 - xi) ** 6.0 * 
                    (35.0 * xi ** 2.0 + 18.0 * xi + 3.0)) * (xi < 1.0)
            elif (self.basis == 'wendland7'):
                psi_set = ((1.0 - xi) ** 7.0 *
                    (16.0 * xi ** 2.0 + 7.0 * xi + 1.0)) * (xi < 1.0)
            elif (self.basis == 'wendland8'):
                psi_set = (((1.0 - xi) ** 8.0 *
                    (32.0 * xi ** 3.0 + 25.0 * xi ** 2.0 + 8.0 * xi + 1.0)) *
                    (xi < 1.0))
        psi_set = np.nan_to_num(psi_set)
        return psi_set

    def gen_psi(self, s):
        '''
        Generates the activity of the basis functions for a given canonical
        system rollout.
         s : array containing the rollout of the canonical system
        '''
        c = np.reshape(self.c, [self.n_bfs + 1, 1])
        w = np.reshape(self.width, [self.n_bfs + 1,1 ])
        if (self.basis == 'gaussian'):
            xi = w * (s - c) * (s - c)
            psi_set = np.exp(- xi)
        else:
            xi = np.abs(w * (s - c))
            if (self.basis == 'mollifier'):
                psi_set = (np.exp(- 1.0 / (1.0 - xi * xi))) * (xi < 1.0)
            elif (self.basis == 'wendland2'):
                psi_set = ((1.0 - xi) ** 2.0) * (xi < 1.0)
            elif (self.basis == 'wendland3'):
                psi_set = ((1.0 - xi) ** 3.0) * (xi < 1.0)
            elif (self.basis == 'wendland4'):
                psi_set = ((1.0 - xi) ** 4.0 * (4.0 * xi + 1.0)) * (xi < 1.0)
            elif (self.basis == 'wendland5'):
                psi_set = ((1.0 - xi) ** 5.0 * (5.0 * xi + 1)) * (xi < 1.0)
            elif (self.basis == 'wendland6'):
                psi_set = ((1.0 - xi) ** 6.0 * 
                    (35.0 * xi ** 2.0 + 18.0 * xi + 3.0)) * (xi < 1.0)
            elif (self.basis == 'wendland7'):
                psi_set = ((1.0 - xi) ** 7.0 *
                    (16.0 * xi ** 2.0 + 7.0 * xi + 1.0)) * (xi < 1.0)
            elif (self.basis == 'wendland8'):
                psi_set = (((1.0 - xi) ** 8.0 *
                    (32.0 * xi ** 3.0 + 25.0 * xi ** 2.0 + 8.0 * xi + 1.0)) *
                    (xi < 1.0))
        psi_set = np.nan_to_num(psi_set)
        return psi_set

    def gen_weights(self, f_target):
        '''
        Generate a set of weights over the basis functions such that the
        target forcing term trajectory is matched.
         f_target shaped n_dim x n_time_steps
        '''
        # Generate the basis functions
        s_track = self.cs.rollout()
        psi_track = self.gen_psi(s_track)
        # Compute useful quantities
        sum_psi = np.sum(psi_track, 0)
        P = psi_track / sum_psi * s_track
        # Compute the weights using linear regression
        self.w = np.nan_to_num(f_target @ np.linalg.pinv(P))

    #own generate weight function without unknown optimization
    def generate_weights(self,f_target):
        

         ## Step 2: Learning of the weights using linear regression
        self.w = np.zeros([self.n_dmps, self.n_bfs + 1])
        s_track = self.cs.rollout()

        psi_set = self.gen_psi(s_track)
        psi_sum = np.sum(psi_set,0)
        psi_sum_2 = psi_sum * psi_sum
        s_track_2 = s_track * s_track
        A = np.zeros([self.n_bfs + 1, self.n_bfs + 1])
        for k in range(self.n_bfs + 1):
            A[k, k] = scipy.integrate.simps(
                psi_set[k,:] * psi_set[k,:] * s_track_2 / psi_sum_2, s_track)
            for h in range(k + 1, self.n_bfs + 1):
                A[h, k] = scipy.integrate.simps(
                    psi_set[k,:] * psi_set[h,:] * s_track_2 / psi_sum_2,
                    s_track)
                A[k, h] = A[h, k].copy()

       

        LU = scipy.linalg.lu_factor(A)

        # The weights are learned dimension by dimension
        for d in range(self.n_dmps):
            f_d_set = f_target[d,:].copy()
            # Set up the minimization problem
            b = np.zeros([self.n_bfs + 1])
            for k in range(self.n_bfs + 1):
                b[k] = scipy.integrate.simps(
                    f_d_set * psi_set[k,:] * s_track / psi_sum,
                    s_track)

            # Solve the minimization problem
           
            self.w[d, :] = np.nan_to_num(scipy.linalg.lu_solve(LU, b))
        self.learned_position = np.ones(self.n_dmps)

    def retrain_weights(self, f_target, f_target_original, s0, s1, s0_tilde, s1_tilde, indexes):
        
        #s1_tilde = s1-np.abs(1/self.width[indexes[-1]])
        #s0_tilde = s0+np.abs(1/self.width[indexes[0]])

        # Check for right order of s0_tilde and s1_tilde
        if (s0_tilde < s1_tilde):
            raise ValueError('s0_tilde must be greater than s1_tilde')

        idx = indexes
        n_rbfs = len(idx)
        
        
        ## Step 2: Learning of the retrained weights using linear regression
        rw = np.zeros([self.n_dmps, n_rbfs])
        

        new_target = f_target_original

        new_s_track = []
        original_target = []
        temp = 0
        s1_tilde_index = 0
        found_s1_tilde = False
        s0_tilde_index = 0
        found_s0_tilde = False
        s1_index = 0
        found_s1 = False
        s0_index = 0
        found_s0 = False
        started = False

        s_track = self.cs.rollout_interval(start=s0_tilde, end=s1_tilde)
        s_track_full = self.cs.rollout()

        for i in range(s_track_full.shape[0]):
            if s_track_full[i] <= s0_tilde and not found_s0_tilde:
                s0_tilde_index = i
                found_s0_tilde = True 
            elif s_track_full[i] <= s0 and not found_s0:
                s0_index = i
                found_s0 = True
            elif s_track_full[i] <= s1 and not found_s1:
                s1_index = i
                found_s1 = True
            elif s_track_full[i] <= s1_tilde and not found_s1_tilde:
                s1_tilde_index = i
                found_s1_tilde = True

            

        original_target = copy.deepcopy(f_target_original)
        original_target[:,s0_index+1:s0_index+f_target.shape[1]-2] = f_target[:,1:-2]
        #original_target = f_target_original[:,s1_tilde_index:s0_tilde_index]
        #original_target = f_target_original[:,s1_tilde_index:s0_tilde_index]
        #original_target[:,s1_index-s1_tilde_index:f_target.shape[1]-(s0_tilde_index-s0_index)]
        #original_target = np.nan_to_num(original_target)
        #original_target = np.transpose(original_target)


        original_target = original_target[:,s0_tilde_index:s1_tilde_index]




        psi_set = self.gen_psi_retrain(s_track,idx)
        psi_sum = np.sum(psi_set,0)
        psi_sum_2 = psi_sum * psi_sum
        s_track_2 = s_track * s_track
        A = np.zeros([n_rbfs, n_rbfs])
        for k in range(n_rbfs):
            A[k, k] = scipy.integrate.simps(
                np.nan_to_num(psi_set[k,:] * psi_set[k,:] * s_track_2 / psi_sum_2), s_track)
            for h in range(k + 1, n_rbfs):
                A[h, k] = scipy.integrate.simps(
                    np.nan_to_num(psi_set[k,:] * psi_set[h,:] * s_track_2 / psi_sum_2),
                    s_track)
                A[k, h] = A[h, k].copy()

       
        A = np.nan_to_num(A)
        LU = scipy.linalg.lu_factor(A)

        # The weights are learned dimension by dimension
        for d in range(self.n_dmps):
            f_d_set = original_target[d,:].copy()
            # Set up the minimization problem
            b = np.zeros([n_rbfs])
            for k in range(n_rbfs):
                b[k] = scipy.integrate.simps(
                        np.nan_to_num(f_d_set * psi_set[k,:] * s_track / psi_sum),
                    s_track)

            # Solve the minimization problem
            b = np.nan_to_num(b)
            rw[d, :] = np.nan_to_num(scipy.linalg.lu_solve(LU, b))

        self.w[:,idx] = rw

    def gen_width(self):
        '''
        Set the "widths" for the basis functions.
        '''
        if (self.basis == 'gaussian'):
            self.width = 1.0 / np.diff(self.c) / np.diff(self.c)
            self.width = np.append(self.width, self.width[-1])
        else:
            self.width = 1.0 / np.diff(self.c)
            self.width = np.append(self.width[0], self.width)


    def imitate_path(self, x_des, dx_des = None, ddx_des = None, t_des = None,
        g_w = True, **kwargs):
        '''
        Takes in a desired trajectory and generates the set of system
        parameters that best realize this path.
          x_des array shaped num_timesteps x n_dmps
          t_des 1D array of num_timesteps component
          g_w boolean, used to separate the one-shot learning from the
                       regression over multiple demonstrations
        '''

        ## Set initial state and x_goal
        self.x_0 = x_des[0].copy()
        self.x_goal = x_des[-1].copy()

        ## Set t_span
        if t_des is None:
            # Default value for t_des
            t_des = np.linspace(0, self.cs.run_time, x_des.shape[0])
        else:
            # Warp time to start from zero and end up to T
            t_des -= t_des[0]
            t_des /= t_des[-1]
            t_des *= self.cs.run_time
        time = np.linspace(0., self.cs.run_time, self.cs.timesteps)

        ## Piecewise linear interpolation
        # Interpolation function
        path_gen = scipy.interpolate.interp1d(t_des, x_des.transpose())
        # Evaluation of the interpolant
        path = path_gen(time)
        x_des = path.transpose()

        ## Second order estimates of the derivatives
        ## (the last non centered, all the others centered)
        if dx_des is None:
            D1 = compute_D1(self.cs.timesteps, self.cs.dt)
            dx_des = np.dot(D1, x_des)
        else:
            dpath = np.zeros([self.cs.timesteps, self.n_dmps])
            dpath_gen = scipy.interpolate.interp1d(t_des, dx_des)
            dpath = dpath_gen(time)
            dx_des = dpath.transpose()
        if ddx_des is None:
            D2 = compute_D2(self.cs.timesteps, self.cs.dt)
            ddx_des = np.dot(D2, x_des)
        else:
            ddpath = np.zeros([self.cs.timesteps, self.n_dmps])
            ddpath_gen = scipy.interpolate.interp1d(t_des, ddx_des)
            ddpath = ddpath_gen(time)
            ddx_des = ddpath.transpose()

        ## Find the force required to move along this trajectory
        s_track = self.cs.rollout()
        f_target = ((ddx_des / self.K - (self.x_goal - x_des) + 
            self.D / self.K * dx_des).transpose() +
            np.reshape((self.x_goal - self.x_0), [self.n_dmps, 1]) * s_track)
        if g_w:
            # Efficiently generate weights to realize f_target
            # (only if not called by paths_regression)
            #self.gen_weights(f_target)
            self.generate_weights(f_target)
            self.reset_state()
            self.learned_position = self.x_goal - self.x_0
        return f_target


    def imitate_retrained_path(self, x_des,s1, s0,t0,t1, dx_des = None, ddx_des = None, t_des = None):
        '''
        Takes in a desired trajectory and generates the set of system
        parameters that best realize this path.
          x_des array shaped num_timesteps x n_dmps
          t_des 1D array of num_timesteps component
          g_w boolean, used to separate the one-shot learning from the
                       regression over multiple demonstrations
        '''
        

        ## Set initial state and x_goal
        x_0 = self.x_0.copy()
        x_goal = self.x_goal.copy()

      
        # Default value for t_des
        t_des = np.linspace(0, t1-t0, x_des.shape[0])
       

        time = np.linspace(0., t1-t0, x_des.shape[0])

        ## Piecewise linear interpolation
        # Interpolation function
        path_gen = scipy.interpolate.interp1d(t_des, x_des.transpose())
        # Evaluation of the interpolant
        path = path_gen(time)
        x_des = path.transpose()

        ## Second order estimates of the derivatives
        ## (the last non centered, all the others centered)
        D1 = compute_D1(x_des.shape[0], self.cs.dt)
        dx_des = np.dot(D1, x_des)

        D2 = compute_D2(x_des.shape[0], self.cs.dt)
        ddx_des = np.dot(D2, x_des)
        

        ## Find the force required to move along this trajectory
        s_track = self.cs.rollout_interval(start=s0, end=s1)

        f_target = ((ddx_des / self.K - (x_goal - x_des) + 
            self.D / self.K * dx_des).transpose() +
            np.reshape((x_goal - x_0), [self.n_dmps, 1]) * s_track)
       
        return f_target

    def paths_regression(self, traj_set, t_set = None):
        '''
        Takes in a set (list) of desired trajectories (with possibly the
        execution times) and generate the weight which realize the best
        approximation.
          each element of traj_set should be shaped num_timesteps x n_dim
          trajectories
        '''

        ## Step 1: Generate the set of the forcing terms
        f_set = np.zeros([len(traj_set), self.n_dmps, self.cs.timesteps])
        g_new = np.ones(self.n_dmps)
        for it in range(len(traj_set)):
            if t_set is None:
                t_des_tmp = None
            else:
                t_des_tmp = t_set[it]
                t_des_tmp -= t_des_tmp[0]
                t_des_tmp /= t_des_tmp[-1]
                t_des_tmp *= self.cs.run_time

            # Alignment of the trajectory so that
            # x_0 = [0; 0; ...; 0] and g = [1; 1; ...; 1].
            x_des_tmp = copy.deepcopy(traj_set[it])
            x_des_tmp -= x_des_tmp[0] # translation to x_0 = 0
            g_old = x_des_tmp[-1] # original x_goal position
            R = roto_dilatation(g_old, g_new) # rotodilatation

            # Rescaled and rotated trajectory
            x_des_tmp = np.dot(x_des_tmp, np.transpose(R))

            # Learning of the forcing term for the particular trajectory
            f_tmp = self.imitate_path(x_des = x_des_tmp, t_des = t_des_tmp,
                g_w = False, add_force = None)
            f_set[it, :, :] = f_tmp.copy() # add the new forcing term to the set

        ## Step 2: Learning of the weights using linear regression
        self.w = np.zeros([self.n_dmps, self.n_bfs + 1])
        s_track = self.cs.rollout()
        psi_set = self.gen_psi(s_track)
        psi_sum = np.sum(psi_set, 0)
        psi_sum_2 = psi_sum * psi_sum
        s_track_2 = s_track * s_track
        A = np.zeros([self.n_bfs + 1, self.n_bfs + 1])
        for k in range(self.n_bfs + 1):
            A[k, k] = scipy.integrate.simps(
                psi_set[k, :] * psi_set[k, :] * s_track_2 / psi_sum_2, s_track)
            for h in range(k + 1, self.n_bfs + 1):
                A[h, k] = scipy.integrate.simps(
                    psi_set[k, :] * psi_set[h, :] * s_track_2 / psi_sum_2,
                    s_track)
                A[k, h] = A[h, k].copy()
        A *= len(traj_set)
        LU = scipy.linalg.lu_factor(A)

        # The weights are learned dimension by dimension
        for d in range(self.n_dmps):
            f_d_set = f_set[:, d, :].copy()
            # Set up the minimization problem
            b = np.zeros([self.n_bfs + 1])
            for k in range(self.n_bfs + 1):
                b[k] = scipy.integrate.simps(
                    f_d_set * psi_set[k, :] * s_track / psi_sum,
                    s_track)

            # Solve the minimization problem
            self.w[d, :] = scipy.linalg.lu_solve(LU, b)
        self.learned_position = np.ones(self.n_dmps)

    def reset_state(self, v0 = None, **kwargs):
        '''
        Reset the system state
        '''
        self.x = self.x_0.copy()
        if v0 is None:
            v0 = 0.0 * self.x_0
        self.dx = v0
        self.ddx = np.zeros(self.n_dmps)
        self.cs.reset_state()

    def retrain(self, x_new,f_target_original, t0, t1, s0, s1):
        '''
        Retrain the DMPs using the new trajectory
        '''

        print("check s0 and s1 tilde")
        retrain_idx = []
        for i in range(self.n_bfs):
            if self.c[i]-np.abs(1/self.width[i]) <= s0 and self.c[i]+np.abs(1/self.width[i]) >=s1:
                retrain_idx.append(i)

       
        s1_tilde = s1-np.abs(1/self.width[retrain_idx[-1]])
        s0_tilde = s0+np.abs(1/self.width[retrain_idx[0]])


        # Generate new DMP for training
        #temp_dmp = DMPs_cartesian(n_dmps = self.n_dmps, n_bfs= len(retrain_idx),dt = self.cs.dt, T = tend, basis='mollifier')



        #dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')

        print("check f_target")

        
        f_target = self.imitate_retrained_path(x_des = x_new,s1 = s1,s0 = s0,t0=t0,t1=t1)

        #MP = dmp(n_dmps = n_dmp, n_bfs=n_bfs, dt = dt, T = ts[-1], basis='mollifier')
        #original_target = MP.imitate_path(x_des=p)

        # Retrain the basis functions
        self.retrain_weights(f_target = f_target, f_target_original = f_target_original, s0 = s0, s1 = s1, s0_tilde = s0_tilde, s1_tilde = s1_tilde, indexes = retrain_idx)
        print("check retrain_weights")
        return f_target
        

    def rollout(self, tau = 1.0, v0 = None, **kwargs):
        '''
        Generate a system trial, no feedback is incorporated.
          tau scalar, time rescaling constant
          v0 scalar, initial velocity of the system
        '''

        # Reset the state of the DMP
        if v0 is None:
            v0 = 0.0 * self.x_0
        self.reset_state(v0 = v0)
        x_track = np.array([self.x_0])
        dx_track = np.array([v0])
        t_track = np.array([0])
        state = np.zeros(2 * self.n_dmps)
        state[range(0, 2*self.n_dmps, 2)] = copy.deepcopy(v0)
        state[range(1, 2*self.n_dmps + 1, 2)] = copy.deepcopy(self.x_0)
        if self.rescale == 'rotodilatation':
            M = roto_dilatation(self.learned_position, self.x_goal - self.x_0)
        elif self.rescale == 'diagonal':
            M = np.diag((self.x_goal - self.x_0) / self.learned_position)
        else:
            M = np.eye(self.n_dmps)
        psi = self.gen_psi(self.cs.s)
        f0 = (np.dot(self.w, psi[:, 0])) / (np.sum(psi[:, 0])) * self.cs.s
        f0 = np.nan_to_num(np.dot(M, f0))
        ddx_track = np.array([-self.D * v0 + self.K*f0])
        err = np.linalg.norm(state[range(1, 2*self.n_dmps + 1, 2)] - self.x_goal)
        P = phi1(self.cs.dt * self.linear_part / tau)
        while err > self.tol:
            psi = self.gen_psi(self.cs.s)
            f = (np.dot(self.w, psi[:, 0])) / (np.sum(psi[:, 0])) * self.cs.s
            f = np.nan_to_num(np.dot(M, f))
            beta = np.zeros(2 * self.n_dmps)
            beta[range(0, 2*self.n_dmps, 2)] = \
                self.K * (self.x_goal * (1.0 - self.cs.s) + 
                self.x_0 * self.cs.s + f) / tau
            vect_field = np.dot(self.linear_part / tau, state) + beta
            state += self.cs.dt * np.dot(P, vect_field)
            x_track = np.append(x_track,
                np.array([state[range(1, 2*self.n_dmps + 1, 2)]]), axis=0)
            dx_track = np.append(dx_track,
                np.array([state[range(0, 2*self.n_dmps, 2)]]), axis=0)
            t_track = np.append(t_track, t_track[-1] + self.cs.dt)
            err = np.linalg.norm(state[range(1, 2*self.n_dmps + 1, 2)] - self.x_goal)
            self.cs.step(tau=tau)
            ddx_track = np.append(ddx_track,
                np.array([self.K * (self.x_goal - x_track[-1]) -
                    self.D * dx_track[-1] -
                    self.K * (self.x_goal - self.x_0) * self.cs.s +
                    self.K * f]), axis=0)
        return x_track, dx_track, ddx_track, t_track

    def step(self, tau = 1.0, error = 0.0, external_force = None,
        adapt=False, tols=None, **kwargs):
        '''
        Run the DMP system for a single timestep.
          tau float: time rescaling constant
          error float: optional system feedback
          external_force 1D array: external force to add to the system
          adapt bool: says if using adaptive step
          tols float list: [rel_tol, abs_tol]
        '''

        ## Initialize
        if tols is None:
            tols = [1e-03, 1e-06]
        ## Setup
        # Scaling matrix
        if self.rescale == 'rotodilatation':
            M = roto_dilatation(self.learned_position, self.x_goal - self.x_0)
        elif self.rescale == 'diagonal':
            M = np.diag((self.x_goal - self.x_0) / self.learned_position)
        else:
            M = np.eye(self.n_dmps)
        # Coupling term in canonical system
        error_coupling = 1.0 + error
        alpha_tilde = - self.cs.alpha_s / tau / error_coupling
        # State definition
        state = np.zeros(2 * self.n_dmps)
        state[0::2] = self.dx
        state[1::2] = self.x
        # Linear part of the dynamical system
        A_m = self.linear_part / tau
        # s-dep part of the dynamical system
        def beta_s(s, x, v):
            psi = self.gen_psi(s)
            f = (np.dot(self.w, psi[:, 0])) / (np.sum(psi[:, 0])) * self.cs.s
            f = np.nan_to_num(np.dot(M, f))
            out = np.zeros(2 * self.n_dmps)
            out[0::2] = self.K * (self.x_goal * (1.0 - s) + self.x_0 * s + f)
            if external_force is not None:
                out[0::2] += external_force(x, v)
            return out / tau
        ## Initialization of the adaptive step
        flag_tol = False
        while not flag_tol:
            # Bogacki–Shampine method
            # Defining the canonical system in the time of the scheme
            s1 = copy.deepcopy(self.cs.s)
            s2 = s1 * np.exp(-alpha_tilde * self.cs.dt * 1.0/2.0)
            s3 = s1 * np.exp(-alpha_tilde * self.cs.dt * 3.0/4.0)
            s4 = s1 * np.exp(-alpha_tilde * self.cs.dt)
            xi1 = np.dot(A_m, state) + beta_s(s1, state[1::2], state[0::2])
            xi2 = np.dot(A_m, state + self.cs.dt * xi1 * 1.0/2.0) + \
                beta_s(s2, state[1::2] + self.cs.dt * xi1[1::2] * 1.0/2.0,
                    state[0::2] + self.cs.dt * xi1[0::2] * 1.0/2.0)
            xi3 = np.dot(A_m, state + self.cs.dt * xi2 * 3.0/4.0) + \
                beta_s(s3, state[1::2] + self.cs.dt * xi2[1::2] * 3.0/4.0,
                state[0::2] + self.cs.dt * xi2[0::2] * 3.0/4.0)
            xi4 = np.dot(A_m, state + self.cs.dt * (2.0 * xi1 + 3.0 * xi2 + 4.0 * xi3) / 9.0) + \
                beta_s(s4, state[1::2] + self.cs.dt * (2.0 * xi1[1::2] + 3.0 * xi2[1::2] + 4.0 * xi3[1::2]) / 9.0,
                state[0::2] + self.cs.dt * (2.0 * xi1[0::2] + 3.0 * xi2[0::2] + 4.0 * xi3[0::2]) / 9.0)
            y_ord2 = state + self.cs.dt * (2.0 * xi1 + 3.0 * xi2 + 4.0 * xi3) / 9.0
            y_ord3 = state + self.cs.dt * (7.0 * xi1 + 6.0 * xi2 + 8.0 * xi3 + 3.0 * xi4) / 24.0
            if (np.linalg.norm(y_ord2 - y_ord3) < tols[0] * np.linalg.norm(state) + tols[1]) or (not adapt):
                flag_tol = True
                state = copy.deepcopy(y_ord3)
            else:
                self.cs.dt /= 1.1
        self.cs.step(tau=tau, error_coupling=error_coupling)
        self.x = copy.deepcopy(state[1::2])
        self.dx = copy.deepcopy(state[0::2])
        psi = self.gen_psi(self.cs.s)
        f = (np.dot(self.w, psi[:, 0])) / (np.sum(psi[:, 0])) * self.cs.s
        f = np.nan_to_num(np.dot(M, f))
        self.ddx = (self.K * (self.x_goal - self.x) - self.D * self.dx \
            - self.K * (self.x_goal - self.x_0) * self.cs.s + self.K * f) / tau
        if external_force is not None:
            self.ddx += external_force(self.x, self.dx) / tau
        return self.x, self.dx, self.ddx
