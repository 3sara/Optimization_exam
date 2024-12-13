import numpy as np
import random

def compute_end_pos(links, theta, alpha):
    '''
    compute the end effector position given the links lengths, 
    theta and alpha values
    '''
    pos=np.array([0,0,0,1])
    for i in range(len(links)-1,-1,-1):
        st=np.sin(theta[i])
        ct=np.cos(theta[i])
        sa=np.sin(alpha[i])
        ca=np.cos(alpha[i])
        transf=[[ct, -ca*st, st*sa,  links[i]*(ct-ca*st)],
                [st, ca*ct, -ca*sa,  links[i]*(st+ca*ct)],
                [0,  sa,    ca,      sa*links[i]        ],
                [0,  0,     0,       1                  ]]
        pos=np.dot(transf,pos)
    return pos[:3]

def check_constraints(indiv, vel, constraints_theta, type):
    '''
    check if the individual is within the constraints,
    if not update position and velocity accordiing to type
    '''
    if constraints_theta is None:
        return indiv, vel
    if type == 'absorb':
        for direction in range(len(constraints_theta)):
            if indiv[direction]<constraints_theta[direction][0]:
                indiv[direction]=constraints_theta[direction][0]
                vel[direction]=0
            elif indiv[direction]>constraints_theta[direction][1]:
                indiv[direction]=constraints_theta[direction][1]
                vel[direction]=0
    elif type == 'reflect':
        for direction in range(len(constraints_theta)):
            if indiv[direction]<constraints_theta[direction][0]:
                indiv[direction]=constraints_theta[direction][0]
                vel[direction]=-vel[direction]
            elif indiv[direction]>constraints_theta[direction][1]:
                indiv[direction]=constraints_theta[direction][1]
                vel[direction]=-vel[direction]
    elif type == 'damping':
        for direction in range(len(constraints_theta)):
            if indiv[direction]<constraints_theta[direction][0]:
                indiv[direction]=constraints_theta[direction][0]
                vel[direction]=-random.uniform(0,1)*vel[direction]
            elif indiv[direction]>constraints_theta[direction][1]:
                indiv[direction]=constraints_theta[direction][1]
                vel[direction]=-random.uniform(0,1)*vel[direction]
    return indiv, vel

def PSO(links, target, pop_size, max_iter, w, c_soc, c_cog, alpha_values=None, constraints_theta=None, type='absorb'):
    '''
    !!!     CONSTRAINT FOR ALPHA : directly specify ALPHA VALUES        !!!
    !!!     CONSTRAINT FOR THETA : specify CONSTRAINTS for THETA        !!!
    input:
        -links: list of links lengths
        -target: target position
        -pop_size: population size
        -max_iter: maximum number of iterations
        -w: inertia weight
        -c_soc: social component
        -c_cog: cognitive component
        -alpha_values: list of alpha values
        -constraints_theta: list of constraints for theta values

    output:
        -theta_best_hist: best theta values for each iteration
        -alpha_best_hist: best alpha values for each iteration
        -fit: final fitness value
        -it: number of iterations
        -exit: exit status (1,0,0) if target reached, (0,1,0) if max_iter reached, (0,0,1) if stationary.
    '''
    #define function to udate alpha according to constraints
    if alpha_values==None:
        dim=len(links)*2
        def update_alpha(i):
            return pop[i][len(links):]
        def update_best_alpha():
            return gbest[len(links):].copy()
    else:
        dim=len(links)
        def update_alpha(i):
            return alpha_values
        def update_best_alpha():
            return alpha_values

    #initialize population
    if constraints_theta==None:
        #random initialization of the population
        pop=np.random.rand(pop_size,dim)*2*np.pi
    else:
        pop=np.random.rand(pop_size,dim)*2*np.pi
        for i in range(pop_size):
            #ensure that theta are initialized randomly within constraints
            elem=[random.uniform(constrain[0],constrain[1]) for constrain in constraints_theta]
            pop[i,:len(links)]=elem
    
    #initialize velocity 
    vel=np.zeros((pop_size,dim))

    #initialize personal and global best
    pbest=pop.copy()
    pbest_fit=np.zeros(pop_size)
    gbest=pop[0].copy()
    gbest_fit=1e10

    #initialize history
    theta_best_hist=[]
    alpha_best_hist=[]

    #initialize pbest and gbest
    for i in range(pop_size):
        theta= pop[i][0:len(links)]
        alpha= update_alpha(i)
        pos=compute_end_pos(links, theta, alpha)
        pbest_fit[i]=np.linalg.norm(pos-target)
        if pbest_fit[i]<gbest_fit:
            gbest=pop[i].copy()
            gbest_fit=pbest_fit[i]

    it=0
    stat_it=0
    theta_best_hist.append(gbest[0:len(links)].copy())
    alpha_best_hist.append(update_best_alpha())

    #main loop
    while it<max_iter:
        it+=1
        for i in range(pop_size):
            #compute position and fitness
            theta= pop[i][0:len(links)]
            alpha= update_alpha(i)
            end_pos = compute_end_pos(links, theta, alpha)
            fit=np.linalg.norm(end_pos-target)

            #chech for pbest update
            if fit<pbest_fit[i]:
                pbest[i]=pop[i].copy()
                pbest_fit[i]=fit
                #chech for gbest update
                if fit<gbest_fit:
                    gbest=pop[i].copy()
                    gbest_fit=fit
                    #check for solution found
                    if fit<1e-3:
                        theta_best_hist.append(gbest[0:len(links)].copy())
                        alpha_best_hist.append(update_best_alpha())
                        exit=np.array([1,0,0])
                        return theta_best_hist, alpha_best_hist, gbest_fit, it, exit
                    
            #update velocity and position
            vel[i]=w*vel[i]+c_soc*np.random.rand()*(gbest-pop[i])+c_cog*np.random.random()*(pbest[i]-pop[i])
            pop[i]+=vel[i] 
            pop[i], vel[i] = check_constraints(pop[i], vel[i], constraints_theta, type)

        theta_best_hist.append(gbest[0:len(links)].copy())
        alpha_best_hist.append(update_best_alpha())

        #check for stationary
        if it>1 and np.linalg.norm(vel)<1e-3:
            stat_it+=1
            if stat_it>5:
                exit=np.array([0,0,1])
                return theta_best_hist, alpha_best_hist, gbest_fit, it, exit

    exit=np.array([0,1,0])
    return theta_best_hist, alpha_best_hist, gbest_fit, it, exit