import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


def DisplayNlinkarm(links, theta, alpha, target):
    '''
    display a plot of the n-link arm in 3D space 
    given the links lengths, alpha and theta values and the target
    '''

    #compute the end position of each link
    trasf=np.eye(4)
    x=[0]
    y=[0]
    z=[0]
    for i in range(len(links)):
        st=np.sin(theta[i])
        ct=np.cos(theta[i])
        sa=np.sin(alpha[i])
        ca=np.cos(alpha[i])
        new_trasf=[[ct, -ca*st, st*sa,  links[i]*(ct-ca*st)],
                   [st, ca*ct, -ca*sa,  links[i]*(st+ca*ct)],
                   [0,  sa,    ca,      sa*links[i]        ],
                   [0,  0,     0,       1                  ]]
        trasf=np.dot(trasf,new_trasf)
        x.append(np.dot(trasf,[0,0,0,1])[0])
        y.append(np.dot(trasf,[0,0,0,1])[1])
        z.append(np.dot(trasf,[0,0,0,1])[2])
    print(f"end effector position: {x[-1],y[-1],z[-1]}")

    #plot in 3D space
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z)
    plt.plot(target[0],target[1],target [2], 'ro')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def Display_animated(links,theta, alpha, target, title, jump=1):
    '''
    display an animated plot of the n-link arm in 3D space
    given the links lengths, alpha and theta values and the target
    each frame is computed every jump iterations
    '''
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')  
    line = ax.plot([], [], [])
    #plot target
    plt.plot(target[0], target[1], target[2], 'ro')

    #function to update the plot
    def animate(i, line):
        trasf=np.eye(4)
        x=[0]
        y=[0]
        z=[0]
        for j in range(len(links)):
            st=np.sin(theta[i*jump][j])
            ct=np.cos(theta[i*jump][j])
            sa=np.sin(alpha[i*jump][j])
            ca=np.cos(alpha[i*jump][j])
            new_trasf=[[ct, -ca*st, st*sa,  links[j]*(ct-ca*st)],
                    [st, ca*ct, -ca*sa,  links[j]*(st+ca*ct)],
                    [0,  sa,    ca,      sa*links[j]        ],
                    [0,  0,     0,       1                  ]]
            trasf=np.dot(trasf,new_trasf)
            x.append(np.dot(trasf,[0,0,0,1])[0])
            y.append(np.dot(trasf,[0,0,0,1])[1])
            z.append(np.dot(trasf,[0,0,0,1])[2])
        line.set_data(x,y)
        line.set_3d_properties(z)
        if i==len(theta)/jump-1:
            plt.title('Iteration: '+str(i*jump)+" Converged")
        else:
            plt.title('Iteration: '+str(i*jump))
        return line,

    anim = animation.FuncAnimation(fig, animate,
                                frames=len(theta)//jump, fargs=(line), interval=1000)
    anim.save('gif/'+title+'.gif', fps=2)
    plt.show()
    plt.close()

