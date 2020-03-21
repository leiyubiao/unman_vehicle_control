import numpy as np
import math
import matplotlib.pyplot as plt

K = 0.1 #前视距离系数
Lfc = 2.0 #前视距离,调速K和Lfc，将决定算法的跟踪算法。如果Lfc变大，则跟踪会变得更平顺，如果变小则跟踪会更精确。
Kp = 1.0 #速度P控制器系数
dt = 0.1 #时间间隔，单件 s
L = 2.9 #车辆轴距，单位：m

class VehicleState:

    def __init__(self,x=0.0,y=0.0,yaw=0.0,v=0.0):
        self.x = x
        self.y = y # the position of car
        self.yaw = yaw # the yaw of car, 0-360 degree
        self.v = v  # the speed of car, constant;
    
def update(state,a,delta): # update the state of car
    
    state.x = state.x +state.v*math.cos(state.yaw)*dt
    state.y = state.y + state.v*math.sin(state.yaw)*dt
    state.yaw = state.yaw +  math.tan(delta)*state.v / L * dt # the rate of yaw: dot(yaw) = v*tan(delta_f)/L;  the delta is chaing, 
                                                              # so the position is chaging.
    state.v = state.v + a * dt # PID controller 
    return state

def calc_target_index(state,cx,cy):
    dx = [state.x - icx for icx in cx] #对象state的位置(x,y)在不断地变化
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx**2+idy**2)) for (idx,idy) in zip(dx,dy)]
    ind = d.index(min(d)) # the nearsest point, which is nearest to the position of car.
    L = 0.0

    Lf = K*state.v + Lfc

    while L < Lf and (ind+1) < len(cx):#from the nearest to the target point
        dx_t = cx[ind+1] - cx[ind]
        dy_t = cy[ind+1] - cx[ind]
        L += math.sqrt(dx_t**2 + dy_t ** 2)
        ind += 1

    return ind # the index of target point 

def PContorl(target,current):
    a = Kp * (target - current)
    return a

def pure_pursuit_control(state, cx, cy, pind):
    ind = calc_target_index(state, cx, cy)

    if pind >= ind: # what is pind?
        ind = pind
    
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1] # endif to get the tx and ty4

    alpha = math.atan2(ty-state.y,tx-state.x) - state.yaw

    if state.v < 0:
        alpha = math.pi - alpha
    
    Lf = K * state.v + Lfc

    delta = math.atan2(2.0*L*math.sin(alpha)/Lf, 1.0) # the output: delta_f, the angle of front wheels.

    return delta, ind

def main():
    cx = np.arange(0,50,1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 30.0 / 3.6

    T = 100.0

    state = VehicleState(x = -0.0, y = -3.0, yaw = 0.0, v = 0.0)

    lastIndex = len(cx) - 1 # the index is from 0;
    time = 0.0 
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state,cx,cy)

    while T >= time and lastIndex > target_ind:
        ai = PContorl(target_speed,state.v)
        di, target_ind = pure_pursuit_control(state,cx,cy,target_ind)
        state = update(state,ai,di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        plt.cla()
        plt.plot(cx,cy,".r",label="course")
        plt.plot(x,y,"-b",label="trajectory")
        plt.plot(cx[target_ind],cy[target_ind],"go",label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("speed[Km/h]:"+ str(state.v*3.6)[:4])
        plt.pause(0.001)

if __name__ == '__main__':
    main()


    



