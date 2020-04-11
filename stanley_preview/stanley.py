import  math
import matplotlib.pyplot as plt
import  numpy as np

k = 1 # k应该随着前轮车速变化而变化 k = a*v+b,其中系数需要自己调速
kp = 2
dt = 0.02
L = 1.8
Lambda = 0.8 #stanley模型计算出角度输出的权重，越大时转弯效果越好，但是走直线时效果比较差。

class VehicleState:
    def __init__(self,x=0.0,y=0.0,yaw=0.0,v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.a = 0.0
        self.delta = 0.0
        #self.e = []

     
    def Update(self):
        self.x = self.x + self.v * dt * math.cos(self.yaw)
        self.y = self.y + self.v * dt * math.sin(self.yaw)
        self.yaw = self.yaw + self.v * math.sin(self.delta) / L
        self.yaw = self.NormalizeAngle(self.yaw)
        self.v = self.v + self.a * dt
    
    def NormalizeAngle(self,angle):
        if (angle > math.pi):
            angle = angle - 2*math.pi
        elif (angle < -math.pi):
            angle = 2*math.pi + angle
        else:
            angle = angle

        return angle

    def AxisTrans(self,x,y):
        tx = (x-self.x) * math.cos(self.yaw) + (y - self.y) * math.sin(self.yaw)
        ty = -(x -self.x) * math.sin(self.yaw) + (y - self.y) * math.cos(self.yaw)
        return tx,ty
       
    def PID(self,targetV):
        self.a = kp * (targetV - self.v)

    def Calculate(self,cx,cy):
        dx = [self.x - ix for ix in cx ]
        dy = [self.y - iy for iy in cy]
        d = [math.sqrt(tx**2 + ty**2) for (tx,ty) in zip(dx,dy)]
        e = min(d)
        targetIndex = d.index(e)
        preViewInd = targetIndex + 3
        
        if (preViewInd >= len(cx)):
            preViewInd = len(cx) - 1
            
        preX,preY = self.AxisTrans(cx[preViewInd],cy[preViewInd])
        preViewDelta = self.NormalizeAngle(math.atan2(preY,preX))

        tmp_y = -(cx[targetIndex] -self.x)* math.sin(self.yaw) + (cy[targetIndex]-self.y)*math.cos(self.yaw)
        if (tmp_y < 0):
            e = -e
        elif (tmp_y == 0):
            e = 0
        else:
            e = e

        index = []
        ind = 0

        while ind < (len(cx)-1):
            ind_tmp = math.atan2(cy[ind+1]-cy[ind],cx[ind+1]-cx[ind])
            ind_tmp = self.NormalizeAngle(ind_tmp)
            index.append(ind_tmp)
            ind += 1
        
        if targetIndex < len(cx)-1:
            targetYaw = index[targetIndex]
        else:
            targetYaw = index[len(cx)-2]

        return e,targetYaw,targetIndex,preViewDelta
    
    def StanleyControl(self,e,targetYaw,preViewDelta):
        phi = 0.0
        deltaStanley = 0.0
        deltaPreview = 0.0

        deltaMax = 26 * math.pi /180

        if (targetYaw - self.yaw) < math.pi:
            phi = targetYaw- self.yaw
        else:
            if (targetYaw > 0):
                phi = -2*math.pi + targetYaw - self.yaw
            else:
                phi = 2*math.pi - targetYaw + self.yaw

        deltaStanley = phi + math.atan2(k * e , self.v ) # 要对前轮转角加以约束
        delta = Lambda * deltaStanley + (1-Lambda)* preViewDelta
        #delta = deltaStanley
        #delta = preViewDelta

        if (delta > deltaMax):
            self.delta = deltaMax
        elif (delta < -deltaMax):
            self.delta = -deltaMax
        else:
            self.delta = delta
    
def main():

    cx = np.arange(0,50,1)
    cy =[math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 30.0 / 3.6
    maxIndex = len(cx) - 1
    #print(maxIndex)
    T = 100.0

    state = VehicleState(x = -0.0, y = -3.0, yaw = 0.0, v = 0.0)

    time = 0
    x = [state.x]
    y = [state.y]
    v = [state.v]
    t = [0.0]
    targetIndex = 0

    while time <= T and targetIndex < maxIndex:
        
        state.PID(target_speed)
        #print(state.a,state.v)
        e, targetYaw, targetIndex,preViewDelta = state.Calculate(cx,cy)
        #print(targetIndex)
        state.StanleyControl(e,targetYaw,preViewDelta)
        state.Update()
        time += dt
        #print(state.delta)

        x.append(state.x)
        y.append(state.y)
        t.append(time)
        v.append(state.v)
        
    #plt.cla()
    plt.plot(cx,cy,".r",label="course")
    plt.plot(x,y,"-b",label="trajectory")
    #plt.plot(cx[targetIndex],cy[targetIndex],"go",label="target")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
    #plt.title("speed[Km/h]:"+ str(state.v*3.6)[:4])
    #plt.pause(0.005)

if __name__ == "__main__":
    main()



    

    


