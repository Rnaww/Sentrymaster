import math
from PIL import Image

# 701: 行对应 map X，列对应 map Y，灰度值换算为米。
class Terrain:
    def __init__(self, filename):
        self.image = Image.open(filename).convert('L')
        self.pixels = self.image.load()
        self.offsets = {5:[], 6:[]}
        for radius in self.offsets:
            for i in range(-radius,radius+1):
                width = round(math.sqrt(radius*radius-i*i))
                for j in range(-width,width+1): self.offsets[radius].append((i,j))

    def flat(self, x, y, radius=5):
        # 车体旋转包络加栅格误差，半径约 0.5m。
        dx = max(14.2-x,0,x-16.2)
        dy = max(2.2-y,0,y-6.8)
        # 补给区停稳时仍检查完整车体高度；行驶时保留起伏路段禁转缓冲。
        supply_stopped = radius == 5 and math.hypot(x-15.45,y-1.95) <= 0.4
        if dx*dx+dy*dy <= (radius*0.1)**2 and not supply_stopped: return False
        row,col = int((x+25)/0.1),int((y+25)/0.1)
        heights = []
        for i,j in self.offsets[radius]:
            r,c = row+i,col+j
            if not (0 <= r < self.image.height and 0 <= c < self.image.width): return False
            h = self.pixels[c,r]*0.8/255
            heights.append(h)
        return max(heights)-min(heights) <= 0.015

    def spin_allowed(self, x, y, vx=0, vy=0):
        # 行驶增加 0.1m 缓冲，避免拐弯后方向变化及反馈延迟导致坡面残留旋转。
        # 停稳使用实际包络，堡垒平顶能正常旋转。
        # 与停留节点的0.05m/s停稳标准一致，微小定位抖动不触发行驶缓冲。
        radius = 6 if math.hypot(vx,vy) >= 0.05 else 5
        return all(self.flat(x+vx*0.1*t,y+vy*0.1*t,radius) for t in [0,0.5,1])

def rotate_vector(q, v):
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if norm < 1e-9: return (0,0,0)
    x,y,z,w = q.x/norm,q.y/norm,q.z/norm,q.w/norm
    tx,ty,tz = 2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])
    return (v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx)
