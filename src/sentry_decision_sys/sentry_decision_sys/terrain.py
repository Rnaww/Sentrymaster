import math
from PIL import Image

# 701: 行对应 map X，列对应 map Y，灰度值换算为米。
class Terrain:
    def __init__(self, filename):
        self.image = Image.open(filename).convert('L')
        self.pixels = self.image.load()
        self.offsets = []
        for i in range(-5,6):
            width = round(math.sqrt(25-i*i))
            for j in range(-width,width+1): self.offsets.append((i,j))

    def flat(self, x, y):
        # 车体旋转包络加栅格误差，半径约 0.5m。
        dx = max(14.2-x,0,x-16.2)
        dy = max(2.2-y,0,y-6.8)
        if dx*dx+dy*dy <= 0.5*0.5: return False
        row,col = int((x+25)/0.1),int((y+25)/0.1)
        heights = []
        for i,j in self.offsets:
            r,c = row+i,col+j
            if not (0 <= r < self.image.height and 0 <= c < self.image.width): return False
            h = self.pixels[c,r]*0.8/255
            heights.append(h)
        return max(heights)-min(heights) <= 0.015

    def spin_allowed(self, x, y, vx=0, vy=0):
        # 提前 0.1s 停止旋转，不等车体已经进入坡面才发停止命令。
        return all(self.flat(x+vx*0.1*t,y+vy*0.1*t) for t in [0,0.5,1])

def rotate_vector(q, v):
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if norm < 1e-9: return (0,0,0)
    x,y,z,w = q.x/norm,q.y/norm,q.z/norm,q.w/norm
    tx,ty,tz = 2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])
    return (v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx)
