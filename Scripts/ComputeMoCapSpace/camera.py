from __future__ import print_function, division
import numpy as np

from py3dmath import Vec3, Rotation

class Camera:
    def __init__(self, pos, centralPoint, rotationAngle, horizFOV_deg, verFOV_deg):
        self.pos = pos
        self.normal = (centralPoint-pos).to_unit_vector()
        
        yaw   = np.arctan2(self.normal[1], self.normal[0])
        pitch = -np.arcsin(self.normal[2])
        self.rot = Rotation.from_euler_YPR([yaw, pitch, rotationAngle*np.pi/180])

        self.rotAng = rotationAngle*np.pi/180

        self.hFOV = np.deg2rad(horizFOV_deg)
        self.vFOV = np.deg2rad(verFOV_deg)
        
        rh1 = Vec3(0, -np.tan(0.5*self.hFOV), 0)
        rh2 = Vec3(0,  np.tan(0.5*self.hFOV), 0)
        rv1 = Vec3(0, 0, -np.tan(0.5*self.vFOV))
        rv2 = Vec3(0, 0,  np.tan(0.5*self.vFOV))
        
        #rotate
        self.fov1 = self.rot*(rh1+rv1)
        self.fov2 = self.rot*(rh2+rv1)
        self.fov3 = self.rot*(rh1+rv2)
        self.fov4 = self.rot*(rh2+rv2)
        
        self.fov1 += self.normal
        self.fov2 += self.normal
        self.fov3 += self.normal
        self.fov4 += self.normal

        v1 = self.pos - self.pos+self.fov1
        v2 = self.pos - self.pos+self.fov2
        v3 = self.pos - self.pos+self.fov3
        v4 = self.pos - self.pos+self.fov4
        
        self.n = []
        self.n.append(v2.cross(v1).to_unit_vector())
        self.n.append(v3.cross(v4).to_unit_vector())
        self.n.append(v1.cross(v3).to_unit_vector())
        self.n.append(v4.cross(v2).to_unit_vector())
        return
        
        
    def get_is_seen(self, point):
        vToPoint = (point - self.pos).to_unit_vector()
        
        for n in self.n:
            if vToPoint.dot(n) > 0:
                return False
            
        return True
    
    
    def plot(self, ax):
        ax.plot([self.pos[0]], [self.pos[1]], [self.pos[2]], 'ko')
        
        
        plot_vector(ax, self.pos, self.pos+self.fov1, 'r:')
        plot_vector(ax, self.pos, self.pos+self.fov2, 'r:')
        plot_vector(ax, self.pos, self.pos+self.fov3, 'r:')
        plot_vector(ax, self.pos, self.pos+self.fov4, 'r:')
        plot_vector(ax, self.pos+self.fov1, self.pos+self.fov2, 'r--')
        plot_vector(ax, self.pos+self.fov3, self.pos+self.fov4, 'r--')
        plot_vector(ax, self.pos+self.fov1, self.pos+self.fov3, 'r--')
        plot_vector(ax, self.pos+self.fov2, self.pos+self.fov4, 'r--')
        
        
    def get_view_points(self, roomxlims, roomylims, roomzlims):
        '''The space is assumed to consist of walls aligned with axes,
           and is defined by the limits roomNlims (which is each a pair 
           of numbers)
           point 0 is center view point, 
                 1-4 extrema
                 remainder are just in-between
        '''
        
        planes = ((Vec3(roomxlims[0], 0, 0), Vec3(+1,0,0)),
                  (Vec3(roomxlims[1], 0, 0), Vec3(-1,0,0)),
                  (Vec3(0, roomylims[0], 0), Vec3(0,+1,0)),
                  (Vec3(0, roomylims[1], 0), Vec3(0,-1,0)),
                  (Vec3(0, 0, roomzlims[0]), Vec3(0,0,+1)),
                  (Vec3(0, 0, roomzlims[1]), Vec3(0,0,-1)))
        
        #project the camera's view onto the planes, find the projection that's in the room:
        
        outPoints = []
        
        
        normals = [self.normal, self.fov1, self.fov2, self.fov3, self.fov4]
        
        npoints = 20
        for alpha in np.linspace(0,1,npoints):
            for beta in np.linspace(0,1,npoints):
                normals.append( (self.fov1*alpha + self.fov2*(1-alpha))*beta + (self.fov3*alpha + self.fov4*(1-alpha))*(1-beta))
        
        for norm in normals:
            norm /= norm.norm2()#make sure it's a unit vector

            minDist = np.inf
            for (p,n) in planes:
                relPos = p-self.pos

                if norm.dot(n) == 0:
                    #this plane is parallel to camera
                    continue

                distToWallAlongNormal = n.dot(relPos)/n.dot(norm)
                
                if distToWallAlongNormal < 1e-3:
                    #want to have points in front of the camera
                    continue
                
                if distToWallAlongNormal < minDist:
                    minDist = distToWallAlongNormal 
                    
            if not minDist < np.inf:
                print('Couldn''t find view point projection to wall for camera at',self.pos)

            outPoints.append(self.pos + minDist*norm)
            
        return outPoints


def check_coverage(cameras, minCorner, maxCorner, resolution):
    '''Return a list of 3d points visible to the cameras
    '''
    
    results = []
    for i in range(len(cameras)+1):
        results.append([])
        
    
    for x in np.arange(minCorner[0], maxCorner[0]+resolution, resolution):
        for y in np.arange(minCorner[1], maxCorner[1]+resolution, resolution):
            for z in np.arange(minCorner[2], maxCorner[2]+resolution, resolution):
                p = Vec3(x,y,z)
                countVisible = 0
                for c in cameras:
                    if c.get_is_seen(p):
                        countVisible += 1
                
                results[countVisible].append(p.to_list())
    
    out = []
    for r in results:
        out.append(np.array(r))
    return out


def plot_vector(ax, start, end, style=''):
    ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], style)
    return
    

def plot_room(ax, cameras, c1,c2,c3,c4,c5,c6,c7,c8):
    #plot the room's outline:
    plot_vector(ax, c1, c2, 'k:')
    plot_vector(ax, c2, c4, 'k:')
    plot_vector(ax, c4, c3, 'k:')
    plot_vector(ax, c3, c1, 'k:')
    plot_vector(ax, c5, c6, 'k:')
    plot_vector(ax, c6, c8, 'k:')
    plot_vector(ax, c8, c7, 'k:')
    plot_vector(ax, c7, c5, 'k:')
    plot_vector(ax, c1, c5, 'k:')
    plot_vector(ax, c2, c6, 'k:')
    plot_vector(ax, c3, c7, 'k:')
    plot_vector(ax, c4, c8, 'k:')

    for c in cameras:
        c.plot(ax)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    return

