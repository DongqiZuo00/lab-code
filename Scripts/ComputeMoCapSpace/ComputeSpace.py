from __future__ import print_function, division
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm

from py3dmath import Vec3
from camera import Camera, check_coverage, plot_room

#===============================================================================
# Problem setup
#===============================================================================

#camera properties
hFov = 70
vFov = 49

#assume we only get to use this fraction of the FOV 
FOV_reductionFactor = 1.0

#set this to true to run the visibility analysis
runVisiblityAnalysis = True

#set this to true to have the tool return the projected viewpoint center onto the opposite walls (also shown in figure)
runCameraViewpoint = True

#Three segments of cameras:
#offset of camera from walls
d = 15/100
if False:
    print("Idealized room dimensions")
    # Room dimensions
    length = 6 #x limits
    width  = 5 #y limits
    hCeil  = 4.7 #ceiling height

    xmin = -length/2
    xmax =  length/2
    ymin = -width/2
    ymax =  width/2
    zmin =  0
    zmax =  hCeil
    #ideal positions
    # on thin edge, by door (negative x values)
    cam1 = Camera(Vec3(-length/2+d, -width/2+d, hCeil-d), Vec3( length/2,  2, 2.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam2 = Camera(Vec3(-length/2+d,  width/2-d, 2.7),   Vec3( length/2, -2, 1.5), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)

    # on thin edge, far from door
    cam3 = Camera(Vec3( length/2-d, -width/2+d, hCeil-d), Vec3(0, 2, 0.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam4 = Camera(Vec3( length/2-d,  width/2-d, hCeil-d), Vec3(0,-2, 0.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)

    cam5 = Camera(Vec3(-length/2+d,        0, hCeil-d), Vec3( 0, 0, 0.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam6 = Camera(Vec3( length/2-d,        0, hCeil-d), Vec3( 0, 0, 0.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
           
    #on central edge
    cam7 = Camera(Vec3( 0, -width/2+d, hCeil-d),  Vec3( 2.0, 2.0, 0.0), 90, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam8 = Camera(Vec3( 0, +width/2-d, hCeil-d),  Vec3( 2.0,-2.0, 0.0), 90, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
else:
    print("True room dimensions")
    #measured positions
    # on thin edge, by door (negative x values)
    xmin = -2.51#-2.333 - d
    xmax =  5.365 #5.239 + d 
    ymin = -3.218#3.117 - d
    ymax =  3.218 #+ d
    zmin =  0
    zmax =  5.126#5.060 + d
    cam1 = Camera(Vec3(5.185,  3.049, 5.063), Vec3( xmin, -2, 0.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam2 = Camera(Vec3(5.239, -2.950, 3.182), Vec3( xmin, +2, 0.5), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)

    # on thin edge, far from door
    cam3 = Camera(Vec3(-2.337, 3.049, 5.032),  Vec3((xmin+xmax)/2,-1, 1.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam4 = Camera(Vec3(-2.200,-3.117, 5.071),  Vec3((xmin+xmax)/2,+1, 1.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)

    cam5 = Camera(Vec3( 5.057,  0, 5.046),  Vec3( (xmin+xmax)/2-6, 0, 0.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam6 = Camera(Vec3(-2.2187, 0, 5.030),  Vec3( (xmin+xmax)/2+6,   0, 0.0), 0, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
           
    #on central edge
    cam7 = Camera(Vec3(1.288,  2.947, 5.050),  Vec3(0, 0, 0), 90, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    cam8 = Camera(Vec3(1.447, -2.950, 4.998),  Vec3(0, 0, 0), 90, hFov*FOV_reductionFactor, vFov*FOV_reductionFactor)
    


cameras = [cam1, cam2, cam3, cam4, cam5, cam6, cam7, cam8]

#Room corners
c1 = Vec3(xmin, ymin, zmin)
c2 = Vec3(xmax, ymin, zmin)
c3 = Vec3(xmin, ymax, zmin)
c4 = Vec3(xmax, ymax, zmin)
c5 = Vec3(xmin, ymin, zmax)
c6 = Vec3(xmax, ymin, zmax)
c7 = Vec3(xmin, ymax, zmax)
c8 = Vec3(xmax, ymax, zmax)

resolution = 0.25 #[m]

#===============================================================================
# Computation
#===============================================================================

if runVisiblityAnalysis:
    print('Computing coverage')
    out = check_coverage(cameras, Vec3(xmin,ymin,zmin), Vec3(xmax,ymax,zmax), resolution)

    print('Plotting')

    fig = plt.figure()
    ax3d = fig.add_subplot(111, projection='3d')

    fig = plt.figure()
    ax_x = fig.add_subplot(221)
    ax_y = fig.add_subplot(222)
    ax_z = fig.add_subplot(223)

    ax_x.set_xlabel('y [m]')
    ax_x.set_ylabel('z [m]')
    ax_y.set_xlabel('x [m]')
    ax_y.set_ylabel('z [m]')
    ax_z.set_xlabel('x [m]')
    ax_z.set_ylabel('y [m]')

    ax_slider_minCams = fig.add_subplot(10,2,12)
    ax_slider_maxCams = fig.add_subplot(10,2,14)
    ax_slider_1 = fig.add_subplot(10,2,16)
    ax_slider_2 = fig.add_subplot(10,2,18)
    ax_slider_3 = fig.add_subplot(10,2,20)
    
    #plot room edges:
    ax_x.plot([ymin, ymax], [zmin, zmin], 'k:')
    ax_x.plot([ymin, ymax], [zmax, zmax], 'k:')
    ax_x.plot([ymin, ymin], [zmin, zmax], 'k:')
    ax_x.plot([ymax, ymax], [zmin, zmax], 'k:')

    ax_y.plot([xmin, xmax], [zmin, zmin], 'k:')
    ax_y.plot([xmin, xmax], [zmax, zmax], 'k:')
    ax_y.plot([xmin, xmin], [zmin, zmax], 'k:')
    ax_y.plot([xmax, xmax], [zmin, zmax], 'k:')

    ax_z.plot([xmin, xmax], [ymin, ymin], 'k:')
    ax_z.plot([xmin, xmax], [ymax, ymax], 'k:')
    ax_z.plot([xmin, xmin], [ymin, ymax], 'k:')
    ax_z.plot([xmax, xmax], [ymin, ymax], 'k:')

    from matplotlib.widgets import Slider
    # sliderHeight   = Slider(ax_z, 'z-val', 0, zmax, valinit=0)
    sliderMinNumCams  = Slider(ax_slider_minCams, 'minCam', 0, len(cameras)+1, valinit=3)
    sliderMaxNumCams  = Slider(ax_slider_maxCams, 'maxCam', 0, len(cameras)+1, valinit=len(cameras)+1)
    slider_x   = Slider(ax_slider_1, 'x-val', xmin, xmax, valinit=0)
    slider_y   = Slider(ax_slider_2, 'y-val', ymin, ymax, valinit=0)
    slider_z   = Slider(ax_slider_3, 'z-val', zmin, zmax, valinit=0)

    c_x = []
    c_y = []
    c_z = []
    #indices
    i_x = [1,2]
    i_y = [0,2]
    i_z = [0,1]

    c3d = [] 

    for i, o in enumerate(out):
        if not len(o):
            continue

        c_x.append(ax_x.plot(o[:,1], o[:,2], '.', label=str(i))[0])
        c_y.append(ax_y.plot(o[:,0], o[:,2], '.', label=str(i))[0])
        c_z.append(ax_z.plot(o[:,0], o[:,1], '.', label=str(i))[0])
        
        c3d.append(ax3d.plot(o[:,0], o[:,1], o[:,2], '.', label=str(i))[0])


    def update_plots(val):
        for n,(c,ind) in enumerate(zip([c_x, c_y, c_z], [i_x, i_y, i_z])):
            for i, o in enumerate(out):
                if not len(o):
                    continue

                if n == 0:
                    #x-vals
                    maskIndex = np.where(np.abs(o[:,0]-slider_x.val)<resolution)
                elif n == 1:
                    #y-vals
                    maskIndex = np.where(np.abs(o[:,1]-slider_y.val)<resolution)
                else:
                    #z-vals
                    maskIndex = np.where(np.abs(o[:,2]-slider_z.val)<resolution)
                
                if i < np.floor(sliderMinNumCams.val) or i > np.floor(sliderMaxNumCams.val):
                    c[i].set_xdata(o[maskIndex,ind[0]]*np.NaN)
                    c[i].set_ydata(o[maskIndex,ind[1]]*np.NaN)
                else:
                    c[i].set_xdata(o[maskIndex,ind[0]])
                    c[i].set_ydata(o[maskIndex,ind[1]])
                

        for i, o in enumerate(out):
            if not len(o):
                continue

            if i < np.floor(sliderMinNumCams.val) or i > np.floor(sliderMaxNumCams.val):
                c3d[i].set_data(o[:,0]*np.NaN, o[:,1]*np.NaN)
                c3d[i].set_3d_properties(o[:,2]*np.NaN)
                continue
                  
            if len(o) == 0:
                c3d[i].set_data(o[:,0]*np.NaN, o[:,1]*np.NaN)
                c3d[i].set_3d_properties(o[:,2]*np.NaN)
                continue
                
            c3d[i].set_data(o[:,0], o[:,1])
            c3d[i].set_3d_properties(o[:,2])
                
                  
           
        ax3d.set_title(str(np.floor(sliderMinNumCams.val))+'<=N<='+str(np.floor(sliderMaxNumCams.val)))
        return


    sliderMinNumCams.on_changed(update_plots)
    sliderMaxNumCams.on_changed(update_plots)
    slider_x.on_changed(update_plots)
    slider_y.on_changed(update_plots)
    slider_z.on_changed(update_plots)

    ax_x.legend()
    ax3d.legend()

    update_plots(0)

    plot_room(ax3d,cameras,c1,c2,c3,c4,c5,c6,c7,c8)

if runCameraViewpoint:
    while True:
        print('Camera locations:')
        for i,cam in enumerate(cameras):
            print('\t',i+1,cam.pos)

        camListIn = raw_input('Which cameras to plot (comma seperated, blank to quit)? > ')
        if not camListIn:
            break
        camList = [int(v)-1 for v in camListIn.split(',')]
        
        fig = plt.figure()
        axViewPoints = fig.add_subplot(111, projection='3d')
        plot_room(axViewPoints,[],c1,c2,c3,c4,c5,c6,c7,c8)
        axViewPoints .set_title('Camera view centers')

        for i,camNo in enumerate(camList):
            cam = cameras[camNo]
            points = cam.get_view_points((xmin, xmax), (ymin, ymax), (zmin, zmax))
            print('Camera at',cam.pos,'sees',points[0])

            for p in points[:5]:
                axViewPoints.plot((cam.pos[0], p[0]), (cam.pos[1], p[1]), (cam.pos[2], p[2]), 's--', c=cm.hsv(i/len(camList)))

            for p in points[5:]:
                axViewPoints.plot((p[0], p[0]), (p[1], p[1]), (p[2], p[2]), 'o', c=cm.hsv(i/len(camList)))
                
        plt.show()

        
        
plt.show()
