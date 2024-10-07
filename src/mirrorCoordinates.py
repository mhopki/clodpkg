'''
Mirror stand on left 
cable on right
camera above

x- down 
x+ up
y- left
y+ right

Camera pos Z direction
Object 0,0 pos X direction 
wire pos Y direction

camera unit vector = [0,0,1]
'''
import numpy as np
import matplotlib.pyplot as plt

def deg2xy(deg):
    '''
    Input 1,2 array of mirror degree change and it will give x and y for mirror controller
    '''
    xy = np.empty([2])
    xy[0] = np.tan(np.radians(deg[0]))/np.tan(np.radians(50))
    xy[1] = np.tan(np.radians(deg[1]))/np.tan(np.radians(50))
    return xy

def xy2deg(xy):
    '''
    input 1,2 array of xy in mirror controller for actual angle change
    '''
    deg = np.empty([2])
    deg[0] = np.degrees(np.arctan( xy[0] * np.tan(np.radians(50))))/2
    deg[1] = np.degrees(np.arctan( xy[1] * np.tan(np.radians(50))))/2

    deg = np.empty([3])

    deg = np.linalg.norm
    return deg

def unit_vec(vec): 
    v_hat = vec / np.linalg.norm(vec) if np.linalg.norm(vec) != 1 else vec
    return np.vstack(v_hat)

def angle_between(vec1,vec2):
    dot = np.dot(vec1,vec2)
    mag = np.linalg.norm(vec1)*np.linalg.norm(vec2)
    angle = np.degrees(np.arccos(dot/mag))
    return angle

def reflection_matrix(vec):
    '''
    input vector normal to reflection plane
    this will build the reflection matrix
    '''
    vec = unit_vec(vec)
    a = vec[0]
    b = vec[1]
    c = vec[2]

    A = np.array([[1-(2*a**2), -2*a*b, -2*a*c],
                 [-2*a*b, 1-(2*b**2), -2*b*c],
                 [-2*a*c, -2*b*c, 1-(2*c**2)]])
    A = np.squeeze(A)
    return A

def reflect(incident,ref_mat):
    reflected = ref_mat @ incident
    return reflected

def solve_for_mirror_norm(reflected):
    '''
    Give code the xyz position you want the beam to hit and it will return the mirror normal vector needed
    '''
    # only works if camera vec is [0,0,1]
    reflected = unit_vec(reflected)
    c = np.sqrt((reflected[2] + 1) / 2)
    b = reflected[1]/(2 * c)
    a = reflected[0]/(2 * c)
    norm_v = np.vstack([a,b,c])
    return norm_v

def mirror_norm2xy(normvec):
    '''
    Put in the mirror's normal vector and get the xy mirror controller inputs to get that normal vector
    '''
    theta = np.radians(-45)
    rotation_matrix = np.squeeze(np.array([[np.cos(theta), 0, np.sin(theta)],
                                           [0, 1, 0],
                                           [-np.sin(theta), 0, np.cos(theta)]]))
    xyplane = rotation_matrix @ normvec
    # ang0 = angle_between(np.squeeze([xyplane[0],xyplane[2]]),np.array([0,1]))
    # ang1 = angle_between(np.squeeze([xyplane[1],xyplane[2]]),np.array([0,1]))
    # xy_nosign = deg2xy([ang1,ang0])
    # xy1 = xy_nosign * np.sign(np.array([xyplane[1],xyplane[0]]).T)

    #original 
    # xy1 = np.vstack([xyplane[1],-xyplane[0]])/np.cos(np.radians(65))
    xy1 = np.vstack([xyplane[1],-xyplane[0]])*xyplane[2]/np.tan(np.radians(50))

    return xy1

def spatial_position2xy(position):
    '''
    inputs an [distance,width,height] position and returns the xy for the mirror
    '''
    mirror_norm = solve_for_mirror_norm(position)
    xy = mirror_norm2xy(mirror_norm)
    return np.squeeze(xy)

def pixel_position2xy(position):
    '''
    inputs [x,y] position for the center pixel in a scene
    '''
    # dist = 70e-3/7.4e-6
    # dist = 70e-3/5e-6
    dist = 9460
    position = np.array([dist,position[0],position[1]])
    mirror_norm = solve_for_mirror_norm(position)
    xy = mirror_norm2xy(mirror_norm)
    return np.squeeze(xy)

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def getXYFromNormalVector(n, d, D):
    r_C = np.array([0, 0, d]) # center of rotation
    # xy coordinate system defined with respect to 90° incidence angle
    n_0 = normalize(np.array([0,0,1]))
    r_OP_0 = np.array([0,0, -1])

    # target plane
    n_t = np.array([0,0,1])
    r_OT = np.array([0,0,-D]) 
    
    scaling = D*np.tan(np.deg2rad(50))
    
    r_OP_2 = getSpotOnTargetPlane(n, r_C, d, n_0, r_OP_0, r_OT, n_t)
    x = r_OP_2[0]/scaling
    y = r_OP_2[1]/scaling
    return x,y

def getNormalVectorFromXY(x,y, d):
    #distance d between center of rotation and mirror surface (mm)
    if d==0:
        return getNormalVectorFromXY_d0(x,y) # faster
    
    # Parameters as used in OQC calibration
    r_C = np.array([0, 0, d]) # center of rotation
    n_0 = np.array([0,0,1])
    r_OP_0 = np.array([0,0, -1])

    # target plane
    D = 90 # Distance center of rotation to target plane, mm
    n_t = np.array([0,0,1])
    r_OT = np.array([0,0,-D])
    
    n_m1, n_m2 = sy.symbols("n_m1 n_m2")
    n_m = sy.matrices.Matrix([n_m1, n_m2, -sy.sqrt(1-(n_m1**2+n_m2**2))])
    r_OP_2 = sy_getSpotOnTargetPlane(n_m, r_C, d, n_0, r_OP_0, r_OT, n_t)
    
    A_TI = np.diag([1,1,1])
    I_r_TP_2 = r_OP_2 - sy.matrices.Matrix(r_OT)
    T_r_TP_2 = np.dot(A_TI, I_r_TP_2)
    T_r_TP_2 = sy.simplify(T_r_TP_2)
    
    
    scaling = D*np.tan(50/180*np.pi)
    res = sy.solvers.solvers.nsolve((T_r_TP_2[0][0]-x*scaling,
                                     T_r_TP_2[1][0]-y*scaling),
                                    (n_m1, n_m2), (0, 0))
    n_x = float(res[0])
    n_y = float(res[1])
    
    n = np.array([n_x, n_y, -np.sqrt(1-(n_x**2+n_y**2))])
    
    return n

def getNormalVectorFromXY_d0(x,y): 
    #approximation for centered beam and d -> 0
    n0 = np.array([0,0, 1]) # xy coordinate system defined with respect to 90° incidence angle
    
    r = normalize(np.array([x, y, -1/np.tan(np.deg2rad(50))])) # direction of reflected beam
    return normalize(r - n0)

def getNormalVectorFromTargetXY_d0(x_t, y_t, A_IT, D, n_0):
    #approximation for centered beam and d -> 0
    n_t = np.dot(A_IT, np.array([0,0,1]))
    r_OT = np.dot(A_IT, np.array([0,0,-D]))
    T_r_TP_2 =  np.array([x_t[0], y_t[0], 0])
    I_r_TP_2 =  np.dot(A_IT, T_r_TP_2)
    r_OP_2 = r_OT + I_r_TP_2
    n_1 = normalize(r_OP_2)
    
    n_m = normalize(n_1-n_0)
    return n_m

def getSpotOnTargetPlane(n_m, r_C, d, n_0, r_OP_0, r_OT, n_t):
    #r_M = r_C + d * n_m # center of mirror surface
    
    # intersection of incoming beam with mirror (beam clipping not checked)
    t_1 = (np.dot(r_C-r_OP_0, n_m)+d) / np.dot(n_0, n_m)
    r_OP_1 = r_OP_0 + t_1 * n_0 
    
    n_1 = n_0 - 2*np.dot(n_0, n_m)*n_m # reflected beam
    
    # intersection of reflected beam with target plane (beam clipping not checked)
    # no check of whether the intersection happens in the correct half-space (t2 > 0)
    t_2 = np.dot(r_OT-r_OP_1, n_t) / np.dot(n_1, n_t)
    if t_2 < 0:
        print('Warning: intersection in wrong half-space')
    r_OP_2 = r_OP_1 + t_2 * n_1
    return r_OP_2

def sy_getSpotOnTargetPlane(n_m, r_C, d, n_0, r_OP_0, r_OT, n_t):
    # same as above but for symbolic calculations
    #r_M = r_C + d * n_m # center of mirror surface
    n_m = sy.matrices.Matrix(n_m)
    r_C = sy.matrices.Matrix(r_C)
    n_0 = sy.matrices.Matrix(n_0)
    r_OP_0 = sy.matrices.Matrix(r_OP_0)
    r_OT = sy.matrices.Matrix(r_OT)
    n_t = sy.matrices.Matrix(n_t)
    
    t_1 = (n_m.dot(r_C-r_OP_0)+d) / n_0.dot(n_m)
    r_OP_1 = r_OP_0 + t_1 * n_0 # center point of incoming beam on mirror
    
    n_1 = n_0 - 2*n_0.dot(n_m)*n_m # reflected beam
    
    t_2 = n_t.dot(r_OT-r_OP_1) / n_1.dot(n_t)
    r_OP_2 = r_OP_1 + t_2 *n_1
    return r_OP_2


def grid(x_min, x_max, y_min, y_max, NxNy):
    
    if isinstance(NxNy, tuple):
        Nx, Ny = NxNy
    else:
        Nx = NxNy
        Ny = NxNy
    
    x_grid = np.linspace(x_min, x_max, endpoint=True, num=Nx)
    y_grid = np.linspace(y_min, y_max, endpoint=True, num=Ny)
    xx, yy = np.meshgrid(x_grid, y_grid)
    x_grid = xx.ravel()
    y_grid = yy.ravel()
    
    return x_grid, y_grid

def hscGrid(x_min, x_max, y_min, y_max, NxNy):
    
    if isinstance(NxNy, tuple):
        Nx, Ny = NxNy
    else:
        Nx = int(NxNy/320)
        Ny = NxNy
    
    x_grid = np.linspace(x_min, x_max, endpoint=True, num=Nx)
    y_grid = np.linspace(y_min, y_max, endpoint=True, num=Ny)
    xx, yy = np.meshgrid(x_grid, y_grid)
    x_grid = xx.ravel()
    y_grid = yy.ravel()
    
    return x_grid, y_grid, xx, yy, Nx, Ny

def circleBoundedGrid(x_min, x_max, y_min, y_max, N):
    x_grid, y_grid = grid(x_min, x_max, y_min, y_max, N)
    x_bounded = []
    y_bounded = []
    
    # discard grid-points outside unit circle
    for x,y in zip(x_grid, y_grid):
        if x**2+y**2 <= 1:
            x_bounded.append(x)
            y_bounded.append(y)
    return np.array(x_bounded), np.array(y_bounded)    

def main():
    x = np.linspace(-0.7,0.7,20)
    y = np.linspace(-0.7,0.7,20)
    x,y = np.meshgrid(x,y)

    xt = np.linspace(-640*10, 640*10,20)
    yt = np.linspace(-640*10,640*10,20)
    xt,yt = np.meshgrid(xt,yt)

    results = np.zeros([20*20,2])

    count = 0
    for j in range(20):
        for i in range(20):
            print(i,j)
            print(xt[i,j],yt[i,j])
            results[count,:] = pixel_position2xy([xt[i,j],yt[i,j]])
            count += 1

    plt.scatter(results[:,0],results[:,1])
    plt.show()

    plt.scatter(xt,yt)
    plt.show()

if __name__ == "__main__":
    # print("module lol")
    main()