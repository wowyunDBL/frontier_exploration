
import numpy as np
import math
from scipy.spatial.transform import Rotation as Rot
from shapely.geometry import Polygon, LineString
import tf.transformations as tf

""" create different polygons """
name_lst = ['rectangle_triangle',
            'rectangle',
            'triangle',
            'circle',
            ]
simple_polygon = {
    'rectangle': {
        "BD": [(0,0),(7,0),(7,4.5),(0,4.5),(0,0)],
        "obs_lst": [
            ]
    },
    
    'rectangle_2': {
        "BD": [(-2.25,-2),(-2.25,2),(2.25,2),(2.25,-2),(-2.25,-2)],
        "obs_lst": [
            ]
    },
    'rectangle_rectangle_': {
        "BD": [(0,0),(10,0),(10,10),(0,10),(0,0)],
        "obs_lst": [
            [(3,6),(5,6),(5,3),(3,3),(3,6)],
            ]
    },
    'triangle': {
        "BD": [(0,0),(10,0),(5,10),(0,0)],
        "obs_lst": [
            ]
    },
    'rectangle_triangle': {
        "BD": [(0,0),(10,0),(10,10),(0,10),(0,0)],
        "obs_lst": [
            np.array([(7.5,5),(10.5,5),(9.0,8),(7.5,5)])-(3,3),
            ]
    },
    'circle': {
        "BD": np.array(list(zip(np.sqrt(10)*np.cos(np.linspace(0, 2*np.pi, 50)),
                             np.sqrt(10)*np.sin(np.linspace(0, 2*np.pi, 50)))))+(5,5),
        "obs_lst": [
            
            ]
    },
    'rectangle_circle': {
        "BD": [(0,0),(10,0),(10,10),(0,10),(0,0)],
        "obs_lst": [
            np.array(list(zip(np.sqrt(1)*np.cos(np.linspace(0, 2*np.pi, 100)),
                             np.sqrt(1)*np.sin(np.linspace(0, 2*np.pi, 100)))))+(5,5),
            ]
    },
    'convex': {
        "BD":  [(0,0),(5,0),(7,3),(10,10),(-1.5,10),(-1.5,2.5),(0,0)],
        "obs_lst": [
            ]
    },
    'concave': {
        "BD": [(0,3),(7,0),(10,4),(10,6),(7,10),(0,7),(4,7),(4,3),(0,3)],
        "obs_lst": [
            ]
    },
    'pentagon': {
        "BD": [(5,0),(10,0),(5,5),(10,10),(5,10),(0,5),(5,0)],
        "obs_lst": [
            ]
    },
    'hexagon': {
        "BD": [(0,0),(5,0),(10,0),(10,5),(5,10),(0,5),(0,0)],
        "obs_lst": [
            ]
    },
    'heptagon': {
        "BD": [(0,4),(4,5),(9,0),(5,6),(10,9),(5,10),(4,6),(0,4)],
        "obs_lst": [
            ]
    },
    'octagon': {
        "BD": [(0,3),(7,3),(7,0),(10,3),(10,3),(10,7),(7,10),(7,7),(0,7),(0,3)],
        "obs_lst": [
            ]
    },
    
    }
    
def get_simple_polygon(name='concave'):
    
    BD = simple_polygon.get(name, 'error')['BD']
    obs_lst = simple_polygon.get(name, 'error')['obs_lst']
    bx, by = zip(*BD)
    plt.plot(bx, by,'*-',lw=2, c='k')
    for obs in obs_lst:
        ox, oy = zip(*obs)
        plt.plot(ox, oy,'*-')
        # plt.fill(ox, oy,hatch='//',fill=False)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.axis('equal')  
    plt.tight_layout()
    
    return BD, obs_lst
    
def find_start_posi(ox, oy):
    # find sweep_direction
    max_dist = 0.0
    vec = [0.0, 0.0]
    sweep_start_pos = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i]
        dy = oy[i + 1] - oy[i]
        d = np.sqrt(dx ** 2 + dy ** 2)

        if d > max_dist:
            max_dist = d
            vec = [dx, dy]
            sweep_start_pos = [ox[i], oy[i]]
    return sweep_start_pos, vec


def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi):
    tx = [ix - sweep_start_posi[0] for ix in ox]
    ty = [iy - sweep_start_posi[1] for iy in oy]

    th = math.atan2(sweep_vec[1], sweep_vec[0])
    r = Rot.from_euler("z", th)
    r = np.asarray(r.as_quat())
    rot = tf.quaternion_matrix(r)[0:2, 0:2]
    converted_xy = np.dot(np.stack([tx, ty]).T,(rot))

    return converted_xy[:, 0], converted_xy[:, 1]


def convert_global_coordinate(x, y, sweep_vec, sweep_start_posi):
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    # rot = Rot.from_euler("z", -th).as_matrix()[0:2, 0:2]
    r = Rot.from_euler("z", -th)
    r = np.asarray(r.as_quat())
    rot = tf.quaternion_matrix(r)[0:2, 0:2]
    converted_xy = np.dot(np.stack([x, y]).T,(rot))
    rx = [ix + sweep_start_posi[0] for ix in converted_xy[:, 0]]
    ry = [iy + sweep_start_posi[1] for iy in converted_xy[:, 1]]

    return rx, ry


def planning(BD, obs_utm_list, reso, sweep_start_posi=[0.0], sweep_vec=[-1, 0]):
    bx, by = zip(*BD)
    rbx, rby = convert_grid_coordinate(bx, by, sweep_vec, sweep_start_posi)
    obs_map_list = []
    for obs in obs_utm_list:
        ox, oy = zip(*obs)
        rox, roy = convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi)
        obs_map_list.append(list(zip(rox, roy)))
    ROI = Polygon(list(zip(rbx, rby)))
    for obs in obs_map_list:
        ROI = ROI.difference(Polygon(obs))
       
    cuty = min(rby)+reso/2
    MAX_R = 1000
    system = {i: [] for i in range(MAX_R)}
    last_intersect_num = 0
    lastcenterlst = []
    num = 0
    
    """ use a line to gradually increase resolution and collide with boundary. if collide with bds, then recorded """
    while cuty < max(rby):
        cutting_line = LineString([(min(rbx), cuty), (max(rbx), cuty)])
        try:
            line = tuple(cutting_line.intersection(ROI).coords)
            if not last_intersect_num == 0:
                last_intersect_num = 0
                for num in range(MAX_R):
                    if system[num] == []:
                        break
            system[num].extend(line)
        except:
            flag = False
            multistring = cutting_line.intersection(ROI)
            if not last_intersect_num == len(multistring):
                last_intersect_num = len(multistring)
                flag = True
                lastcenterlst = [line.centroid.coords[:][0] for line in multistring]
            else:
                centerlst = [line.centroid.coords[:][0] for line in multistring]
                if lastcenterlst :
                    for i in range(len(lastcenterlst)): 
                        if abs(lastcenterlst[i][0] - centerlst[i][0]) > 1.5 :
                            flag = True
                            break
                lastcenterlst = centerlst.copy()
            if flag:
                for num in range(MAX_R):
                    if system[num] == []:
                        break
            for idx, line in enumerate(multistring):
                system[num + idx].extend(line.coords)
        cuty += reso
    print('MAX_R: '+str(MAX_R))
    for idx in range(MAX_R):
        if system[idx] == []:
            break
        """ only first polygon won't reverse"""
        # if idx > 0:
        system[idx] = system[idx][::-1]
        """generate zigzag"""
        print(system[idx])
        for i, item in enumerate(system[idx]):
            if (i + 1) % 4 == 0:
                system[idx][i], system[idx][i - 1] = system[idx][i - 1], system[idx][i]
    """accumulate waypoints"""
    global_waypoint_list = []
    # import matplotlib.cm as cm
    # lrx,lry = None,None
    # colors = cm.rainbow(np.linspace(0,1, idx))
    for i in range(idx):
        px, py = zip(*system[i])
        px, py = px[::-1], py[::-1]
        # plt.plot(px,py,'*-',c= colors[i])
        # plt.axis('equal')
        rx, ry = convert_global_coordinate(px, py, sweep_vec, sweep_start_posi)
        global_waypoint_list.append(list(zip(rx, ry)))
        # if not lrx==None and not lry==None: 
        #     plt.plot([lrx,px[0]],[lry,py[0]],'k*-',lw=5)
        # lrx,lry = px[-1], py[-1]
    return global_waypoint_list


if __name__ == '__main__':
    # for i in simple_polygon:
    #     if i== 'rectangle_2':
    #         BD, obs_utm_list = get_simple_polygon(name = i)
    # a = planning(BD, obs_utm_list, reso=0.5, sweep_start_posi=[2,2.0], sweep_vec=[0,1])
    # path = a[0]
    # for i, item in enumerate(path):
    #     if i % 2:
    #         path[i], path[i-1] = path[i-1], path[i] 
    # x,y = zip(*path)
    # import matplotlib.pyplot as plt
    # plt.plot(x,y)
    # path = [ tuple([round(x,2) if isinstance(x, float) else x for x in t]) for t in path]
    # print(path)

    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(5,5))
    plt.tight_layout()
    BD, obs_utm_list = get_simple_polygon(name = 'rectangle')
    a = planning(BD, obs_utm_list, reso=2, sweep_start_posi=[2,2.0], sweep_vec=[1,0])
    path = a[0]
    print(path)
    for i, item in enumerate(path):
        if i % 2:
            path[i], path[i-1] = path[i-1], path[i] 
    x,y = zip(*path)
    import matplotlib.pyplot as plt
    plt.plot(x, y, '--', c='gray',lw=3)
    import matplotlib.cm as cm
    colors = cm.rainbow(np.linspace(1, 0, len(x)))
    plt.scatter(x, y, c=colors,s=100)
    plt.axis('equal')
    plt.title('zigzag_driven path', fontsize=18)
    plt.savefig('/home/anny/path_zigzag.png')
    plt.show()
    plt.close()
    import json
    path_dict = {'path_x':x, 'path_y':y}
    json_object = json.dumps(path_dict, indent=4)      
    with open('/home/anny/path_rect.json', 'a+') as outfile:
        outfile.write(json_object)
    path = [ tuple([round(x,2) if isinstance(x, float) else x for x in t]) for t in path]
    print(path)