from shapely.geometry import Polygon
from matplotlib import pyplot as plt

def test_iou(ipoints, jpoints):
    #ipoints = [(30.6079, 9.36615), (34.4813, 9.37658), (34.4863, 7.72858), (30.6123, 7.71815)]
    #jpoints = [(30.4381, 9.03328), (34.412, 9.64375), (34.6554, 8.05922), (30.6816, 7.44875)]
    irec = Polygon(ipoints)
    jrec = Polygon(jpoints)
    inter = irec.intersection(jrec)
    union = irec.union(jrec)
    print "inter area: ", inter.area
 
    inter_points = list(inter.exterior.coords)
    print "inter points: ", inter_points
    

    draw = True
    if draw:
        #draw rec
        ix = [p[0] for p in ipoints]
        ix.append(ipoints[0][0])
        iy = [p[1] for p in ipoints]
        iy.append(ipoints[0][1])

        jx = [p[0] for p in jpoints]
        jx.append(jpoints[0][0])
        jy = [p[1] for p in jpoints]
        jy.append(jpoints[0][1])
        plt.plot(ix, iy, 'b-')
        plt.plot(jx, jy, 'g-')
        #draw points
        inter_px = [p[0] for p in inter_points]
        inter_py = [p[1] for p in inter_points]
        plt.plot(inter_px, inter_py, 'yo')
        plt.show()


if __name__ == "__main__":
    ipoints = [(30.6079, 9.36615), (34.4813, 9.37658), (34.4863, 7.72858), (30.6123, 7.71815)]
    jpoints = [(30.4381, 9.03328), (34.412, 9.64375), (34.6554, 8.05922), (30.6816, 7.44875)]
    #test_iou(ipoints, jpoints)

    ipoints = [(1.5, -1), (-1.5, -1), (-1.5, 3), (1.5, 3)]
    jpoints = [(2.8243, 0), (0, -2.82843), (-2.82843, 0), (0, 2.82843)]
    test_iou(ipoints, jpoints)
