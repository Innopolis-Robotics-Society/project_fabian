from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

EDGES = [
    (5,7),(7,9),(6,8),(8,10),(5,6),
    (11,13),(13,15),(12,14),(14,16),
    (5,11),(6,12),(11,12)
]

def make_markers(header, kpts, ids):
    ma = MarkerArray()
    mid = 0
    for i in range(len(ids)):
        # lines
        line = Marker()
        line.header = header
        line.ns = "skeleton"
        line.id = mid; mid += 1
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = 0.01
        line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 1.0
        for a,b in EDGES:
            pa = kpts[i,a]; pb = kpts[i,b]
            if pa[2] > 0 and pb[2] > 0:
                line.points.append(Point(x=float(pa[0]), y=float(pa[1]), z=0.0))
                line.points.append(Point(x=float(pb[0]), y=float(pb[1]), z=0.0))
        ma.markers.append(line)
    return ma
