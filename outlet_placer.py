import sys
import numpy as np
import json

from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
import shapely.affinity as affinity

# ==============================================================
# ==============================================================
# Globals
# ==============================================================

# handle data/measurement errors
_ERROR_THRES = 0.001

# NEC defines walls as 24 inches
# and max dist to an outlet along a wall is 72 inches
_NEC_WALL_MIN_LEN = 24
_NEC_OUTLET_DISTANCE = 72.0

# outlet dimensions in inches
_OUTLET_WIDTH = 4
_OUTLET_DEPTH = 2
_OUTLET_WALL_BUFFER = 1 # so outlets aren't flushed into a wall corner

# as per specifications
_NONWALL_CATEGORIES = ['doors', 'kitchens']
_GENERIC_ROOMS = 'generic_rooms'

# ==============================================================
# ==============================================================
# Helpers
# ==============================================================

def same(a, b):
    # Compares two almost similar values
    return abs(a - b) < _ERROR_THRES

# ==============================================================
# ==============================================================
# Classes
# ==============================================================

class Line(LineString):
    """ Wrapper for LineString to represents a line defined by 2 points
        Includes support for 1D projection approach
    """
    def __init__(self, pts):
        super(Line, self).__init__(pts)
        self.p1 = Point(pts[0])
        self.p2 = Point(pts[1])

    def contains_point(self, pt):
        return self.distance(Point(pt)) < _ERROR_THRES

    def unproject_2d(self, val):
        """ Unprojects a float into its 2D room space mapping on line, assuming relative value. """
        per_len = val / self.length
        x = self.p1.x * (1 - per_len) + self.p2.x * (per_len)
        y = self.p1.y * (1 - per_len) + self.p2.y * (per_len)
        return (x, y)

    def overlaps(self, line):
        """ Checks whether we contain another line. """
        return self.contains_point(line.p1) and self.contains_point(line.p2)

class Wall():
    """ Container to manage ordered line segment representing a wall of the room
        Includes support for 1D projection approach
        Attributes:
            line (Line)
            starting_len (float): 1D projection value for first vertex.
            total_len (float): 1D projection value for second vertex.
    """
    def __init__(self, line, starting_len):
        self.line = line
        self.starting_len = starting_len
        self.total_len = starting_len + line.length

    def contains_point(self, pt):
        return self.line.contains_point(pt)

    def project_1d(self, pt):
        """ Projects a 2D room position into its 1D value on the wall, relative to our starting value. """
        if self.contains_point(pt):
            return self.starting_len + self.line.p1.distance(Point(pt))
        return -1

    def unproject_2d(self, val):
        """ Unprojects a float into its 2D room space mapping, correcting for our starting value. """
        return self.line.unproject_2d(val - self.starting_len)

class RoomPolygon:
    """ Represents the wall enclosing a generic room
        Includes support for 1D projection approach
        Attributes:
            walls (list of Wall)
            total_len (float): 1D projection value, total length of walls.
            polygon (Polygon): Polygon reference for convenience.
    """
    def __init__(self, lines):
        # Init with an ordered series of line segments
        self.walls = []
        total_len = 0
        for line in lines:
            next_segment = Wall(line, total_len)
            total_len = next_segment.total_len
            self.walls.append(next_segment)

        self.total_len = total_len
        self.polygon = Polygon([p.p1 for p in lines])

    def unproject_2d(self, val):
        """ Unprojects a float into its 2D room space mapping. """
        for wall in self.walls:
            if val >= wall.starting_len and val <= wall.total_len:
                return wall.unproject_2d(val)
        return None

    def unproject_dist_2d(self, start_val, end_val):
        """ Unprojects a range represented by floats into a set of ordered lines.
            Handles case which range extends across corners, covering multiple walls.
        """
        lines, is_appending, p1, p2 = [], False, None, None
        for wall in self.walls:
            if not is_appending:
                # trying to find the edge that contains starting value
                if start_val >= wall.starting_len and start_val <= wall.total_len:
                    p1 = wall.unproject_2d(start_val)

                    # is the end point also on the same edge?
                    if end_val >= wall.starting_len and end_val <= wall.total_len:
                        p2 = wall.unproject_2d(end_val)
                        return [Line([p1, p2])] # case that edge contains both start and end
                    else:
                        lines.append(Line([p1, wall.line.p2])) # otherwise, we create segments
                        is_appending = True
            else:
                # trying to find the edge that contains end value
                if end_val >= wall.starting_len and end_val <= wall.total_len:
                    p2 = wall.unproject_2d(end_val)
                    lines.append(Line([wall.line.p1, p2]))
                    return lines
                else:
                    lines.append(Line([wall.line.p1, wall.line.p2])) # otherwise, keep creating segments
        return lines

    def project_1d(self, pt):
        """ Projects a 2D room position into its 1D value on the wall, if possible. """
        for wall in self.walls:
            if wall.contains_point(pt):
                return wall.project_1d(pt)
        return -1

    def overlaps(self, line):
        """ Checks if a line is on the wall, typically used for checking wall-flushed objects. """
        for wall in self.walls:
            if wall.line.overlaps(line):
                return True

    def contains_point(self, pt):
        """ Checks if a point is inside the room, primarily for directional offset to position new outlets. """
        return self.polygon.contains(pt)

class Outlet:
    """ Represents an outlet flushed against the wall.
        Attributes:
            wall_pt (Point): Center position on the wall.
            vertices (list of tuples): Vertices of the outlet.
            polygon (Polygon): As a polygon for convenience.
    """
    def __init__(self, wall_pt, vertices):
        self.wall_pt = wall_pt #
        self.vertices = vertices #
        self.polygon = Polygon(vertices)

    def data_3d(self):
        """ Helper for saving out formatted data. """
        return [[v[0],v[1],0.0] for v in self.vertices]

# ==============================================================
# ==============================================================
# Logic to get NEC spec walls from room polygon
# ==============================================================

def get_wall_cuts_1d(room, room_info, categories_to_check):
    """ Cut the room by doors and kitchen polygons.
        Return the 1D projections on the room wall.
    """
    wall_cuts_1d = []
    for check in categories_to_check:
        shapes = [[Line([r[i][:2], r[(i+1) % len(r)][:2]]) for i in range(len(r))] for r in room_info[check]]
        for shape in shapes:
            for l in shape:
                if room.overlaps(l):
                    pt1_1d, pt2_1d = room.project_1d(l.p1), room.project_1d(l.p2)
                    wall_cuts_1d.append((min(pt1_1d, pt2_1d), max(pt1_1d, pt2_1d)))
    # reorder
    return sorted(wall_cuts_1d, key=lambda x: x[0])

def get_walls_1d(room, cuts_1d):
    """ Splice wall polygon into wall segments based on cut polygons projected into 1D space.
        Return as 1D ranges that represent the wall segments.
    """

    nec_walls_1d, current_pos, max_pos = [], 0, room.total_len
    for cut in cuts_1d:
        wall_start, wall_end = min(current_pos, cut[0]), cut[0]
        current_pos = cut[1]
        if abs(wall_start - wall_end) >= _NEC_WALL_MIN_LEN:
            nec_walls_1d.append((wall_start, wall_end))

    # add the final wall
    wall_start, wall_end = min(current_pos, max_pos), max_pos
    if abs(wall_start - wall_end) > _NEC_WALL_MIN_LEN:
        nec_walls_1d.append((wall_start, wall_end))

    return nec_walls_1d

def nec_walls_2d(room, walls_1d):
    """ Unproject the 1D wall segment ranges back into 2D room space positions.
        Returns NEC compliant wall segments, each an ordered list of line(s)
    """
    nec_walls = [room.unproject_dist_2d(w[0], w[1]) for w in walls_1d]

    # remember to merge last and first if connected (loop)
    if same(walls_1d[-1][1], room.total_len):
        nec_walls[0] = nec_walls[-1] + nec_walls[0]
        nec_walls = nec_walls[:-1]
    return nec_walls

# ==============================================================
# ==============================================================
# Logic to generate outlet placement options
# ==============================================================

def generate_outlet(p1, p2, offset):
    """ Helper to construct outlets with flushed center pts on wall and vertices.
    """
    flushed_center = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    vertices = [p1, p2, (p2[0] + offset[0], p2[1] + offset[1]), (p1[0] + offset[0], p1[1] + offset[1])]
    return Outlet(flushed_center, vertices)

def generate_all_outlets(nec_walls, room_polygon):
    """ Generates all possible positions of outlets along NEC walls. Not necessarily valid.
    """
    nec_walls_flatten = [line for wall in nec_walls for line in wall]
    outlets = []
    for line in nec_walls_flatten:
        # get outlet offset when flushed to wall, determine direction facing inside the room
        ctr, offset = line.centroid, (0,0)

        # using parallel lines to position offsets
        left = line.parallel_offset(_OUTLET_DEPTH, 'left').centroid
        right = line.parallel_offset(_OUTLET_DEPTH, 'right').centroid
        if room_polygon.contains_point(left):
            offset = (left.x - ctr.x, left.y - ctr.y)
        if room_polygon.contains_point(right):
            offset = (right.x - ctr.x, right.y - ctr.y)

        # generate all possible outlets along the wall, each with X inch difference
        _OUTLET_POS_DIFF = 2
        start_pos = _OUTLET_WALL_BUFFER + int(_OUTLET_WIDTH/2)
        end_pos = int(line.length + 1) - int(_OUTLET_WIDTH/2) - _OUTLET_WALL_BUFFER
        for i in range(start_pos, end_pos, _OUTLET_POS_DIFF):
            outlets.append(generate_outlet(line.unproject_2d(i-_OUTLET_WIDTH/2), line.unproject_2d(i+_OUTLET_WIDTH/2), offset))
    return outlets

def outlet_placement_options(nec_walls, room_polygon, blocking_polygons):
    """ Select valid positions of outlets, comparing against blocking elements like pucks.
    """
    # Optimize to only check blockers near the wall
    _MAX_DIST_FROM_WALL = _OUTLET_WIDTH * 2
    wall_blocks = []
    for block in blocking_polygons:
        wall_dist = room_polygon.polygon.boundary.distance(block.centroid)
        if wall_dist <= _MAX_DIST_FROM_WALL:
            wall_blocks.append(block)

    # Generate possible outlets then remove any that blocks key features like pucks
    # (#blocks) x (#outlets) x (intersection calculation) process, may consider future optimizations
    outlets, valid_outlets = generate_all_outlets(nec_walls, room_polygon), []
    for outlet in outlets:
        blocked_puck = False
        for block in wall_blocks:
            if not outlet.polygon.intersection(block).is_empty:
                blocked_puck = True
                break
        if not blocked_puck:
            valid_outlets.append(outlet)
    return valid_outlets

def get_valid_outlet_placements(room, room_info, floor_info, nec_walls):
    """ Helper to load blockers and then find valid outlet placement options. """

    # Load up blockers, e.g. pucks and windows, with some extra padding
    blockers = [Polygon(p) for p in floor_info['pucks']] + [Polygon(p) for p in room_info['windows']] + [Polygon(p) for p in room_info['doors']]
    _BLOCKER_PADDING = 0.4
    blockers = [affinity.scale(b, 1+_BLOCKER_PADDING, 1+_BLOCKER_PADDING) for b in blockers]

    return outlet_placement_options(nec_walls, room, blockers)

# ==============================================================
# ==============================================================
# Logic to select outlet recommendations
# ==============================================================

def basic_greedy_walk(start_val, end_val, valid_options):
    """ Very simple greedy based 1D traversal approach based on walking NEC specs.
        First outlet is 6 ft away, then subsequent outlets are 12 ft apart.
    """
    recommendations, walk_pos = [], start_val
    max_walk_dist = _NEC_OUTLET_DISTANCE # first outlet from wall is the NEC max outlet distance
    while walk_pos < (end_val - _NEC_OUTLET_DISTANCE) or ((end_val - start_val) < _NEC_OUTLET_DISTANCE and walk_pos < (end_val - _NEC_WALL_MIN_LEN)):
        possible = [op for op in valid_options if op[1] > walk_pos and op[1] < min(end_val, (walk_pos + max_walk_dist))]

        if len(possible) > 0:
            ideal_outlet = sorted(possible, key=lambda x: x[1])[-1]
            recommendations.append(ideal_outlet)
            walk_pos = ideal_outlet[1] # update walk position

            # subsequent distance between two outlets is double NEC specs
            max_walk_dist = _NEC_OUTLET_DISTANCE * 2.0
        else:
            # circumstance where pucks/windows/etc block the first set of possible outlets
            # which likely violates NEC but we try to handle it anyway in most cases
            # should probably throw a visual warning for the user
            walk_pos += 1

    return recommendations

def try_smoother_distribution(reccs, start_val, valid_options):
    """ Polynominal smoothing for wall sections with multiple outlets
        to try to better distribute their relative positioning over 1D space.
    """
    if len(reccs) > 3:
        try:
            # polynomial fitting to smooth deltas between outlets
            delta = [reccs[0][1]-start_val] + [(reccs[i+1][1]-reccs[i][1]) for i in range(len(reccs)-1)]
            poly = np.polyfit(range(len(reccs)), delta, 4)
            smooth = np.poly1d(poly)(range(len(reccs)))

            # target values we're trying for, but may not get
            targets = np.cumsum(smooth) + start_val

            recommendations = []
            for t in targets:
                ideal = sorted([(op, abs(op[1]-t)) for op in valid_options], key=lambda x: x[1])[0][0]
                recommendations.append(ideal)
                valid_options.remove(ideal)
            return recommendations
        except:
            pass
    return reccs

# ==============================================================
# ==============================================================
# Main logic
# ==============================================================

def place_outlets(studio, floors):
    outlets = []

    # Load in rooms as custom objects to support 1D-2D projection approach
    rooms = [RoomPolygon([Line([r[i][:2], r[(i+1) % len(r)][:2]]) for i in range(len(r))]) for r in studio[_GENERIC_ROOMS]]
    for room in rooms:
        # Project room into 1D, then perform cuts based on nonwall categories to splice room into 1D NEC wall segments
        wall_cuts_1d = get_wall_cuts_1d(room, studio, _NONWALL_CATEGORIES)
        nec_walls_1d = get_walls_1d(room, wall_cuts_1d)

        # Then unproject back to 2D room space
        nec_walls = nec_walls_2d(room, nec_walls_1d)

        # Get valid outlet 2D placements then get 1D projection mappings
        valid_placements = get_valid_outlet_placements(room, studio, floors, nec_walls)
        projected_options = sorted([(vp, room.project_1d(vp.wall_pt)) for vp in valid_placements], key=lambda x: x[1])

        # Finalize recommendations for outlets
        for nw in nec_walls:
            start_val, end_val = room.project_1d(nw[0].p1), room.project_1d(nw[-1].p2)
            if start_val < end_val:
                valid_options = [op for op in projected_options if op[1] >= start_val and op[1] <= end_val]
            else:
                # special case of loop around so we offset the outlet options and end value by the projected room length
                valid_options = [(op[0], op[1] + room.total_len) for op in projected_options if op[1] <= end_val] + [op for op in projected_options if op[1] >= start_val]
                end_val = end_val + room.total_len

            # First run a greedy approach and then smooth the results
            recommendations = basic_greedy_walk(start_val, end_val, valid_options)
            recommendations = try_smoother_distribution(recommendations, start_val, valid_options)
            outlets += [[r[0].data_3d() for r in recommendations]]
    return outlets

def main():
    # room info
    studio = "json/studio_info.json"
    with open(studio) as json_file:
        studio = json.load(json_file)

    # floor info
    floors = "json/floor_info.json"
    with open(floors) as json_file:
        floors = json.load(json_file)

    outlets = place_outlets(studio, floors)
    with open('outlets.json', 'w+') as json_out:
        json.dump(outlets, json_out)

if __name__ == "__main__":
    main()