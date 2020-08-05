import sys
import numpy as np
import json

from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
import shapely.affinity as affinity

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

# helper
def dist(p1, p2):
	return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# helper to compare two almost similar values
def same(a, b):
	return abs(a - b) < _ERROR_THRES

# reader
def read_data_as_polygons(filepath, key):
	f = open(filepath)
	data = json.load(f)
	f.close()
	return [Polygon(p) for p in data[key]]

class Line:
	# represents a line defined by 2 points
    def __init__(self, p1, p2):
    	self.p1 = p1
    	self.p2 = p2
    	self._slope = None

    def __repr__(self):
    	return '[%.5f, %.5f, 0.0], [%.5f, %.5f, 0.0]' % (self.p1[0], self.p1[1], self.p2[0], self.p2[1])

    def slope(self):
    	if self._slope:
    		return self._slope # cache

    	if abs(self.p2[0] - self.p1[0]) < _ERROR_THRES:
    		self._slope = np.inf
        else:
        	m = (self.p2[1] - self.p1[1]) / (self.p2[0] - self.p1[0])
        	self._slope = m if m >= _ERROR_THRES else 0

        return self._slope

    def length(self):
    	return dist(self.p1, self.p2)

    def isvert(self):
    	return np.isinf(self.slope())

    def ishorz(self):
    	return abs(self.slope()) < _ERROR_THRES

    def center(self):
    	return ((self.p1[0] + self.p2[0])/2, (self.p1[1] + self.p2[1])/2)

    def contains_point(self, pt):
		if self.isvert() and abs(pt[0]-self.p1[0]) < _ERROR_THRES:
			if pt[1] >= min(self.p1[1], self.p2[1]) and pt[1] <= max(self.p1[1], self.p2[1]):
				return True

		if self.ishorz() and abs(pt[1]-self.p1[1]) < _ERROR_THRES:
			if pt[0] >= min(self.p1[0], self.p2[0]) and pt[0] <= max(self.p1[0], self.p2[0]):
				return True

		# todo diagonal lines
		return False

    def astuple(self):
		return (self.p1, self.p2)

    def unproject_2d(self, val):
		per_len = val / self.length()
		x = self.p1[0] * (1 - per_len) + self.p2[0] * (per_len)
		y = self.p1[1] * (1 - per_len) + self.p2[1] * (per_len)
		return (x,y)

    def overlaps(self, line):
    	if self.isvert() and line.isvert() and abs(self.p1[0]-line.p1[0]) < _ERROR_THRES:
    		miny = min(self.p1[1], self.p2[1])
    		maxy = max(self.p1[1],self.p2[1])
    		return (line.p1[1] >= miny and line.p1[1] <= maxy) or (line.p2[1] >= miny and line.p2[1] <= maxy)

    	if self.ishorz() and line.ishorz() and abs(self.p1[1]-line.p1[1]) < _ERROR_THRES:
    		minx = min(self.p1[0], self.p2[0])
    		maxx = max(self.p1[0],self.p2[0])
    		return (line.p1[0] >= minx and line.p1[0] <= maxx) or (line.p2[0] >= minx and line.p2[0] <= maxx)

    	# todo diagonal lines
    	return False

class Wall():
	"""helper to manage each ordered line segment representing a wall of the room"""
	def __init__(self, line, starting_len):
		self.line = line
		self.starting_len = starting_len
		self.total_len = starting_len + line.length()

	def contains_point(self, pt):
		return self.line.contains_point(pt)

	def project_1d(self, pt):
		if self.contains_point(pt):
			return self.starting_len + dist(self.line.p1, pt)
		return -1

	def unproject_2d(self, val):
		return self.line.unproject_2d(val - self.starting_len)

	def overlaps(self, line):
		return self.line.overlaps(line)

class RoomPolygon:
	"""represents the wall enclosing a room"""
	def __init__(self, lines):

		# create an ordered series of line segments
		self.walls = []
		total_len = 0
		for line in lines:
			next_segment = Wall(line, total_len)
			total_len = next_segment.total_len
			self.walls.append(next_segment)
		self.total_len = total_len

		# polygon ref
		self.polygon = Polygon([(p.p1[0],p.p1[1]) for p in lines])

	def unproject_2d(self, val):
		for wall in self.walls:
			if val >= wall.starting_len and val <= wall.total_len:
				return wall.unproject_2d(val)
		return None

	def unproject_dist_2d(self, start_val, end_val):
		lines = []

		is_appending = False
		p1, p2 = None, None
		for wall in self.walls:
			if not is_appending:
				# trying to find the edge that contains starting value
				if start_val >= wall.starting_len and start_val <= wall.total_len:
					p1 = wall.unproject_2d(start_val)

					# is the end point also on the same edge?
					if end_val >= wall.starting_len and end_val <= wall.total_len:
						p2 = wall.unproject_2d(end_val)
						return [Line(p1, p2)] # case that edge contains both start and end
					else:
						lines.append(Line(p1, wall.line.p2)) # otherwise, we create segments
						is_appending = True
			else:
				# trying to find the edge that contains end value
				if end_val >= wall.starting_len and end_val <= wall.total_len:
					p2 = wall.unproject_2d(end_val)
					lines.append(Line(wall.line.p1, p2))
					return lines
				else:
					lines.append(Line(wall.line.p1, wall.line.p2)) # otherwise, keep creating segments

		return lines

	def project_1d(self, pt):
		for wall in self.walls:
			if wall.contains_point(pt):
				return wall.project_1d(pt)
		return -1

	def overlaps(self, line):
		for wall in self.walls:
			if wall.overlaps(line):
				return True

	def contains_point(self, pt):
		return self.polygon.contains(Point(pt[0], pt[1]))

class Outlet:
	"""represents an outlet flushed against the wall"""
	def __init__(self, wall_pt, vertices):
		self.wall_pt = wall_pt
		self.vertices = vertices
		self.polygon = Polygon(vertices)

def generate_outlet(p1, p2, offset):
	wall_pt = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
	vertices = [p1, p2, (p2[0] + offset[0], p2[1] + offset[1]), (p1[0] + offset[0], p1[1] + offset[1])]
	return Outlet(wall_pt, vertices) #[Line(corners[i], corners[(i+1)%len(corners)]) for i in range(len(corners))]

def generate_all_outlets(nec_walls, room_polygon):
	nec_lines = [line for wall in nec_walls for line in wall] # flatten

	outlets = []
	for line in nec_lines:

		# get outlet offset when flushed to wall
		ctr, offset = line.center(), (0,0)
		if line.ishorz():
			offset = (0, _OUTLET_DEPTH) if room_polygon.contains_point((ctr[0], ctr[1]+_OUTLET_DEPTH)) else (0, -_OUTLET_DEPTH)
		if line.isvert():
			offset = (_OUTLET_DEPTH, 0) if room_polygon.contains_point((ctr[0]+_OUTLET_DEPTH, ctr[1])) else (-_OUTLET_DEPTH, 0)

		# TODO diagonal walls

		# generate all possible outlets along the wall, each with X inch difference
		_OUTLET_POS_DIFF = 2
		start_pos = _OUTLET_WALL_BUFFER + int(_OUTLET_WIDTH/2)
		end_pos = int(line.length() + 1) - int(_OUTLET_WIDTH/2) - _OUTLET_WALL_BUFFER
		for i in range(start_pos, end_pos, _OUTLET_POS_DIFF):
			outlets.append(generate_outlet(line.unproject_2d(i-_OUTLET_WIDTH/2), line.unproject_2d(i+_OUTLET_WIDTH/2), offset))

	return outlets

def outlet_placement_options(nec_walls, room_polygon, blocking_polygons):
	# optimize to only check blockers near the wall
	_MAX_DIST_FROM_WALL = _OUTLET_WIDTH * 2
	wall_blocks = []
	for block in blocking_polygons:
	 	wall_dist = room_polygon.polygon.boundary.distance(block.centroid)
	 	if wall_dist <= _MAX_DIST_FROM_WALL:
	 		wall_blocks.append(block)

	# generate all possible outlets, then remove any that blocks key features like pucks
	outlets = generate_all_outlets(nec_walls, room_polygon)
	valid_outlets = []
	for outlet in outlets:
		blocked_puck = False
		for block in wall_blocks:
			if not outlet.polygon.intersection(block).is_empty:
				blocked_puck = True
				break
		if not blocked_puck:
			valid_outlets.append(outlet)

	return valid_outlets

def main():
	f = open('json/studio_info.json')
	data = json.load(f)
	f.close()

	vals = {dt:np.array(data[dt]) for dt in data.keys()}
	rooms = [RoomPolygon([Line(r[i], r[(i+1) % len(r)]) for i in range(len(r))]) for r in vals['generic_rooms']]

	room = rooms[0]
	nec_walls_1d, nec_wall_breaks_1d = [], []

	categories_to_check = ['doors', 'kitchens']
	for check in categories_to_check:
		shapes = [[Line(r[i], r[(i+1) % len(r)]) for i in range(len(r))] for r in vals[check]]

		for shape in shapes:
			for l in shape:
				if room.overlaps(l):
					pt1_1d = room.project_1d(l.p1)
					pt2_1d = room.project_1d(l.p2)
					nec_wall_breaks_1d.append((min(pt1_1d, pt2_1d), max(pt1_1d, pt2_1d)))
				#print l.p1, l.p2, room.overlaps(l), check

	# reorder
	nec_wall_breaks_1d.sort(key=lambda x: x[0])

	# splice wall polygon into wall segments based on checks
	current_pos = 0
	max_pos = room.total_len

	for cut in nec_wall_breaks_1d:
		wall_start, wall_end = min(current_pos, cut[0]), cut[0]
		current_pos = cut[1]
		if abs(wall_start - wall_end) >= _NEC_WALL_MIN_LEN:
			nec_walls_1d.append((wall_start, wall_end))
			# otherwise skip, because it's a corner

	# add the end
	wall_start, wall_end = min(current_pos, max_pos), max_pos
	if abs(wall_start - wall_end) > _NEC_WALL_MIN_LEN:
		nec_walls_1d.append((wall_start, wall_end))

	# NEC compliant wall segments, each an ordered list of line(s)
	nec_walls = [room.unproject_dist_2d(w[0], w[1]) for w in nec_walls_1d]

	# remember to merge last and first if connected
	if same(nec_walls_1d[-1][1], room.total_len):
		nec_walls[0] = nec_walls[-1] + nec_walls[0]
		nec_walls = nec_walls[:-1]

	blockers = read_data_as_polygons('json/floor_info.json', 'pucks') + read_data_as_polygons('json/studio_info.json', 'windows') + read_data_as_polygons('json/studio_info.json', 'doors')

	# adding some extra padding around blockers e.g., pucks and windows
	_BLOCKER_PADDING = 0.1
	blockers = [affinity.scale(b, 1+_BLOCKER_PADDING, 1+_BLOCKER_PADDING) for b in blockers]

	valid_placements = outlet_placement_options(nec_walls, room, blockers)
	projected_options = sorted([(vp, room.project_1d(vp.wall_pt)) for vp in valid_placements], key=lambda x: x[1])

	# for vp in projected_options:
	# 	print vp

	outlets_recommendations = []
	for nw in nec_walls:
		start_val, end_val = room.project_1d(nw[0].p1), room.project_1d(nw[-1].p2)
		walk_pos = start_val

		# first time from wall is just the outlet distance
		max_walk_dist = _NEC_OUTLET_DISTANCE

		if start_val < end_val:

			while walk_pos < (end_val - _NEC_OUTLET_DISTANCE):
				possible = [op for op in projected_options if op[1] > walk_pos and op[1] < min(end_val, (walk_pos + max_walk_dist))]

				if len(possible) > 0:
					ideal_outlet = sorted(possible, key=lambda x: x[1])[-1]
					print ideal_outlet
					outlets_recommendations.append(ideal_outlet[0])

					# subsequent distance between two outlets is double NEC specs
					max_walk_dist = _NEC_OUTLET_DISTANCE * 2.0

					walk_pos = ideal_outlet[1] # update walk position
				else:
					# circumstance where pucks/windows/etc block the first set of possible outlets
					# which likely violates NEC but we try to handle it anyway in most cases
					# should throw a visual warning for the user
					walk_pos += _NEC_OUTLET_DISTANCE
		else:
			# special case of loop around
			pass

		# print outlets_recommendations

	# for vp in valid_placements:
	# 	print '[%s],' % ','.join(['[%s,%s,0.0]' % (pt[0],pt[1]) for pt in vp])

if __name__ == '__main__':
	main()