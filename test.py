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
_STUDIO_INFO_FILE = 'json/studio_info.json'
_FLOOR_INFO_FILE = 'json/floor_info.json'

# ==============================================================
# ==============================================================
# Helpers
# ==============================================================

def dist(p1, p2):
	return np.linalg.norm(np.array(p1[:2])-np.array(p2[:2]))

# helper to compare two almost similar values
def same(a, b):
	return abs(a - b) < _ERROR_THRES

# I/O helpers
def read_data(filepath, key):
	with open(filepath, 'r') as f:
		data = json.load(f)
	return data[key]

def read_data_as_polygons(filepath, key):
	return [Polygon(p) for p in read_data(filepath, key)]

def save_data(base_fp, output_fp, new_data):
	data = {}
	with open(base_fp, 'r') as f:
		data = json.load(f)

	data.update(new_data)

	with open(output_fp, 'w+') as f:
		json.dump(data, f, indent=4, sort_keys=True)

# ==============================================================
# ==============================================================
# Classes
# ==============================================================

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

    	if same(self.p2[0], self.p1[0]):
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
		if self.isvert() and same(pt[0], self.p1[0]):
			if pt[1] >= min(self.p1[1], self.p2[1]) and pt[1] <= max(self.p1[1], self.p2[1]):
				return True

		if self.ishorz() and same(pt[1], self.p1[1]):
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
    	if self.isvert() and line.isvert() and same(self.p1[0], line.p1[0]):
    		miny = min(self.p1[1], self.p2[1])
    		maxy = max(self.p1[1],self.p2[1])
    		return (line.p1[1] >= miny and line.p1[1] <= maxy) or (line.p2[1] >= miny and line.p2[1] <= maxy)

    	if self.ishorz() and line.ishorz() and same(self.p1[1], line.p1[1]):
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

def get_wall_cuts_1d(room, categories_to_check):
	wall_cuts_1d = []
	for check in categories_to_check:
		shapes = [[Line(r[i], r[(i+1) % len(r)]) for i in range(len(r))] for r in read_data(_STUDIO_INFO_FILE, check)]
		for shape in shapes:
			for l in shape:
				if room.overlaps(l):
					pt1_1d = room.project_1d(l.p1)
					pt2_1d = room.project_1d(l.p2)
					wall_cuts_1d.append((min(pt1_1d, pt2_1d), max(pt1_1d, pt2_1d)))
	# reorder
	return sorted(wall_cuts_1d, key=lambda x: x[0])

def get_walls_1d(room, cuts_1d):
	# splice wall polygon into wall segments based on checks
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
	# NEC compliant wall segments, each an ordered list of line(s)
	nec_walls = [room.unproject_dist_2d(w[0], w[1]) for w in walls_1d]

	# remember to merge last and first if connected
	if same(walls_1d[-1][1], room.total_len):
		nec_walls[0] = nec_walls[-1] + nec_walls[0]
		nec_walls = nec_walls[:-1]

	return nec_walls

# ==============================================================
# ==============================================================
# Logic to generate outlet placement options
# ==============================================================

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

def get_valid_outlet_placements(room, nec_walls):
	# load up blockers, e.g. pucks and windows, with some extra padding
	blockers = read_data_as_polygons(_FLOOR_INFO_FILE, 'pucks') + read_data_as_polygons(_STUDIO_INFO_FILE, 'windows') + read_data_as_polygons(_STUDIO_INFO_FILE, 'doors')
	_BLOCKER_PADDING = 0.1
	blockers = [affinity.scale(b, 1+_BLOCKER_PADDING, 1+_BLOCKER_PADDING) for b in blockers]

	return outlet_placement_options(nec_walls, room, blockers)

# ==============================================================
# ==============================================================
# Logic to select outlet recommendations
# ==============================================================

def basic_greedy_walk(start_val, end_val, valid_options):
	recommendations = []

	# first time from wall is just the outlet distance
	max_walk_dist = _NEC_OUTLET_DISTANCE

	walk_pos = start_val
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
			# should throw a visual warning for the user
			walk_pos += 1

	return recommendations

def try_smoother_distribution(reccs, start_val, valid_options):
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

def main():
	outlets_recommendations = []
	rooms = [RoomPolygon([Line(r[i], r[(i+1) % len(r)]) for i in range(len(r))]) for r in read_data(_STUDIO_INFO_FILE, 'generic_rooms')]

	for room in rooms:
		# project room into 1D, then perform cuts based on nonwall categories to splice room into 1D NEC wall segments
		wall_cuts_1d = get_wall_cuts_1d(room, _NONWALL_CATEGORIES)
		nec_walls_1d = get_walls_1d(room, wall_cuts_1d)

		# then unproject back to 2D room space
		nec_walls = nec_walls_2d(room, nec_walls_1d)

		# get valid outlet 2D placements then get 1D projection mappings
		valid_placements = get_valid_outlet_placements(room, nec_walls)
		projected_options = sorted([(vp, room.project_1d(vp.wall_pt)) for vp in valid_placements], key=lambda x: x[1])

		# finalize recommendations for outlets
		for nw in nec_walls:
			start_val, end_val = room.project_1d(nw[0].p1), room.project_1d(nw[-1].p2)
			if start_val < end_val:
				valid_options = [op for op in projected_options if op[1] >= start_val and op[1] <= end_val]
			else:
				# special case of loop around so we offset the outlet options and end value by the projected room length
				valid_options = [(op[0], op[1] + room.total_len) for op in projected_options if op[1] <= end_val] + [op for op in projected_options if op[1] >= start_val]
				end_val = end_val + room.total_len

			recommendations = basic_greedy_walk(start_val, end_val, valid_options)
			recommendations = try_smoother_distribution(recommendations, start_val, valid_options)
			outlets_recommendations += [r[0] for r in recommendations]

	# save out data to visualize
	save_data('json/studio_info.json', 'json/output_info.json', {'outlets':[vp.data_3d() for vp in outlets_recommendations]})

if __name__ == '__main__':
	main()