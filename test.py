import sys
import numpy as np
import json

# handle data/measurement errors
_ERROR_THRES = 0.001

# helper
def dist(p1, p2):
	return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

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

    def contains_point(self, pt):
		if self.isvert() and abs(pt[0]-self.p1[0]) < _ERROR_THRES:
			if pt[1] >= min(self.p1[1], self.p2[1]) and pt[1] <= max(self.p1[1], self.p2[1]):
				return True

		if self.ishorz() and abs(pt[1]-self.p1[1]) < _ERROR_THRES:
			if pt[0] >= min(self.p1[0], self.p2[0]) and pt[0] <= max(self.p1[0], self.p2[0]):
				return True

		# todo diagonal lines
		return False

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
		per_len = (val - self.starting_len) / (self.line.length())
		x = self.line.p1[0] * (1 - per_len) + self.line.p2[0] * (per_len)
		y = self.line.p1[1] * (1 - per_len) + self.line.p2[1] * (per_len)
		return (x,y)

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
	print nec_wall_breaks_1d

	# splice wall polygon into wall segments based on checks
	current_pos = 0
	max_pos = room.total_len

	for cut in nec_wall_breaks_1d:
		wall_start, wall_end = min(current_pos, cut[0]), cut[0]
		current_pos = cut[1]
		if abs(wall_start - wall_end) > _ERROR_THRES:
			nec_walls_1d.append((wall_start, wall_end))
			# otherwise skip, because it's a corner

	# add the end
	wall_start, wall_end = min(current_pos, max_pos), max_pos
	if abs(wall_start - wall_end) > _ERROR_THRES:
		nec_walls_1d.append((wall_start, wall_end))

	# valid 1d projected wall segments
	print nec_walls_1d

	# NEC compliant wall segments, each an ordered list of line(s)
	nec_walls = [room.unproject_dist_2d(w[0], w[1]) for w in nec_walls_1d]



	print nec_walls
	# print
	# print room.project_1d([9756.538397487642,
 #                -497.9888348060059])
	# print room.unproject_2d(501.37236532459883)

if __name__ == '__main__':
	main()