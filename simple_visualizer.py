from PIL import Image, ImageDraw
import sys
import numpy as np
import json

# consts to define size/format of output
_W, _H, _BORDER = 900, 1200, 0.1
_INNER = (1 - _BORDER * 2)

# table for color keyed data sources
_COLOR_TABLE = {
	'bathrooms': '#4CC3D9',
	'closets': '#7BC8A4',
	'doors': '#F16745',
	'generic_rooms': '#404040',
	'kitchens': '#FFC65D',
	'windows': '#93648D',
	'debug': '#00ff00',
	'debug2':'#00ff00',
	'outlets': '#00ff00'
}

def wcs2pix(x_coord, y_coord, min_coords, w, h, w2p_ratio, offset):
	x = (x_coord-min_coords[0]) * 1/w2p_ratio
	y = (y_coord-min_coords[1]) * 1/w2p_ratio
	return int(x + offset[0]), h - int(y + offset[1])

def load_file_with_pixel_coords(filepath):
	f = open(filepath)
	data = json.load(f)
	f.close()

	vals = {dt:np.array(data[dt]) for dt in data.keys() if not 'pucks' in dt}

	# get min/max for mapping
	min_coords = np.amin([np.amin(np.amin(vals[k], axis=0), axis=0) for k in vals], axis=0)
	max_coords = np.amax([np.amax(np.amax(vals[k], axis=0), axis=0) for k in vals], axis=0)

	# coords to pixels ratio
	w2p_ratio = max((max_coords[0]-min_coords[0]) / (_W * _INNER), (max_coords[1]-min_coords[1]) / (_H * _INNER))

	# offset to center the rendering
	offset = (_W - (max_coords[0]-min_coords[0]) * 1/w2p_ratio) / 2 , (_H - (max_coords[1]-min_coords[1]) * 1/w2p_ratio) / 2

	# convert to pix coords
	vals = {dt:np.array(data[dt]) for dt in data.keys()}
	for k in vals:
		d = {
			'raw': vals[k],
			'xy': [[wcs2pix(b[0], b[1], min_coords, _W, _H, w2p_ratio, offset) for b in a] for a in vals[k]]
		}
		vals[k] = d

	# return data with {'raw',[...], 'xy':[...]} format where xy is the converted coords into screen space
	return vals

def main():
	a = np.full((_H, _W, 3), 255.0)
	img = Image.fromarray(np.uint8(a))
	viz = ImageDraw.Draw(img)

	data = load_file_with_pixel_coords('json/output_info.json')

	for k in data:
		for shape in data[k]['xy']:
			viz.polygon(shape, fill ="#fff", outline = _COLOR_TABLE[k] if k in _COLOR_TABLE else 'black')
	img.show()

if __name__ == '__main__':
	main()
