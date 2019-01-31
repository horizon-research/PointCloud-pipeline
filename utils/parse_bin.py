import struct

def parse_bin(dataset):
	'''
		Args:
			- dataset: path to the Kitti binary file
		Returns: 
			- a list in which each element is a list containing
			the coordinates of one point
	'''
	points = list()
	with open(dataset, "rb") as f:
		byte_ = f.read(4 * 4)
		x0, y0, z0, i_ = struct.unpack('4f', byte_)
		
		# coordinate system conversion
		x_ = -y0
		y_ = - z0
		z_ = x0

		points.append([x_, y_, z_])
		while byte_ != b"":
			byte_ = f.read(4 * 4)
			if byte_ != b"":
				x0, y0, z0, i_ = struct.unpack('4f', byte_)

				# coordinate system conversion
				x_ = -y0
				y_ = - z0
				z_ = x0
				points.append([x_, y_, z_])
	return points

if __name__ == '__main__':
	
	dataset = "./sample/000000.bin"
	dst = "000000.txt"
	points = parse_bin(dataset)

	with open(dst, 'w') as f:
		for p_ in points:
			f.write("%s," % p_[0])
			f.write("%s," % p_[1])
			f.write("%s\n" % p_[2])
			