import struct
import os

def parse_bin(data):
	'''
		Args:
			- data: path to the Kitti binary file
		Returns: 
			- a list in which each element is a list containing
			the coordinates of one point
	'''
	points = list()
	with open(data, "rb") as f:
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
	
	# dataset = "./sample/000000.bin"
	# dst = "000000.txt"
	
	for j in range(10):
	    i = j + 1
	    seq_id = "%02d" % i   
	    dataset = "/localdisk/pointCloud_bin/Dataset/sequences/" + seq_id + "/velodyne/"
	    all_bins =[b for b in os.listdir(dataset) if ".bin" in b]
	    print(len(all_bins))
	   
	    dst_dir = "/localdisk/pointCloud_txt/Dataset/sequences/" + seq_id + "/velodyne/"
	    if not os.path.exists(dst_dir):
		os.makedirs(dst_dir)            

	    for bin_ in all_bins[:]:
	        src_ = dataset + bin_
		points = parse_bin(src_)
		dst_ = dst_dir + bin_.split(".")[0] + ".txt"
 		# print(dst_)
		# print(points)
		
		with open(dst_, 'w') as f:
		    for p_ in points:
			f.write("%s," % p_[0])
			f.write("%s," % p_[1])
			f.write("%s\n" % p_[2])	
