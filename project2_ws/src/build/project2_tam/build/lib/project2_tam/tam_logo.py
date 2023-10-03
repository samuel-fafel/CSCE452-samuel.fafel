class Point:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f"(x:{self.x}, y:{self.y}, theta:{self.theta})"


class LineSegment:
    def __init__(self, start=Point(), end=Point()):
        self.start = start
        self.end = end

    def __str__(self):
        return f"Start: {self.start}, End: {self.end}"

my_points = [
	# A
	Point(72 ,429), # 0
	Point(72 ,396), # 1
	Point(92 ,396), # 2
	Point(138,287), # 3
	Point(127,287), # 4
	Point(127,252), # 5
	Point(203,252), # 6
	Point(203,287), # 7
	Point(194,287), # 8
	Point(240,396), # 9
	Point(260,396), # 10
	Point(260,429), # 11
	Point(192,429), # 12
	Point(192,396), # 13
	Point(201,396), # 14
	Point(195,378), # 15
	Point(138,378), # 16
	Point(130,396), # 17
	Point(139,396), # 18
	Point(139,429), # 19
	Point(72 ,429), # 20

	Point(181,342), # 21   Triangle within A
	Point(151,342), # 22
	Point(166,312), # 23
	Point(181,342), # 24
	
	# T
	Point(203,227), # 25
	Point(127,227), # 26
	Point(127,117), # 27
	Point(518,117), # 28
	Point(518,227), # 29
	Point(440,227), # 30
	Point(440,185), # 31
	Point(361,185), # 32
	Point(361,453), # 33
	Point(403,453), # 34
	Point(403,530), # 35
	Point(240,530), # 36
	Point(240,452), # 37
	Point(283,452), # 38
	Point(283,183), # 39
	Point(203,183), # 40
	Point(203,227), # 41
	
	# M
	Point(385,429), # 42
	Point(385,396), # 43
	Point(403,396), # 44
	Point(403,287), # 45
	Point(385,287), # 46
	Point(385,252), # 47
	Point(440,252), # 48
	Point(478,330), # 49
	Point(517,252), # 50
	Point(574,252), # 51
	Point(574,287), # 52
	Point(560,287), # 53
	Point(560,396), # 54
	Point(574,396), # 55
	Point(574,429), # 56
	Point(510,429), # 57
	Point(510,396), # 58
	Point(524,396), # 59
	Point(524,319), # 60
	Point(478,412), # 61
	Point(435,319), # 62
	Point(435,396), # 63
	Point(450,396), # 64
	Point(450,429), # 65
	Point(385,429), # 66
]

for point in my_points:
	point.x = point.x * 2 / 100.0 - 1
	point.y = point.y * -2 / 100.0 + 12

my_lines = [
	# A
	LineSegment(my_points[0],  my_points[1]),  # 0
	LineSegment(my_points[1],  my_points[2]),  # 1
	LineSegment(my_points[2],  my_points[3]),  # 2
	LineSegment(my_points[3],  my_points[4]),  # 3
	LineSegment(my_points[4],  my_points[5]),  # 4 
	LineSegment(my_points[5],  my_points[6]),  # 5
	LineSegment(my_points[6],  my_points[7]),  # 6
	LineSegment(my_points[7],  my_points[8]),  # 7
	LineSegment(my_points[8],  my_points[9]),  # 8
	LineSegment(my_points[9],  my_points[10]), # 9
	LineSegment(my_points[10], my_points[11]), # 10
	LineSegment(my_points[11], my_points[12]), # 11
	LineSegment(my_points[12], my_points[13]), # 12
	LineSegment(my_points[13], my_points[14]), # 13
	LineSegment(my_points[14], my_points[15]), # 14
	LineSegment(my_points[15], my_points[16]), # 15
	LineSegment(my_points[16], my_points[17]), # 16
	LineSegment(my_points[17], my_points[18]), # 17
	LineSegment(my_points[18], my_points[19]), # 18
	LineSegment(my_points[19], my_points[20]), # 19

	LineSegment(my_points[21], my_points[22]), # 20    Triangle within A
	LineSegment(my_points[22], my_points[23]), # 21
	LineSegment(my_points[23], my_points[24]), # 22

	# T
	LineSegment(my_points[25], my_points[26]), # 23
	LineSegment(my_points[26], my_points[27]), # 24
	LineSegment(my_points[27], my_points[28]), # 25
	LineSegment(my_points[28], my_points[29]), # 26
	LineSegment(my_points[29], my_points[30]), # 27
	LineSegment(my_points[30], my_points[31]), # 28
	LineSegment(my_points[31], my_points[32]), # 29
	LineSegment(my_points[32], my_points[33]), # 30
	LineSegment(my_points[33], my_points[34]), # 31
	LineSegment(my_points[34], my_points[35]), # 32
	LineSegment(my_points[35], my_points[36]), # 33
	LineSegment(my_points[36], my_points[37]), # 34
	LineSegment(my_points[37], my_points[38]), # 35
	LineSegment(my_points[38], my_points[39]), # 36
	LineSegment(my_points[39], my_points[40]), # 37
	LineSegment(my_points[40], my_points[41]), # 38

	# M
	LineSegment(my_points[42], my_points[43]), # 39
	LineSegment(my_points[43], my_points[44]), # 40
	LineSegment(my_points[44], my_points[45]), # 41
	LineSegment(my_points[45], my_points[46]), # 42
	LineSegment(my_points[46], my_points[47]), # 43
	LineSegment(my_points[47], my_points[48]), # 44
	LineSegment(my_points[48], my_points[49]), # 45
	LineSegment(my_points[49], my_points[50]), # 46
	LineSegment(my_points[50], my_points[51]), # 47
	LineSegment(my_points[51], my_points[52]), # 48
	LineSegment(my_points[52], my_points[53]), # 49
	LineSegment(my_points[53], my_points[54]), # 50
	LineSegment(my_points[54], my_points[55]), # 51
	LineSegment(my_points[55], my_points[56]), # 52
	LineSegment(my_points[56], my_points[57]), # 53
	LineSegment(my_points[57], my_points[58]), # 54
	LineSegment(my_points[58], my_points[59]), # 55
	LineSegment(my_points[59], my_points[60]), # 56
	LineSegment(my_points[60], my_points[61]), # 57
	LineSegment(my_points[61], my_points[62]), # 58
	LineSegment(my_points[62], my_points[63]), # 59
	LineSegment(my_points[63], my_points[64]), # 60
	LineSegment(my_points[64], my_points[65]), # 61
	LineSegment(my_points[65], my_points[66]), # 62
]

def main():
	# Example usage:
	start_point = Point(1, 2)
	end_point = Point(3, 4)
	segment = LineSegment(start_point, end_point)
	print(segment)	

if __name__ == '__main__':
	main()
		
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
