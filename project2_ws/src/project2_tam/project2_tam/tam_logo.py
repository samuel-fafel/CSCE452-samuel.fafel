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
	Point(72 ,429),
	Point(72 ,396),
	Point(92 ,396),
	Point(138,287),
	Point(127,287),
	Point(127,252),
	Point(203,252),
	Point(203,287),
	Point(194,287),
	Point(240,396),
	Point(260,396),
	Point(260,429),
	Point(192,429),
	Point(192,396),
	Point(201,396),
	Point(195,378),
	Point(138,378),
	Point(130,396),
	Point(139,396),
	Point(139,429),
	Point(72 ,429),
	Point(181,342),
	Point(151,342),
	Point(166,312),
	Point(181,342),
	
	# T
	Point(203,227),
	Point(127,227),
	Point(127,117),
	Point(518,117),
	Point(518,227),
	Point(440,227),
	Point(440,185),
	Point(361,185),
	Point(361,453),
	Point(403,453),
	Point(403,530),
	Point(240,530),
	Point(240,452),
	Point(283,452),
	Point(283,183),
	Point(203,183),
	Point(203,227),
	
	# M
	Point(385,429),
	Point(385,396),
	Point(403,396),
	Point(403,287),
	Point(385,287),
	Point(385,252),
	Point(440,252),
	Point(478,330),
	Point(517,252),
	Point(574,252),
	Point(574,287),
	Point(560,287),
	Point(560,396),
	Point(574,396),
	Point(574,429),
	Point(510,429),
	Point(510,396),
	Point(524,396),
	Point(524,319),
	Point(478,412),
	Point(435,319),
	Point(435,396),
	Point(450,396),
	Point(450,429),
	Point(385,429),
]

for point in my_points:
	point.x = point.x * 2 / 100.0 - 1
	point.y = point.y * -2 / 100.0 + 12

if __name__ == '__main__':
	# Example usage:
	start_point = Point(1, 2)
	end_point = Point(3, 4)
	segment = LineSegment(start_point, end_point)
	print(segment)	
	
	for p in range(len(my_points)-1):
		segment = LineSegment(my_points[p], my_points[p+1])
		print(segment)
		
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
