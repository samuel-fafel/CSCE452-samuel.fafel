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
	Point(72,429),
	Point(72,396),
	Point(91,396),
	Point(139,287),
	Point(127,287),
	Point(127,252),
	Point(203,252),
	Point(203,285),
	Point(194,285),
	Point(242,396),
	Point(260,396),
	Point(260,430),
	Point(192,430),
	Point(192,396),
	Point(201,396),
	Point(195,378),
	Point(138,378),
	Point(130,396),
	Point(139,396),
	Point(139,429),
	Point(181,342),
	Point(151,342),
	Point(166,312)
]

for point in my_points:
	point.x = point.x * 2 / 100.0
	point.y = point.y * 2 / 100.0

if __name__ == '__main__':
	# Example usage:
	start_point = Point(1, 2)
	end_point = Point(3, 4)
	segment = LineSegment(start_point, end_point)
	print(segment)	
	
	for p in range(len(my_points)-1):
		segment = LineSegment(my_points[p], my_points[p+1])
		print(segment)
		
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
