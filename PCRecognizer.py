import time
import math
import numpy as np
from tkinter import *
from tkinter import ttk


# match two point_cloud by calculating distance between their points
# between our points and the template
def greedy_cloud_match(points, pc):
	e = 0.50
	step = math.floor(math.pow(len(points), 1.0 - e))
	minimum = math.inf
	for c in range(0, len(points)):
		d1 = cloud_distance(points, pc.points, c)
		d2 = cloud_distance(pc.points, points, c)
		minimum = min(minimum, min(d1, d2))
		c += step

	return minimum

# geometric distance between two point_clouds
def cloud_distance(pc1, pc2, start):
	aux = [False] * len(pc1)
	print("->",len(aux)," y ",len(pc2))

	suma = 0
	w = start
	while True: # kindof do while?
		index = -1
		minimum = math.inf
		for j in range(0, len(aux)):
			if False == aux[j]:
				dist = distance(pc1[w], pc2[j])
				if dist < minimum:
					minimum = dist
					index = j

		aux[index] = True
		weigth = 1 - ((w - start + len(pc1) % len(pc1)) / len(pc1))
		suma += weigth * minimum
		w = (w + 1) % len(pc1)

		if w == start:
			break

	return suma

# resamples a pc in order to set homogenous lengths for compare them properly.
# resample_length indicates the length which to resample the pc.
def resample(points, resample_len):
	interval = path_length(points) / (resample_len - 1)
	d = 0.0
	new_points = []
	new_points.append(points[0])
	c = 1
	for p in points:
		try:
			if points[c].id == points[c - 1].id:  # we are int he same stroke
				dist = distance(points[c - 1], points[c])
				if d + dist >= interval:
					print("!!")
					px = points[c - 1].x + ((interval - d) / dist) * (points[c].x - points[c - 1].x)
					py = points[c - 1].y + ((interval - d) / dist) * (points[c].y - points[c - 1].y)
					p = Point(px, py, points[c].id)
					new_points.append(p)
					points.insert(c, p)  # insert p in c position, reasigning all elements
					d = 0.0

				else:
					d += dist

			c += 1
		except:
			break

	print("C: ",c)
	if len(new_points) == resample_len - 1:
		new_points.append(Point(points[len(points) - 1].x,
							    points[len(points) - 1].y,
							    points[len(points) - 1].id))

	print("np_l: ",len(new_points))
	return new_points

# provides same point_cloud in different scales in order to compare
def scale(points):
	min_x = math.inf
	min_y = math.inf
	max_x = math.inf * -1
	max_y = math.inf * -1
	for c in range(len(points)):
		min_x = min(min_x, points[c].x)
		min_y = min(min_y, points[c].y)
		max_x = max(max_x, points[c].x)
		max_y = max(max_y, points[c].y)

	scale = max(max_x - min_x, max_y - min_y)
	new_points = []
	for c in range(len(points)):
		px = (points[c].x - min_x) / scale
		py = (points[c].y - min_y) / scale
		new_points.append(Point(px, py, points[c].id))

	return new_points

# translates a point_cloud to the provided centroid. It maps all pc to origin, 
# in order to recognize pc that are similar but in different coordinates
def translate_to(points, where):
	centroid = get_centroid(points)
	new_points = []
	for c in range(0, len(points)):
		px = points[c].x + where.x - centroid.x
		py = points[c].y + where.y - centroid.y
		new_points.append(Point(px, py, points[c].id))

	return new_points

# calculates the centroid of given cloud of points
def get_centroid(points):
	x = 0.0
	y = 0.0
	for c in range(0, len(points)):
		x += points[c].x
		y += points[c].y

	x /= len(points)
	y /= len(points)

	return Point(x, y, 0)

# calculates the length of a single point in a point_cloud
def path_length(points):
	dist = 0.0
	for c in range(1, len(points)):
		if points[c].id == points[c - 1].id:
			dist += distance(points[c - 1], points[c])

	return dist

# calculates distance between two given points
def distance(p1, p2):
	dx = p2.x - p1.x
	dy = p2.y - p1.y
	return math.sqrt(dx*dx + dy*dy)


# class definitions
class Point:
	def __init__(self, x, y, id):
		self.x = x
		self.y = y
		self.id = id

class Point_cloud:
	def __init__(self, name, points):
		self.name = name
		self.points = []
		algo = resample(points, 32)
		self.points = algo
		self.points = scale(self.points)
		self.points = translate_to(self.points, origin)

class Result:
	def __init__(self, name, score, ms):
		self.name = name
		self.score = score
		self.ms = ms


num_templates = 1
num_points = 32  # points number to resample to
origin = Point(0, 0, 0)
class PCRecognizer:
	templates = []
	templates.append(Point_cloud("T", [Point(30, 7, 1), Point(103, 7, 1), 
									   Point(66, 7, 2), Point(66, 87, 2)]))

	def recognize(self, points):
		t_ini = time.clock()

		# normalizing point_cloud, points, figure
		points = resample(points, 32)
		points = scale(points)
		points = translate_to(points, origin)
		
		score = math.inf
		template_n = -1
		for c in range(0, len(self.templates)):
			# normalizing template
			dist = greedy_cloud_match(points, self.templates[c])
			if dist < score:
				score = dist
				template_n = c

		t_fin = time.clock()
		if template_n == -1:
			return Result("no match", 0.0, t_fin - t_ini)

		else:
			print("score: "+str(score))
			return Result(self.templates[template_n].name, max((score - 2.0) / -2.0, 0.0), t_fin - t_ini)


if __name__ == "__main__":
	canvas_width = 420
	canvas_height = 400
	points = []
	stroke_id = 0
	result = -1
	clicked = False
	pcr = PCRecognizer()

	def click(e):
		print("click")
		if e.num == 1:
			print("start point: (",e.x,",",e.y,")")
			x1, y1 = (e.x - 1), (e.y - 1)
			x2, y2 = (e.x + 1), (e.y + 1)

			clicked = True
			global stroke_id, num_points
			stroke_id += 1
			points.append(Point(e.x, e.y, stroke_id))
			canvas.create_oval(x1, y1, x2, y2, fill = "#dddddd")

	def move(e):
		print("move")
		x1, y1 = (e.x - 1), (e.y - 1)
		x2, y2 = (e.x + 1), (e.y + 1)
		global stroke_id, points
		points.append(Point(e.x, e.y, stroke_id))
		canvas.create_oval(x1, y1, x2, y2, fill = "#dddddd")

	def release(e):
		print("release")
		if e.num == 1:
			print("ending point: (",e.x,",",e.y,")")

		if e.num == 3:
			print("recognizing stroke")
			global stroke_id, pcr, points
			stroke_id = 0
			print("points_l: ",len(points))
			result = pcr.recognize(points)

			score = "Result: matched with "+result.name+" about "+str(round(result.score, 2))
			str_var.set(score)

			points = []  # clear points array

	def clear():
		canvas.delete("all")
		str_var.set("");

	window = Tk()
	window.title("python canvas")
	window.bind("q", lambda e: window.destroy())
	window.bind("c", lambda e: clear())
	str_var = StringVar() # variable for actualize label text

	tab_control = ttk.Notebook(window)
	tab1 = ttk.Frame(tab_control)
	tab_control.add(tab1, text="canvas")
	tab_control.pack(expand=1, fill="both")

	canvas = Canvas(tab1, width=canvas_width, height=canvas_height)
	canvas.pack(expand=YES, fill=BOTH)
	canvas.bind("<Button>", click)
	canvas.bind("<B1-Motion>", move)
	canvas.bind("<ButtonRelease>", release)
	message = Label(tab1, text="Left click to recognize. q for quit. c for clear.\nTemplates: T", font=("Helvetica", 14))
	message.pack(side=BOTTOM)
	str_var.set("")
	label_score = Label(tab1, textvariable=str_var, font=("Helvetica", 14))
	label_score.pack(side=TOP)

	tab2 = ttk.Frame(tab_control)
	tab_control.add(tab2, text="templates")
	tab_control.pack(expand=1, fill="both")

	img = PhotoImage(file="templates.png")
	label = Label(tab2, image=img)
	label.pack()

	mainloop()