#Written by Gerard Kruisheer 2018
# from numba import jit
from   datetime import datetime
import numpy as np
import pickle
#To Do: Change to quaternion operations one day

def meters_to_geo(x, y):
	earths_radius = 6371000 #converted to meters

	geo_y = np.multiply(np.true_divide(y, earths_radius), np.true_divide(180, np.pi)) # geo_y =(y / earths_radius) * (180 / np.pi)
	geo_x = np.multiply(np.true_divide(x, earths_radius), np.true_divide(180, np.pi)) # geo_x =(x / earths_radius) * (180 / np.pi) / np.cos(latitude * np.pi/180)
	# print(f'This is the resulting value for x {geo_x} and y {geo_y}')
	return(geo_y, geo_x)

#There is two values because lat and lon scales are different
def geo_to_meters(x, y): #This is not precise
	R = 6373.0 #converted to meters
	x = np.divide((x * y), (R * R))
	return(x)

# def meters_to_geo(self, dlon, dlat):
# 	dlon = lon2 - lon1
# 	dlat = lat2 - lat1
# 	a = (sin(dlat/2))**2 + cos(lat1) * cos(lat2) * (sin(dlon/2))**2
# 	c = 2 * atan2(sqrt(a), sqrt(1-a))
# 	distance = R * c

# def distance(boat_lat, boat_lng, pt_lat, pt_lng,):
# 	delta_pts = np.sqrt((boat_lat-pt_lat)*(boat_lat-pt_lat) + (boat_lng-pt_lng)*(boat_lng-pt_lng))
# 	return(delta_pts)

# def haversine(lon1, lat1, lon2, lat2): #Adapted from Anishes fleet-light

def haversine(lat1, lon1, lat2, lon2): #Adapted from Anishes fleet-light # For 610 pts time = -0.00016379356384277344

	"""
	Calculate the great circle distance (Orthodromic in literature - OrthD because I'm lazy
	between two points on the earth (specified in decimal degrees)    
	"""
	# convert decimal degrees to radians         
	if lat1 is None:
		lat1 = 0
	if lon1 is None:
		lon1 = 0
	if lat2 is None:
		lat2 = 0
	if lon2 is None:
		lon2 = 0

	lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])

	# haversine formula 
	dlon = np.subtract(lon2, lon1) 
	dlat = np.subtract(lat2, lat1)
	# argh - sequential multiplies and trig - this will be slow in production.
	# Find a vector ready trig library or start thinking about LUTs fucko, or the 
	# RPA is gonna be sitting on its ass thinking for a while.
	# a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
	a = (np.sin(dlat/2))**2 + np.cos(lat1) * np.cos(lat2) * (np.sin(dlon/2))**2
	c = 2 * np.arcsin(np.sqrt(a)) 
	# c = 2 * np.atan2(sqrt(a), sqrt(1-a))
	# r = 6371000 # Radius of earth in meters. Use 3956 for miles
	r = 6373.0 # Radius of earth in meters. Use 3956 for miles
	meter_dist = c * r

	return meter_distmain.py

def distance(boat_lat, boat_lng, pt_lat, pt_lng,): #For 610 pts time = -3.361701965332031e-05
	delta_pts = np.sqrt(np.multiply(np.subtract(boat_lat,pt_lat),np.subtract(boat_lat, pt_lat)) + np.multiply(np.subtract(boat_lng, pt_lng),np.subtract(boat_lng, pt_lng)))
	return(delta_pts)

def distance_spherical(a_lat,a_log,b_lat,b_log): #For 610 pts time = -0.00010967254638671875
	R = 6373.0
	lat1 = np.radians(a_lat)
	lon1 = np.radians(a_log)
	lat2 = np.radians(b_lat)
	lon2 = np.radians(b_log)
	dlon = np.subtract(lon2, lon1)
	dlat = np.subtract(lat2, lat1)
	a = np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2
	cvar = np.multiply(np.arctan2(np.sqrt(a), np.sqrt(1 - a)), 2)
	distance = R * cvar
	return distance

def distance_pt_to_line(my_lat, my_lng, curr_path_seg_x0, curr_path_seg_y0, curr_path_seg_x1, curr_path_seg_y1): #Added from anishes ShipController
	# verified pen and paper - this is accurate
	point_pt0 = float(my_lat)
	point_pt1 = float(my_lng) 
	line1_pt0 = float(curr_path_seg_x0)
	line1_pt1 = float(curr_path_seg_y0)
	line2_pt0 = float(curr_path_seg_x1)
	line2_pt1 = float(curr_path_seg_y1)

	numerator = abs(((line2_pt1 - line1_pt1)*point_pt0) -((line2_pt0-line1_pt0)*point_pt1) + (line2_pt0*line1_pt1) -(line2_pt1*line1_pt0))


	denominator = np.sqrt((line2_pt1-line1_pt1)**2 + (line2_pt0-line1_pt0)**2)

	distance = numerator/denominator
	# distance = abs(((line2_pt1 - line1_pt1)*point_pt0) -((line2_pt0-line1_pt0)*point_pt1) + (line2_pt0*line1_pt1) -(line2_pt1*line1_pt0))
	return(distance)   


def get_world_polar_bb(my_lat, my_lng, range_m, az_deg, range_size, az_size):

	"""
	Given a target center, and a size in range and azimth - get the lat/lng corners of the bounding arc segment
	:param float my_lat in degrees        
	:param float my_lng in degrees        
	:param float range to target center in m
	:param float az_deg to target center in m
	:param float range_size range span in m
	:param float az_size to azimuth span in m
	:return: float (arc bounding box in lat/lng coords)
	"""

	box = np.zeros((2, 4))

	box[0, 0], box[1, 0], _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg - az_size/2), range_m - range_size/2))
	box[0, 1], box[1, 1], _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg + az_size/2), range_m - range_size/2))
	box[0, 2], box[1, 2], _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg - az_size/2), range_m + range_size/2))
	box[0, 3], box[1, 3], _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg + az_size/2), range_m + range_size/2))
	# p0_lat, p0_lng, _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg - az_size/2), range_m - range_size/2))
	# p1_lat, p1_lng, _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg + az_size/2), range_m - range_size/2))
	# p2_lat, p2_lng, _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg - az_size/2), range_m + range_size/2))
	# p3_lat, p3_lng, _ = map(np.degrees, vinc_pt(np.radians(my_lat), np.radians(my_lng), np.radians(az_deg + az_size/2), range_m + range_size/2))
	# print(f"This is the box dimentions {box}")
	return(box) #p0_lat, p0_lng, p1_lat, p1_lng, p2_lat, p2_lng, p3_lat, p3_lng) 

def boxcenter(box):

	center  = np.divide(box[:, 1] + box[:, 3], 2)
	
	# center2 = np.divide(box[:, 1] + box[:, 3], 2)
	# print(f" Center calculation the following two numbers should be similar {center} {center2}")

	return(center)

def get_radar_image_pix_bb(range_center, az_center, range_size, az_size  ):
	"""
	Given a target center, and a size in range and azimth - get the pixel coords 
	(assume 4096x4096 radar image) of its bounding box 
	:param float range_center (distance to center) in meters
	:param float az_center (angle to center) in degrees
	:param float range_size range span in m
	:param float az_size to azimuth span in m
	:return: float (cartesian bounding box in pixel coords)
	"""
	
	CELL_DUR = 19998646e-15
	CELERITY = 299792458

	RADAR_MAX_RANGE_PX = 2047
	m_to_pix_scaler = 1/(CELL_DUR * CELERITY/2)

	center_x_px, center_y_px    = RADAR_MAX_RANGE_PX, RADAR_MAX_RANGE_PX
	range_center_px             = range_center*m_to_pix_scaler
	range_size_px               =   range_size*m_to_pix_scaler
	
	r_min       = range_center_px - range_size_px/2
	r_max       = range_center_px + range_size_px/2
	az_min      = az_center  - az_size/2   
	az_max      = az_center  + az_size/2   

	x0_in = r_min*np.cos(np.radians(az_min)) + center_x_px
	y0_in = r_min*np.sin(np.radians(az_min)) + center_y_px
	x1_in = r_min*np.cos(np.radians(az_max)) + center_x_px
	y1_in = r_min*np.sin(np.radians(az_max)) + center_y_px

	x2_out = r_max*np.cos(np.radians(az_min)) + center_x_px
	y2_out = r_max*np.sin(np.radians(az_min)) + center_y_px
	x3_out = r_max*np.cos(np.radians(az_max)) + center_x_px
	y3_out = r_max*np.sin(np.radians(az_max)) + center_y_px
	
	return  ((x0_in,y0_in), (x1_in,y1_in), (x2_out,y2_out), (x3_out,y3_out)) 


def radius_detection(boat_lat, boat_lng, pt_lat, pt_lng, radius):

	r2pt = np.sqrt((boat_lat-pt_lat)*(boat_lat-pt_lat) + (boat_lng-pt_lng)*(boat_lng-pt_lng))

	if r2pt < radius:
		in_radius = True
	else:
		in_radius = False

	return(in_radius)


def delta_headings(boat_lat, boat_lng, pt_lat, pt_lng):
	opposit = (pt_lat - boat_lat)
	adjacent = (pt_lng - boat_lng)
	theta = np.rad2deg(np.arctan(np.absolute(opposit/adjacent)))
	if opposit > 0 and adjacent > 0:
		theta =  90 - theta
		#print("Q1")
	if opposit < 0 and adjacent > 0: 
		theta = theta + 90
		#print("Q2")
	if opposit < 0 and adjacent < 0:
		theta = 90 - theta + 180
		#print("Q3")
	if opposit > 0 and adjacent < 0:
		theta = theta + 270
		#print("Q4")
	return(theta)

def heading_angle_between_points(lat1, lng1, lat2, lng2):	#Modified from Anishes implementation to support numpy	

	X =  np.cos(np.radians(lat2)) * np.sin(np.radians(lng2 - lng1))
	Y = (np.cos(np.radians(lat1)) * np.sin(np.radians(lat2))) - (np.sin(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.cos(np.radians(lng2 - lng1)))

	bearing_rad = np.arctan2(X,Y)
	bearing_deg = np.degrees(bearing_rad)

	return(bearing_deg) #, X, Y)

#The idea of this is to create generic way of current speed to distance in the future. Unfortunatly maps change length based on heading
#knots is the speed you are traveling
#heading is the way you are facing 
#Delta time is the how long it will travel for (future estimate)
def knots_to_distance(knots, heading, delta_time):
	earths_radius = 6371
	km_h = 1.85 * knots

	km = (km_h/3600) * delta_time
	heading_rad = np.radians(heading)
	
	dy = np.sin(heading_rad) * km 
	dx = np.cos(heading_rad) * km 

	lat = ((dy * 180)/(6371 * np.pi))
	lng = (dx/(earths_radius*np.cos((lat*np.pi)/180))) 
	distance = np.sqrt(np.square(lat) + np.square(lng))
	print(f'This is the calculated distance {distance}')
	
	return(distance)


def map_gk(value, left_min, left_max, right_min, right_max):

	left_range = left_max - left_min
	right_range = right_max - right_min
	scaler = (value - left_min) / (left_range)
	output = right_min + (scaler * right_range)

	#floor the values if it exceeds bounds
	if output < right_min:
		output = right_min
	if output > right_max:
		output = right_max
	
	return(output)


def ship_box_transform(heading, boat_lat, boat_lng, a, b, c, d):		
	#Change from rotation matrix to method to something else? 

	heading = 360 - heading - 90
	g_t, g_a = meters_to_geo(a, a)
	g_t, g_b = meters_to_geo(b, b)
	g_t, g_c = meters_to_geo(c, c)
	g_t, g_d = meters_to_geo(d, d)

	x1 = -g_a 
	x2 =  g_b  
	y1 =  g_c  
	y2 = -g_d 
 
	A = np.array([x1, y2])
	B = np.array([x2, y2])
	C = np.array([x2, y1])
	D = np.array([x1, y1])

	theta = np.radians(np.subtract(360, heading+180)) #(360 - heading -90)

	c, s = np.cos(theta), np.sin(theta)
	j = np.matrix([[c, s], [-s, c]])

	p0 = np.array([boat_lat, boat_lng])
	p1 = np.add(p0, np.dot(j, A)) 
	p2 = np.add(p0, np.dot(j, B)) 
	p3 = np.add(p0, np.dot(j, C))
	p4 = np.add(p0, np.dot(j, D))

	square = np.array([p1, p2, p3, p4]) #, p1])
	return(square.T)

def test_distance():
	import time
	for i in range(100):
		tic = time.clock()
		distance,(4.41601846, 51.90056363, 4.43075069, 51.90165002)
		toc = time.clock()
		print(toc - tic)

# test_distance()
# print(distance(4.41601846, 51.90056363, 4.43075069, 51.90165002))
# function getGeoCoordinatesForMesh(lon, lat, angle, vertices){
#     var rotated = new Array();
#     for(var i in vertices) {
#         var v = vertices[i];
#         var bearing = Math.atan2(v[0], v[1]) + toRadians(angle); 
#         var distance = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
#         var lat1 = toRadians(lat);    
#         var lon1 = toRadians(lon);    
#         var centralAngle = Math.acos(1 - (Math.pow(distance, 2) / (2 * Math.pow(earthRadius * 1000, 2))));
#         var lat2 = Math.asin(Math.sin(lat1) * Math.cos(centralAngle) + Math.cos(lat1) * Math.sin(centralAngle) * Math.cos(bearing));    
#         var lon2 = lon1 + Math.atan2(Math.sin(bearing) * Math.sin(centralAngle) * Math.cos(lat1), Math.cos(centralAngle)-Math.sin(lat1) * Math.sin(lat2));
		
#         rotated.push([toDegrees(lon2), toDegrees(lat2), 1]);            
#     }
#     return rotated;


# Formules voor benadering zijn gebaseerd op http://www.dekoepel.nl/pdf/Transformatieformules.pdf
# Bovenstaande link werkt helaas niet meer, daar Stiching de Koepel opgeheven is. Backup link: http://media.thomasv.nl/2015/07/Transformatieformules.pdf

def fromRdToWgs(x, y):
    X0      = 155000.0
    Y0      = 463000.0
    phi0    = 52.15517440
    lam0    = 5.38720621

    Kp = [0,2,0,2,0,2,1,4,2,4,1]
    Kq = [1,0,2,1,3,2,0,0,3,1,1]
    Kpq = [3235.65389,-32.58297,-0.24750,-0.84978,-0.06550,-0.01709,-0.00738,0.00530,
           -0.00039,0.00033,-0.00012]

    Lp = [1,1,1,3,1,3,0,3,1,0,2,5]
    
    Lq = [0,1,2,0,3,1,1,2,4,2,0,0]
    
    Lpq = [5260.52916, 105.94684, 2.45656, -0.81885,
           0.05594, -0.05607, 0.01199, -0.00256,
           0.00128, 0.00022, -0.00022, 0.00026]

    x = np.subtract(x, X0) * 0.00001
    y = np.subtract(y, Y0) * 0.00001
    
    phi = np.zeros_like(x)
    lam = np.zeros_like(y)

    for k in range(len(Kpq)):
        phi = phi + np.multiply(Kpq[k], np.multiply(np.power(x, Kp[k]), np.power(y, Kq[k])))

    phi = phi0 + np.divide(phi, 3600)
    for l in range(len(Lpq)):
        lam = lam + np.multiply(Lpq[l], np.multiply(np.power(x, Lp[l]), np.power(y, Lq[l])))
        
    lam = lam0 + np.divide(lam, 3600)

    return(phi, lam)

def fromWgsToRd(coords):

	X0      = 155000
	Y0      = 463000
	phi0    = 52.15517440
	lam0    = 5.38720621
	
	Rp = [0,1,2,0,1,3,1,0,2]
	Rq = [1,1,1,3,0,1,3,2,3]
	Rpq = [190094.945,-11832.228,-114.221,-32.391,-0.705,-2.340,-0.608,-0.008,0.148]

	Sp = [1,0,2,1,3,0,2,1,0,1]
	Sq = [0,2,0,2,0,1,2,1,4,4]
	Spq = [309056.544,3638.893,73.077,-157.984,59.788,0.433,-6.439,-0.032,0.092,-0.054]

	dPhi = 0.36 * ( coords[0] - self.phi0 )
	dLam = 0.36 * ( coords[1] - self.lam0 )

	X = 0
	Y = 0

	for r in range( len( Rpq ) ):
		X = X + ( Rpq[r] * dPhi**Rp[r] * dLam**Rq[r] ) 
	X = self.X0 + X

	for s in range( len( Spq ) ):
		Y = Y + ( Spq[s] * dPhi**Sp[s] * dLam**Sq[s] )
	Y = self.Y0 + Y

	return [X,Y]

def loads57Pickle(pickleFileName):

    land_fname = pickleFileName
    
    land_main_poly_list = []
    land_main_poly_list.append([])
    land_main_poly_list.append([])

    land_main_poly_list = (pickle.load(open(land_fname,"rb")))
    return land_main_poly_list

def get_time():
	millis  = float(datetime.now().strftime('.%f')[:-4])
	seconds = int(datetime.now().strftime('%S'))
	minuts  = int(datetime.now().strftime('%M'))
	hours   = int(datetime.now().strftime('%H'))
	time_2  = float(np.multiply(minuts,60) + np.multiply(hours,3600) + seconds)
	time_2  = float(np.multiply(minuts,60) + np.multiply(hours,3600) + seconds + millis)
	return(time_2)



def vinc_pt(phi1, lembda1, alpha12, s):
    """
	Vincenty direct formula
    Returns the lat and long of projected point and reverse azimuth
    given a reference point and a distance and azimuth to project.
    :parameters: phi-lat, lemba-longs and alpha-azimuths passed in radians (!) , s is the distance in meters
    :Returns: ( phi2,  lambda2,  alpha21 ) as a tuple
    """

    a = 6224136.1166648 #6378137.0
    b = 6224136.1166648 #6356752.314245
    f = (a-b)/a    

    two_pi = 2.0 * np.pi

    if (alpha12 < 0.0):
        alpha12 = alpha12 + two_pi
    if (alpha12 > two_pi):
        alpha12 = alpha12 - two_pi

    b = a * (1.0 - f)

    tan_u1 = (1 - f) * np.tan(phi1)
    U1 = np.arctan(tan_u1)
    sigma1 = np.arctan2(tan_u1, np.cos(alpha12))
    sinalpha = np.cos(U1) * np.sin(alpha12)
    cosalpha_sq = 1.0 - sinalpha * sinalpha

    u2 = cosalpha_sq * (a * a - b * b) / (b * b)
    A = 1.0 + (u2 / 16384) * (4096 + u2 * (-768 + u2 *
                                           (320 - 175 * u2)))
    B = (u2 / 1024) * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))

    # Starting with the approximation
    sigma = (s / (b * A))

    last_sigma = 2.0 * sigma + 2.0  # something impossible

    # Iterate the following three equations
    # until there is no significant change in sigma
    # two_sigma_m , delta_sigma
    while (abs((last_sigma - sigma) / sigma) > 1.0e-9):
        two_sigma_m = 2 * sigma1 + sigma

        delta_sigma = B * np.sin(sigma) * \
            (np.cos(two_sigma_m)
             + (B / 4) * (np.cos(sigma) *
                          (-1 + 2 * np.power(
                              np.cos(two_sigma_m), 2) -
                           (B / 6) * np.cos(two_sigma_m) *
                           (-3 + 4 * np.power(np.sin(sigma), 2)) *
                           (-3 + 4 * np.power(
                               np.cos(two_sigma_m), 2)))))

        last_sigma = sigma
        sigma = (s / (b * A)) + delta_sigma

    phi2 = np.arctan2((np.sin(U1) *
                       np.cos(sigma) +
                       np.cos(U1) *
                       np.sin(sigma) *
                       np.cos(alpha12)),
                      ((1 - f) *
                       np.sqrt(np.power(sinalpha, 2) +
                                 pow(np.sin(U1) *
                                     np.sin(sigma) -
                                     np.cos(U1) *
                                     np.cos(sigma) *
                                     np.cos(alpha12), 2))))

    lembda = np.arctan2((np.sin(sigma) * np.sin(alpha12)),
                        (np.cos(U1) * np.cos(sigma) -
                         np.sin(U1) * np.sin(sigma) * np.cos(alpha12)))

    C = (f / 16) * cosalpha_sq * (4 + f * (4 - 3 * cosalpha_sq))

    omega = lembda - (1 - C) * f * sinalpha *  \
        (sigma + C * np.sin(sigma) *
         (np.cos(two_sigma_m) +
          C * np.cos(sigma) *
          (-1 + 2 * np.power(np.cos(two_sigma_m), 2))))

    lembda2 = lembda1 + omega

    alpha21 = np.arctan2(
        sinalpha,
        (-
         np.sin(U1) *
         np.sin(sigma) +
         np.cos(U1) *
         np.cos(sigma) *
         np.cos(alpha12)))

    alpha21 = alpha21 + two_pi / 2.0
    if (alpha21 < 0.0):
        alpha21 = alpha21 + two_pi
    if (alpha21 > two_pi):
        alpha21 = alpha21 - two_pi

    return phi2, lembda2, alpha21
