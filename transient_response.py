


# Constraints:
# - assumes moment of inertia about center of gravity axis is idealized as a point mass MOI
# - assumes chassis torsional stiffness is infinite (front roll = rear roll)


import csv
import math
import kinematics


def getConstants(file):
	pointDict = {}
	with open(file) as csvfile:
		csvfile = csv.reader(csvfile, delimiter=',')
		for row in csvfile:
			if (row[0] == 'Front toe'):
				pointDict['FT'] = float(row[1])
			elif (row[0] == 'Front tire spring rate'):
				pointDict['FTSR'] = float(row[1])
			elif (row[0] == 'Front shock spring rate'):
				pointDict['FSSR'] = float(row[1])
			elif (row[0] == 'Front motion ratio'):
				pointDict['FMR'] = float(row[1])
			elif (row[0] == 'Front anti-roll stiffness'):
				pointDict['FARS'] = float(row[1])
			elif (row[0] == 'Front damping ratio'):
				pointDict['FDR'] = float(row[1])
			elif (row[0] == 'Front corner unsprung mass'):
				pointDict['FUM'] = float(row[1])
			elif (row[0] == 'Front corner sprung mass'):
				pointDict['FSM'] = float(row[1])
			elif (row[0] == 'Rear toe'):
				pointDict['RT'] = float(row[1])
			elif (row[0] == 'Rear tire spring rate'):
				pointDict['RTSR'] = float(row[1])
			elif (row[0] == 'Rear shock spring rate'):
				pointDict['RSSR'] = float(row[1])
			elif (row[0] == 'Rear motion ratio'):
				pointDict['RMR'] = float(row[1])
			elif (row[0] == 'Rear anti-roll stiffness'):
				pointDict['RARS'] = float(row[1])
			elif (row[0] == 'Rear damping ratio'):
				pointDict['RDR'] = float(row[1])
			elif (row[0] == 'Rear corner unsprung mass'):
				pointDict['RUM'] = float(row[1])
			elif (row[0] == 'Rear corner sprung mass'):
				pointDict['RSM'] = float(row[1])
			elif (row[0] == 'Center of gravity height'):
				pointDict['COG'] = float(row[1])
			elif (row[0] == 'Polar moment of inertia about COG z-axis'):
				pointDict['PMOI'] = float(row[1])
			elif (row[0] == 'Front tire contact patch'):
				pointDict['FTCP'] = [float(row[1]), float(row[2]), float(row[3])]
			elif (row[0] == 'Rear tire contact patch'):
				pointDict['RTCP'] = [float(row[1]), float(row[2]), float(row[3])]

	if (len(pointDict) < 20):
		raise ValueError('ERROR: Suspension points file does not have all required points. Check that point names haven\'t been changed or have spaces added to end.')
	
	print('Suspension values read:')
	for key in pointDict:
		print (key + ':		' + str(pointDict[key]))

	return pointDict

def getFittingFunction(file):
	fitFunc = []
	with open(file) as csvfile:
		csvfile = csv.reader(csvfile, delimiter=',')
		next(csvfile, None)  # skip the header
		for row in csvfile:
			fitFunc.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
	return fitFunc

def getLateralForce(fitFunc, slipangle, camber, normalforce):
	cf = 0
	for i in range(0,len(fitFunc)):
		cf += fitFunc[i][3]*(slipangle**fitFunc[i][0])*(camber**fitFunc[i][1])*(normalforce**fitFunc[i][2])
	return (cf*normalforce)

def getWheelRates(constants, front_or_rear):
	if (front_or_rear == 'FRONT'):
		tirerate = constants['FTSR']
		wheelrate = constants['FSSR']*constants['FMR']*constants['FMR']
		return ((1.0/tirerate + 1.0/wheelrate)**(-1))
	elif (front_or_rear == 'REAR'):
		tirerate = constants['RTSR']
		wheelrate = constants['RSSR']*constants['RMR']*constants['RMR']
		return ((1.0/tirerate + 1.0/wheelrate)**(-1))

def getRollStiffness(constants, front_or_rear, track):
	# roll stiffness is torque/deg of roll. Assume +- 1 inch displacement on each wheel.
	# F = 1in*wheelrate on each wheel. Torque = F*track
	# angle = arctan(2in/track)
	wheelrate = getWheelRates(constants, front_or_rear)
	angle = math.degrees(math.atan(2.0/track))
	return (12*wheelrate/angle)

def interpolate(xvals, yvals, x):
	for i in range(0, len(xvals)-1):
		if (x >= xvals[i] and x <= xvals[i+1]):
			x0 = xvals[i]
			x1 = xvals[i+1]
			y0 = yvals[i]
			y1 = yvals[i+1]
			return ((x - x0)/(x1 - x0) * (y1 - y0) + y0)

def getCamberFromNormalForce(curves, normal_force, nominal_normal_force):
	dF = nominal_normal_force - normal_force
	return interpolate(curves[0], curves[1], dF/12.)

def getRCHfromNormalForce(curves, normal_force, nominal_normal_force):
	dF = nominal_normal_force - normal_force
	return interpolate(curves[0], curves[2], dF/12.)





fitFunc = getFittingFunction('TireFit/lateral_tire_fitting_function.csv')
constants = getConstants('suspension_points.csv')

# right and up are positive from the driver's view


# ------------- Constants ----------------
front_track = constants['FTCP'][1] * 2
rear_track = constants['RTCP'][1] * 2

wheelbase = constants['RTCP'][0] - constants['FTCP'][0]
h_cog = constants['COG']
longitudinal_MOI = 0.5*(constants['FSM'] + constants['RSM'])*h_cog*h_cog

FR_ss_normal_force = constants['FUM'] + constants['FSM']
FL_ss_normal_force = constants['FUM'] + constants['FSM']
RR_ss_normal_force = constants['RUM'] + constants['RSM']
RL_ss_normal_force = constants['RUM'] + constants['RSM']

total_mass = 2*(RR_ss_normal_force + FR_ss_normal_force)
rear_weight_bias = RR_ss_normal_force/(RR_ss_normal_force + FR_ss_normal_force)

# --------------------------------------

# ------------ Get curves --------------
# get camber and RCH curves
curves = kinematics.main(3, .05)
frontcurves = curves[0]
rearcurves = curves[1]
frontwheelrate = getWheelRates(constants, 'FRONT')
rearwheelrate = getWheelRates(constants, 'REAR')

# transform curves as a function of delta z to delta normal force
dF = [x * frontwheelrate for x in frontcurves[0]] #scale dZ to dF
frontcurves[0] = dF

dF = [x * rearwheelrate for x in rearcurves[0]]
rearcurves[0] = dF

#--------------------------------------


# ------- Initial conditions -----------
yaw = 0
dyaw_dt = 0
roll = 0
droll_dt = 0
d2roll_dt2 = 0

FR_lateral_force = 0
FL_lateral_force = 0
RR_lateral_force = 0
RL_lateral_force = 0

FR_normal_force = FR_ss_normal_force
FL_normal_force = FL_ss_normal_force
RR_normal_force = RR_ss_normal_force
RL_normal_force = RL_ss_normal_force
# --------------------------------------


dt = 0.1
const_slip_angle = 8 #step input for front 

front_RCH = 0
rear_RCH = 0
front_roll_stiffness = getRollStiffness(constants, 'FRONT', front_track) + 12*constants['FARS'] 
rear_roll_stiffness =  getRollStiffness(constants, 'REAR', front_track) + 12*constants['RARS'] 


# find critical damping coefficients. gamma_crit = 2*sqrt(k*m). k = roll stiffness . m = longitudinal moment of inertia
front_crit_damping_coeff = 2*math.sqrt(front_roll_stiffness*longitudinal_MOI)
rear_crit_damping_coeff = 2*math.sqrt(rear_roll_stiffness*longitudinal_MOI)

# set damping coefficients = damping_ratio*crit_damping_coeff
front_gamma = constants['FDR']*front_crit_damping_coeff
rear_gamma = constants['RDR']*rear_crit_damping_coeff



# Open output file
csvfile = open('output.csv', 'w')
writer = csv.writer(csvfile, delimiter=',')
writer.writerow(['t', 'YawGradient', 'Roll', 'FL normal force', 'FR normal force', 'RL normal force', 'RR normal force'])


for i in range(0, 10000):


	# get new roll center heights and cambers from the normal forces
	#front_RCH = .5*getRCHfromNormalForce(frontcurves, FR_normal_force, FR_ss_normal_force) + .5*getRCHfromNormalForce(frontcurves, FL_normal_force, FL_ss_normal_force)
	#rear_RCH = .5*getRCHfromNormalForce(rearcurves, RR_normal_force, RR_ss_normal_force) + .5*getRCHfromNormalForce(rearcurves, RL_normal_force, RL_ss_normal_force)
	frontleft_camber = getCamberFromNormalForce(frontcurves, FL_normal_force, FL_ss_normal_force) 
	frontright_camber = getCamberFromNormalForce(frontcurves, FR_normal_force, FR_ss_normal_force) 
	rearleft_camber = getCamberFromNormalForce(rearcurves, RL_normal_force, RL_ss_normal_force)
	rearright_camber = getCamberFromNormalForce(rearcurves, RR_normal_force, RR_ss_normal_force) 
	

	# deltaF = roll (deg)  * roll_stiffness(ft-lbs/deg) / track (in) * (12in/ft)
	deltaF = roll*front_roll_stiffness*(h_cog - front_RCH)/h_cog/front_track

	# deltaF is split between the two wheels
	FR_normal_force = FR_ss_normal_force + (0.5*deltaF)
	FL_normal_force = FL_ss_normal_force - (0.5*deltaF)

	# Repeat for rear
	deltaF = roll*rear_roll_stiffness*(h_cog - rear_RCH)/h_cog/rear_track
	RR_normal_force = RR_ss_normal_force + (0.5*deltaF)
	RL_normal_force = RR_ss_normal_force - (0.5*deltaF)

	#check if normal force goes negative. If it does, set it to 0
	if (FR_normal_force < 0):
		FR_normal_force = 0
	if (FL_normal_force < 0):
		FL_normal_force = 0
	if (RR_normal_force < 0):
		RR_normal_force = 0
	if (RL_normal_force < 0):
		RL_normal_force = 0	


	# calculate lateral forces using tire fit function for a given slip angle, camber, and normal force
	FR_lateral_force = getLateralForce(fitFunc, const_slip_angle, frontright_camber, FR_normal_force)
	FL_lateral_force = getLateralForce(fitFunc, const_slip_angle, -frontleft_camber, FL_normal_force)
	RR_lateral_force = getLateralForce(fitFunc, yaw - constants['RT'], rearright_camber, RR_normal_force)
	RL_lateral_force = getLateralForce(fitFunc, yaw + constants['RT'], -rearleft_camber, RL_normal_force)

	# calculate total front and rear lateral forces
	front_force = FR_lateral_force - FL_lateral_force
	rear_force = RR_lateral_force - RL_lateral_force

	# calculate roll torque as a function of the lateral forces and their moment arms with damping in the yz plane
	front_roll_torque = front_force*(h_cog - front_RCH) - front_gamma*droll_dt
	rear_roll_torque = rear_force*(h_cog - rear_RCH) - rear_gamma*droll_dt


	# 2nd derivative of roll is the roll torque divided by moment of inertia
	d2roll_dt2 = (front_roll_torque + rear_roll_torque)/longitudinal_MOI

	# yaw torque is the difference in torques between the front lateral force and rear lateral force on the yaw center (assumed to be center of gravity) in xy plane
	yawtorque = front_force*(1 - rear_weight_bias)*wheelbase - rear_force*rear_weight_bias*wheelbase
	# derivative of yaw is the yaw torque divided by the polar moment of inertia. (Not 2nd derivative because we consider yaw to be in the car's reference frame)
	dyaw_dt = yawtorque/constants['PMOI']

	# Step in time
	yaw += dt*dyaw_dt
	roll += dt*droll_dt
	droll_dt += dt*d2roll_dt2

	#write solution
	if (i%100 == 0):
		writer.writerow([i/1000.*dt, -dyaw_dt, roll, FL_normal_force, FR_normal_force, RL_normal_force, RR_normal_force])

