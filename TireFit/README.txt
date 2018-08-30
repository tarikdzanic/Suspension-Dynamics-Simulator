To do the simulations, the code needs to know what coefficient of friction to expect for a given slip angle, camber, and normal force. For this, you need a 
3-variable fitted function based on some tire data to interpolate the coefficient of friction. This code relies on polynomial fits, which generally work well 
for the given data range but can go very wrong if trying to extrapolate data from outside the given range. Be careful to keep the simulation bounds within 
the data range you've fitted the functions on.

The lateral_tire_fitting_function.csv file holds the constants and exponents of the function. Querying the coefficient of friction for a given slip angle,
camber, and normal force will return a sum of the constants times the variables with their given exponents. Example:

Slip angle exponent		Camber exponent		Normal force exponent		Constant
3				1			0				.12
1				0			2				.55

will give the function:
CF = .12*(s^3)*c + .55*s*n^2

The given fitting function is a fit for the Goodyear G19 Eagle 20.0x7.0-13 tire.