// Attitude estimator based on the fusion of 3-axis accelerometer, gyroscope and magnetometer data
// File: attitude_estimator.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include "attitude_estimator.h"
#include <cmath>

// Namespaces
using namespace stateestimation;

// Constructor
AttitudeEstimator::AttitudeEstimator(bool quickLearn)
{
	// Initialise the entire class
	resetAll(quickLearn);
}

// Reset functions
void AttitudeEstimator::reset(bool quickLearn, bool resetGyroBias)
{
	// Reset the estimator state
	resetState(resetGyroBias);

	// Reset the lambda value to reinitiate quick learning (if requested)
	if(quickLearn) resetLambda();
	else setLambda();
}
void AttitudeEstimator::resetAll(bool quickLearn)
{
	// Initialise the configuration variables
	setPIGains(2.20, 2.65, 10.0, 1.25);
	setQLTime(3.0);

	// Initialise the magnetometer calibration
	setMagCalib(1.0, 0.0, 0.0);

	// Reset the attitude estimator
	reset(quickLearn);
}
void AttitudeEstimator::resetState(bool resetGyroBias)
{
	// Declare variables
	int i;
	
	// Initialise the attitude estimate
	setAttitude(1.0, 0.0, 0.0, 0.0);

	// Initialise the gyro bias estimate
	if(resetGyroBias)
		setGyroBias(0.0, 0.0, 0.0);

	// Initialise the remaining size 3 internal variables
	for(i = 0;i < 3;i++)
	{
		m_w[i] = 0.0;
		m_wold[i] = 0.0;
		m_omega[i] = 0.0;
		m_base[i] = 0.0;
	}

	// Initialise the remaining size 4 internal variables
	for(i = 0;i < 4;i++)
	{
		m_Qy[i] = 0.0;
		m_Qtilde[i] = 0.0;
		m_dQ[i] = 0.0;
		m_dQold[i] = 0.0;
	}
	m_Qy[0] = 1.0;
	m_Qtilde[0] = 1.0;

	// Initialise the remaining size 9 internal variables
	for(i = 0;i < 9;i++)
	{
		m_Ry[i] = 0.0;
	}
	m_Ry[0] = 1.0;
	m_Ry[4] = 1.0;
	m_Ry[8] = 1.0;
}

// Get/set functions for the current attitude estimate
void AttitudeEstimator::setAttitude(double yaw, double pitch, double roll)
{
	// Declare variables
	double cphi, sphi, cth, sth, cpsi, spsi;
	double w, x, y, z;

	// Halve the yaw, pitch and roll values (for calculation purposes only)
	yaw   /= 2.0;
	pitch /= 2.0;
	roll  /= 2.0;

	// Precalculate the required sin and cos values
	cphi = cos(yaw);
	sphi = sin(yaw);
	cth  = cos(pitch);
	sth  = sin(pitch);
	cpsi = cos(roll);
	spsi = sin(roll);

	// Calculate the required quaternion components
	w = cphi*cth*cpsi + sphi*sth*spsi;
	x = cphi*cth*spsi - sphi*sth*cpsi;
	y = cphi*sth*cpsi + sphi*cth*spsi;
	z = sphi*cth*cpsi - cphi*sth*spsi;

	// Set the current attitude estimate to the calculated quaternion orientation
	setAttitude(w, x, y, z);
}
void AttitudeEstimator::setAttitude(double w, double x, double y, double z)
{
	// Calculate the quaternion square norm
	double qscale = w*w + x*x + y*y + z*z;

	// Update the current attitude estimate
	if(qscale < QNORM_TOL_SQ) // Reset the attitude estimate to the identity orientation if the norm is too close to zero
	{
		m_Qhat[0] = 1.0;
		m_Qhat[1] = 0.0;
		m_Qhat[2] = 0.0;
		m_Qhat[3] = 0.0;
	}
	else
	{
		qscale = 1.0 / std::sqrt(qscale);
		m_Qhat[0] = qscale*w;
		m_Qhat[1] = qscale*x;
		m_Qhat[2] = qscale*y;
		m_Qhat[3] = qscale*z;
	}

	// Update the Euler angle representation of the current attitude estimate
	updateEuler();
}

// Get/set functions for the configuration variables
void AttitudeEstimator::getPIGains(double &Kp, double &Ti, double &KpQuick, double &TiQuick)
{
	// Retrieve the current gains
	Kp = m_Kp;
	Ti = m_Ti;
	KpQuick = m_KpQuick;
	TiQuick = m_TiQuick;
}
void AttitudeEstimator::setPIGains(double Kp, double Ti, double KpQuick, double TiQuick)
{
	// Set the standard PI gains
	if((Kp > 0.0) && (Ti > 0.0))
	{
		m_Kp = Kp;
		m_Ti = Ti;
	}

	// Set the quick learning PI gains
	if((KpQuick > 0.0) && (TiQuick > 0.0))
	{
		m_KpQuick = KpQuick;
		m_TiQuick = TiQuick;
	}
}

// Estimation update functions
/**
* This function requires (in a logically grouped sense) four inputs:
* - @c dt: The time since the last call to `update()` (in the first call should just be the nominal loop period, must be in seconds)
* - @c gyro: New gyroscope reading (must be in rad/s)
* - @c acc: New accelerometer reading (can be in any self-consistent units, preferably <tt>ms<sup>-2</sup></tt>)
* - @c mag: New magnetometer reading (can be in any self-consistent units, preferably gauss and preferably the same units as `MagCalib`, but this is not required)
* 
* The algorithm at work is based on the Passive Complementary Filter described in:
* > R. Mahoney, T. Hamel, and J.-M. Pflimlin, "Nonlinear complementary filters on the special orthogonal group",
* > IEEE Transactions on Automatic Control, vol. 53, no. 5, pp. 1203-1218.
* This implementation has been made ultimately robust, with detailed consideration and mathematical analysis having been performed to avoid
* all possible numerical issues, and to deal with all possible special cases. Nevertheless, the implementation has simultaneously been
* optimised to use the least number of floating point operations as possible. The source code of this and the subordinate functions thereof
* has been heavily documented.
**/
void AttitudeEstimator::update(double dt, double gyroX, double gyroY, double gyroZ, double accX, double accY, double accZ, double magX, double magY, double magZ)
{
	// Note: The algorithm implemented in this function is based on the Passive Complementary Filter described in:
	//       --> R. Mahoney, T. Hamel, and J.-M. Pflimlin, "Nonlinear complementary filters on the special orthogonal group",
	//       --> IEEE Transactions on Automatic Control, vol. 53, no. 5, pp. 1203-1218.
	//       Do not modify *anything* unless you know *exactly* what you're doing (even if you think you're not changing anything significant),
	//       or it could affect the unconditional and unadulterated robustness of the entire estimator. This applies to updateQy() as well.

	// Update lambda
	if(m_lambda < 1.0)
	{
		m_lambda += dt / m_QLTime;
		if(m_lambda <= 0.0) m_lambda = 0.0;
		if(m_lambda >= 1.0) m_lambda = 1.0;
	}

	// Calculate the required filter gains for this update (Note: Ki = Kp / Ti)
	double Kp = m_lambda*m_Kp + (1-m_lambda)*m_KpQuick;
	double Ti = m_lambda*m_Ti + (1-m_lambda)*m_TiQuick;

	// Save the old values of the required variables
	m_wold[0] = m_w[0];
	m_wold[1] = m_w[1];
	m_wold[2] = m_w[2];
	m_dQold[0] = m_dQ[0];
	m_dQold[1] = m_dQ[1];
	m_dQold[2] = m_dQ[2];
	m_dQold[3] = m_dQ[3];

	// Calculate Qy, the current quaternion orientation measurement, based on the acc and mag readings
	updateQy(accX, accY, accZ, magX, magY, magZ); // Sets m_Qy internally...

	// Calculate the rotational error between the current Qhat and the new measured Qy
	m_Qtilde[0] = m_Qhat[0]*m_Qy[0] + m_Qhat[1]*m_Qy[1] + m_Qhat[2]*m_Qy[2] + m_Qhat[3]*m_Qy[3];
	m_Qtilde[1] = m_Qhat[0]*m_Qy[1] - m_Qhat[1]*m_Qy[0] - m_Qhat[2]*m_Qy[3] + m_Qhat[3]*m_Qy[2];
	m_Qtilde[2] = m_Qhat[0]*m_Qy[2] + m_Qhat[1]*m_Qy[3] - m_Qhat[2]*m_Qy[0] - m_Qhat[3]*m_Qy[1];
	m_Qtilde[3] = m_Qhat[0]*m_Qy[3] - m_Qhat[1]*m_Qy[2] + m_Qhat[2]*m_Qy[1] - m_Qhat[3]*m_Qy[0];

	// Calculate the angular velocity feedback term required to act in the direction of reducing Qtilde
	double wscale = 2.0 * Kp * m_Qtilde[0];
	m_w[0] = wscale * m_Qtilde[1];
	m_w[1] = wscale * m_Qtilde[2];
	m_w[2] = wscale * m_Qtilde[3];

	// Update the estimated gyro bias (trapezoidal integration of -Ki*w)
	double bscale = 0.5 * dt / Ti;
	m_bhat[0] -= bscale*(m_w[0] + m_wold[0]);
	m_bhat[1] -= bscale*(m_w[1] + m_wold[1]);
	m_bhat[2] -= bscale*(m_w[2] + m_wold[2]);

	// Calculate the required (combined predictive/corrective) angular velocity to apply to our current attitude estimate
	m_omega[0] = gyroX - m_bhat[0] + m_w[0];
	m_omega[1] = gyroY - m_bhat[1] + m_w[1];
	m_omega[2] = gyroZ - m_bhat[2] + m_w[2];

	// Convert the calculated angular velocity into a quaternion velocity (the missing factor of 0.5 here has been taken into dscale below)
	m_dQ[0] = -m_Qhat[1]*m_omega[0] - m_Qhat[2]*m_omega[1] - m_Qhat[3]*m_omega[2];
	m_dQ[1] =  m_Qhat[0]*m_omega[0] + m_Qhat[2]*m_omega[2] - m_Qhat[3]*m_omega[1];
	m_dQ[2] =  m_Qhat[0]*m_omega[1] - m_Qhat[1]*m_omega[2] + m_Qhat[3]*m_omega[0];
	m_dQ[3] =  m_Qhat[0]*m_omega[2] + m_Qhat[1]*m_omega[1] - m_Qhat[2]*m_omega[0];

	// Update the attitude estimate using the calculated quaternion velocity (trapezoidal integration of dQ)
	double dscale = 0.25 * dt;
	m_Qhat[0] += dscale*(m_dQ[0] + m_dQold[0]);
	m_Qhat[1] += dscale*(m_dQ[1] + m_dQold[1]);
	m_Qhat[2] += dscale*(m_dQ[2] + m_dQold[2]);
	m_Qhat[3] += dscale*(m_dQ[3] + m_dQold[3]);

	// Renormalise the current attitude estimate
	double qscale = m_Qhat[0]*m_Qhat[0] + m_Qhat[1]*m_Qhat[1] + m_Qhat[2]*m_Qhat[2] + m_Qhat[3]*m_Qhat[3];
	if(qscale < QNORM_TOL_SQ) { reset(true); return; } // The quaternion is so far away from being normalised that something must be dreadfully wrong... (avoid potential division by zero below)
	qscale = 1.0 / std::sqrt(qscale);
	m_Qhat[0] *= qscale;
	m_Qhat[1] *= qscale;
	m_Qhat[2] *= qscale;
	m_Qhat[3] *= qscale;

	// Update the Euler angle representation of the current attitude estimate
	updateEuler();
}
void AttitudeEstimator::updateQy(double accX, double accY, double accZ, double magX, double magY, double magZ)
{
	// Note: This function uses the value of m_magCalib = (mtx,mty,mtz). Only the projection of this vector
	//       onto the global xG-yG plane is considered however, which is equivalent to (mtx,mty,0). As such,
	//       only the values of mtx and mty are used by this function, and m_magCalib is effectively taken
	//       to be (mtx,mty,0), irrespective of what the value of mtz is. The only difference this makes is
	//       in the comments, where for example ||m_magCalib|| is used as shorthand for sqrt(mtx^2 + mty^2).
	//       In actual fact, all that really matters about the m_magCalib vector is its yaw, because in the
	//       calculation of m_Qy that's all it's effectively reduced to. More precisely stated, only the
	//       value of atan2(mty,mtx) is important. The magnetometer readings do not affect any component of
	//       the resulting rotation other than the yaw about the measured acceleration vector!
	
	// Declare variables
	double dot, nacc, nxG, nyG, r, s, t;
	
	// Calculate the norm (squared) of the acc measurement
	nacc = accX*accX + accY*accY + accZ*accZ; // = ||acc||^2
	
	// If the acc norm is too close to zero then we actually have no idea where we are, so just set Qy to the unit quaternion and return
	if(nacc < ACC_TOL_SQ)
	{
		m_Qy[0] = 1.0; // w
		m_Qy[1] = 0.0; // x
		m_Qy[2] = 0.0; // y
		m_Qy[3] = 0.0; // z
		return;
	}
	
	// Define zGhat as the unit vector pointing in the direction of acc
	// Note: To machine precision the resulting zGhat will have unit norm, i.e. ||zGhat|| == 1
	nacc = std::sqrt(nacc);
	m_Ry[6] = accX/nacc; // zGhat -> x
	m_Ry[7] = accY/nacc; // zGhat -> y
	m_Ry[8] = accZ/nacc; // zGhat -> z
	
	// Project mag into the xG-yG plane using the formula mtilde := mag - dot(mag,zGhat)*zGhat
	// Note: To machine precision the resulting mtilde is perpendicular to zGhat, and ||mtilde|| = ||mag||*sin(angle(acc,mag)),
	//       where the angle is taken to be in the range [0,pi].
	dot = magX*m_Ry[6] + magY*m_Ry[7] + magZ*m_Ry[8]; // = dot(mag,zGhat)
	magX -= dot*m_Ry[6];                              // mtilde -> x
	magY -= dot*m_Ry[7];                              // mtilde -> y
	magZ -= dot*m_Ry[8];                              // mtilde -> z
	
	// Generate a second orthogonal basis vector for the xG-yG plane, complementary to mtilde, using the cross product (define X = m_base / Y = mtilde / Z = zGhat)
	// Note: To machine precision ||m_base|| == ||mtilde|| and dot(m_base,mtilde) == 0
	m_base[0] = magY*m_Ry[8] - magZ*m_Ry[7]; // m_base -> x
	m_base[1] = magZ*m_Ry[6] - magX*m_Ry[8]; // m_base -> y
	m_base[2] = magX*m_Ry[7] - magY*m_Ry[6]; // m_base -> z
	
	// Calculate orthogonal xG and yG such that mtilde is collinear with m_magCalib (a vector in xG-yG-zG coordinates) projected into the xG-yG plane
	// Note: To machine precision ||xG|| == ||yG|| == ||m_magCalib||*||mtilde|| == ||m_magCalib||*||mag||*sin(angle(acc,mag)), where the
	//       angle is taken to be in the range [0,pi]. Zero xG/yG arise iff m_magCalib is zero, or acc and mag are collinear.
	m_Ry[0] = m_magCalib[1]*m_base[0] + m_magCalib[0]*magX; // xG -> x
	m_Ry[1] = m_magCalib[1]*m_base[1] + m_magCalib[0]*magY; // xG -> y
	m_Ry[2] = m_magCalib[1]*m_base[2] + m_magCalib[0]*magZ; // xG -> z
	m_Ry[3] = m_magCalib[1]*magX - m_magCalib[0]*m_base[0]; // yG -> x
	m_Ry[4] = m_magCalib[1]*magY - m_magCalib[0]*m_base[1]; // yG -> y
	m_Ry[5] = m_magCalib[1]*magZ - m_magCalib[0]*m_base[2]; // yG -> z
	
	// Calculate the xG and yG vector (square) norms
	// Note: The calculated nxG and nyG should theoretically be identical.
	nxG = m_Ry[0]*m_Ry[0] + m_Ry[1]*m_Ry[1] + m_Ry[2]*m_Ry[2]; // = ||xG||^2
	nyG = m_Ry[3]*m_Ry[3] + m_Ry[4]*m_Ry[4] + m_Ry[5]*m_Ry[5]; // = ||yG||^2
	
	// Check whether the basis vector generation was successful (non-zero xG/yG)
	// Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
	if((nxG < XGYG_NORM_TOL_SQ) || (nyG < XGYG_NORM_TOL_SQ))
	{
		// Note: This IF block executes if the algorithm was unable to resolve the acc and mag measurements into a consistent and unambiguous
		//       3D rotation. Assuming the m_magCalib is not the zero vector, this occurs iff acc and mag are collinear. In this case the mag
		//       vector gives us no valid information and we are forced to discard it. Instead we now try to use the current orientation
		//       estimate m_Qhat to resolve the single acc vector into a complete 3D rotation that agrees "as much as possible" with m_Qhat.
		//       The code in this IF block assumes that m_Qhat is a unit quaternion. Within reason this code can deal with numeric deviations
		//       thereof, but eventually these act to reduce the numerical accuracy of the computed values. The terms "ZYX yaw" and "ZXY yaw"
		//       in the comments below refer to the yaw of a rotation (about the z-axis) in the case of ZYX and ZXY Euler angles respectively.
		
		// Calculate orthogonal xG and yG so that the ZYX yaw of m_Qy, the rotation defined by the resulting orthogonal basis xG-yG-zGhat, is equal to the ZYX yaw of m_Qhat
		// Note: The critical underlying observation behind this efficient calculation is that the ZYX yaw of a rotation rotates the global
		//       x-axis so that it is collinear with the [global] projection of the body-fixed (i.e. rotated) x-axis into the [global] xy-plane.
		//       As such, the problem becomes finding an xG such that xG projected into the xG-yG plane (== xG) is equal to xh (the global x-axis
		//       expressed in body-fixed coordinates if the body is at an orientation of m_Qhat) projected into the xG-yG plane. Hence we never
		//       actually need to calculate a ZYX yaw, and can instead just set xG to be exactly the projection of xh into the plane perpendicular
		//       to zGhat. If m_Qhat is a unit quaternion, to machine precision ||0.5*xh|| = 0.5 and ||xG|| = ||yG|| = 0.5*sin(angle(xh,zGhat)),
		//       where the angle is taken to be in the range [0,pi]. Zero xG/yG arise iff xh and zGhat/acc are collinear. yG is calculated as the
		//       cross product of zGhat and xG.
		m_Ry[0] = 0.5 - m_Qhat[2]*m_Qhat[2] - m_Qhat[3]*m_Qhat[3]; // 0.5*xh -> x
		m_Ry[1] = m_Qhat[1]*m_Qhat[2] - m_Qhat[3]*m_Qhat[0];       // 0.5*xh -> y
		m_Ry[2] = m_Qhat[1]*m_Qhat[3] + m_Qhat[2]*m_Qhat[0];       // 0.5*xh -> z
		dot = m_Ry[0]*m_Ry[6] + m_Ry[1]*m_Ry[7] + m_Ry[2]*m_Ry[8]; // = dot(0.5*xh,zGhat)
		m_Ry[0] -= dot*m_Ry[6];                                    // xG -> x
		m_Ry[1] -= dot*m_Ry[7];                                    // xG -> y
		m_Ry[2] -= dot*m_Ry[8];                                    // xG -> z
		m_Ry[3] = m_Ry[2]*m_Ry[7] - m_Ry[1]*m_Ry[8];               // yG -> x
		m_Ry[4] = m_Ry[0]*m_Ry[8] - m_Ry[2]*m_Ry[6];               // yG -> y
		m_Ry[5] = m_Ry[1]*m_Ry[6] - m_Ry[0]*m_Ry[7];               // yG -> z
		
		// Calculate the xG and yG vector (square) norms
		// Note: The calculated nxG and nyG should theoretically be identical.
		nxG = m_Ry[0]*m_Ry[0] + m_Ry[1]*m_Ry[1] + m_Ry[2]*m_Ry[2]; // = ||xG||^2
		nyG = m_Ry[3]*m_Ry[3] + m_Ry[4]*m_Ry[4] + m_Ry[5]*m_Ry[5]; // = ||yG||^2
		
		// Check whether the basis vector generation was successful (non-zero xG/yG)
		// Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
		if((nxG < XGYG_NORM_TOL_SQ) || (nyG < XGYG_NORM_TOL_SQ))
		{
			// Note: This IF block executes if the mag vector had to be discarded (i.e. if m_magCalib is zero, or acc and mag are collinear) and
			//       m_Qhat is such that xh is collinear with acc. That is, mag was discarded and our current attitude estimate places the global
			//       x-axis in the same (or 180 degree opposite) direction as the acc we just measured. Assuming m_Qhat is a unit quaternion, this
			//       can only happen if our estimate is *exactly* out by 90 degrees, and perchance in *exactly* the right direction. If m_Qhat
			//       deviates from unit norm by more than a negligible eps-ish amount however (which it should never do), then funkier and slightly
			//       counterintuitive things can happen that produce further special cases below. The xh (ZYX) and yh (ZXY) methods produce
			//       identical results for collinear zGhat and zh. As the angle between these two vectors increases, the xh and yh methods
			//       continuously (but only gradually) start to differ more and more in their output (in terms of global yaw only though, the zGhat
			//       vector always points directly opposite to the measured acc). As the angle between zGhat (opposite of measured acc) and zh
			//       (current estimate of the up direction in body-fixed coordinates) approaches 90 degrees, there are two singularities
			//       (corresponding to gimbal lock of the two different Euler angle conventions) that make the output of the xh and yh methods
			//       start to differ vastly and in an unstable way. For example, close to zGhat == xh the xh output becomes extremely sensitive to
			//       small variations in zGhat (and nxG/nyG tend to zero, which is the condition that is checked for by this IF block), while the
			//       yh output remains stable, and vice versa for zGhat == yh. We wish to have a continuous xG/yG output for as much of the domain
			//       as possible though, so we nominally always use xh (as it corresponds to matching global yaw in a sense consistent with our
			//       nominal ZYX Euler angle convention) unless it produces an effectively zero output, in which case we switch to yh, which in
			//       this neighbourhood is guaranteed to be stable (with the unit quaternion technical caveat). The switching is of course a
			//       locally discontinuous operation though, but we really have no choice here due to the nasty directional dependence of the
			//       singularities. So in summary, the important points to take home from all this are:
			//       1) This yh method/IF block will practically never execute, but it cannot be neglected.
			//       2) The singularities and regions of progressive instability in xG/yG do NOT depend on the absolute orientation of acc and
			//          m_Qhat at all (i.e. it doesn't matter if we're upright, upside-down or sideways), it depends only on the *relative*
			//          orientation/rotation between acc and m_Qhat. What's more, it can only even possibly be a problem if acc and m_Qhat
			//          disagree in what direction is up by almost exactly 90 degrees, and by coincidence *exactly* in the wrong direction.
			
			// Calculate orthogonal xG and yG so that the ZXY yaw of m_Qy, the rotation defined by the resulting orthogonal basis xG-yG-zGhat, is equal to the ZXY yaw of m_Qhat
			// Note: The critical underlying observation behind this efficient calculation is that the ZXY yaw of a rotation rotates the global
			//       y-axis so that it is collinear with the [global] projection of the body-fixed (i.e. rotated) y-axis into the [global] xy-plane.
			//       As such, the problem becomes finding a yG such that yG projected into the xG-yG plane (== yG) is equal to yh (the global y-axis
			//       expressed in body-fixed coordinates if the body is at an orientation of m_Qhat) projected into the xG-yG plane. Hence we never
			//       actually need to calculate a ZXY yaw, and can instead just set yG to be exactly the projection of yh into the plane perpendicular
			//       to zGhat. If m_Qhat is a unit quaternion, to machine precision ||0.5*yh|| = 0.5 and ||yG|| = ||xG|| = 0.5*sin(angle(yh,zGhat)),
			//       where the angle is taken to be in the range [0,pi]. Zero xG/yG arise iff yh and zGhat/acc are collinear. xG is calculated as the
			//       cross product of yG and zGhat.
			m_Ry[3] = m_Qhat[1]*m_Qhat[2] + m_Qhat[3]*m_Qhat[0];       // 0.5*yh -> x
			m_Ry[4] = 0.5 - m_Qhat[1]*m_Qhat[1] - m_Qhat[3]*m_Qhat[3]; // 0.5*yh -> y
			m_Ry[5] = m_Qhat[2]*m_Qhat[3] - m_Qhat[1]*m_Qhat[0];       // 0.5*yh -> z
			dot = m_Ry[3]*m_Ry[6] + m_Ry[4]*m_Ry[7] + m_Ry[5]*m_Ry[8]; // = dot(0.5*yh,zGhat)
			m_Ry[3] -= dot*m_Ry[6];                                    // yG -> x
			m_Ry[4] -= dot*m_Ry[7];                                    // yG -> y
			m_Ry[5] -= dot*m_Ry[8];                                    // yG -> z
			m_Ry[0] = m_Ry[4]*m_Ry[8] - m_Ry[5]*m_Ry[7];               // xG -> x
			m_Ry[1] = m_Ry[5]*m_Ry[6] - m_Ry[3]*m_Ry[8];               // xG -> y
			m_Ry[2] = m_Ry[3]*m_Ry[7] - m_Ry[4]*m_Ry[6];               // xG -> z
			
			// Calculate the xG and yG vector (square) norms
			// Note: The calculated nxG and nyG should theoretically be identical.
			nxG = m_Ry[0]*m_Ry[0] + m_Ry[1]*m_Ry[1] + m_Ry[2]*m_Ry[2]; // = ||xG||^2
			nyG = m_Ry[3]*m_Ry[3] + m_Ry[4]*m_Ry[4] + m_Ry[5]*m_Ry[5]; // = ||yG||^2
			
			// Check whether the basis vector generation was successful (non-zero xG/yG)
			// Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
			if((nxG < XGYG_NORM_TOL_SQ) || (nyG < XGYG_NORM_TOL_SQ))
			{
				// Note: Ok, so you're asking me why I'm even checking this case, seeing as it's impossible anyway, right? Right...?
				//       Well, almost. Somewhat surprisingly it *is* actually possible for the calculated xh and yh to turn out collinear, even though
				//       the corresponding expressions used to calculate them are orthogonal - as long as m_Qhat is a unit quaternion! That exactly is
				//       the catch. This entire function was written with the mindset that it should never ever break, even if it receives completely
				//       rubbish inputs. In that case it should simply output whatever it thinks is most appropriate, and *never* start performing
				//       divisions by zero or such. No matter what happens, this function *must* be guaranteed to return a valid non-Inf/NaN quaternion,
				//       or the effects of one bad call could ripple through and break the entire attitude estimator, which is not robust. I've managed
				//       to prove mathematicallly that the previously calculated xh and yh are collinear iff m_Qhat is a pure vector quaternion of norm
				//       1/sqrt(2). If, furthermore, the resulting collinear xh and yh also happen to be collinear with the measured acc, AND with the
				//       measured mag (or if m_magCalib is zero), then this case is invoked! If this is the case, then we know that m_Qhat is useless,
				//       and so we are forced to discard it also. In order to still be able to resolve our single acc measurement into a full 3D
				//       orientation, some extra (arbitrary) assumption is required. The two assumptions in use below amount to assumptions of zero yaw
				//       in terms of either ZYX yaw or ZXY yaw, with preference to the (nominal) former convention for Euler angles.
				
				// Check whether zGhat is collinear with (1,0,0), and produce an appropriate output either way, with the assumption of zero ZYX yaw or ZXY yaw
				// Note: If zGhat is collinear with (1,0,0), then the code in the first case doesn't work because (1,0,0) is the ZYX gimbal lock situation.
				//       In this case however we know that zGhat can't also be collinear with (0,1,0), the ZXY gimbal lock situation, so it is safe to
				//       calculate a new xG/yG pair based on a zero ZXY yaw assumption without any further checks.
				if((std::abs(m_Ry[7]) >= ZGHAT_ABS_TOL) || (std::abs(m_Ry[8]) >= ZGHAT_ABS_TOL)) // If zGhat is *not* collinear with (1,0,0)...
				{
					// Assume zero ZYX yaw: xG is the projection of (1,0,0) into the plane perpendicular to zGhat, yG is the cross product of zGhat and xG
					// Note: To machine precision ||xG|| = ||yG|| = sin(angle(zGhat,(1,0,0))), where the angle is taken to be in the range [0,pi].
					m_Ry[0] = 1.0 - m_Ry[6]*m_Ry[6]; // xG -> x
					m_Ry[1] = -m_Ry[6]*m_Ry[7];      // xG -> y
					m_Ry[2] = -m_Ry[6]*m_Ry[8];      // xG -> z
					m_Ry[3] = 0.0;                   // yG -> x
					m_Ry[4] = m_Ry[8];               // yG -> y
					m_Ry[5] = -m_Ry[7];              // yG -> z
				}
				else // If zGhat *is* collinear with (1,0,0), and hence not collinear with (0,1,0) (as it is a unit vector)...
				{
					// Assume zero ZXY yaw: yG is the projection of (0,1,0) into the plane perpendicular to zGhat, xG is the cross product of yG and zGhat
					// Note: To machine precision ||xG|| = ||yG|| = sin(angle(zGhat,(0,1,0))), where the angle is taken to be in the range [0,pi].
					//       This case is only invoked if m_Qhat is either exactly (0,+-1/sqrt(2),0,0) or (0,0,0,+-1/sqrt(2)), acc is non-zero and
					//       along the body-fixed x-axis, and mag is collinear with acc or m_magCalib is zero.
					m_Ry[0] = m_Ry[8];               // xG -> x
					m_Ry[1] = 0.0;                   // xG -> y
					m_Ry[2] = -m_Ry[6];              // xG -> z
					m_Ry[3] = -m_Ry[7]*m_Ry[6];      // yG -> x
					m_Ry[4] = 1.0 - m_Ry[7]*m_Ry[7]; // yG -> y
					m_Ry[5] = -m_Ry[7]*m_Ry[8];      // yG -> z
				}
				
				// Calculate the xG and yG vector (square) norms
				// Note: The calculated nxG and nyG should theoretically be identical.
				nxG = m_Ry[0]*m_Ry[0] + m_Ry[1]*m_Ry[1] + m_Ry[2]*m_Ry[2]; // = ||xG||^2
				nyG = m_Ry[3]*m_Ry[3] + m_Ry[4]*m_Ry[4] + m_Ry[5]*m_Ry[5]; // = ||yG||^2
			}
		}
	}

	// Normalise xG and yG to obtain an orthonormal basis (in conjunction with zGhat) that forms the rows of the orthogonal rotation matrix m_Ry
	// Note: The calculated orthonormal basis <xGhat, yGhat, zGhat> is placed in the rows of this matrix (as opposed to the columns) as we want
	//       the inverse (i.e. transpose) rotation for m_Ry. That is, the global to body-fixed frame rotation, not the body-fixed to global rotation.
	nxG = std::sqrt(nxG);
	nyG = std::sqrt(nyG);
	m_Ry[0] /= nxG; // xGhat -> x
	m_Ry[1] /= nxG; // xGhat -> y
	m_Ry[2] /= nxG; // xGhat -> z
	m_Ry[3] /= nyG; // yGhat -> x
	m_Ry[4] /= nyG; // yGhat -> y
	m_Ry[5] /= nyG; // yGhat -> z

	// Convert the rotation matrix m_Ry into the quaternion m_Qy
	// Note: m_Qy and -m_Qy are both valid and completely equivalent outputs here, so we have
	//       the freedom to arbitrarily choose the sign of *one* of the quaternion parameters.
	t = m_Ry[0] + m_Ry[4] + m_Ry[8];
	if(t >= 0.0)                                          // Option 1: Centred at identity rotation... [Condition ensures |w| >= 0.5, WLOG choose the sign w >= 0.5]
	{
		r = std::sqrt(1.0 + t);                           // = 2*|w|
		s = 0.5/r;                                        // = 1/(4*|w|)
		m_Qy[0] = 0.5*r;                                  // = |w|           = w*sgn(w) = w
		m_Qy[1] = s*(m_Ry[7] - m_Ry[5]);                  // = (4xw)/(4*|w|) = x*sgn(w) = x
		m_Qy[2] = s*(m_Ry[2] - m_Ry[6]);                  // = (4yw)/(4*|w|) = y*sgn(w) = y
		m_Qy[3] = s*(m_Ry[3] - m_Ry[1]);                  // = (4zw)/(4*|w|) = z*sgn(w) = z
	}
	else if((m_Ry[8] >= m_Ry[4]) && (m_Ry[8] >= m_Ry[0])) // Option 2: Centred at 180 deg z-rotation... [Conditions ensure |z| > 0.5, WLOG choose the sign z > 0.5]
	{
		r = std::sqrt(1.0 - m_Ry[0] - m_Ry[4] + m_Ry[8]); // = 2*|z|
		s = 0.5/r;                                        // = 1/(4*|z|)
		m_Qy[0] = s*(m_Ry[3] - m_Ry[1]);                  // = (4zw)/(4*|z|) = w*sgn(z) = w
		m_Qy[1] = s*(m_Ry[2] + m_Ry[6]);                  // = (4xz)/(4*|z|) = x*sgn(z) = x
		m_Qy[2] = s*(m_Ry[7] + m_Ry[5]);                  // = (4yz)/(4*|z|) = y*sgn(z) = y
		m_Qy[3] = 0.5*r;                                  // = |z|           = z*sgn(z) = z
	}
	else if(m_Ry[4] >= m_Ry[0])                           // Option 3: Centred at 180 deg y-rotation... [Conditions ensure |y| > 0.5, WLOG choose the sign y > 0.5]
	{
		r = std::sqrt(1.0 - m_Ry[0] + m_Ry[4] - m_Ry[8]); // = 2*|y|
		s = 0.5/r;                                        // = 1/(4*|y|)
		m_Qy[0] = s*(m_Ry[2] - m_Ry[6]);                  // = (4yw)/(4*|y|) = w*sgn(y) = w
		m_Qy[1] = s*(m_Ry[3] + m_Ry[1]);                  // = (4xy)/(4*|y|) = x*sgn(y) = x
		m_Qy[2] = 0.5*r;                                  // = |y|           = y*sgn(y) = y
		m_Qy[3] = s*(m_Ry[7] + m_Ry[5]);                  // = (4yz)/(4*|y|) = z*sgn(y) = z
	}
	else                                                  // Option 4: Centred at 180 deg x-rotation... [Conditions ensure |x| > 0.5, WLOG choose the sign x > 0.5]
	{
		r = std::sqrt(1.0 + m_Ry[0] - m_Ry[4] - m_Ry[8]); // = 2*|x|
		s = 0.5/r;                                        // = 1/(4*|x|)
		m_Qy[0] = s*(m_Ry[7] - m_Ry[5]);                  // = (4xw)/(4*|x|) = w*sgn(x) = w
		m_Qy[1] = 0.5*r;                                  // = |x|           = x*sgn(x) = x
		m_Qy[2] = s*(m_Ry[3] + m_Ry[1]);                  // = (4xy)/(4*|x|) = y*sgn(x) = y
		m_Qy[3] = s*(m_Ry[2] + m_Ry[6]);                  // = (4xz)/(4*|x|) = z*sgn(x) = z
	}

	// Note: Any deviations from being a unit quaternion (that might be experienced here due to the inaccuracies
	//       of floating point arithmetic) are pretty much irrelevant. This is firstly because they will only
	//       ever be extremely minor eps deviations, given the mathematical correctness of this algorithm, but also
	//       because any scaling in Qy is swallowed up by the Kp in the expression for m_w in the update()
	//       function anyway. This is why no quaternion normalisation step has been added here.
}
void AttitudeEstimator::updateEuler()
{
	// Note: The calculation of the theta Euler angle (second angle parameter = about Y = pitch)
	//       relies on the assumption that m_Qhat is a unit quaternion!
	//       The angle output ranges are:
	//           Yaw = phi = m_Ehat[0] is in (-pi,pi]
	//           Pitch = theta = m_Ehat[1] is in [-pi/2,pi/2]
	//           Roll = psi = m_Ehat[2] is in (-pi,pi]

	// Declare variables
	double tmp;

	// Calculate pitch
	tmp = 2.0*(m_Qhat[0]*m_Qhat[2] - m_Qhat[3]*m_Qhat[1]); // m_Qhat needs to be a unit quaternion for this!
	if(tmp >=  1.0) tmp =  1.0;
	if(tmp <= -1.0) tmp = -1.0;
	m_Ehat[1] = std::asin(tmp);

	// Calculate yaw
	tmp = 0.5 - m_Qhat[2]*m_Qhat[2];
	m_Ehat[0] = std::atan2(m_Qhat[0]*m_Qhat[3]+m_Qhat[1]*m_Qhat[2], tmp-m_Qhat[3]*m_Qhat[3]);

	// Calculate roll
	m_Ehat[2] = std::atan2(m_Qhat[0]*m_Qhat[1]+m_Qhat[2]*m_Qhat[3], tmp-m_Qhat[1]*m_Qhat[1]);
}
// EOF