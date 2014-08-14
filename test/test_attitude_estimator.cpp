// Unit testing of the attitude estimator
// File: test_attitude_estimator.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Header configuration defines
#define ATT_EST_PRIVATE_ARE_AVAILABLE // Uncomment this define to enable the additional tests that require all update functions, m_Qy and m_Qhat to be public
//#define ATT_EST_SPEED_TEST            // Uncomment this define to enable speed testing

// Defines
#define SPEED_TEST_ITS  1e8

// Includes
#include <cmath>
#include <cstdio>
#include <iostream>
#include <gtest/gtest.h>
#include "attitude_estimator.h" // Configured by ATT_EST_PRIVATE_ARE_AVAILABLE

// Namespaces
using namespace std;
using namespace stateestimation;

// Test: Test the reset/get/set characteristics of the attitude estimator
TEST(AttitudeEstimatorTest, test_reset_get_set)
{
	// Declare variables
	double q1[4] = {0.0}, q2[4] = {0.0};
	double v1[3] = {0.0}, v2[3] = {0.0};
	double Kp = 0.0, Ti = 0.0, KpQ = 0.0, TiQ = 0.0, norm = 0.0;
	AttitudeEstimator::AccMethodEnum method = (AttitudeEstimator::AccMethodEnum) -1;

	// Create an attitude estimator
	AttitudeEstimator Est(true); // Explicitly specify the default value of true for quickLearn...

	// Check the initial estimator state
	method = Est.getAccMethod();
	EXPECT_EQ(Est.ME_FUSED_YAW, method);
	Est.getAttitude(q1);
	EXPECT_DOUBLE_EQ(1.0, q1[0]);
	EXPECT_DOUBLE_EQ(0.0, q1[1]);
	EXPECT_DOUBLE_EQ(0.0, q1[2]);
	EXPECT_DOUBLE_EQ(0.0, q1[3]);
	EXPECT_DOUBLE_EQ(0.0, Est.eulerYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerRoll());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedRoll());
	EXPECT_TRUE(Est.fusedHemi());
	EXPECT_DOUBLE_EQ(0.0, Est.getLambda());
	Est.getMagCalib(v1);
	EXPECT_DOUBLE_EQ(1.0, v1[0]);
	EXPECT_DOUBLE_EQ(0.0, v1[1]);
	EXPECT_DOUBLE_EQ(0.0, v1[2]);
	Est.getPIGains(Kp, Ti, KpQ, TiQ);
	EXPECT_DOUBLE_EQ(2.20, Kp);
	EXPECT_DOUBLE_EQ(2.65, Ti);
	EXPECT_DOUBLE_EQ(10.0, KpQ);
	EXPECT_DOUBLE_EQ(1.25, TiQ);
	EXPECT_DOUBLE_EQ(3.0, Est.getQLTime());
	Est.getGyroBias(v1);
	EXPECT_DOUBLE_EQ(0.0, v1[0]);
	EXPECT_DOUBLE_EQ(0.0, v1[1]);
	EXPECT_DOUBLE_EQ(0.0, v1[2]);

	// Test modifying the acc-only resolution method
	Est.setAccMethod(Est.ME_ZYX_YAW);
	method = Est.getAccMethod();
	EXPECT_EQ(Est.ME_ZYX_YAW, method);
	Est.setAccMethod(Est.ME_FUSED_YAW);
	method = Est.getAccMethod();
	EXPECT_EQ(Est.ME_FUSED_YAW, method);
	Est.setAccMethod(Est.ME_ABS_FUSED_YAW);
	method = Est.getAccMethod();
	EXPECT_EQ(Est.ME_ABS_FUSED_YAW, method);
	Est.setAccMethod((AttitudeEstimator::AccMethodEnum) -1);
	method = Est.getAccMethod();
	EXPECT_EQ(Est.ME_FUSED_YAW, method);
	Est.setAccMethod(Est.ME_ZYX_YAW);
	method = Est.getAccMethod();
	EXPECT_EQ(Est.ME_ZYX_YAW, method);
	Est.setAccMethod(Est.ME_COUNT);
	method = Est.getAccMethod();
	EXPECT_EQ(Est.ME_FUSED_YAW, method);

	// Test modifying the bias estimate
	v2[0] = 2.1;
	v2[1] = 3.2;
	v2[2] = 4.3;
	Est.setGyroBias(v2);
	Est.getGyroBias(v1);
	EXPECT_DOUBLE_EQ(2.1, v1[0]);
	EXPECT_DOUBLE_EQ(3.2, v1[1]);
	EXPECT_DOUBLE_EQ(4.3, v1[2]);
	Est.setGyroBias(5.4, 4.3, 3.2);
	Est.getGyroBias(v1);
	EXPECT_DOUBLE_EQ(5.4, v1[0]);
	EXPECT_DOUBLE_EQ(4.3, v1[1]);
	EXPECT_DOUBLE_EQ(3.2, v1[2]);

	// Test modifying the lambda value
	Est.setLambda();
	EXPECT_DOUBLE_EQ(1.0, Est.getLambda());
	Est.resetLambda();
	EXPECT_DOUBLE_EQ(0.0, Est.getLambda());
	Est.setLambda(0.481);
	EXPECT_DOUBLE_EQ(0.481, Est.getLambda());

	// Test modifying the magnetometer calibration
	v2[0] = 1.2;
	v2[1] = 3.4;
	v2[2] = 5.6;
	Est.setMagCalib(v2);
	Est.getMagCalib(v1);
	EXPECT_DOUBLE_EQ(1.2, v1[0]);
	EXPECT_DOUBLE_EQ(3.4, v1[1]);
	EXPECT_DOUBLE_EQ(5.6, v1[2]);
	Est.setMagCalib(6.5, 4.3, 2.1);
	Est.getMagCalib(v1);
	EXPECT_DOUBLE_EQ(6.5, v1[0]);
	EXPECT_DOUBLE_EQ(4.3, v1[1]);
	EXPECT_DOUBLE_EQ(2.1, v1[2]);
	Est.setMagCalib(9.0, 0.0, 0.0);

	// Test modifying the configuration variables
	Est.setPIGains(4.00, 3.00, 1.70, 0.00);
	Est.getPIGains(Kp, Ti, KpQ, TiQ);
	EXPECT_DOUBLE_EQ(4.00, Kp);
	EXPECT_DOUBLE_EQ(3.00, Ti);
	EXPECT_DOUBLE_EQ(10.0, KpQ);
	EXPECT_DOUBLE_EQ(1.25, TiQ);
	Est.setPIGains(0.00, 7.40, 8.00, 3.00);
	Est.getPIGains(Kp, Ti, KpQ, TiQ);
	EXPECT_DOUBLE_EQ(4.00, Kp);
	EXPECT_DOUBLE_EQ(3.00, Ti);
	EXPECT_DOUBLE_EQ(8.00, KpQ);
	EXPECT_DOUBLE_EQ(3.00, TiQ);
	Est.setQLTime(2.0);
	EXPECT_DOUBLE_EQ(2.0, Est.getQLTime());

	// Test modifying the attitude estimate
	q2[0] = 1.2;
	q2[1] = 3.4;
	q2[2] = 5.6;
	q2[3] = 7.8;
	Est.setAttitude(q2);
	norm = sqrt(q2[0]*q2[0] + q2[1]*q2[1] + q2[2]*q2[2] + q2[3]*q2[3]);
	Est.getAttitude(q1);
	EXPECT_DOUBLE_EQ(q2[0]/norm, q1[0]);
	EXPECT_DOUBLE_EQ(q2[1]/norm, q1[1]);
	EXPECT_DOUBLE_EQ(q2[2]/norm, q1[2]);
	EXPECT_DOUBLE_EQ(q2[3]/norm, q1[3]);
	EXPECT_NE(0.0, Est.eulerYaw());
	EXPECT_NE(0.0, Est.eulerPitch());
	EXPECT_NE(0.0, Est.eulerRoll());
	EXPECT_NE(0.0, Est.fusedYaw());
	EXPECT_NE(0.0, Est.fusedPitch());
	EXPECT_NE(0.0, Est.fusedRoll());
	Est.setAttitude(0.0, 0.0, 0.0, 0.0);
	Est.getAttitude(q1);
	EXPECT_DOUBLE_EQ(1.0, q1[0]);
	EXPECT_DOUBLE_EQ(0.0, q1[1]);
	EXPECT_DOUBLE_EQ(0.0, q1[2]);
	EXPECT_DOUBLE_EQ(0.0, q1[3]);
	EXPECT_DOUBLE_EQ(0.0, Est.eulerYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerRoll());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedRoll());
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitude(8.7, 6.5, 4.3, 2.1);
	norm = sqrt(8.7*8.7 + 6.5*6.5 + 4.3*4.3 + 2.1*2.1);
	Est.getAttitude(q1);
	EXPECT_DOUBLE_EQ(8.7/norm, q1[0]);
	EXPECT_DOUBLE_EQ(6.5/norm, q1[1]);
	EXPECT_DOUBLE_EQ(4.3/norm, q1[2]);
	EXPECT_DOUBLE_EQ(2.1/norm, q1[3]);
	EXPECT_NE(0.0, Est.eulerYaw());
	EXPECT_NE(0.0, Est.eulerPitch());
	EXPECT_NE(0.0, Est.eulerRoll());
	EXPECT_NE(0.0, Est.fusedYaw());
	EXPECT_NE(0.0, Est.fusedPitch());
	EXPECT_NE(0.0, Est.fusedRoll());
	Est.setAttitudeEuler(1.4, -0.5, 2.7);
	EXPECT_DOUBLE_EQ(1.4, Est.eulerYaw());
	EXPECT_DOUBLE_EQ(-0.5, Est.eulerPitch());
	EXPECT_DOUBLE_EQ(2.7, Est.eulerRoll());
	Est.setAttitudeFused(0.6, -0.4, 0.7, false);
	EXPECT_DOUBLE_EQ(0.6, Est.fusedYaw());
	EXPECT_DOUBLE_EQ(-0.4, Est.fusedPitch());
	EXPECT_DOUBLE_EQ(0.7, Est.fusedRoll());
	EXPECT_FALSE(Est.fusedHemi());

	// Check the reset function does its job
	Est.reset(false);
	Est.getAttitude(q1);
	EXPECT_DOUBLE_EQ(1.0, q1[0]);
	EXPECT_DOUBLE_EQ(0.0, q1[1]);
	EXPECT_DOUBLE_EQ(0.0, q1[2]);
	EXPECT_DOUBLE_EQ(0.0, q1[3]);
	EXPECT_DOUBLE_EQ(0.0, Est.eulerYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerRoll());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedRoll());
	EXPECT_TRUE(Est.fusedHemi());
	EXPECT_DOUBLE_EQ(1.0, Est.getLambda());
	Est.getGyroBias(v1);
	EXPECT_DOUBLE_EQ(0.0, v1[0]);
	EXPECT_DOUBLE_EQ(0.0, v1[1]);
	EXPECT_DOUBLE_EQ(0.0, v1[2]);

	// Check that the configuration variables and magnetometer calibration were left untouched by the reset
	Est.getMagCalib(v1);
	EXPECT_DOUBLE_EQ(9.0, v1[0]);
	EXPECT_DOUBLE_EQ(0.0, v1[1]);
	EXPECT_DOUBLE_EQ(0.0, v1[2]);
	Est.getPIGains(Kp, Ti, KpQ, TiQ);
	EXPECT_DOUBLE_EQ(4.00, Kp);
	EXPECT_DOUBLE_EQ(3.00, Ti);
	EXPECT_DOUBLE_EQ(8.00, KpQ);
	EXPECT_DOUBLE_EQ(3.00, TiQ);
	EXPECT_DOUBLE_EQ(2.0, Est.getQLTime());

	// Check that resetAll resets everything it should (more than reset)
	Est.resetAll(true);
	EXPECT_DOUBLE_EQ(1.0, q1[0]);
	EXPECT_DOUBLE_EQ(0.0, q1[1]);
	EXPECT_DOUBLE_EQ(0.0, q1[2]);
	EXPECT_DOUBLE_EQ(0.0, q1[3]);
	EXPECT_DOUBLE_EQ(0.0, Est.eulerYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.eulerRoll());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedYaw());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedPitch());
	EXPECT_DOUBLE_EQ(0.0, Est.fusedRoll());
	EXPECT_TRUE(Est.fusedHemi());
	EXPECT_DOUBLE_EQ(0.0, Est.getLambda());
	Est.getGyroBias(v1);
	EXPECT_DOUBLE_EQ(0.0, v1[0]);
	EXPECT_DOUBLE_EQ(0.0, v1[1]);
	EXPECT_DOUBLE_EQ(0.0, v1[2]);
	Est.getMagCalib(v1);
	EXPECT_DOUBLE_EQ(1.0, v1[0]);
	EXPECT_DOUBLE_EQ(0.0, v1[1]);
	EXPECT_DOUBLE_EQ(0.0, v1[2]);
	Est.getPIGains(Kp, Ti, KpQ, TiQ);
	EXPECT_DOUBLE_EQ(2.20, Kp);
	EXPECT_DOUBLE_EQ(2.65, Ti);
	EXPECT_DOUBLE_EQ(10.0, KpQ);
	EXPECT_DOUBLE_EQ(1.25, TiQ);
	EXPECT_DOUBLE_EQ(3.0, Est.getQLTime());
}

// Test: Test the correctness of the setAttitudeEuler() function
TEST(AttitudeEstimatorTest, test_setAttitudeEuler) // Note: setAttitudeEuler() internally has a conversion to quaternions and back, where the quaternion is normalised in-between
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Case: Normal Euler angles
	Est.setAttitudeEuler(0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.0, Est.eulerYaw(), 1e-11);
	EXPECT_NEAR( 0.0, Est.eulerPitch(), 1e-11);
	EXPECT_NEAR( 0.0, Est.eulerRoll(), 1e-11);
	Est.setAttitudeEuler(0.5, -0.5, 2.8);
	EXPECT_NEAR( 0.5, Est.eulerYaw(), 1e-11);
	EXPECT_NEAR(-0.5, Est.eulerPitch(), 1e-11);
	EXPECT_NEAR( 2.8, Est.eulerRoll(), 1e-11);
	Est.setAttitudeEuler(1.9, -1.1, -2.3);
	EXPECT_NEAR( 1.9, Est.eulerYaw(), 1e-11);
	EXPECT_NEAR(-1.1, Est.eulerPitch(), 1e-11);
	EXPECT_NEAR(-2.3, Est.eulerRoll(), 1e-11);
	Est.setAttitudeEuler(-0.2, 1.3, 0.4);
	EXPECT_NEAR(-0.2, Est.eulerYaw(), 1e-11);
	EXPECT_NEAR( 1.3, Est.eulerPitch(), 1e-11);
	EXPECT_NEAR( 0.4, Est.eulerRoll(), 1e-11);
	Est.setAttitudeEuler(-2.6, 0.1, -0.9);
	EXPECT_NEAR(-2.6, Est.eulerYaw(), 1e-11);
	EXPECT_NEAR( 0.1, Est.eulerPitch(), 1e-11);
	EXPECT_NEAR(-0.9, Est.eulerRoll(), 1e-11);

	// Case: Near-singular Euler angles
	Est.setAttitudeEuler(0.7, 1.57079, -0.3);
	EXPECT_NEAR( 0.70000, Est.eulerYaw(), 1e-9);
	EXPECT_NEAR( 1.57079, Est.eulerPitch(), 1e-9);
	EXPECT_NEAR(-0.30000, Est.eulerRoll(), 1e-9);
	Est.setAttitudeEuler(1.8, -1.57079, 3.0);
	EXPECT_NEAR( 1.80000, Est.eulerYaw(), 1e-9);
	EXPECT_NEAR(-1.57079, Est.eulerPitch(), 1e-9);
	EXPECT_NEAR( 3.00000, Est.eulerRoll(), 1e-9);
}

// Test: Test the correctness of the setAttitudeFused() function
TEST(AttitudeEstimatorTest, test_setAttitudeFused) // Note: setAttitudeFused() internally has a conversion to quaternions and back, where the quaternion is normalised in-between
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Case: Normal fused angles
	Est.setAttitudeFused(0.0, 0.0, 0.0, true);
	EXPECT_NEAR( 0.0, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 0.0, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 0.0, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitudeFused(-1.2389912679385777, 0.4504108626120746, 0.2309101949807438, true);
	EXPECT_NEAR(-1.2389912679385777, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 0.4504108626120746, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 0.2309101949807438, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitudeFused(-1.1245333812991449, -1.1152938107481036, -0.1505999801369221, false);
	EXPECT_NEAR(-1.1245333812991449, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-1.1152938107481036, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR(-0.1505999801369221, Est.fusedRoll(), 1e-14);
	EXPECT_FALSE(Est.fusedHemi());
	Est.setAttitudeFused(-2.9940777491210464, -0.0180413873657193, 1.0164835253856090, false);
	EXPECT_NEAR(-2.9940777491210464, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-0.0180413873657193, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 1.0164835253856090, Est.fusedRoll(), 1e-14);
	EXPECT_FALSE(Est.fusedHemi());
	Est.setAttitudeFused(-0.8827575035984855, -1.0299638573151797, 0.2067085358389749, true);
	EXPECT_NEAR(-0.8827575035984855, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-1.0299638573151797, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 0.2067085358389749, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());

	// Case: Invalid fused angles (ones that fail the sine sum criterion)
	Est.setAttitudeFused(-0.2909856257679569, -1.1630855231035317, -0.6314208789414353, false);
	EXPECT_NEAR(-0.2909856257679569, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-0.9993590078458838, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR(-0.5714373189490128, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitudeFused(-1.7489003673032701, 1.1472271331811683, -0.8937378526800521, true);
	EXPECT_NEAR(-1.7489003673032701, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 0.8634210833398374, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR(-0.7073752434550591, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitudeFused(2.4583008656951644, -1.3607318026738779, 0.9501187069481475, false);
	EXPECT_NEAR( 2.4583008656951644, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-0.8769820089949256, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 0.6938143177999709, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitudeFused(-1.5523060420811190, 0.5956084196409952, 1.4156676884054307, true);
	EXPECT_NEAR(-1.5523060420811190, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 0.5164300632429631, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 1.0543662635519337, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());

}

// Test: Test the correctness of the updateEuler() function
TEST(AttitudeEstimatorTest, test_updateEuler) // Note: updateEuler() is called implicitly in retrieving the Euler angle components
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Set the current attitude estimate and magnetometer calibration
	Est.setMagCalib(1.0, 0.0, 0.0);

	// Case: Normal Euler angles
	Est.setAttitude(0.592552850843098, 0.707894251703962, 0.111054036045993, 0.368013380789475);
	EXPECT_NEAR( 0.7, Est.eulerYaw(), 1e-14);
	EXPECT_NEAR(-0.4, Est.eulerPitch(), 1e-14);
	EXPECT_NEAR( 1.6, Est.eulerRoll(), 1e-14);
	Est.setAttitude(0.310125868152194, -0.652539210120707, 0.620265075419198, 0.305427178510867);
	EXPECT_NEAR(-1.5, Est.eulerYaw(), 1e-14);
	EXPECT_NEAR( 0.9, Est.eulerPitch(), 1e-14);
	EXPECT_NEAR(-3.1, Est.eulerRoll(), 1e-14);

	// Case: Near-singular Euler angles (more numerical issues with asin/atan2 => looser verification tolerances)
	Est.setAttitude(-0.126037693525368, -0.695781316126912, -0.126040295812392, 0.695784955119778);
	EXPECT_NEAR( 2.70000, Est.eulerYaw(), 1e-9);
	EXPECT_NEAR( 1.57079, Est.eulerPitch(), 1e-9);
	EXPECT_NEAR(-0.80000, Est.eulerRoll(), 1e-9);
	Est.setAttitude(0.439544782044344, -0.553893538424300, -0.439544465585940, -0.553898000937147);
	EXPECT_NEAR(-2.40000, Est.eulerYaw(), 1e-9);
	EXPECT_NEAR(-1.57079, Est.eulerPitch(), 1e-9);
	EXPECT_NEAR( 0.60000, Est.eulerRoll(), 1e-9);
}

// Test: Test the correctness of the updateFused() function
TEST(AttitudeEstimatorTest, test_updateFused) // Note: updateFused() is called implicitly in retrieving the fused angle components
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Set the current attitude estimate and magnetometer calibration
	Est.setMagCalib(1.0, 0.0, 0.0);

	// Case: Normal fused angles
	Est.setAttitude(0.4821785020739293, 0.5681941774354589, 0.4837559843360982, 0.4589546998181812);
	EXPECT_NEAR( 1.5214534918546789, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-0.0550651280646345, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 1.4441114693495201, Est.fusedRoll(), 1e-14);
	EXPECT_FALSE(Est.fusedHemi());
	Est.setAttitude(-0.5747725522069820, 0.6113337041221015, -0.5417310040241355, -0.0493470841370070);
	EXPECT_NEAR( 0.1712899139689803, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 0.7519705909187845, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR(-0.7066504864647706, Est.fusedRoll(), 1e-14);
	EXPECT_FALSE(Est.fusedHemi());
	Est.setAttitude(-0.3208918327799606, 0.3458339031220823, 0.4891541368196635, 0.7335908761282912);
	EXPECT_NEAR(-2.3168957445381952, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-0.9637426788118773, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 0.5186722733459991, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitude(0.6233964573859717, 0.5317992944741803, -0.3207546279525302, 0.4750608760594984);
	EXPECT_NEAR( 1.3023404766264970, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-1.1318234423525451, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 0.3664331247282780, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());

	// Case: Fused angles at the crossover between the positive and negative z hemispheres
	Est.setAttitude(0.7064483840022956, -0.6632516485507483, 0.2451473908572840, 0.0305071826153391);
	EXPECT_NEAR( 0.0863141334826512, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 0.3971978161591546, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR(-1.1735985106357418, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitude(0.3183170816271715, -0.7070179228893456, 0.0112096705230417, -0.6314065532953872);
	EXPECT_NEAR(-2.2076849618654277, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR(-1.0879889487677572, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR(-0.4828073780271395, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitude(0.0871006265553665, 0.6107171579599902, -0.3564050406114823, -0.7017217973340023);
	EXPECT_NEAR(-2.8946075869572829, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 0.9190419991352411, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR( 0.6517543276596556, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
	Est.setAttitude(0.6912675571926681, 0.0400591627670449, 0.7059711454514606, -0.1488259709904069);
	EXPECT_NEAR(-0.4241148971008855, Est.fusedYaw(), 1e-14);
	EXPECT_NEAR( 1.4154214376340313, Est.fusedPitch(), 1e-14);
	EXPECT_NEAR(-0.1553748891608649, Est.fusedRoll(), 1e-14);
	EXPECT_TRUE(Est.fusedHemi());
}

// Test: Test the correctness of the attitude estimator
TEST(AttitudeEstimatorTest, test_update)
{
	// Declare variables
	double q[4];
	double b[3], mt[3];
	double Kp, Ti, KpQ, TiQ;

	// Create an attitude estimator
	AttitudeEstimator Est(false);

	// Initialise the attitude estimator
	Est.setAttitude(1.0, 0.0, 0.0, 0.0);
	Est.setGyroBias(0.0, 0.5, 1.0);
	Est.setMagCalib(1.0, 0.0, 0.0);
	Est.setPIGains(5.00, 1.50, 10.0, 1.25);

	// Verify the magnetometer calibration and PI gains
	Est.getMagCalib(mt);
	EXPECT_NEAR(1.0, mt[0], 1e-15);
	EXPECT_NEAR(0.0, mt[1], 1e-15);
	EXPECT_NEAR(0.0, mt[2], 1e-15);
	Est.getPIGains(Kp, Ti, KpQ, TiQ);
	EXPECT_DOUBLE_EQ(5.00, Kp);
	EXPECT_DOUBLE_EQ(1.50, Ti);
	EXPECT_DOUBLE_EQ(10.0, KpQ);
	EXPECT_DOUBLE_EQ(1.25, TiQ);

	// Check that our initial estimator state is correct
	Est.getAttitude(q);
	EXPECT_NEAR(1.0, q[0], 1e-15);
	EXPECT_NEAR(0.0, q[1], 1e-15);
	EXPECT_NEAR(0.0, q[2], 1e-15);
	EXPECT_NEAR(0.0, q[3], 1e-15);
	Est.getGyroBias(b);
	EXPECT_NEAR(0.0, b[0], 1e-15);
	EXPECT_NEAR(0.5, b[1], 1e-15);
	EXPECT_NEAR(1.0, b[2], 1e-15);

	// Define the update function inputs
	double dt = 0.02;
	double acc[3]  = {-4.102821582419913e-01,  7.575655021799962e-01, 6.712045437704806e+00};
	double gyro[3] = { 6.403921931759489e-01,  9.360723675810693e-01, 7.894952671157460e-01};
	double mag[3]  = {-1.151522662792782e+00, -1.118054993770523e+00, 5.233922602069686e-01};

	// Run the estimation and print the outputs
	for(int k = 0; k < 500; k++)
	{
		// Perform the update
		Est.update(dt, gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);

		// Retrieve the new estimates
		Est.getAttitude(q);
		Est.getGyroBias(b);

		// Verify a couple of estimates, spread evenly throughout the estimation run
		if(k == 99)
		{
			EXPECT_NEAR( 3.4292612121324739e-01, q[0], 1e-15);
			EXPECT_NEAR(-1.9209071503758710e-02, q[1], 1e-15);
			EXPECT_NEAR( 8.8186908413074208e-02, q[2], 1e-15);
			EXPECT_NEAR( 9.3501644699232156e-01, q[3], 1e-15);
			EXPECT_NEAR( 5.0366756023736658e-01, b[0], 1e-15);
			EXPECT_NEAR( 7.2184655689754496e-01, b[1], 1e-15);
			EXPECT_NEAR( 3.4812977832258141e-01, b[2], 1e-15);
		}
		if(k == 199)
		{
			EXPECT_NEAR( 3.8226606451658535e-01, q[0], 1e-15);
			EXPECT_NEAR(-8.6435175083959797e-03, q[1], 1e-15);
			EXPECT_NEAR( 6.8603975753226451e-02, q[2], 1e-15);
			EXPECT_NEAR( 9.2146157816532670e-01, q[3], 1e-15);
			EXPECT_NEAR( 6.1223321502854988e-01, b[0], 1e-15);
			EXPECT_NEAR( 8.9189462536685826e-01, b[1], 1e-15);
			EXPECT_NEAR( 6.9850786079300486e-01, b[2], 1e-15);
		}
		if(k == 299)
		{
			EXPECT_NEAR( 3.9026384986863527e-01, q[0], 1e-15);
			EXPECT_NEAR(-6.4499730438504254e-03, q[1], 1e-15);
			EXPECT_NEAR( 6.4564403996416844e-02, q[2], 1e-15);
			EXPECT_NEAR( 9.1841382996448406e-01, q[3], 1e-15);
			EXPECT_NEAR( 6.3458476919266138e-01, b[0], 1e-15);
			EXPECT_NEAR( 9.2696127681439977e-01, b[1], 1e-15);
			EXPECT_NEAR( 7.7073028932131382e-01, b[2], 1e-15);
		}
		if(k == 399)
		{
			EXPECT_NEAR( 3.9190829854563736e-01, q[0], 1e-15);
			EXPECT_NEAR(-5.9976183670213913e-03, q[1], 1e-15);
			EXPECT_NEAR( 6.3730744907061473e-02, q[2], 1e-15);
			EXPECT_NEAR( 9.1777464895178817e-01, q[3], 1e-15);
			EXPECT_NEAR( 6.3919442338859822e-01, b[0], 1e-15);
			EXPECT_NEAR( 9.3419322300964502e-01, b[1], 1e-15);
			EXPECT_NEAR( 7.8562502731071371e-01, b[2], 1e-15);
		}
		if(k == 499)
		{
			EXPECT_NEAR( 3.9224726363482920e-01, q[0], 1e-15);
			EXPECT_NEAR(-5.9043191047627097e-03, q[1], 1e-15);
			EXPECT_NEAR( 6.3558774382524219e-02, q[2], 1e-15);
			EXPECT_NEAR( 9.1764236246257147e-01, q[3], 1e-15);
			EXPECT_NEAR( 6.4014515495033797e-01, b[0], 1e-15);
			EXPECT_NEAR( 9.3568479682624373e-01, b[1], 1e-15);
			EXPECT_NEAR( 7.8869703594960638e-01, b[2], 1e-15);
		}
	}
}

//
// Special testing
//

// Only execute if speed testing is desired
#ifdef ATT_EST_SPEED_TEST

// Run a certain number of cycles on a given estimator
void executeCycles(AttitudeEstimator& Est, int N, bool useMag = true)
{
	// Declare variables
	double magX, magY, magZ;
	double q[4] = {0};

	// Set the magnetometer values
	if(useMag)
	{
		magX = -0.4;
		magY = 0.7;
		magZ = -0.1;
	}
	else
	{
		magX = 0.0;
		magY = 0.0;
		magZ = 0.0;
	}
	
	// Execute the required number of cycles of the estimator
	for(int i = 0; i < N; i++)
	{
		Est.update(0.010, 0.0, 0.05, -0.05, 0.4, -0.2, -6.0, magX, magY, magZ);
		Est.getAttitude(q);
	}
}

// Test: Test the execution time of the magnitude method
TEST(AttitudeEstimatorTest, test_speedMag)
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Set up the estimator
	Est.setAccMethod(Est.ME_FUSED_YAW);
	Est.setAttitude(0.4, 0.7, -0.3, -0.5);
	Est.setMagCalib(1.0, 0.0, 0.0);
	executeCycles(Est, SPEED_TEST_ITS, true);
}

// Test: Test the execution time of the fused yaw method
TEST(AttitudeEstimatorTest, test_speedFusedYaw)
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Set up the estimator
	Est.setAccMethod(Est.ME_FUSED_YAW);
	Est.setAttitude(0.4, 0.7, -0.3, -0.5);
	Est.setMagCalib(0.0, 0.0, 0.0);
	executeCycles(Est, SPEED_TEST_ITS, false);
}

// Test: Test the execution time of the ZYX yaw method
TEST(AttitudeEstimatorTest, test_speedZYXYaw)
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Set up the estimator
	Est.setAccMethod(Est.ME_ZYX_YAW);
	Est.setAttitude(0.4, 0.7, -0.3, -0.5);
	Est.setMagCalib(0.0, 0.0, 0.0);
	executeCycles(Est, SPEED_TEST_ITS, false);
}

#endif /* ATT_EST_SPEED_TEST */

// Only execute if private class members are available for testing
#ifdef ATT_EST_PRIVATE_ARE_AVAILABLE

// Test: Test the correctness of the updateQy() function
TEST(AttitudeEstimatorTest, test_updateQy) // Note: For this test to compile the updateQy() function, m_Qy and m_Qhat need to be made public (only ever do this temporarily)
{
	// Create an attitude estimator
	AttitudeEstimator Est;

	// Note: If some of the quaternion tests here fail, the first thing that you should check is whether its actually correct,
	//       just the wrong sign. i.e. If we're testing for (1,0,0,0) and we get (-1,0,0,0) back, then the unit test will fail,
	//       even though a correct output was returned.

	//
	// Test Fused Yaw Method
	//

	// Change the acc-only resolution method
	Est.resetAll();
	Est.setAccMethod(Est.ME_FUSED_YAW);

	// Set the current attitude estimate and magnetometer calibration
	Est.setAttitude(1.0, 0.0, 0.0, 0.0);
	Est.setMagCalib(1.0, 0.0, 0.0);

	// Case: Acc is zero
	Est.updateQy(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
	EXPECT_NEAR(1.0, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[3], 1e-15);

	// Case: Acc and mag are both valid and unambiguous (m_Qhat should be irrelevant)
	Est.updateQy(0.0, 0.0, 5.0, 3.0, 0.0, 0.0);
	EXPECT_NEAR(1.0, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[3], 1e-15);
	Est.updateQy(-0.2, 0.4, 1.0, -1.2, 0.6, -0.7);
	EXPECT_NEAR(-0.261154630414682, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.139805215990429, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.154980763303808, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.942461523671184, Est.m_Qy[3], 1e-15);
	Est.setMagCalib(0.5, 1.3, -0.8);
	Est.updateQy(1.97, -4.93, -2.18, 0.65, -2.37, 1.14);
	EXPECT_NEAR( 0.5031410611323102, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.8289865209174649, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.0519991639383328, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.2385927653754926, Est.m_Qy[3], 1e-15);

	// Prevent mag from being used in the subsequent tests
	Est.setMagCalib(0.0, 0.0, 0.0);

	// Case: Fused angles representation of H with respect to G is not in singularity (normal case)
	Est.setAttitude(0.7872358201641929, -0.2225070728459767, -0.0727954675120195, -0.5704832915113571);
	Est.updateQy(0.1487586731012882, -1.0318259348182854, 0.0520870133587073, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.6405374228953400, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.6490624955723698, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.2318772906953779, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.3386186775293635, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.0954283450355833, 0.7576104902164565, 0.5540402995009882, -0.3316008483200932);
	Est.updateQy(0.0758641771452659, 0.1295823593777883, 0.5303498834750111, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.5651321197459419, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.0106502828191484, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(-0.1371028890010138, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.8134586999564921, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.2261653189676135, 0.8102974724140080, -0.5265224232161255, -0.1226433550844352);
	Est.updateQy(1.0683744235889208, 0.6702037497975503, -1.2053860886134233, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.3694091065234676, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.7255253534421092, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(-0.5648710729361709, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.1344267253568828, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.2124070617737524, -0.8257433870186585, 0.2402744656478841, 0.4640035345352651);
	Est.updateQy(0.2234554384126400, -0.7510358746009479, 0.5507195208552991, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.8301785872662097, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.3668826991417991, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(-0.2790864373533426, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.3135464220821645, Est.m_Qy[3], 1e-15);
	Est.setAttitude(-0.5170315313926657, -0.3618330389030689, -0.5052351757087615, 0.5886362754116177);
	Est.updateQy(0.0060329438221631, -0.0119560450268350, 0.0132053470655010, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.3457637147069603, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.2903157038678429, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(-0.2542677246050404, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.8552848472315745, Est.m_Qy[3], 1e-15);

	// Case: Fused angles representation of H with respect to G is in singularity (acc is *exactly* opposite to what we expect to have)
	Est.setAttitude(0.1315050313554053, 0.0628385160278410, 0.5635331902597460, 0.8131347312145174);
	Est.updateQy(0.0460225400289575, -0.9329839804258333, -0.3569633287578298, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.0628385160280403, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.1315050313552642, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.8131347312144415, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.5635331902598663, Est.m_Qy[3], 1e-15);
	Est.setAttitude(-0.5996819528906139, -0.1391235269691680, 0.7878730512597646, -0.0168004380772317);
	Est.updateQy(-0.9496211724184447, -0.1403865118709524, 0.2801986013156501, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.1391235269691764, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.5996819528905156, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.0168004380772349, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.7878730512598381, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.7445714456406411, -0.0297643106282695, -0.0256722525065951, -0.6663845613462749);
	Est.updateQy(-0.0778986064867350, 0.0101081261354275, -0.9969100425276831, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.0297643106282849, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.7445714456406466, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.6663845613462737, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.0256722525064528, Est.m_Qy[3], 1e-15);
	Est.setAttitude(-0.5282105669075481, 0.4966906285813930, -0.0198348597853515, -0.6884029305747638);
	Est.updateQy(0.7048005336720857, 0.4974057257937632, -0.5058099956333281, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.4966906285813809, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.5282105669075945, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.6884029305747393, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.0198348597852607, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.1972171024904266, -0.2741338695282224, 0.0214845912999399, 0.9410071457749077);
	Est.updateQy(0.5243981179390351, 0.0676934670115582, -0.8487780678285322, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.2741338695283436, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.1972171024904762, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.9410071457748665, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.0214845912997490, Est.m_Qy[3], 1e-15);

	//
	// Test Absolute Fused Yaw Method
	//

	// Change the acc-only resolution method
	Est.resetAll();
	Est.setAccMethod(Est.ME_ABS_FUSED_YAW);

	// Set the current attitude estimate and magnetometer calibration
	Est.setAttitude(1.0, 0.0, 0.0, 0.0);
	Est.setMagCalib(1.0, 0.0, 0.0);

	// Case: Acc is zero
	Est.updateQy(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
	EXPECT_NEAR(1.0, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[3], 1e-15);

	// Case: Acc and mag are both valid and unambiguous (m_Qhat should be irrelevant)
	Est.updateQy(0.0, 0.0, 5.0, 3.0, 0.0, 0.0);
	EXPECT_NEAR(1.0, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[3], 1e-15);
	Est.updateQy(-0.2, 0.4, 1.0, -1.2, 0.6, -0.7);
	EXPECT_NEAR(-0.261154630414682, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.139805215990429, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.154980763303808, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.942461523671184, Est.m_Qy[3], 1e-15);
	Est.setMagCalib(0.5, 1.3, -0.8);
	Est.updateQy(1.97, -4.93, -2.18, 0.65, -2.37, 1.14);
	EXPECT_NEAR( 0.5031410611323102, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.8289865209174649, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.0519991639383328, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.2385927653754926, Est.m_Qy[3], 1e-15);

	// Prevent mag from being used in the subsequent tests
	Est.setMagCalib(0.0, 0.0, 0.0);

	// Case: Match the fused yaw of Qy to the fused yaw of Qhat
	Est.setAttitude(0.0593498238343210, -0.3684008345914626, -0.6950113107487008, -0.6145874237360555);
	Est.updateQy(0.0653130869941388, 0.1408317758534581, 0.1502788389860246, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.0885030635828862, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.1293711432148016, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(-0.3680984596463857, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.9164790445207663, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.2985621434353671, 0.4008598917686713, -0.6370341556565788, 0.5868215045526177);
	Est.updateQy(-0.7511787061021573, -0.0191764762381048, -1.2509137048592693, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.1211565733914288, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.8697464744764664, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.4149158638592946, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.2381322758000264, Est.m_Qy[3], 1e-15);
	Est.setAttitude(-0.7985544995138126, 0.3888651411734571, 0.4144559864391484, -0.1982948526592248);
	Est.updateQy(-1.0031569075807398, 0.6182787236419769, 1.3007056332429565, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.9055306095563346, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.1093983127730271, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(-0.3427605795961903, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.2248588654997435, Est.m_Qy[3], 1e-15);

	// Case: Match the fused yaw of Qy to the ZYX yaw of Qhat
	Est.setAttitude(-0.0000000000001761, -0.8298054517748469, -0.5580527862171661, 0.0000000000004423);
	Est.updateQy(-0.3563023186939954, 0.7114454379278298, -0.7365563262840626, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.3322763925454150, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.4508929952701395, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.7977178922176832, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.2234593256257253, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.0000000000004696, 0.6242462264374359, -0.7812276548987634, 0.0000000000003575);
	Est.updateQy(-1.1986018908768623, -0.0602429958299813, -0.6726418006292408, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.3155616000244389, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.6461733136022717, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.5717704839058757, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.3949170028149167, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.0000000000004138, -0.7882642741276257, 0.6153368460721717, -0.0000000000004642);
	Est.updateQy(-0.4992701502361627, -0.1169565130132472, -0.6190842722316107, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.2672406851831245, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.3944964989331197, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.8540697200568568, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.2086140977843245, Est.m_Qy[3], 1e-15);

	// Case: Match the ZYX yaw of Qy to the fused yaw of Qhat
	Est.setAttitude(0.4255784371869725, -0.7491239509678703, 0.0811700907363579, -0.5011064919332353);
	Est.updateQy(0.0000000000001955, -0.0000000000002696, -1.0000000000000000, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.0000000000000128, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.6473286269887536, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.7622110263443157, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.0000000000001660, Est.m_Qy[3], 1e-15);
	Est.setAttitude(-0.5477793720826180, -0.7052459255081034, -0.3189083535083264, -0.3175899969104817);
	Est.updateQy(-0.0000000000002232, -0.0000000000001510, -1.0000000000000000, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.0000000000000094, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.8651148256862977, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.5015738613382547, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.0000000000001344, Est.m_Qy[3], 1e-15);
	Est.setAttitude(-0.1245519154975099, -0.9090227287084450, 0.3439131685051232, 0.1997203834517672);
	Est.updateQy(0.0000000000003709, -0.0000000000002079, -1.0000000000000000, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.0000000000001023, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.5291636424714049, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.8485197932201671, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.0000000000001863, Est.m_Qy[3], 1e-15);

	// Case: Match the ZYX yaw of Qy to the ZYX yaw of Qhat
	Est.setAttitude(0.0000000000001691, 0.6285284121279358, 0.7777866257193780, -0.0000000000000190);
	Est.updateQy(-0.0000000000002650, -0.0000000000004164, -1.0000000000000000, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.0000000000000278, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.6285284121279358, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.7777866257193780, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.0000000000002452, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.0000000000000094, -0.6993407901302812, 0.7147884017385523, -0.0000000000004479);
	Est.updateQy(-0.0000000000000741, 0.0000000000001827, -1.0000000000000000, 0.0, 0.0, 0.0);
	EXPECT_NEAR(-0.0000000000000374, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.6993407901302812, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.7147884017385523, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.0000000000000912, Est.m_Qy[3], 1e-15);
	Est.setAttitude(0.0000000000000062, -0.1197310989733336, -0.9928063577247266, 0.0000000000002385);
	Est.updateQy(-0.0000000000001517, 0.0000000000003649, -1.0000000000000000, 0.0, 0.0, 0.0);
	EXPECT_NEAR( 0.0000000000000971, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.1197310989733335, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.9928063577247266, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.0000000000001720, Est.m_Qy[3], 1e-15);

	//
	// Test ZYX Yaw Method
	//

	// Set the acc-only resolution method
	Est.setAccMethod(Est.ME_ZYX_YAW);

	// Set the current attitude estimate and magnetometer calibration
	Est.setAttitude(1.0, 0.0, 0.0, 0.0);
	Est.setMagCalib(1.0, 0.0, 0.0);

	// Case: Acc is zero
	Est.updateQy(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
	EXPECT_NEAR(1.0, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[3], 1e-15);

	// Case: Acc and mag are both valid and unambiguous (m_Qhat should be irrelevant)
	Est.updateQy(0.0, 0.0, 5.0, 3.0, 0.0, 0.0);
	EXPECT_NEAR(1.0, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(0.0, Est.m_Qy[3], 1e-15);
	Est.updateQy(-0.2, 0.4, 1.0, -1.2, 0.6, -0.7);
	EXPECT_NEAR(-0.261154630414682, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.139805215990429, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.154980763303808, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.942461523671184, Est.m_Qy[3], 1e-15);
	Est.setMagCalib(0.5, 1.3, -0.8);
	Est.updateQy(1.97, -4.93, -2.18, 0.65, -2.37, 1.14);
	EXPECT_NEAR( 0.5031410611323102, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.8289865209174649, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.0519991639383328, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.2385927653754926, Est.m_Qy[3], 1e-15);

	// Case: Mag measurement needs to be discarded (match ZYX yaw)
	Est.setMagCalib(1.0, 1.0, 0.0);
	Est.updateQy(-1.7, 1.3, -0.2, -3.4, 2.6, -0.4); // Acc and mag are collinear
	EXPECT_NEAR( 0.584556422371574, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.681365550628110, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.286819101416163, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.334319575472664, Est.m_Qy[3], 1e-15);
	Est.setMagCalib(0.0, 0.0, 0.0); // Magnetometer calibration is zero
	Est.updateQy(-0.9, -3.2, 1.9, 0.3, -0.7, 1.0);
	EXPECT_NEAR( 0.8629533806823695, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.4912251049844832, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.1028632093928026, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.0585535580068824, Est.m_Qy[3], 1e-15);

	// Case: Mag measurement needs to be discarded (match ZXY yaw)
	Est.setMagCalib(0.0, 0.0, 0.0); // Magnetometer calibration is zero
	Est.setAttitudeEuler(0.5, -0.3, 1.4);
	Est.updateQy(-0.83838664359420356, 0.33705645287673092, -0.42836991422951487, 1.0, 1.0, 1.0); // Acc is along the global x-axis according to m_Qhat
	EXPECT_NEAR( 0.468159171699882, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.654665634720466, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.534413570028688, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.258151182136598, Est.m_Qy[3], 1e-15);
	Est.setMagCalib(0.0, 0.0, 0.0); // Magnetometer calibration is zero
	Est.setAttitudeEuler(-1.8, 0.9, 3.1);
	Est.updateQy(-0.257224178522156, -1.785612820648891, 0.250112508119670, 3.0, 2.0, 1.0); // Acc is along the global x-axis according to m_Qhat
	EXPECT_NEAR( 0.727313453229958, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.602228026219683, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.262027139930978, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR(-0.199194184467945, Est.m_Qy[3], 1e-15);

	// Case: Zero ZYX yaw assumption required
	Est.setMagCalib(0.0, 0.0, 0.0);     // Magnetometer calibration is zero
	Est.m_Qhat[0] =  0.000000000000000; // Note: Normally this would not be possible as m_Qhat is private and setAttitude() normalises the passed quaternion.
	Est.m_Qhat[1] =  0.478828002247735; //       This case is only achievable by breaking our model and having a non-normalised m_Qhat.
	Est.m_Qhat[2] =  0.232545603493783; //       This particular quaternion has norm 1/sqrt(2) and a zero scalar component.
	Est.m_Qhat[3] = -0.465452775863626;
	Est.updateQy(-0.256320938468470, -0.124483754175689, 0.249161059424410, 1.0, 2.0, 3.0); // Acc is along both the global x-axis and global y-axis according to m_Qhat
	EXPECT_NEAR( 0.9067313428959584, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR(-0.2139014942956990, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.3537249448410367, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.0834451073782317, Est.m_Qy[3], 1e-15);

	// Case: Zero ZXY yaw assumption required
	Est.setMagCalib(0.0, 0.0, 0.0);     // Magnetometer calibration is zero
	Est.m_Qhat[0] =  0.000000000000000; // Note: Normally this would not be possible as m_Qhat is private and setAttitude() normalises the passed quaternion.
	Est.m_Qhat[1] =  0.000000000000000; //       This case is only achievable by breaking our model and having a non-normalised m_Qhat.
	Est.m_Qhat[2] =  0.000000000000000; //       This particular quaternion has norm 1/sqrt(2) and all components zero except for in the z coordinate.
	Est.m_Qhat[3] =  0.707106781186547;
	Est.updateQy(-3.4, 0.0, 0.0, 4.0, 5.0, 6.0); // Acc is along the body-fixed x-axis, as well as the global x-axis and global y-axis according to m_Qhat
	EXPECT_NEAR( 0.707106781186547, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.000000000000000, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR( 0.707106781186548, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.000000000000000, Est.m_Qy[3], 1e-15);
	Est.setMagCalib(0.0, 0.0, 0.0);     // Magnetometer calibration is zero
	Est.m_Qhat[0] =  0.000000000000000; // Note: Normally this would not be possible as m_Qhat is private and setAttitude() normalises the passed quaternion.
	Est.m_Qhat[1] = -0.707106781186547; //       This case is only achievable by breaking our model and having a non-normalised m_Qhat.
	Est.m_Qhat[2] =  0.000000000000000; //       This particular quaternion has norm 1/sqrt(2) and all components zero except for in the x coordinate.
	Est.m_Qhat[3] =  0.000000000000000;
	Est.updateQy(1.7, 0.0, 0.0, 6.0, 5.0, 4.0); // Acc is along the body-fixed x-axis, as well as the global x-axis and global y-axis according to m_Qhat
	EXPECT_NEAR( 0.707106781186547, Est.m_Qy[0], 1e-15);
	EXPECT_NEAR( 0.000000000000000, Est.m_Qy[1], 1e-15);
	EXPECT_NEAR(-0.707106781186548, Est.m_Qy[2], 1e-15);
	EXPECT_NEAR( 0.000000000000000, Est.m_Qy[3], 1e-15);
}
#endif /* ATT_EST_PRIVATE_ARE_AVAILABLE */

//
// Main function
//
int main(int argc, char **argv)
{
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF
