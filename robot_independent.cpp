#include <robot_independent.h>
#include <iostream>
#include <cmath>
using namespace std;



// This function should implement the robot independent mapping (i.e. mapping arc parameters in configuration space to a series of discrete frames in task space)
// Inputs:
// init_frame			4x4 Matrix, specifying the initial frame of the curve
// kappa				m-dimensional vector, storing the curvature of each segment of the curve
// l					m-dimensional vector, storing the length of each segment of the curve
// phi					m-dimensional vector, storing the angle of the bending plane of each segment of the curve
// n					number of requested frames to be returned for each segment (equally distributed along the circular arc)
// bishop				boolean value, specifying whether the material frame of the curve should be maintained or not (meaning a constant local rotation around the z-axis)
//
// Output (return):
//
// Eigen::MatrixXd		4x4(m*n+1) dimensional matrix storing all of the returned frames along the curve (stacked 4x4 blocks). First, leftmost 4x4 block should store the initial frame (init_frame).


Eigen::MatrixXd tmatrix(double k, double s, double phi) {
	Eigen::Matrix4d T;

	if (k != 0.0) { 
		T << cos(phi) * cos(k * s), -1 * sin(phi), cos(phi) * sin(k *s), (cos(phi) * (1 - cos(k * s))) / k,
			sin(phi) * cos (k * s), cos(phi), sin(phi) * sin(k * s), (sin(phi) * (1 - cos(k * s))) / k,
			-1 * sin(k * s), 0, cos(k * s), sin(k * s) /k,
			0, 0, 0, 1;
	}
	else {
		T << cos(phi) * cos(k * s), -1 * sin(phi), cos(phi) * sin(k * s), 0,
			sin(phi) * cos (k * s), cos(phi), sin(phi) * sin(k * s), 0,
			-1 * sin(k * s), 0, cos(k * s), s,
			0, 0, 0, 1;
	}
	return T;
}

Eigen::MatrixXd bimat(Eigen::Matrix4d T, double phi) {
	phi = -1 * phi;
	Eigen::Matrix3d rz;
	rz << cos(phi), -1 * sin(phi), 0,
		sin(phi), cos(phi), 0,
		0, 0, 1;
	Eigen::Matrix3d fr;
	for (int r=0; r <3; r++){
		for (int c=0; c<3; c++){
			fr(r, c) = T(r, c);
		}
	}
	Eigen::Matrix3d ff;
	ff = fr * rz;
	for (int rw=0; rw<3; rw++){
		for (int cw=0; cw<3; cw++){
			T(rw, cw) = ff(rw, cw);
		}
	}
	return T;
}

Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop)
{
	//Eigen::MatrixXd curve = init_frame;
	int counter = 0;
	int pnt = 4;
	int m = kappa.size();
	// std::cout << "value of m  " << m << "  value of n  " << n << std::endl;
	Eigen::MatrixXd curve(4, ((m * n) + 1) * 4);
	Eigen::Matrix4d prev_frame = init_frame;
	Eigen::Matrix4d trmat;

	for (int rw=0; rw < 4; rw++) {
		for (int cl=0; cl < 4; cl++) {
			curve(rw, cl) = init_frame(rw, cl);
		}
	}
	for (int i = 0; i < m; i++) {	
		for (int k = 1; k <= n; k++) {
			double s = (l[i] / n) * k;
			trmat = prev_frame * tmatrix(kappa[i], s, phi[i]);	
			if (bishop == true) {
				trmat = bimat(trmat, phi[i]);
			}
			for (int r=0; r < 4; r++) {
				for (int c=0; c<4; c++) {
					curve(r, pnt + c) = trmat(r, c);
				}
			}
			pnt = pnt + 4;
		}
		prev_frame = trmat;
	}
	return curve;
}

// This function should implement mapping from SE(3) (Sepcial Euclidean Lie Group) to the corresponding lie algebra se(3)
// Inputs:
// T					4x4 Matrix, specifying a transformation matrix T = [R p; 0 0 0 1] in SE(3)
// Output (return):
//
// Eigen::Matrix4d		4x4 Matrix, being the product of [S]*theta, where [S] is the screw axis in matrix form consisting of an angular (skew symmetric) and translational part
//						and theta is the displacement along this screw axis
Eigen::Matrix4d matrix_log(Eigen::Matrix4d T)
{
	Eigen::Matrix4d matrix_log; // = Eigen::Matrix4d::Identity();
	double theta = 0;
	Eigen::Matrix3d omega_hat;
	Eigen::Matrix<double, 3, 1> v;
	
	double trace = T(1,1) + T(2,2) + T(0, 0);

	//cout << "first \n";
	theta = acos( (trace - 1) / 2);

	if (abs(theta) == 0 || std::isnan(theta)) {

		double n;
		n = sqrt( pow(T(0,3), 2) + pow(T(1, 3), 2) + pow(T(2, 3), 2) );
		
		//cout << "\n" << T(0, 3) << "   t1  " << T(1, 3) << "  t2  "<< T(2, 3) << endl;
		//cout << n << " n " << T(0, 3) << " 1 " << T(1, 3) << T(2, 3);

		v << T(0, 3) / n, T(1, 3) / n, T(2, 3) / n;
		v = v * n;
		//cout << "2.3 \n";
		matrix_log << 0., 0., 0., v(0, 0), 0., 0., 0., v(1, 0), 0., 0., 0., v(2, 0), 0., 0., 0., 0.;
	
	}
	else {
		//cout << "2 in else  \n";
		Eigen::Matrix3d R;
		R << T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2); 
		Eigen::Matrix3d g;
		Eigen::Matrix<double, 3, 1> p;
		
		p << T(0, 3), T(1, 3), T(2, 3);
		omega_hat = (1 / (2 * sin(theta))) * (R - R.transpose());
		
		g = ((1. / theta) * Eigen::Matrix3d::Identity()) - (0.5 * omega_hat) + 
		( ( (1./theta) - (0.5 * ( 1./tan(theta/2) ) ) ) *  (omega_hat * omega_hat) );
		//cout << g << "g \n";
		v = g * p;
	
		matrix_log << omega_hat(0, 0), omega_hat(0, 1), omega_hat(0, 2), v(0, 0), 
					  omega_hat(1, 0), omega_hat(1, 1), omega_hat(1, 2), v(1, 0), 
					  omega_hat(2, 0), omega_hat(2, 1), omega_hat(2, 2), v(2, 0),
					  0, 0, 0, 0; 

		matrix_log = matrix_log * theta;
	}
	
	return matrix_log;

}


// This function should calculate and return a desired twist in the body frame based on a current body frame and a desired body frame (both frames expressed w.r.t. the space frame)
// Inputs:
// T_cur					4x4 Matrix, specifying the current body frame T_sb
// T_target					4x4 Matrix, specifying the desired target body frame T_sd
// Output (return):
//
// Eigen::MatrixXd			6x1 Matrix, expressing the desired body frame twist V_b to move from the current body frame to the desired frame
//							The first three entries should hold the rotational part, while the last thee entries should hold the translational part
Eigen::MatrixXd calculate_desired_body_twist(Eigen::Matrix4d T_target, Eigen::Matrix4d T_cur)
{
	Eigen::Matrix<double, 6, 1> body_twist;
	body_twist.setZero();
	Eigen::Matrix4d T;
	
	T = matrix_log( T_cur.inverse() * T_target );
	body_twist << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

	return body_twist;
}

