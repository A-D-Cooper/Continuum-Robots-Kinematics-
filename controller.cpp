#include <controller.h>
#include <iostream>
#include <cmath>
using namespace std;



Controller::Controller(TDCRModel* tdcr, CTCRModel* ctcr) {
	
	mp_TDCR = tdcr;
	mp_CTCR = ctcr;

}

Controller::~Controller() {

}


// This function implements a single control iteration for a three segment TDCR.
// It implements pose control, meaning it takes a target frame as an input and calculates a step in Q for the TDCR to approach this new frame (in terms of orientation and position).
// It then applies this step to the TDCR and updates its state for the next control iteration, while returning its new tip frame and shape.  
//
// Inputs:
// T_target					4x4 matrix, specifying the target tip frame for the current control iteration T_sd.
// wdls_jac					Boolean value, specifying if a damped (singularity robust) Jacobian should be utilized in the control iteration
// gain						Double value, specifying the proportional gain of the controller
//
// Outputs:
// ee_frame					4x4 matrix, specifying the end-effector frame of the updated TDCR after the control iteration
// disk_frames				4x4(3*n+1) matrix, storing the disk frames of the updated TDCR after the control iteration
void Controller::execute_tdcr_control_iteration(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &disk_frames, Eigen::Matrix4d T_target, bool wdls_jac, double gain)
{
	//Get current frame and configuration of TDCR
	Eigen::Matrix4d T_cur = mp_TDCR->get_ee_frame();
	Eigen::MatrixXd q_cur = mp_TDCR->get_current_config();
	
	//YOUR CODE GOES HERE
	Eigen::Matrix<double, 6, 1> v;
	Eigen::MatrixXd J;
	Eigen::Matrix<double, 6, 1> q_dot;
	Eigen::Matrix<double, 9, 1> q_new;
	Eigen::Matrix<double, 6, 6> W;
	
	for (int i=0; i<6; i++) {
		for (int r=0;r<6; r++){
			if (i==r) {
					W(i,r) = 2.5;
				}
			else { 
				W(i,r) = 0;
				}
			}
		}

	//cout << "11 \n";
	
	v = calculate_desired_body_twist(T_target, T_cur);

	//cout << v << " v vlaues \n";
	//cout << "\n" << "T_target  \n" << T_target << "\n  T_cur  \n" << T_cur << "\n";
	
	//cout << v(0, 0) << "  " << v(1, 0) << "  " << v(2, 0) << "  " << v(3, 0) << "  " << v(4, 0) << "  " << v(5, 0) << " \n";
	v = v * gain;
	
	bool b = mp_TDCR->get_body_jacobian(J, q_cur);
	//cout << "12 \n";
	
	if (wdls_jac==false) {
		q_dot = J.inverse() * v;
	}

	else {
		q_dot = ( ( (J.transpose() * J) + W).inverse() * J.transpose() ) * v;
	}
	//cout << "\n Jacobian \n" << J << "\n";
	q_new << q_cur(0, 0) + q_dot(0, 0), q_cur(1, 0) + q_dot(1, 0), q_cur(2, 0) - q_dot(0, 0) - q_dot(1, 0),
			 q_cur(3, 0) + q_dot(2, 0), q_cur(4, 0) + q_dot(3, 0), q_cur(5, 0) - q_dot(2, 0) - q_dot(3, 0),
			 q_cur(6, 0) + q_dot(4, 0), q_cur(7, 0) + q_dot(5, 0), q_cur(8, 0) - q_dot(4, 0) - q_dot(5, 0);	

	bool fk1 = mp_TDCR->forward_kinematics(ee_frame, disk_frames, q_new);
	//cout<< "\n" << fk1 << "  fk1  \n";

	//YOUR CODE ENDS HERE
}


// This function implements a single control iteration for a three tube CTCR.
// It implements position control, meaning it takes a target frame as an input and calculates a step in Q for the CTCR to approach the new position of this frame (Note: ignoring orientation!).
// It then applies this step to the CTCR and updates its state for the next control iteration, while returning its new tip frame and shape.  
//
// Inputs:
// T_target					4x4 matrix, specifying the target tip frame for the current control iteration T_sd.
// wdls_jac					Boolean value, specifying if a damped (singularity robust) Jacobian should be utilized in the control iteration
// gain						Double value, specifying the proportional gain of the controller
//
// Outputs:
// ee_frame					4x4 matrix, specifying the end-effector frame of the updated CTCR after the control iteration
// backbone_centerline		4x4(m*n+1) dimensional matrix storing n frames for each of the m subsegments of the updated CTCR after the control iteration
// tube_ind					Vector with m entries, specifying the outermost tube for each of the m subsegments of the updated CTCR after the control iteration
void Controller::execute_ctcr_control_iteration(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::Matrix4d T_target, bool wdls_jac, double gain)
{
	//Get current frame and configuration of CTCR
	Eigen::Matrix4d T_cur = mp_CTCR->get_ee_frame();
	Eigen::MatrixXd q_cur = mp_CTCR->get_current_config();
	
	//YOUR CODE GOES HERE
	
	//YOUR CODE ENDS HERE
}
