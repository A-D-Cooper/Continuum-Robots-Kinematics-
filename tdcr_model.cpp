#include <tdcr_model.h>
#include <cmath>
#include <iostream>
using namespace std;


TDCRModel::TDCRModel(std::array<double,3> length, int disks_per_seg, std::array<double,3> pradius_disks, Eigen::Matrix4d base_frame) {
	
	m_length[0]         = length[0];
    m_length[1]         = length[1];
    m_length[2]         = length[2];
    m_disks_per_seg		= disks_per_seg;
    m_pradius_disks[0]  = pradius_disks[0];
    m_pradius_disks[1]  = pradius_disks[1];
    m_pradius_disks[2]  = pradius_disks[2];
    m_base_frame		= base_frame;
    
    
    m_current_config.resize(9,1);
    m_current_config.setZero();
}

TDCRModel::~TDCRModel() {

}



// This function should implement the forward kinematics of a tendon driven continuum robot (i.e. mapping tendon lengths changes in joint space to the robot's end-effector and disk frames)
// Inputs:
// q						9x1 matrix/vector holding the tendon displacement (changes in tendon lengths) for each tendon.
//							The first three tendons belong to segment one etc (3 Tendons per 3 Segments).
//							A negative value indicates that the tendon is shortened (i.e. pulled).
//
// Outputs:
// ee_frame					4x4 matrix storing the end-effector frame resulting from the actuation q.
// disk_frames				4x4(3*n+1) dimensional matrix storing the frames for each of the n disks for each segment.
//							The leftmost 4x4 block should store the initial base/disk frame of the robot.
// boolean return value		True if kinematics have been calculated successfully, false if not.
//							Also return false, if the tendon length constraints are invalidated (the sum of tendon length changes in each segment has to equal zero).
bool TDCRModel::forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &disk_frames, Eigen::MatrixXd q)
{
	//YOUR CODE GOES HERE
	//return false; // Delete this line once you start coding
	
	double kappa_1;
	double kappa_2;
	double kappa_3;
	double phi_1;
	double phi_2;
	double phi_3;
	double d_1;
	double d_2;
	double d_3;
	double beta = (2 * 3.14159265358979323) / 3;
	
	double sum = q(0, 0) + q(1, 0) + q(2, 0) + q(3, 0) + q(4, 0) + q(5, 0) + q(6, 0) + q(7, 0) + q(8, 0);
	double sumabs = abs(q(0, 0)) + abs(q(1, 0)) + abs(q(2, 0)) + abs(q(3, 0)) + abs(q(4, 0)) + abs(q(5, 0)) + abs(q(6, 0)) + abs(q(7, 0)) + abs(q(8, 0));

	//std::cout << sum << "  " << sumabs << std::endl;

	if (abs(sum) > 0.0000000001 && sumabs > 0)
	{
		return false;
	}


	if (q(0, 0) != 0) {
		phi_1 = atan2( (q(0, 0) * cos(beta)) - q(1, 0), -q(0, 0) * sin(beta));
	}
	else {
		phi_1 = atan2( (q(1, 0) * cos(beta)) - q(2, 0), -q(1, 0) * sin(beta));
	}
	if (q(3, 0) != 0) {
		phi_2 = atan2( (q(3, 0) * cos(beta)) - q(4, 0), -q(3, 0) * sin(beta));
	}
	else {
		phi_2 = atan2( (q(4, 0) * cos(beta)) - q(5, 0), -q(6, 0) * sin(beta));
	}
	if (q(6, 0) != 0) {
		phi_3 = atan2( (q(6, 0) * cos(beta)) - q(7, 0), -q(6, 0) * sin(beta));
	}
	else {
		phi_3 = atan2( (q(7, 0) * cos(beta)) - q(8, 0), -q(7, 0) * sin(beta));
	}

	d_1 = m_pradius_disks[0] * cos(phi_1) ;
	d_2 = m_pradius_disks[1] * cos(phi_2) ;
	d_3 = m_pradius_disks[2] * cos(phi_3) ;

	if (q(0, 0) != 0) {
		kappa_1 = -q(0, 0) / (d_1 * m_length[0]);
	}
	else {
		kappa_1 = -q(1, 0) / (d_1 * m_length[0]);
	}
	if (q(3, 0) !=0 ) {
		kappa_2 = -q(3, 0) / (d_2 * m_length[1]);
	}
	else {
		kappa_2 = -q(4, 0) / (d_2 * m_length[1]);
	}
	if (q(6, 0) != 0) {
		kappa_3 = -q(6, 0) / (d_3 * m_length[2]);
	}
	else {
		kappa_3 = -q(7, 0) / (d_3 * m_length[2]);
	}

	std::vector<double> v_kappa(3);
	std::vector<double> v_phi(3);
	std::vector<double> v_length(3);

	v_length[0] = m_length[0];
	v_length[1] = m_length[1];
	v_length[2] = m_length[2];
	v_kappa[0] = abs(kappa_1);
	v_kappa[1] = abs(kappa_2);
	v_kappa[2] = abs(kappa_3);
	v_phi[0] = phi_1;
	v_phi[1] = phi_2;
	v_phi[2] = phi_3;


	//Eigen::MatrixXd disk_frames;
	disk_frames = arc_to_x(m_base_frame, v_kappa, v_length, v_phi, m_disks_per_seg, true);

	int col = disk_frames.cols() - 1;
	int row = disk_frames.rows() - 1;

	//Eigen::Matrix4d ee_frame;
	ee_frame << disk_frames(row -3, col-3), disk_frames(row-3, col-2), disk_frames(row-3, col-1), disk_frames(row-3, col),
			disk_frames(row-2, col-3), disk_frames(row-2, col-2), disk_frames(row-2, col-1), disk_frames(row-2, col),
			disk_frames(row-1, col-3), disk_frames(row-1, col-2), disk_frames(row-1, col-1), disk_frames(row-1, col),
			disk_frames(row, col-3), disk_frames(row, col-2), disk_frames(row, col-1), disk_frames(row, col); 
	
	//YOUR CODE ENDS HERE
	
	//Setting the member variables accordingly
	m_ee_frame = ee_frame;
	m_disk_frames = disk_frames;
	m_current_config = q;
	
	return true;

}


// This function should calculate and return the body Jacobian of a tendon driven continuum robot using a simple finite differences approach
// Inputs:
// q						9x1 matrix/vector holding the tendon displacement (changes in tendon lengths) for each tendon.
//							The first three tendons belong to segment one etc (3 Tendons per 3 Segments).
// J						6x6 body Jacobian matrix, relating joint space velocities with end-effector twist (beware the tendon constraints are manually resolved to reduce Q to six dimensions, representing the robot's DoF)
//							The first three rows correspond to the rotational part of a twist, while the last three rows correspond to the translational part
//
// Outputs:
// boolean return value		Return false, if if the tendon length constraints are invalidated.
//							Return true otherwise (and proceed to calculate and return J).
bool TDCRModel::get_body_jacobian(Eigen::MatrixXd &J, Eigen::MatrixXd q)
{
	//Resize J and set to zero
	J.setZero(6,6);
	
	//Evaluate at current configuration q
	Eigen::Matrix4d init_ee_frame;
	Eigen::MatrixXd init_disk_frames;
	Eigen::MatrixXd init_q = q;
	
	if(!forward_kinematics(init_ee_frame, init_disk_frames, q))
	{
		//Return false if joint value constraints are not met
		return false;
	}
	
	//Calculate the Body Jacobian using Finite Differences here (YOUR CODE GOES HERE)
	
	//cout << "7 \n";
	
	
	double h = 1e-4;
	Eigen::Matrix4d ee_frame;
	Eigen::MatrixXd disk_frames;
	Eigen::Matrix<double, 9, 1> nq1;
	Eigen::Matrix<double, 9, 1> nq2;
	Eigen::Matrix<double, 9, 1> nq3;
	Eigen::Matrix<double, 9, 1> nq4;
	Eigen::Matrix<double, 9, 1> nq5;
	Eigen::Matrix<double, 9, 1> nq6;



	//cout << "8 \n";

	bool fk = forward_kinematics(init_ee_frame, init_disk_frames, q);
	nq1 << q(0, 0)+h, q(1, 0), q(2, 0)-h, q(3, 0), q(4, 0), q(5, 0), q(6, 0), q(7, 0), q(8, 0);  
	nq2 << q(0, 0), q(1, 0)+h, q(2, 0)-h, q(3, 0), q(4, 0), q(5, 0), q(6, 0), q(7, 0), q(8, 0);
	nq3 << q(0, 0), q(1, 0), q(2, 0), q(3, 0)+h, q(4, 0), q(5, 0) -h, q(6, 0), q(7, 0), q(8, 0);
	nq4 << q(0, 0), q(1, 0), q(2, 0), q(3, 0), q(4, 0)+h, q(5, 0) -h, q(6, 0), q(7, 0), q(8, 0);
	nq5 << q(0, 0), q(1, 0), q(2, 0), q(3, 0), q(4, 0), q(5, 0), q(6, 0)+h, q(7, 0), q(8, 0) - h;
	nq6 << q(0, 0), q(1, 0), q(2, 0), q(3, 0), q(4, 0), q(5, 0), q(6, 0), q(7, 0)+h, q(8, 0) - h;

	bool fk2;
	Eigen::Matrix<double, 6, 1> twist;

	
	for (int i=0;i<6; i++) {
		if (i==0) {
			fk2 = forward_kinematics(ee_frame, disk_frames, nq1);
		}
		if (i==1){
			fk2 = forward_kinematics(ee_frame, disk_frames, nq2);
		}
		if (i==2) {
			fk2 = forward_kinematics(ee_frame, disk_frames, nq3);
		}
		if (i==3) {
			fk2 = forward_kinematics(ee_frame, disk_frames, nq4);
		}
		if (i==4) {
			fk2 = forward_kinematics(ee_frame, disk_frames, nq5);
		}
		if (i==5) {
			fk2 = forward_kinematics(ee_frame, disk_frames, nq6);
		}
			
		//cout << "\n fk2 " << fk2 << endl;
		twist = calculate_desired_body_twist(ee_frame, init_ee_frame);
		
		for (int c=0; c<6; c++) {
			J(c, i) = twist(c, 0) / h;
		}
	}

	//cout << "10 \n";
	//YOUR CODE ENDS HERE

	//Setting the member variables accordingly
	m_ee_frame = init_ee_frame;
	m_disk_frames = init_disk_frames;
	m_current_config = init_q;
	
	return true;
}

Eigen::MatrixXd TDCRModel::get_current_config()
{
	return m_current_config;
}

Eigen::Matrix4d TDCRModel::get_ee_frame()
{
	return m_ee_frame;	
}

Eigen::MatrixXd TDCRModel::get_disk_frames()
{
	return m_disk_frames;	
}

Eigen::Matrix4d TDCRModel::get_base_frame()
{
	return m_base_frame;	
}

