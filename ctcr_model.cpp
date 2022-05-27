#include <ctcr_model.h>


CTCRModel::CTCRModel(std::array<double,3> length, std::array<double,3> ro, std::array<double,3> ri, std::array<double,3> straight_length, std::array<double,3> curvature, double youngs_modulus, int pts_per_seg, Eigen::Matrix4d base_frame) {
	
	m_length[0]         = length[0];
    m_length[1]         = length[1];
    m_length[2]         = length[2];
    
	m_ro[0]         = ro[0];
    m_ro[1]         = ro[1];
    m_ro[2]         = ro[2];
    
	m_ri[0]         = ri[0];
    m_ri[1]         = ri[1];
    m_ri[2]         = ri[2];
    
	m_straight_length[0]  = straight_length[0];
    m_straight_length[1]  = straight_length[1];
    m_straight_length[2]  = straight_length[2];
    
	m_curvature[0]      = curvature[0];
    m_curvature[1]      = curvature[1];
    m_curvature[2]      = curvature[2];
    
	m_youngs_modulus    = youngs_modulus;
	m_points_per_seg	= pts_per_seg;
    m_base_frame		= base_frame;
    
    
    m_current_config.resize(6,1);
    m_current_config.setZero();
}

CTCRModel::~CTCRModel() {

}

// This function should implement the forward kinematics of a concnetric tube continuum robot (i.e. mapping tube rotations and translations in joint space to the robot's end-effector and shape)
// Inputs:
// q						6x1 matrix/vector holding actuation values.
//							The first three entries are each tube's rotation, while the last three are the tube's translations.
//
// Outputs:
// ee_frame					4x4 matrix storing the end-effector frame resulting from the actuation q.
// backbone_centerline		4x4(m*n+1) dimensional matrix storing n frames for each of the m subsegments of the CTCR.
//							The leftmost 4x4 block should store the initial base/disk frame of the robot.
// tube_ind					Vector with m entries, specifying the outermost tube for each of the m subsegments.
// boolean return value		True if kinematics have been calculated successfully, false if not.
//							Also return false, if the joints limits and inequality constraints are invalidated.
bool CTCRModel::forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::MatrixXd q)
{
	
	
	//YOUR CODE GOES HERE
	return false; // Delete this line once you start coding
	
	
	//YOUR CODE ENDS HERE

	
	//Setting the member variables accordingly
	m_ee_frame = ee_frame;
	m_backbone_centerline = backbone_centerline;
	m_current_config = q;
	
	return true;
	
}


// This function should calculate and return the body Jacobian of a concnetric tube continuum robot using a simple finite differences approach
// Inputs:
// q						6x1 matrix/vector holding actuation values.
//							The first three entries are each tube's rotation, while the last three are the tube's translations.
// J						6x6 body Jacobian matrix, relating joint space velocities with end-effector twist
//							The first three rows correspond to the rotational part of a twist, while the last three rows correspond to the translational part
//
// Outputs:
// boolean return value		Return false, if the joints limits and inequality constraints are invalidated for the requested values in q.
//							Return true otherwise (and proceed to calculate and return J).
bool CTCRModel::get_body_jacobian(Eigen::MatrixXd &J, Eigen::MatrixXd q)
{
	//Resize J and set to zero
	J.setZero(6,6);
	
	//Evaluate at current configuration q
	Eigen::Matrix4d init_ee_frame;
	Eigen::MatrixXd init_backbone_centerline;
	std::vector<int> tube_ind;
	Eigen::MatrixXd init_q = q;
	
	if(!forward_kinematics(init_ee_frame, init_backbone_centerline, tube_ind, q))
	{
		//Return false if joint value constraints are not met
		return false;
	}
	
	

	
	//Calculate the Body Jacobian using Finite Differences here (YOUR CODE GOES HERE)
	
	
	
	//YOUR CODE ENDS HERE
	
	//Setting the member variables accordingly if q was valid
	m_ee_frame = init_ee_frame;
	m_backbone_centerline = init_backbone_centerline;
	m_current_config = init_q;
	
	return true;
}

Eigen::MatrixXd CTCRModel::get_current_config()
{
	return m_current_config;
}

Eigen::Matrix4d CTCRModel::get_ee_frame()
{
	return m_ee_frame;
}

Eigen::MatrixXd CTCRModel::get_backbone_centerline()
{
	return m_backbone_centerline;
}

Eigen::Matrix4d CTCRModel::get_base_frame()
{
	return m_base_frame;
}

