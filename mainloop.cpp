#include <mainloop.h>

MainLoop::MainLoop(Visualizer* vis, Controller* con, TDCRModel* tdcr, CTCRModel* ctcr, double timestep, int assignment, int control_scenario) {
	
	m_timestep = timestep;
	mp_Vis = vis;
	mp_Controller = con;
	mp_TDCR = tdcr;
	mp_CTCR = ctcr;
	m_loopCount = 0; 
	m_assignment = assignment;
	m_control_scenario = control_scenario;
	
	m_control_loop_active = false;
	
	//Initialize each scenario
	if(m_assignment == 1) {
			
	}
	else if(m_assignment == 2) {
		//Initialize counter
		m_counter = 0;
		
		//Define the configurations to check
		m_configs.clear();
		
		// Read configurations from csv file
		std::ifstream configFile("../tdcr_configs.csv");
	
		// Make sure the file is open
		if(!configFile.is_open()) throw std::runtime_error("Could not open file");
		
		// Read data, line by line
		std::string line;
		double value;
		while(std::getline(configFile, line))
		{
			// Create a stringstream of the current line
			std::stringstream ss(line);
			
			// Keep track of the current column index
			int colIdx = 0;
			
			
			Eigen::Matrix<double,9,1> config;
			// Extract each integer
			while(ss >> value){
				
				// Add the current value to the configuration vector
				config(colIdx) = value;
				
				// If the next token is a comma, ignore it and move on
				if(ss.peek() == ',') ss.ignore();
				
				// Increment the column index
				colIdx++;
			}
			m_configs.push_back(config);
		}
	
		// Close file
		configFile.close();
		
		
		
	}
	else if(m_assignment == 3) {
		//Initialize counter
		m_counter = 0;
		
		
		//Define the configurations to check
		m_configs.clear();
		
		// Read configurations from csv file
		std::ifstream configFile("../ctcr_configs.csv");
	
		// Make sure the file is open
		if(!configFile.is_open()) throw std::runtime_error("Could not open file");
		
		// Read data, line by line
		std::string line;
		double value;
		while(std::getline(configFile, line))
		{
			// Create a stringstream of the current line
			std::stringstream ss(line);
			
			// Keep track of the current column index
			int colIdx = 0;
			
			
			Eigen::Matrix<double,6,1> config;
			// Extract each integer
			while(ss >> value){
				
				// Add the current value to the configuration vector
				config(colIdx) = value;
				
				// If the next token is a comma, ignore it and move on
				if(ss.peek() == ',') ss.ignore();
				
				// Increment the column index
				colIdx++;
			}
			m_configs.push_back(config);
		}
	
		// Close file
		configFile.close();
	
	}
	else if(m_assignment == 4) {
		
		
		std::cout << "Enter control gain: ";
		std::cin >> m_control_gain;
		std::cout << std::endl;
		
		
		std::cout << "Enable singularity robust Jacobian (0/1)? ";
		std::cin >> m_wdls_jac;
		std::cout << std::endl;
		
		if(m_control_scenario == 0)
		{
			m_TDCR_path.clear();
			
			Eigen::Matrix4d target;
			
			m_counter = 1;
			
			//Initial point
			target = mp_TDCR->get_ee_frame();
			m_TDCR_path.push_back(target);
				
			//First target
			target(0,3) = target(0,3) - 0.05;
			
			m_TDCR_path.push_back(target);
			
			m_control_target_frame = target;
				
			mp_Vis->drawTargetFrame(target);
			
			//Successive targets
			target(1,3) = target(1,3) + 0.05;
			m_TDCR_path.push_back(target);
			
			
			Eigen::Matrix3d rot_x;
			rot_x << 1, 0, 0,
					 0, std::cos(-M_PI/2), -1*std::sin(-M_PI/2),
					 0, std::sin(-M_PI/2), std::cos(-M_PI/2);
			target.block(0,0,3,3) = rot_x*target.block(0,0,3,3);
			
			
			target(1,3) = target(1,3) + 0.05;
			m_TDCR_path.push_back(target);
			
			target(0,3) = target(0,3) - 0.05;
			m_TDCR_path.push_back(target);
			
			target(0,3) = target(0,3) - 0.05;
			m_TDCR_path.push_back(target);
			
			target(0,3) = target(0,3) - 0.05;
			m_TDCR_path.push_back(target);
			
			target(1,3) = target(1,3) + 0.05;
			target.block(0,0,3,3) = rot_x*target.block(0,0,3,3);
			m_TDCR_path.push_back(target);
			
			target(1,3) = target(1,3) + 0.05;
			target(2,3) = target(2,3) - 0.05;
			m_TDCR_path.push_back(target);
			
			target(2,3) = target(2,3) - 0.05;
			m_TDCR_path.push_back(target);
			
					 
			target(0,3) = target(0,3) + 0.05;
			m_TDCR_path.push_back(target);
			
			
			target(2,3) = target(2,3) + 0.05;
			target(1,3) = target(1,3) - 0.05;
			target.block(0,0,3,3) = rot_x.transpose()*target.block(0,0,3,3);
			
			m_TDCR_path.push_back(target);
			
			target(2,3) = target(2,3) + 0.05;
			target(1,3) = target(1,3) - 0.05;
			target.block(0,0,3,3) = rot_x.transpose()*target.block(0,0,3,3);
			
			m_TDCR_path.push_back(target);
			
			
			target(1,3) = target(1,3) - 0.05;
			m_TDCR_path.push_back(target);
			target(1,3) = target(1,3) - 0.05;
			m_TDCR_path.push_back(target);
			
			target(0,3) = target(0,3) + 0.05;
			m_TDCR_path.push_back(target);
			
			target(0,3) = target(0,3) + 0.05;
			m_TDCR_path.push_back(target);
	
					 
			
		}
		else if(m_control_scenario == 1)
		{
			std::vector<Eigen::Matrix4d> vis_path;
			m_CTCR_path.clear();
			m_counter = 0;
			
			//Define a simple path
			int steps = 400;
			double radius = 0.04;
			double height = radius/8;
			int counter = 5;
			
			double tilt = 1;
			
			Eigen::Matrix4d tilt_matrix;
			
			tilt_matrix << std::cos(tilt), 0, std::sin(tilt), 0,
						   0, 1, 0, 0,
						   -std::sin(tilt), 0, std::cos(tilt), 0,
						   0, 0, 0, 1;
			
			
			
			
			for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/steps)
			{
				Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
				target(0,3) += height*std::sin(4*theta);
				target(1,3) += radius*std::sin(theta);
				target(2,3) += -radius*std::cos(theta)+radius;
				
				target = tilt_matrix*target;
				
				target.block(0,3,3,1) += mp_CTCR->get_ee_frame().block(0,3,3,1);
				
				
				m_CTCR_path.push_back(target);
				
				if(counter == 5)
				{
					counter = 0;
					vis_path.push_back(target);
				}
				else
				{
					counter ++;
				}
				
			}
			vis_path.push_back(m_CTCR_path.back());
			
			m_control_target_frame = m_CTCR_path.at(0);
				
			mp_Vis->drawPath(vis_path);
			
		}
		
	}
	else if(m_assignment == 0) {
		
	}

}

MainLoop::~MainLoop() {
	
}

void MainLoop::Execute(vtkObject *caller, unsigned long eventId, void *vtkNotUsed(callData))
{	
	
	if (vtkCommand::TimerEvent == eventId)
	{	
		// Increase loop count
		m_loopCount++;
		
		//Begin time measurement
		//clock_t begin = std::clock();
		
			
		//Do stuff here in each mainloop iteration for the specific assignment scenario
		if(m_assignment == 1) {
			
		}
		else if(m_assignment == 2) {
			
			
		}
		else if(m_assignment == 3) {
	
		}
		else if(m_assignment == 4) {
			
			Eigen::Matrix4d ee_frame;
			Eigen::MatrixXd disk_frames;
			Eigen::MatrixXd backbone_centerline;
			std::vector<int> tube_ind;
			
			
			//TDCR Controller
			if(m_control_loop_active)
			{
				
				if(m_control_scenario == 0)
				{
					
					mp_Controller->execute_tdcr_control_iteration(ee_frame,disk_frames,m_control_target_frame,m_wdls_jac,m_control_gain);
					
					mp_Vis->updateTDCR(disk_frames);
					
					if(calculate_desired_body_twist(m_control_target_frame,ee_frame).norm() < 1e-3	)
					{
						m_control_loop_active = false;
						
						if(m_counter != m_TDCR_path.size()-1)
							m_counter++;
						else
							m_counter = 0;
						
						m_control_target_frame = m_TDCR_path.at(m_counter);
				
						mp_Vis->moveTargetFrame(m_control_target_frame);
						
						std::cout << "Reached current control goal!" << std::endl;
					}
				}
				else if(m_control_scenario == 1)
				{
					m_counter++;
					
					m_control_target_frame = m_CTCR_path.at(m_counter);
					
					mp_Controller->execute_ctcr_control_iteration(ee_frame,backbone_centerline,tube_ind,m_control_target_frame,m_wdls_jac,m_control_gain);
					
					mp_Vis->updateCTCR(backbone_centerline,tube_ind);
					
					
					if(m_counter == m_CTCR_path.size()-1)
					{
						m_control_loop_active = false;
						m_counter = 0;
						
						m_control_target_frame = m_CTCR_path.at(m_counter);
						
						std::cout << "Finished current control scenario!" << std::endl;
					}
				}
				
				
			}
			
			
		}
		else if(m_assignment == 0) {
			
		}
		
		//Hand new data to visualizer in order to update the scene
		if(m_loopCount*m_timestep >= 0.02) //Update Visualizer with 50 Hz (20 ms)
		{
			mp_Vis->update();
			m_loopCount = 0;
		}
		
		//End time measurement
		//clock_t end = std::clock();
		//double elapsed_msecs = double(end - begin) / CLOCKS_PER_SEC * 1000;
		//std::cout << elapsed_msecs << std::endl;
		
		
	}
	else if(vtkCommand::KeyPressEvent == eventId) // React on user input for each particular assignment scenario
	{
		vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
		
		//For assignment 1, define different sets of curve parameters from a file and plot them once with and without bishop representation (considering some offset between them)
		if(*(iren->GetKeySym()) == 'a' && m_assignment == 1)
		{
			std::cout << "Drawing first set of curves..." << std::endl;
			mp_Vis->clear();
			
			//Define arc parameters
			std::vector<double> kappa;
			std::vector<double> l;
			std::vector<double> phi;
			
			kappa.push_back(8);
			l.push_back(0.15);
			phi.push_back(1.2);
			
			Eigen::Matrix4d init_1 = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d init_2 = init_1;
			init_2(0,3) = 0.1;
			
			Eigen::MatrixXd curve_1 = arc_to_x(init_1,kappa,l,phi,25,false);
			Eigen::MatrixXd curve_2 = arc_to_x(init_2,kappa,l,phi,25,true);
			Eigen::MatrixXd frames_1 = arc_to_x(init_1,kappa,l,phi,5,false);
			Eigen::MatrixXd frames_2 = arc_to_x(init_2,kappa,l,phi,5,true);
			
			
			if(curve_1.rows() != 4 || curve_1.cols() != 4*(25+1) || curve_2.rows() != 4 || curve_2.cols() != 4*(25+1) || frames_1.rows() != 4 || frames_1.cols() != 4*(5+1) || frames_2.rows() != 4 || frames_2.cols() != 4*(5+1))
			{
				std::cout << "Wrong dimensions of matrix storing the discrete frames (returned from arc_to_x)!" << std::endl;
			}
			else
			{
				mp_Vis->drawCurve(curve_1);
				mp_Vis->drawCurve(curve_2);
				mp_Vis->drawFrames(frames_1);
				mp_Vis->drawFrames(frames_2);
			}
			
			
		}
		if(*(iren->GetKeySym()) == 's' && m_assignment == 1)
		{
			std::cout << "Drawing second set of curves..." << std::endl;
			mp_Vis->clear();
			
			//Define arc parameters
			std::vector<double> kappa;
			std::vector<double> l;
			std::vector<double> phi;
			
			kappa.push_back(8);
			l.push_back(0.15);
			phi.push_back(M_PI/2);
			
			kappa.push_back(8);
			l.push_back(0.15);
			phi.push_back(M_PI/4);
			
			Eigen::Matrix4d init_1 = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d init_2 = init_1;
			init_2(0,3) = 0.1;
			
			Eigen::MatrixXd curve_1 = arc_to_x(init_1,kappa,l,phi,25,false);
			Eigen::MatrixXd curve_2 = arc_to_x(init_2,kappa,l,phi,25,true);
			Eigen::MatrixXd frames_1 = arc_to_x(init_1,kappa,l,phi,5,false);
			Eigen::MatrixXd frames_2 = arc_to_x(init_2,kappa,l,phi,5,true);
			
			
			if(curve_1.rows() != 4 || curve_1.cols() != 4*(2*25+1) || curve_2.rows() != 4 || curve_2.cols() != 4*(2*25+1) || frames_1.rows() != 4 || frames_1.cols() != 4*(2*5+1) || frames_2.rows() != 4 || frames_2.cols() != 4*(2*5+1))
			{
				std::cout << "Wrong dimensions of matrix storing the discrete frames (returned from arc_to_x)!" << std::endl;
			}
			else
			{
				mp_Vis->drawCurve(curve_1);
				mp_Vis->drawCurve(curve_2);
				mp_Vis->drawFrames(frames_1);
				mp_Vis->drawFrames(frames_2);
			}
			
		}
		if(*(iren->GetKeySym()) == 'd' && m_assignment == 1)
		{
			std::cout << "Drawing third set of curves..." << std::endl;
			mp_Vis->clear();
			
			//Define arc parameters
			std::vector<double> kappa;
			std::vector<double> l;
			std::vector<double> phi;
			
			kappa.push_back(8);
			l.push_back(0.15);
			phi.push_back(M_PI/4);
			
			kappa.push_back(8);
			l.push_back(0.15);
			phi.push_back(2*M_PI/4);
			
			kappa.push_back(8);
			l.push_back(0.15);
			phi.push_back(3*M_PI/4);
			
			Eigen::Matrix4d init_1 = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d init_2 = init_1;
			init_2(0,3) = 0.1;
			
			Eigen::MatrixXd curve_1 = arc_to_x(init_1,kappa,l,phi,25,false);
			Eigen::MatrixXd curve_2 = arc_to_x(init_2,kappa,l,phi,25,true);
			Eigen::MatrixXd frames_1 = arc_to_x(init_1,kappa,l,phi,5,false);
			Eigen::MatrixXd frames_2 = arc_to_x(init_2,kappa,l,phi,5,true);
			
			
			if(curve_1.rows() != 4 || curve_1.cols() != 4*(3*25+1) || curve_2.rows() != 4 || curve_2.cols() != 4*(3*25+1) || frames_1.rows() != 4 || frames_1.cols() != 4*(3*5+1) || frames_2.rows() != 4 || frames_2.cols() != 4*(3*5+1))
			{
				std::cout << "Wrong dimensions of matrix storing the discrete frames (returned from arc_to_x)!" << std::endl;
			}
			else
			{
				mp_Vis->drawCurve(curve_1);
				mp_Vis->drawCurve(curve_2);
				mp_Vis->drawFrames(frames_1);
				mp_Vis->drawFrames(frames_2);
			}
			
		}
		if(*(iren->GetKeySym()) == 'f' && m_assignment == 1)
		{
			std::cout << "Drawing fourth set of curves..." << std::endl;
			mp_Vis->clear();
			
			//Define arc parameters
			std::vector<double> kappa;
			std::vector<double> l;
			std::vector<double> phi;
			
			kappa.push_back(0);
			l.push_back(0.15);
			phi.push_back(0);
			
			kappa.push_back(0);
			l.push_back(0.15);
			phi.push_back(0);
			
			Eigen::Matrix4d init_1 = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d init_2 = init_1;
			init_2(0,3) = 0.1;
			
			Eigen::MatrixXd curve_1 = arc_to_x(init_1,kappa,l,phi,25,false);
			Eigen::MatrixXd curve_2 = arc_to_x(init_2,kappa,l,phi,25,true);
			Eigen::MatrixXd frames_1 = arc_to_x(init_1,kappa,l,phi,5,false);
			Eigen::MatrixXd frames_2 = arc_to_x(init_2,kappa,l,phi,5,true);
			
			
			if(curve_1.rows() != 4 || curve_1.cols() != 4*(2*25+1) || curve_2.rows() != 4 || curve_2.cols() != 4*(2*25+1) || frames_1.rows() != 4 || frames_1.cols() != 4*(2*5+1) || frames_2.rows() != 4 || frames_2.cols() != 4*(2*5+1))
			{
				std::cout << "Wrong dimensions of matrix storing the discrete frames (returned from arc_to_x)!" << std::endl;
			}
			else
			{
				mp_Vis->drawCurve(curve_1);
				mp_Vis->drawCurve(curve_2);
				mp_Vis->drawFrames(frames_1);
				mp_Vis->drawFrames(frames_2);
			}
		}
		if(*(iren->GetKeySym()) == 'c' && m_assignment == 1)
		{
			std::cout << "Clearing currently drawn curves..." << std::endl;
			mp_Vis->clear();
		}
		
		//Test scenario interactions
		if(strcmp((iren->GetKeySym()),"Up") == 0 && m_assignment == 0)
		{
			Eigen::Vector2d dir(1,0);
			mp_Vis->moveCube(dir);
		}
		if(strcmp((iren->GetKeySym()),"Down") == 0 && m_assignment == 0)
		{
			Eigen::Vector2d dir(-1,0);
			mp_Vis->moveCube(dir);
		}
		if(strcmp((iren->GetKeySym()),"Left") == 0 && m_assignment == 0)
		{
			Eigen::Vector2d dir(0,-1);
			mp_Vis->moveCube(dir);
		}
		if(strcmp((iren->GetKeySym()),"Right") == 0 && m_assignment == 0)
		{
			Eigen::Vector2d dir(0,1);
			mp_Vis->moveCube(dir);
		}
		
		
		//Interactions for Assignment 2
		if(strcmp((iren->GetKeySym()),"Return") == 0 && m_assignment == 2)
		{
			//Calculate kinematics based on current configuration (which is indicated by the counter)
			Eigen::MatrixXd q = m_configs.at(m_counter);
			Eigen::Matrix4d ee_frame;
			Eigen::MatrixXd disk_frames;
			bool success = mp_TDCR->forward_kinematics(ee_frame,disk_frames, q);
			
			//Update visualizer
			if(success)
			{
				mp_Vis->updateTDCR(disk_frames);
				//Print current configuration and EE-Frame
				std::cout << "Current configuration:" << std::endl;
				std::cout << m_configs.at(m_counter).transpose() << std::endl;
				std::cout << "Current end-effector frame:" << std::endl;
				std::cout << ee_frame << std::endl << std::endl;
			}
			else
			{
				std::cout << "TDCR kinematics returned false!" << std::endl; 
			}
			
			
			//Counter up
			m_counter++;
			if(m_counter == m_configs.size())
				m_counter = 0;
				
			
		}
		
		//Interactions for Assignment 3
		if(strcmp((iren->GetKeySym()),"Return") == 0 && m_assignment == 3)
		{
			//Calculate kinematics based on current configuration (which is indicated by the counter)
			Eigen::MatrixXd q = m_configs.at(m_counter);
			Eigen::Matrix4d ee_frame;
			Eigen::MatrixXd backbone;
			std::vector<int> tube_ind;
			bool success = mp_CTCR->forward_kinematics(ee_frame,backbone, tube_ind, q);
			
			//Update visualizer
			if(success)
			{	
				mp_Vis->updateCTCR(backbone,tube_ind);
				//Print current configuration and EE-Frame
				std::cout << "Current configuration:" << std::endl;
				std::cout << m_configs.at(m_counter).transpose() << std::endl;
				std::cout << "Current end-effector frame:" << std::endl;
				std::cout << ee_frame << std::endl << std::endl;
			}
			else
			{
				std::cout << "CTCR kinematics returned false!" << std::endl; 
			}
			
			
			
			//Counter up
			m_counter++;
			if(m_counter == m_configs.size())
				m_counter = 0;
				
			
		}
		
		//Interactions for Assignment 4
		if(strcmp((iren->GetKeySym()),"Return") == 0 && m_assignment == 4)
		{
			m_control_loop_active = true;
				
			
		}

	}
}
