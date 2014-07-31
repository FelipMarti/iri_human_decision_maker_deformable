#include "human_decision_maker_deformable_alg_node.h"

HumanDecisionMakerDeformableAlgNode::HumanDecisionMakerDeformableAlgNode
    (void):algorithm_base::IriBaseAlgorithm <
    HumanDecisionMakerDeformableAlgorithm > (),
pick_place_defo_client_("pick_place_defo", true),
server_IM("desired_pos_marker")
{
	//init class attributes if necessary
	this->My_State = 0;

	this->place_point.X = 0;
	this->place_point.Y = 0;
	this->place_point.Z = 0;
	this->pick_point.X = 0;
	this->pick_point.Y = 0;
	this->pick_point.Z = 0;
	this->yaw_EF_orientation = 0;

	//init Interactive Marker
	init_interactive_markers(this->server_IM);

	//this->loop_rate_ = 2;//in [Hz]

	// [init publishers]

	// [init subscribers]

	// [init services]

	// [init clients]
	interest_points_client_ =
	    this->public_node_handle_.serviceClient <
	    iri_color_interesting_points_deformable::InterestPoints >
	    ("interest_points");

	// [init action servers]

	// [init action clients]
}

HumanDecisionMakerDeformableAlgNode::~HumanDecisionMakerDeformableAlgNode(void)
{
	// [free dynamic memory]
}

void HumanDecisionMakerDeformableAlgNode::mainNodeThread(void)
{
	// [fill msg structures]

	// [fill srv structure and make request to the server]

	// [fill action structure and make request to the action server]

	// [publish messages]

	/// My state machine
	// Estat=0 => Ask the server for the points of interest 
	// Estat=1 => Ask the human which point of interest pick
	// Estat=2 => Ask to the human for the interactive Marker..
	// Estat=3 => Call Pick & Place 
	// Estat=4 => Wait Pick & Place 

	if (this->My_State == 0) {	// Ask the server and store all points of interest

		Num_Int_Points = 0;
		ListInterestPoints.clear();

		//Send Interest Points request
		ROS_INFO
		    ("HumanDecisionMakerDeformableAlgNode:: Sending New Request!");
		if (interest_points_client_.call(interest_points_srv_)) {
			ROS_INFO
			    ("HumanDecisionMakerDeformableAlgNode:: We have Response");
			//Store Points of Interest
			Num_Int_Points = interest_points_srv_.response.amount;
			int i = 0;
			this->ListInterestPoints.resize(Num_Int_Points);
			while (i < Num_Int_Points) {
				ListInterestPoints[i].U =
				    interest_points_srv_.response.U[i];
				ListInterestPoints[i].V =
				    interest_points_srv_.response.V[i];
				ListInterestPoints[i].X =
				    interest_points_srv_.response.X[i];
				ListInterestPoints[i].Y =
				    interest_points_srv_.response.Y[i];
				ListInterestPoints[i].Z =
				    interest_points_srv_.response.Z[i];
				ListInterestPoints[i].color =
				    interest_points_srv_.response.Color[i];
				ListInterestPoints[i].object_type =
				    interest_points_srv_.
				    response.object_type[i];
				i++;
			}
		}
		else
			//The request Failed, so we stay in this state 
			ROS_INFO
			    ("HumanDecisionMakerDeformableAlgNode::FailCall Server topic interest_points ");

		if (Num_Int_Points > 0)
			My_State++;
		else
			ROS_INFO
			    ("HumanDecisionMakerDeformableAlgNode:: NO Interested Points obtained!");

	}
	else if (this->My_State == 1) {	// Ask Human where to move

		//Some interested points has been obtained
		ListGraspPoints.clear();

		// Ask to select one...
		ROS_INFO("[MAIN]: HEY HUMAN! ");
		ROS_INFO("[MAIN]: Where do you want to move the robot?");
		int i = 0;
		int Color_Point_ant = -1;

		//Show diff colors detected
		while (i < Num_Int_Points) {
			if (ListInterestPoints[i].color != Color_Point_ant) {
				ROS_INFO("#%d", ListInterestPoints[i].color);
			}
			Color_Point_ant = ListInterestPoints[i].color;
			i++;
		}

		// Human entrance       
		ROS_INFO("[MAIN]: Insert Number:");
		int GraspPointColor = -1;
		std::cin >> GraspPointColor;

		//Check the entrance
		bool trobat = false;
		i = 0;
		while (i < Num_Int_Points && !trobat) {
			if (ListInterestPoints[i].color == GraspPointColor)
				trobat = true;
			else
				i++;
		}

		//Add all the grasp points of the object/color selected by the human
		if (trobat) {
			ListGraspPoints.push_back(ListInterestPoints[i]);
			My_State++;
		}
		else {
			ROS_INFO
			    ("[MAIN]: Wrong number inserted. Asking new Interested Points");
			My_State = 0;
		}

	}
	else if (this->My_State == 2) {	//Interactive Marker

		int option = 0;

		do {
			ROS_INFO
			    ("[MAIN]: What about the Drop point: (%.4f,%.4f,%.4f) ?",
			     place_point.X, place_point.Y, place_point.Z);
			ROS_INFO
			    ("[MAIN]: If it is not OK move the Interactive Marker. Is it OK? [1=Yes/0=N]");

			std::cin >> option;

		} while (option != 1);

		My_State++;

	}
	else if (this->My_State == 3) {	//Pick & Place

		if (ListGraspPoints[0].object_type == 'C') {	//Grasping Centroide (cloth)
			pick_point.X = ListGraspPoints[0].X;
			pick_point.Y = ListGraspPoints[0].Y;
			pick_point.Z = ListGraspPoints[0].Z;

			pick_place_defoMakeActionRequest();

		}

		My_State = 4;

	}

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void HumanDecisionMakerDeformableAlgNode::
pick_place_defoDone(const actionlib::SimpleClientGoalState & state,
		    const
		    iri_pickandplace_deformable::PaPDeformableResultConstPtr &
		    result)
{
	alg_.lock();
	if (state.toString().compare("SUCCEEDED") == 0) {
		ROS_INFO
		    ("HumanDecisionMakerDeformableAlgNode::pick_place_defoDone: Goal Achieved!");
		My_State = 0;
	}
	else {
		ROS_INFO
		    ("HumanDecisionMakerDeformableAlgNode::pick_place_defoDone: %s",
		     state.toString().c_str());
		My_State = 0;
	}
	//copy & work with requested result 
	alg_.unlock();
}

void HumanDecisionMakerDeformableAlgNode::pick_place_defoActive()
{
	alg_.lock();
	//ROS_INFO("HumanDecisionMakerDeformableAlgNode::pick_place_defoActive: Goal just went active!"); 
	alg_.unlock();
}

void HumanDecisionMakerDeformableAlgNode::
pick_place_defoFeedback(const
			iri_pickandplace_deformable::PaPDeformableFeedbackConstPtr
			& feedback)
{
	alg_.lock();
	//ROS_INFO("HumanDecisionMakerDeformableAlgNode::pick_place_defoFeedback: Got Feedback!"); 

	bool feedback_is_ok = true;

	//analyze feedback 
	//my_var = feedback->var; 

	//if feedback is not what expected, cancel requested goal 
	if (!feedback_is_ok) {
		pick_place_defo_client_.cancelGoal();
		//ROS_INFO("HumanDecisionMakerDeformableAlgNode::pick_place_defoFeedback: Cancelling Action!"); 
	}
	alg_.unlock();
}

/*  [action requests] */
void HumanDecisionMakerDeformableAlgNode::pick_place_defoMakeActionRequest()
{
	ROS_INFO
	    ("HumanDecisionMakerDeformableAlgNode::pick_place_defoMakeActionRequest: Starting New Request!");

	//wait for the action server to start 
	//will wait for infinite time 
	pick_place_defo_client_.waitForServer();
	ROS_INFO
	    ("HumanDecisionMakerDeformableAlgNode::pick_place_defoMakeActionRequest: Server is Available!");

	//send a goal to the action 
	pick_place_defo_goal_.pick_pos[0] = pick_point.X;
	pick_place_defo_goal_.pick_pos[1] = pick_point.Y;
	pick_place_defo_goal_.pick_pos[2] = pick_point.Z;

	pick_place_defo_goal_.place_pos[0] = place_point.X;
	pick_place_defo_goal_.place_pos[1] = place_point.Y;
	pick_place_defo_goal_.place_pos[2] = place_point.Z;

	pick_place_defo_goal_.yaw = yaw_EF_orientation;

	//pick_place_defo_goal_.data = my_desired_goal; 
	pick_place_defo_client_.sendGoal(pick_place_defo_goal_,
					 boost::bind
					 (&HumanDecisionMakerDeformableAlgNode::pick_place_defoDone,
					  this, _1, _2),
					 boost::bind
					 (&HumanDecisionMakerDeformableAlgNode::pick_place_defoActive,
					  this),
					 boost::bind
					 (&HumanDecisionMakerDeformableAlgNode::pick_place_defoFeedback,
					  this, _1));
	ROS_INFO
	    ("HumanDecisionMakerDeformableAlgNode::pick_place_defoMakeActionRequest: Goal Sent. Wait for Result!");
}

void HumanDecisionMakerDeformableAlgNode::node_config_update(Config & config,
							     uint32_t level)
{
	this->alg_.lock();

	this->alg_.unlock();
}

void HumanDecisionMakerDeformableAlgNode::addNodeDiagnostics(void)
{
}

/* INTERACTIVE MARKERS */

void HumanDecisionMakerDeformableAlgNode::
processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &
		feedback)
{
//  ROS_INFO_STREAM( feedback->marker_name << " is now at "
//      << feedback->pose.position.x << ", " << feedback->pose.position.y
//      << ", " << feedback->pose.position.z );

	this->place_point.X = feedback->pose.position.x;
	this->place_point.Y = feedback->pose.position.y;
	this->place_point.Z = feedback->pose.position.z;

}

void HumanDecisionMakerDeformableAlgNode::
init_interactive_markers(interactive_markers::InteractiveMarkerServer & server)
{

	ROS_INFO("[INTERACTIVE_MARKER]: Init!!");

	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "camera_rgb_optical_frame";
	int_marker.name = "my_marker";
	int_marker.description = "Desired position";

	// create a grey box marker
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::CUBE;
	box_marker.scale.x = 0.045;
	box_marker.scale.y = 0.045;
	box_marker.scale.z = 0.045;
	box_marker.color.r = 0.5;
	box_marker.color.g = 0.5;
	box_marker.color.b = 0.5;
	box_marker.color.a = 1.0;

	// create a non-interactive control which contains the box
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;
	box_control.markers.push_back(box_marker);

	// add the control to the interactive marker
	int_marker.controls.push_back(box_control);

	// create a control which will move the box
	// this control does not contain any markers,
	// which will cause RViz to insert two arrows
	visualization_msgs::InteractiveMarkerControl rotate_control;
	rotate_control.name = "move_x";
	rotate_control.interaction_mode =
	    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

	// add the control to the interactive marker
	int_marker.controls.push_back(rotate_control);

	//Y AXIS
	rotate_control.orientation.w = 1;
	rotate_control.orientation.x = 0;
	rotate_control.orientation.y = 0;
	rotate_control.orientation.z = 1;
	rotate_control.name = "move_y";
	rotate_control.interaction_mode =
	    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(rotate_control);

	//Z AXIS
	rotate_control.orientation.w = 1;
	rotate_control.orientation.x = 0;
	rotate_control.orientation.y = 1;
	rotate_control.orientation.z = 0;
	rotate_control.name = "move_z";
	rotate_control.interaction_mode =
	    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(rotate_control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	server.insert(int_marker,
		      boost::
		      bind
		      (&HumanDecisionMakerDeformableAlgNode::processFeedback,
		       this, _1));

	// 'commit' changes and send to all clients
	server.applyChanges();

	ROS_INFO("[INTERACTIVE_MARKER]: FII Init!!");

}

/* main function */
int main(int argc, char *argv[])
{
	return algorithm_base::main < HumanDecisionMakerDeformableAlgNode >
	    (argc, argv, "human_decision_maker_deformable_alg_node");
}
