// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _human_decision_maker_deformable_alg_node_h_
#define _human_decision_maker_deformable_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "human_decision_maker_deformable_alg.h"

// [publisher subscriber headers]

// [service client headers]
#include <iri_color_interesting_points_deformable/InterestPoints.h>

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iri_pickandplace_deformable/PaPDeformableAction.h>

// My Includes 
#include <interactive_markers/interactive_marker_server.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class HumanDecisionMakerDeformableAlgNode:public
    algorithm_base::IriBaseAlgorithm < HumanDecisionMakerDeformableAlgorithm > {
 private:
	// [publisher attributes]

	// [subscriber attributes]

	// [service attributes]

	// [client attributes]
	ros::ServiceClient interest_points_client_;
	iri_color_interesting_points_deformable::InterestPoints
	    interest_points_srv_;

	// [action server attributes]

	// [action client attributes]
	actionlib::SimpleActionClient <
	    iri_pickandplace_deformable::PaPDeformableAction >
	    pick_place_defo_client_;
	iri_pickandplace_deformable::PaPDeformableGoal pick_place_defo_goal_;
	void pick_place_defoMakeActionRequest();
	void pick_place_defoDone(const actionlib::SimpleClientGoalState & state,
				 const
				 iri_pickandplace_deformable::PaPDeformableResultConstPtr
				 & result);
	void pick_place_defoActive();
	void pick_place_defoFeedback(const
				     iri_pickandplace_deformable::PaPDeformableFeedbackConstPtr
				     & feedback);

	/* MY CLASS VARS */
	struct PointInterest {
		int color;
		int U;
		int V;
		float X;
		float Y;
		float Z;
		char object_type;
	};

	// My State Machine variable
	int My_State;

	// Num of interest points that arrive
	int Num_Int_Points;

	// List of Interest Points
	std::vector < PointInterest > ListInterestPoints;
	//List of points to grasp an object
	std::vector < PointInterest > ListGraspPoints;

	// Pick and Drop Robot 3D position   
	HumanDecisionMakerDeformableAlgorithm::point_XYZ pick_point;
	HumanDecisionMakerDeformableAlgorithm::point_XYZ place_point;
	float yaw_EF_orientation;

	// Create an interactive marker server on the topic namespace simple_marker
	interactive_markers::InteractiveMarkerServer server_IM;

 public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
	HumanDecisionMakerDeformableAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
	~HumanDecisionMakerDeformableAlgNode(void);

 protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
	void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
	void node_config_update(Config & config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
	void addNodeDiagnostics(void);

	// [diagnostic functions]

	// [test functions]

	/* MY INTERACTIVE MARKERS FUNCTIONS */
	// TODO: Move to human_decision_maker_deformable_alg
	void processFeedback(const
			     visualization_msgs::InteractiveMarkerFeedbackConstPtr
			     & feedback);
	void init_interactive_markers(interactive_markers::
				      InteractiveMarkerServer & server);

};

#endif
