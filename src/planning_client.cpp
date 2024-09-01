#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
// #include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_msgs/action/get_motion_plan.hpp>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <planning_server/planning_server.h>

#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
// #include <tesseract_command_language/serialization.h>
#include <tesseract_common/serialization.h>
#include <tesseract_command_language/utils.h>
// #include <tesseract_task_composer/core/test_suite/test_programs.hpp>

// #include <tesseract_common/any_poly.h>
// #include <tesseract_command_language/serialization.h>
#include <tesseract_common/serialization.h>


#include <tesseract_common/manipulator_info.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <tesseract_rosutils/utils.h>


// #include <iostream>
// #include <typeinfo>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using tesseract_common::Serialization;
using namespace tesseract_environment;
using namespace tesseract_planning;

CompositeInstruction create_programe(const std::string& freespace_profile = DEFAULT_PROFILE_KEY,
                                                 const std::string& process_profile = "PROCESS")
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), DEFAULT_PROFILE_KEY.c_str()); //-->'DEFAULT'
    //defining oruientation fomr angle axis taken from SWORD
    Eigen::Vector3d axis(1.00, 0.0, 0.04);
    axis.normalize();
    double angle = 180.0 * M_PI / 180.0;
    Eigen::AngleAxisd angleAxis(angle, axis);
    Eigen::Quaterniond rotation(angleAxis);
    std::cout << "Quaternion: " << rotation.coeffs().transpose() << std::endl;


  double X=0.510;
  double Y=0.00264;
  double Z=0.68411;

  CompositeInstruction program(
      DEFAULT_PROFILE_KEY, CompositeInstructionOrder::ORDERED, tesseract_common::ManipulatorInfo("groupe", "world", "tcp_screw_gripper"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t" };

  StateWaypointPoly swp1{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction start_instruction(swp1, MoveInstructionType::FREESPACE, freespace_profile);
  start_instruction.setDescription("Start");

  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
                                               rotation) };

  // Define from start composite instruction
  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start(freespace_profile);
  from_start.setDescription("from_start");
  from_start.appendMoveInstruction(start_instruction);
  from_start.appendMoveInstruction(plan_f0);
  program.push_back(from_start);

  //
  // for (int i = 0; i < 4; ++i)
  // {
    // double x = 0.55 + (i * 0.001);

    
    // CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, y, Z) *
                                                  // rotation);
    CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
                                                  rotation);
    X+= 0.01; 
    CartesianWaypointPoly wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
                                                  rotation);
    X+= 0.01; 
    CartesianWaypointPoly wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
                                                  rotation);
    X+= 0.01; 
    CartesianWaypointPoly wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
                                                  rotation);
    // x+= 0.001; 
    // CartesianWaypointPoly wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
    //                                               rotation);
    // x+= 0.001; 
    // CartesianWaypointPoly wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
    //                                               rotation);
    // x+= 0.001; 
    // CartesianWaypointPoly wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
    //                                               rotation);

    CompositeInstruction raster_segment(process_profile);
    raster_segment.setDescription("Raster # 1");

    // if (i == 0 || i == 2)
    // {
      raster_segment.appendMoveInstruction(MoveInstruction(wp2, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp3, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp4, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp5, MoveInstructionType::LINEAR, process_profile));
      // raster_segment.appendMoveInstruction(MoveInstruction(wp6, MoveInstructionType::LINEAR, process_profile));
      // raster_segment.appendMoveInstruction(MoveInstruction(wp7, MoveInstructionType::LINEAR, process_profile));
    // }
    // else
    // {
    //   raster_segment.appendMoveInstruction(MoveInstruction(wp6, MoveInstructionType::LINEAR, process_profile));
    //   raster_segment.appendMoveInstruction(MoveInstruction(wp5, MoveInstructionType::LINEAR, process_profile));
    //   raster_segment.appendMoveInstruction(MoveInstruction(wp4, MoveInstructionType::LINEAR, process_profile));
    //   raster_segment.appendMoveInstruction(MoveInstruction(wp3, MoveInstructionType::LINEAR, process_profile));
    //   raster_segment.appendMoveInstruction(MoveInstruction(wp2, MoveInstructionType::LINEAR, process_profile));
    //   raster_segment.appendMoveInstruction(MoveInstruction(wp1, MoveInstructionType::LINEAR, process_profile));
    // }
    program.push_back(raster_segment);
    // program.push_back(raster_segment);
    // Add transition
    // if (i == 0 || i == 2)
    // {
    //   CartesianWaypointPoly wp7 =
    //       CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.55 + ((i + 1) * 0.001), 0.3, Z) *
    //                         rotation);

    //   MoveInstruction plan_f1(wp7, MoveInstructionType::FREESPACE, freespace_profile);
    //   plan_f1.setDescription("transition_from_end_plan");

    //   CompositeInstruction transition(freespace_profile);
    //   transition.setDescription("Transition #" + std::to_string(i + 1));
    //   transition.appendMoveInstruction(plan_f1);
    //   program.push_back(transition);
    // }
    // else if (i == 1)
    // {
    //   CartesianWaypointPoly wp1 =
    //       CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.55 + ((i + 1) * 0.001), -0.3, Z) *
    //                         rotation);

    //   MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE, freespace_profile);
    //   plan_f1.setDescription("transition_from_end_plan");

    //   CompositeInstruction transition(freespace_profile);
    //   transition.setDescription("Transition #" + std::to_string(i + 1));
    //   transition.appendMoveInstruction(plan_f1);
    //   program.push_back(transition);
    // }
  // }

  MoveInstruction plan_f2(swp1, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end(freespace_profile);
  to_end.setDescription("to_end");
  to_end.appendMoveInstruction(plan_f2);
  program.push_back(to_end);

  program.print("Program: ");

  return program;
}



class ActionClient : public rclcpp::Node
{
public:

  // USING
  using ActionT = tesseract_msgs::action::GetMotionPlan;
  using GoalHandleAction = rclcpp_action::ClientGoalHandle<ActionT>;

  explicit ActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_client", options)
  {
  //CREATE TRAJ PUBLISHER
  this->trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_topic", 10);
  
  // CREATE DISPLAY_TRAJECTORY_PUBLISHER
  this->display_trajectory_publisher_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("display_planned_path", 10);

  // CREATE ACTION CLIENT
  this->client_ptr_ = rclcpp_action::create_client<ActionT>(this,"tesseract_get_motion_plan");

  // WAIT FOR SERVER
  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      throw std::runtime_error("Action server not available");
    }
  // SEND REQUEST
    this->send_goal();
  }



  void send_goal()
  {
    // Create Goal
    auto goal_msg = ActionT::Goal();
    // Taskflow to use
    goal_msg.request.name = "RasterFtPipeline";
    // goal_msg.request.executor = "tututu";
    goal_msg.request.dotgraph= true;
    goal_msg.request.debug=true;
    goal_msg.request.save_io=true;
    // programme definition
    CompositeInstruction program = create_programe();
    //programme serialisation
    std::string any_composite_instruction_string = tesseract_common::Serialization::toArchiveStringXML<tesseract_common::AnyPoly>(program); 
    
    goal_msg.request.input = any_composite_instruction_string;
    


    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ActionClient::result_callback, this, std::placeholders::_1);
    
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ActionT>::SharedPtr client_ptr_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_publisher_;

  void publish_display_trajectory(trajectory_msgs::msg::JointTrajectory traj_msg)
  {
    // Create RobotTrajectory message
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = traj_msg;

    // Create display_traj message
    moveit_msgs::msg::DisplayTrajectory display_traj_msg;
    display_traj_msg.trajectory.push_back(robot_trajectory);


    RCLCPP_INFO(this->get_logger(), "Publishing DisplayTrajectory");

    this->display_trajectory_publisher_->publish(display_traj_msg);
  };

  void publish_trajectory(trajectory_msgs::msg::JointTrajectory msg)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing JointTrajectory");
    this->trajectory_publisher_->publish(msg);
  };

  void goal_response_callback(std::shared_ptr<GoalHandleAction> goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    std::shared_ptr<GoalHandleAction>,
    const std::shared_ptr<const ActionT::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback");
  }

  void result_callback(const GoalHandleAction::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
      {
        std::cout<<result.result->response.dotgraph<<std::endl;
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        
        //deserialise CI result
        tesseract_planning::InstructionPoly ci = Serialization::fromArchiveStringXML<tesseract_planning::InstructionPoly>(result.result->response.results);
        
        //generate intial state

        tesseract_scene_graph::SceneState scene_state;

        for (const auto& pair : result.result->response.initial_state)
        {
          scene_state.joints[pair.first] = pair.second;
        }

        //CI to tesseract_traj
        tesseract_common::JointTrajectory tesseract_joint_trajectory = toJointTrajectory(ci);
        //est ce que les time stamp sont OK dans la traj tesseract?
          for (const auto& states : tesseract_joint_trajectory)
            {
              // std::cout<<states.time_from_start<<std::endl;
              std::cout<<states.time<<std::endl;
              std::cout<<'22222222222222222'<<std::endl;
              std::cout<<'AAAAAAAAAAAAAAAAA'<<std::endl;
              std::cout<<'22222222222222222'<<std::endl;
            }
              // current_point.time_from_start = rclcpp::Duration::from_seconds(states.time);
        //Ci & initial state to ros msg
        trajectory_msgs::msg::JointTrajectory ros_msg = tesseract_rosutils::toMsg(tesseract_joint_trajectory,scene_state);

        // // tesseract_scene_graph::SceneState scene_state;

        // // initial_state_data=result.result->response.initial_state
        // std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        // std::cout<<typeid(result.result->response.initial_state).name()<<std::endl; //St6vectorIN14tesseract_msgs3msg17StringDoublePair_ISaIvEEESaIS4_EE


        // // for (const auto& pair : initial_state_data)
        // //   {
        // //     scene_state.joints[pair.first] = pair.second;
        // //   }
        // // tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        publish_trajectory(ros_msg);
        publish_display_trajectory(ros_msg);
        break;
      }
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    // Process the result
    RCLCPP_INFO(this->get_logger(), "Result received");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ActionClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////




// **** The name of the taskflow to use ****
// - TrajOptIfoptTask
// - TrajOptPipeline
// - RasterCtPipeline
// - DescartesFTask
// - RasterFtPipeline
// - FreespaceIfoptTask
// - DescartesFNPCPipeline
// - RasterCtGlobalTask
// - DescartesDTask
// - RasterFtGlobalTask
// - RasterCtOnlyGlobalTask
// - DescartesDPipeline
// - DescartesDNPCTask
// - RasterFtOnlyPipeline
// - DescartesDNPCPipeline
// - RasterCtOnlyTask
// - TrajOptIfoptPipeline
// - DescartesFNPCTask
// - CartesianTask
// - CartesianPipeline
// - FreespaceTask
// - RasterFtOnlyTask
// - FreespacePipeline
// - RasterCtGlobalPipeline
// - OMPLPipeline
// - RasterFtTask
// - DescartesFPipeline
// - OMPLTask
// - RasterCtOnlyGlobalPipeline
// - TrajOptTask
// - RasterCtOnlyPipeline
// - RasterCtTask
// - RasterFtGlobalPipeline
// - RasterFtOnlyGlobalPipeline
// - FreespaceIfoptPipeline
// - RasterFtOnlyGlobalTask