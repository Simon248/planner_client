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
using tesseract_common::Serialization;
using namespace tesseract_environment;
using namespace tesseract_planning;



CompositeInstruction rasterExampleProgram(const std::string& freespace_profile = DEFAULT_PROFILE_KEY,
                                                 const std::string& process_profile = "PROCESS")
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), DEFAULT_PROFILE_KEY.c_str()); //-->'DEFAULT'

  //ORIENTATION

    // double w = 2.67935e-08, x = 0.999949, y = -2.7098e-10, z = 0.0101133;
    // Eigen::Quaterniond rotation(w, x, y, z);

    Eigen::Vector3d axis(-0.79, 0.0, -0.61);
    axis.normalize();
    double angle = 180.0 * M_PI / 180.0;
    Eigen::AngleAxisd angleAxis(angle, axis);
    Eigen::Quaterniond rotation(angleAxis);

    // Afficher le quaternion
    std::cout << "Quaternion: " << rotation.coeffs().transpose() << std::endl;

  double Z=1.0;
  CompositeInstruction program(
      DEFAULT_PROFILE_KEY, CompositeInstructionOrder::ORDERED, tesseract_common::ManipulatorInfo("groupe", "world", "tcp_screw_gripper"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t" };
  StateWaypointPoly swp1{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction start_instruction(swp1, MoveInstructionType::FREESPACE, freespace_profile);
  start_instruction.setDescription("Start");

  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.4, 0.0, Z) *
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
    double X=0.89;
    double Y=-0.01;
    
    // CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, y, Z) *
                                                  // rotation);
    CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
                                                  rotation);
    X+= 0.001; 
    CartesianWaypointPoly wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
                                                  rotation);
    X+= 0.001; 
    CartesianWaypointPoly wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(X, Y, Z) *
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
      // raster_segment.appendMoveInstruction(MoveInstruction(wp5, MoveInstructionType::LINEAR, process_profile));
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




void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<tesseract_msgs::action::GetMotionPlan>> goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, waiting for result");
  }
}

void feedback_callback(
  std::shared_ptr<rclcpp_action::ClientGoalHandle<tesseract_msgs::action::GetMotionPlan>>,
  const std::shared_ptr<const tesseract_msgs::action::GetMotionPlan::Feedback> feedback)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received feedback");
}

void result_callback(const rclcpp_action::ClientGoalHandle<tesseract_msgs::action::GetMotionPlan>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result received");
      tesseract_planning::InstructionPoly ci = Serialization::fromArchiveStringXML<tesseract_planning::InstructionPoly>(result.result->response.results);
      tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), trajectory.description.c_str());

      trajectory_msgs::msg::JointTrajectory msg;
  
      // Set the joint names
      msg.joint_names = joint_trajectory.joint_names;
      // Set the points
      for (const auto& point : joint_trajectory.trajectory)
        {
          trajectory_msgs::msg::JointTrajectoryPoint traj_point;
          traj_point.positions = point.positions;
          traj_point.velocities = point.velocities;
          traj_point.accelerations = point.accelerations;
          traj_point.time_from_start = rclcpp::Duration(point.time_from_start);
          msg.points.push_back(traj_point);
        }
    }

      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
      break;
  }
  rclcpp::shutdown();
}


void create_programe(tesseract_planning::CompositeInstruction & program)
{
using namespace tesseract_planning;

// Créer et initialiser le programme
    program = CompositeInstruction(
        "cartesian_program",
        CompositeInstructionOrder::ORDERED,
        tesseract_common::ManipulatorInfo("groupe", "world", "tool0")
    );

  std::vector<std::string> joint_names;
  joint_names.emplace_back("joint_1_s");
  joint_names.emplace_back("joint_2_l");
  joint_names.emplace_back("joint_3_u");
  joint_names.emplace_back("joint_4_r");
  joint_names.emplace_back("joint_5_b");
  joint_names.emplace_back("joint_6_t");

  Eigen::VectorXd joint_pos(6);
  joint_pos(0) = 0.0;
  joint_pos(1) = 0.0;
  joint_pos(2) = 0.0;
  joint_pos(3) = 0.0;
  joint_pos(4) = 0.0;
  joint_pos(5) = 0.0;


StateWaypointPoly wp0{ StateWaypoint(joint_names, joint_pos) };
MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
start_instruction.setDescription("Start Instruction");

// Create cartesian waypoint
CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.5, -0.2, 0.62) *
                                              Eigen::Quaterniond(0, 0, 1.0, 0)) };

CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.5, 0.3, 0.62) *
                                              Eigen::Quaterniond(0, 0, 1.0, 0)) };
// Plan freespace from start
MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "freespace_profile");
plan_f0.setDescription("from_start_plan");

// Plan linear move
MoveInstruction plan_c0(wp2, MoveInstructionType::LINEAR, "RASTER");

// Plan freespace to end
MoveInstruction plan_f1(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
plan_f1.setDescription("to_end_plan");

// Add Instructions to program
program.appendMoveInstruction(start_instruction);
program.appendMoveInstruction(plan_f0);
program.appendMoveInstruction(plan_c0);
program.appendMoveInstruction(plan_f1);

// Print Diagnostics
program.print("Program: ");
// CONSOLE_BRIDGE_logInform("basic cartesian plan example");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Create node
  auto node = std::make_shared<rclcpp::Node>("planning_client");
  publisher_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_topic", 10);
  // Create action client
  auto action_client = rclcpp_action::create_client<tesseract_msgs::action::GetMotionPlan>(node, "tesseract_get_motion_plan");
  // WAIT for action server
  if (!action_client->wait_for_action_server(std::chrono::seconds(10))) 
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return 1;
  }
  auto goal_msg = tesseract_msgs::action::GetMotionPlan::Goal();
  goal_msg.request.name = "TrajOptPipeline";

  // tesseract_planning::CompositeInstruction  program;
  
  // create_programe(program);
  // std::cout << "Type de program: " << typeid(program).name() << std::endl;

  // to anypoly
  // tesseract_common::AnyPoly any_composite_instruction(program);

  CompositeInstruction program2 = rasterExampleProgram();
  // tesseract_planning::InstructionPoly any_composite_instruction(program);
  // std::cout << "Type de any_composite_instruction: " << typeid(any_composite_instruction).name() << std::endl;
  //serialize
  std::string any_composite_instruction_string = 
  tesseract_common::Serialization::toArchiveStringXML<tesseract_common::AnyPoly>(program2); 

  //   result->response.results = Serialization::toArchiveStringXML<tesseract_planning::InstructionPoly>(
  //       results.as<tesseract_planning::CompositeInstruction>());

  // bool test= tesseract_planning::Serialization::toArchiveStringXML(any_composite_instruction,
  //                              "test_tesseract_serialisation.xml",
  //                              "any_composite_instruction");
  // Afficher la chaîne sérialisée
  // std::cout << any_composite_instruction_string << std::endl;
    
  // result->response.results = Serialization::toArchiveStringXML<tesseract_planning::InstructionPoly>(
  //   results.as<tesseract_planning::CompositeInstruction>());


  goal_msg.request.input= any_composite_instruction_string;

  // Remplir le goal_msg ici avec les informations nécessaires

  RCLCPP_INFO(node->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<tesseract_msgs::action::GetMotionPlan>::SendGoalOptions();
  send_goal_options.goal_response_callback = goal_response_callback;
  send_goal_options.feedback_callback = feedback_callback;
  send_goal_options.result_callback = result_callback;

  action_client->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_INFO(node->get_logger(), "Planning Client Running!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
  
//   // Create Process Planning Request
//   tesseract_msgs::msg::PlanningRequest request;
  
//   // request.environment_state =
//   // request.commands =

//   request.name = "TaskflowExecutor"; //c'est pas ca mais le serveur me dire les availables
    
  
//   // request.executor  if empty default one is used.
//   // request.input
//   // request.dotgraph = false;
//   // request.debug = true;
//   // request.save_io = false;
//   // request.move_profile_remapping
//   // request.composite_profile_remapping 
  
//   // request.instructions = Instruction(program);  // EXISTE PLUS
//   // Print Diagnostics
//   // request.instructions.print("Program: "); // EXISTE PLUS
//   // Instruction program = rasterExampleProgram();

//   RCLCPP_INFO(node->get_logger(), "Planning Client Running!");
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }



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