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

#include <tesseract_task_composer/core/test_suite/test_programs.hpp>

// #include <tesseract_common/any_poly.h>
// #include <tesseract_command_language/serialization.h>


using namespace tesseract_environment;
using namespace tesseract_planning;


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
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result received");
      break;
    case rclcpp_action::ResultCode::ABORTED:
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
        tesseract_common::ManipulatorInfo("groupe", "base_link", "tcp")
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
  // Create action client
  auto action_client = rclcpp_action::create_client<tesseract_msgs::action::GetMotionPlan>(node, "tesseract_get_motion_plan");
  // WAIT for action server
  if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
  RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
  rclcpp::shutdown();
  return 1;
}
  auto goal_msg = tesseract_msgs::action::GetMotionPlan::Goal();
  goal_msg.request.name = "OMPLTask";
  tesseract_planning::CompositeInstruction  program;
  
  create_programe(program);
 std::cout << "Type de program: " << typeid(program).name() << std::endl;

  // to anypoly
  tesseract_common::AnyPoly any_composite_instruction(program);

  CompositeInstruction program2 = test_suite::rasterExampleProgram();
  // tesseract_planning::InstructionPoly any_composite_instruction(program);
  std::cout << "Type de any_composite_instruction: " << typeid(any_composite_instruction).name() << std::endl;
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