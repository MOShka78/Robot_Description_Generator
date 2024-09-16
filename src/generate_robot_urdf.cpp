#include <unistd.h>

#include <generate_robot_urdf/generate_robot_urdf.hpp>
#include <rclcpp/rclcpp.hpp>
GenerateRobotURDF::GenerateRobotURDF()
    : xacro_macro_file(
          "src/generate_robot_urdf/config/output/robot_macro.xacro") {
  xacro_macro_file << "<?xml version=\"1.0\"?>\n";
  xacro_macro_file << "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\">\n";
  getURDF();
  generateYAMLlimit();
  setProperty();
  generateURDFmacro();
}
GenerateRobotURDF::~GenerateRobotURDF() {}

void GenerateRobotURDF::getURDF() {
  std::string full_path_descr =
      ament_index_cpp::get_package_share_directory("generate_robot_urdf") +
      "/config/input/urdf_solid.urdf";
  tf_tree_ = urdf::parseURDFFile(full_path_descr);

  xacro_macro_file << " <xacro:macro name=\"" << tf_tree_->name_
                   << "\" params=\"parent_link\">\n\n";
}

void GenerateRobotURDF::generateYAMLlimit() {
  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "joint_limits" << YAML::Value << YAML::BeginMap;

  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    if (joint_value->limits != nullptr) {
      emitter << YAML::Key << joint_name << YAML::Value << YAML::BeginMap;

      emitter << YAML::Key << "min_position" << YAML::Value
              << joint_value->limits->lower;
      emitter << YAML::Key << "max_positiom" << YAML::Value
              << joint_value->limits->upper;
      emitter << YAML::Key << "max_effort" << YAML::Value
              << joint_value->limits->effort;
      emitter << YAML::Key << "max_velocity" << YAML::Value
              << joint_value->limits->velocity;
      emitter << YAML::EndMap;
    }
  }
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "link_mass" << YAML::Value << YAML::BeginMap;
  for (const auto& [link_name, link_value] : tf_tree_->links_) {
    emitter << YAML::Key << link_name << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "mass" << YAML::Value << link_value->inertial->mass;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;

  std::ofstream fout("src/generate_robot_urdf/config/output/joint_limits.yaml");
  fout << emitter.c_str();
  fout.close();
}

void GenerateRobotURDF::setProperty() {
  xacro_macro_file
      << "   <xacro:property name=\"yaml_path\" value=\"$(find "
         "generate_robot_urdf)/config/output/joint_limits.yaml\" />\n";
  xacro_macro_file << "   <xacro:property name=\"yaml_file\" "
                      "value=\"${xacro.load_yaml(yaml_path)}\"/>\n\n";

  xacro_macro_file << "   <!-- link mass [kg] -->\n";
  for (const auto& [link_name, link_value] : tf_tree_->links_) {
    xacro_macro_file << "   <xacro:property name=\"" << link_name << "_mass"
                     << "\" "
                        "value=\"${yaml_file['link_mass']['"
                     << link_name << "']['mass']}\"/>\n";
  }

  xacro_macro_file << "\n   <!-- joint limits [rad] -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    xacro_macro_file << "   <xacro:property name=\"" << joint_name
                     << "_lower_limit"
                     << "\" "
                        "value=\"${yaml_file['joints_limits']['"
                     << joint_name << "']['min_position']}\"/>\n";
    xacro_macro_file << "   <xacro:property name=\"" << joint_name
                     << "_upper_limit"
                     << "\" "
                        "value=\"${yaml_file['joints_limits']['"
                     << joint_name << "']['max_position']}\"/>\n";
  }

  xacro_macro_file << "\n   <!-- joint velocity limits [rad/s] -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    xacro_macro_file << "   <xacro:property name=\"" << joint_name
                     << "_velocity_limit"
                     << "\" "
                        "value=\"${yaml_file['joints_limits']['"
                     << joint_name << "']['max_velocity']}\"/>\n";
  }

  xacro_macro_file << "\n   <!-- joint effort limits -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    xacro_macro_file << "   <xacro:property name=\"" << joint_name
                     << "_effort_limit"
                     << "\" "
                        "value=\"${yaml_file['joints_limits']['"
                     << joint_name << "']['max_effort']}\"/>\n";
  }
}

void GenerateRobotURDF::generateURDFmacro() {
  for (const auto& [link_name, link_value] : tf_tree_->links_) {
    xacro_macro_file << "\n\n   <!-- " << link_name << " -->\n";
    if (link_value->parent_joint != nullptr) {
      xacro_macro_file << "    <joint name=\"" << link_value->parent_joint->name
                       << "\" type=\""
                       << getTypeJoint(link_value->parent_joint->type)
                       << "\">\n";
      if (link_value->parent_joint->limits != nullptr) {
        xacro_macro_file << "     <axis xyz=\""
                         << link_value->parent_joint->axis.x << " "
                         << link_value->parent_joint->axis.y << " "
                         << link_value->parent_joint->axis.z
                         << "\" rpy=\"0 0 0\"/>\n";

        xacro_macro_file << "     <limit effort=\"${"
                         << link_value->parent_joint->name
                         << "_effort_limit}\" lower=\"${"
                         << link_value->parent_joint->name
                         << "_lower_limit}\" "
                            "upper=\"${"
                         << link_value->parent_joint->name
                         << "_upper_limit}\" "
                            "velocity=\"${"
                         << link_value->parent_joint->name
                         << "_velocity_limit}\"/>\n";
      }
      xacro_macro_file << "     <parent link=\""
                       << link_value->getParent()->name << "\"/>\n";
      xacro_macro_file << "     <child link=\"" << link_value->name << "\"/>\n";
      double roll, pitch, yaw;

      link_value->parent_joint->parent_to_joint_origin_transform.rotation
          .getRPY(roll, pitch, yaw);
      xacro_macro_file << "     <origin xyz=\""
                       << link_value->parent_joint
                              ->parent_to_joint_origin_transform.position.x
                       << " "
                       << link_value->parent_joint
                              ->parent_to_joint_origin_transform.position.y
                       << " "
                       << link_value->parent_joint
                              ->parent_to_joint_origin_transform.position.z
                       << "\" rpy=\"" << roll << " " << pitch << " " << yaw
                       << "\" />\n";
      xacro_macro_file << "   </joint>\n";
    } else {
      xacro_macro_file << "    <joint name=\""
                       << "${parent_link}-" << link_value->name
                       << "\" type=\"fixed\">\n";
      xacro_macro_file << "     <parent link=\"${parent_link}\"/>\n";
      xacro_macro_file << "     <child link=\"" << link_value->name << "\"/>\n";
      xacro_macro_file << "     <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
      xacro_macro_file << "   </joint>\n";
    }

    xacro_macro_file << "   <link name=\"" << link_name << "\">\n";
    xacro_macro_file << "     <visual>\n";
    xacro_macro_file << "       <geometry>\n";
    xacro_macro_file
        << "         <mesh filename=\"package://package_name/TODO.STL\"/>\n";
    xacro_macro_file << "       </geometry>\n";
    xacro_macro_file << "       <material name=\""
                     << link_value->visual->material_name << "\">\n";
    xacro_macro_file << "         <color rgba=\""
                     << link_value->visual->material->color.r << " "
                     << link_value->visual->material->color.g << " "
                     << link_value->visual->material->color.b << " "
                     << link_value->visual->material->color.a << "\" />\n";
    xacro_macro_file << "       </material>\n";

    double roll_v, pitch_v, yaw_v;
    link_value->visual->origin.rotation.getRPY(roll_v, pitch_v, yaw_v);

    xacro_macro_file << "       <origin xyz=\""
                     << link_value->visual->origin.position.x << " "
                     << link_value->visual->origin.position.y << " "
                     << link_value->visual->origin.position.z << "\" rpy=\""
                     << roll_v << " " << pitch_v << " " << yaw_v << "\"/>\n";
    xacro_macro_file << "     </visual>\n";
    xacro_macro_file << "     <collision>\n";
    xacro_macro_file << "       <geometry>\n";
    xacro_macro_file
        << "         <mesh filename=\"package://package_name/TODO.STL\"/>\n";
    xacro_macro_file << "       </geometry>\n";

    double roll_c, pitch_c, yaw_c;
    link_value->collision->origin.rotation.getRPY(roll_c, pitch_c, yaw_c);

    xacro_macro_file << "       <origin xyz=\""
                     << link_value->collision->origin.position.x << " "
                     << link_value->collision->origin.position.y << " "
                     << link_value->collision->origin.position.z << "\" rpy=\""
                     << roll_c << " " << pitch_c << " " << yaw_c << "\"/>\n";
    xacro_macro_file << "     </collision>\n";
    xacro_macro_file << "     <inertial>\n";
    xacro_macro_file << "       <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
    xacro_macro_file << "       <mass value=\"${" << link_name << "_mass}"
                     << "\"/>\n";
    xacro_macro_file << "       <inertia ixx=\"" << link_value->inertial->ixx
                     << "\" ixy=\"" << link_value->inertial->ixy << "\" ixz=\""
                     << link_value->inertial->ixz << "\" iyy=\""
                     << link_value->inertial->iyy << "\" iyz=\""
                     << link_value->inertial->iyz << "\" izz=\""
                     << link_value->inertial->izz << "\"/>\n";
    xacro_macro_file << "     </inertial>\n";
    xacro_macro_file << "   </link>\n";
  }
  xacro_macro_file << " </xacro:macro>\n";
  xacro_macro_file << "</robot>\n";
}
std::string GenerateRobotURDF::getTypeJoint(uint8_t type) {
  switch (type) {
    case urdf::Joint::REVOLUTE:
      return "revolute";
    case urdf::Joint::PRISMATIC:
      return "prismatic";
    case urdf::Joint::FLOATING:
      return "floating";
    case urdf::Joint::PLANAR:
      return "planar";
    case urdf::Joint::FIXED:
      return "fixed";
    default:
      return "unknown";
  }
}

int main(int argc, char* argv[]) {
  char current_path[FILENAME_MAX];
  getcwd(current_path, sizeof(current_path));
  for (int i = 0; i < argc; i++) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("generate_robot_urdf"), current_path);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("generate_robot_urdf"), argv[i]);
  }

  auto sd = std::make_shared<GenerateRobotURDF>();
  return 0;
}
