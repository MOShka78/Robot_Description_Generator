
#include <urdf_expansion/urdf_expansion.hpp>
URDFExpansion::URDFExpansion(std::string path_dir, std::string filename)
    : package_path_dir_(path_dir),
      old_path_dir_(path_dir),
      filename_(filename) {
  tf_tree_ = urdf::parseURDFFile(path_dir + filename);

  makeDirPackage();
  createCmakeLists();
  createPackageXML();
  createLaunch();
  copyMeshes();
  generateYAMLlimit();
  generateURDFInc();
  generateURDFmacro();
  generateURDFcommon();
}

void URDFExpansion::generateURDFInc() {
  std::ofstream xacro_macro_inc(package_path_dir_ + "urdf/inc/" +
                                tf_tree_->name_ + "_property.xacro");

  xacro_macro_inc << "<?xml version=\"1.0\"?>\n";
  xacro_macro_inc << "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\">\n";

  setProperty(xacro_macro_inc);

  xacro_macro_inc << "</robot>\n";

  xacro_macro_inc.close();
}

void URDFExpansion::generateYAMLlimit() {
  YAML::Emitter emitter_limit;

  emitter_limit << YAML::BeginMap;
  emitter_limit << YAML::Key << "joint_limits" << YAML::Value << YAML::BeginMap;

  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    if (joint_value->limits != nullptr) {
      emitter_limit << YAML::Key << joint_name << YAML::Value << YAML::BeginMap;

      emitter_limit << YAML::Key << "min_position" << YAML::Value
                    << joint_value->limits->lower;
      emitter_limit << YAML::Key << "max_position" << YAML::Value
                    << joint_value->limits->upper;
      emitter_limit << YAML::Key << "max_effort" << YAML::Value
                    << joint_value->limits->effort;
      emitter_limit << YAML::Key << "max_velocity" << YAML::Value
                    << joint_value->limits->velocity;
      emitter_limit << YAML::EndMap;
    }
  }
  emitter_limit << YAML::EndMap;
  emitter_limit << YAML::EndMap;

  YAML::Emitter emitter_mass;

  emitter_mass << YAML::BeginMap;
  emitter_mass << YAML::Key << "link_mass" << YAML::Value << YAML::BeginMap;
  for (const auto& [link_name, link_value] : tf_tree_->links_) {
    emitter_mass << YAML::Key << link_name << YAML::Value << YAML::BeginMap;
    emitter_mass << YAML::Key << "mass" << YAML::Value
                 << link_value->inertial->mass;
    emitter_mass << YAML::EndMap;
  }
  emitter_mass << YAML::EndMap;
  emitter_mass << YAML::EndMap;

  std::ofstream joint_limits(package_path_dir_ + "config/joint_limits.yaml");
  joint_limits << emitter_limit.c_str();
  joint_limits.close();

  std::ofstream link_mass(package_path_dir_ + "config/link_mass.yaml");
  link_mass << emitter_mass.c_str();
  link_mass.close();
}

void URDFExpansion::setProperty(std::ofstream& file) {
  file << "   <xacro:property name=\"yaml_path_joint_limits\" value=\"$(find " +
              new_package_name + ")/config/joint_limits.yaml\" />\n";
  file << "   <xacro:property name=\"yaml_path_link_mass\" value=\"$(find " +
              new_package_name + ")/config/link_mass.yaml\" />\n";

  file << "   <xacro:property name=\"yaml_file_joint_limits\" "
          "value=\"${xacro.load_yaml(yaml_path_joint_limits)}\"/>\n\n";
  file << "   <xacro:property name=\"yaml_file_link_mass\" "
          "value=\"${xacro.load_yaml(yaml_path_link_mass)}\"/>\n";

  file << "   <!-- link mass [kg] -->\n";
  for (const auto& [link_name, link_value] : tf_tree_->links_) {
    file << "   <xacro:property name=\"" << link_name << "_mass"
         << "\" "
            "value=\"${yaml_file_link_mass['link_mass']['"
         << link_name << "']['mass']}\"/>\n";
  }

  file << "\n   <!-- joint limits [rad] -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    file << "   <xacro:property name=\"" << joint_name << "_lower_limit"
         << "\" "
            "value=\"${yaml_file_joint_limits['joint_limits']['"
         << joint_name << "']['min_position']}\"/>\n";
    file << "   <xacro:property name=\"" << joint_name << "_upper_limit"
         << "\" "
            "value=\"${yaml_file_joint_limits['joint_limits']['"
         << joint_name << "']['max_position']}\"/>\n";
  }

  file << "\n   <!-- joint velocity limits [rad/s] -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    file << "   <xacro:property name=\"" << joint_name << "_velocity_limit"
         << "\" "
            "value=\"${yaml_file_joint_limits['joint_limits']['"
         << joint_name << "']['max_velocity']}\"/>\n";
  }

  file << "\n   <!-- joint effort limits -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    file << "   <xacro:property name=\"" << joint_name << "_effort_limit"
         << "\" "
            "value=\"${yaml_file_joint_limits['joint_limits']['"
         << joint_name << "']['max_effort']}\"/>\n";
  }
}

void URDFExpansion::generateURDFmacro() {
  std::ofstream xacro_macro_robot(package_path_dir_ + "urdf/" +
                                  tf_tree_->name_ + "_macro.xacro");

  xacro_macro_robot << "<?xml version=\"1.0\"?>\n";
  xacro_macro_robot << "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\">\n";
  xacro_macro_robot << " <xacro:macro name=\"" << tf_tree_->name_
                    << "\" params=\"parent_link\">\n\n";

  xacro_macro_robot << "  <xacro:include filename=\"$(find " +
                           new_package_name + ")/urdf/inc/" + tf_tree_->name_ +
                           "_property.xacro\" />\n";

  addJointsLinks(xacro_macro_robot);

  xacro_macro_robot << " </xacro:macro>\n";
  xacro_macro_robot << "</robot>\n";
}

void URDFExpansion::addJointsLinks(std::ofstream& file) {
  for (const auto& [link_name, link_value] : tf_tree_->links_) {
    file << "\n\n   <!-- " << link_name << " -->\n";

    if (link_value->parent_joint != nullptr) {
      file << "    <joint name=\"" << link_value->parent_joint->name
           << "\" type=\"" << getTypeJoint(link_value->parent_joint->type)
           << "\">\n";
      if (link_value->parent_joint->limits != nullptr) {
        file << "     <axis xyz=\"" << link_value->parent_joint->axis.x << " "
             << link_value->parent_joint->axis.y << " "
             << link_value->parent_joint->axis.z << "\" rpy=\"0 0 0\"/>\n";

        file << "     <limit effort=\"${" << link_value->parent_joint->name
             << "_effort_limit}\" lower=\"${" << link_value->parent_joint->name
             << "_lower_limit}\" "
                "upper=\"${"
             << link_value->parent_joint->name
             << "_upper_limit}\" "
                "velocity=\"${"
             << link_value->parent_joint->name << "_velocity_limit}\"/>\n";
      }
      file << "     <parent link=\"" << link_value->getParent()->name
           << "\"/>\n";
      file << "     <child link=\"" << link_value->name << "\"/>\n";
      double roll, pitch, yaw;

      link_value->parent_joint->parent_to_joint_origin_transform.rotation
          .getRPY(roll, pitch, yaw);
      file << "     <origin xyz=\""
           << link_value->parent_joint->parent_to_joint_origin_transform
                  .position.x
           << " "
           << link_value->parent_joint->parent_to_joint_origin_transform
                  .position.y
           << " "
           << link_value->parent_joint->parent_to_joint_origin_transform
                  .position.z
           << "\" rpy=\"" << roll << " " << pitch << " " << yaw << "\" />\n";
      file << "   </joint>\n";
    } else {
      file << "    <joint name=\""
           << "${parent_link}-" << link_value->name << "\" type=\"fixed\">\n";
      file << "     <parent link=\"${parent_link}\"/>\n";
      file << "     <child link=\"" << link_value->name << "\"/>\n";
      file << "     <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
      file << "   </joint>\n";
    }

    file << "   <link name=\"" << link_name << "\">\n";
    file << "     <visual>\n";
    file << "       <geometry>\n";
    file << "         <mesh filename=\"package://" + new_package_name +
                "/meshes/visual/" + link_name + ".STL\"/>\n";
    file << "       </geometry>\n";
    file << "       <material name=\"" << link_value->visual->material_name
         << "\">\n";
    file << "         <color rgba=\"" << link_value->visual->material->color.r
         << " " << link_value->visual->material->color.g << " "
         << link_value->visual->material->color.b << " "
         << link_value->visual->material->color.a << "\" />\n";
    file << "       </material>\n";

    double roll_v, pitch_v, yaw_v;
    link_value->visual->origin.rotation.getRPY(roll_v, pitch_v, yaw_v);

    file << "       <origin xyz=\"" << link_value->visual->origin.position.x
         << " " << link_value->visual->origin.position.y << " "
         << link_value->visual->origin.position.z << "\" rpy=\"" << roll_v
         << " " << pitch_v << " " << yaw_v << "\"/>\n";
    file << "     </visual>\n";
    file << "     <collision>\n";
    file << "       <geometry>\n";
    file << "         <mesh filename=\"package://" + new_package_name +
                "/meshes/collision/" + link_name + ".STL\"/>\n";
    file << "       </geometry>\n";

    double roll_c, pitch_c, yaw_c;
    link_value->collision->origin.rotation.getRPY(roll_c, pitch_c, yaw_c);

    file << "       <origin xyz=\"" << link_value->collision->origin.position.x
         << " " << link_value->collision->origin.position.y << " "
         << link_value->collision->origin.position.z << "\" rpy=\"" << roll_c
         << " " << pitch_c << " " << yaw_c << "\"/>\n";
    file << "     </collision>\n";
    file << "     <inertial>\n";
    file << "       <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
    file << "       <mass value=\"${" << link_name << "_mass}"
         << "\"/>\n";
    file << "       <inertia ixx=\"" << link_value->inertial->ixx << "\" ixy=\""
         << link_value->inertial->ixy << "\" ixz=\""
         << link_value->inertial->ixz << "\" iyy=\""
         << link_value->inertial->iyy << "\" iyz=\""
         << link_value->inertial->iyz << "\" izz=\""
         << link_value->inertial->izz << "\"/>\n";
    file << "     </inertial>\n";
    file << "   </link>\n";
  }
}
void URDFExpansion::generateURDFcommon() {
  std::ofstream xacro_macro_robot(package_path_dir_ + "urdf/" +
                                  tf_tree_->name_ + ".urdf.xacro");

  xacro_macro_robot << "<?xml version=\"1.0\" ?>\n";
  xacro_macro_robot << "<robot name=\"" << tf_tree_->name_
                    << "\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n\n";

  xacro_macro_robot << "  <link name=\"world\"/>\n\n";

  xacro_macro_robot << "  <joint name=\"world2base\" type=\"fixed\">\n";
  xacro_macro_robot << "    <parent link=\"world\"/>\n";
  xacro_macro_robot << "    <child link=\"base_link\"/>\n";
  xacro_macro_robot << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
  xacro_macro_robot << "  </joint>\n\n";

  xacro_macro_robot << "  <xacro:include filename=\"$(find " +
                           new_package_name + ")/urdf/" + tf_tree_->name_ +
                           "_macro.xacro\"/>\n";
  xacro_macro_robot << "  <xacro:" + tf_tree_->name_ +
                           " parent_link=\"world\"/>\n";
  xacro_macro_robot << "</robot>\n";
}

std::string URDFExpansion::getTypeJoint(uint8_t type) {
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

void URDFExpansion::createCmakeLists() {
  std::ifstream example_cmakelists;
  example_cmakelists.open(
      ament_index_cpp::get_package_share_directory("urdf_expansion") +
      "/config/example_cmake.txt");

  std::string line;
  if (example_cmakelists.is_open()) {
    while (getline(example_cmakelists, line)) {
      std::ofstream cmakelists(package_path_dir_ + "CMakeLists.txt",
                               std::ios::app);
      if (line.find("project()") != std::string::npos) {
        line.insert(line.find("(") + 1, new_package_name);
      }
      if (line.find("COMMAND xacro urdf/.urdf.xacro") != std::string::npos) {
        line.insert(line.find("COMMAND xacro urdf/") +
                        sizeof("COMMAND xacro urdf/") - 1,
                    tf_tree_->name_);
      }
      if (line.find("OUTPUT_FILE urdf/.urdf") != std::string::npos) {
        line.insert(
            line.find("OUTPUT_FILE urdf/") + sizeof("OUTPUT_FILE urdf/") - 1,
            tf_tree_->name_);
      }
      cmakelists << line << "\n";
    }
    example_cmakelists.close();
  }
}

void URDFExpansion::createPackageXML() {
  std::ifstream example_package_xml;
  example_package_xml.open(
      ament_index_cpp::get_package_share_directory("urdf_expansion") +
      "/config/example_package.xml");

  std::string line;

  if (example_package_xml.is_open()) {
    while (getline(example_package_xml, line)) {
      std::ofstream package_xml(package_path_dir_ + "package.xml",
                                std::ios::app);
      if (line.find("<name></name>") != std::string::npos) {
        line.insert(line.find("</name>"), new_package_name);
      }
      package_xml << line << "\n";
    }
    example_package_xml.close();
  }
}

void URDFExpansion::createLaunch() {
  std::ifstream example_launch_ros;
  example_launch_ros.open(
      ament_index_cpp::get_package_share_directory("urdf_expansion") +
      "/config/example_ros.launch");

  std::string line;

  if (example_launch_ros.is_open()) {
    while (getline(example_launch_ros, line)) {
      std::ofstream launch_ros(
          package_path_dir_ + "launch/" + new_package_name + ".launch",
          std::ios::app);
      if (line.find("package_name") != std::string::npos) {
        line.replace(line.find("package_name"), sizeof("package_name") - 1,
                     new_package_name);
      }
      if (line.find("robot_name") != std::string::npos) {
        line.replace(line.find("robot_name"), sizeof("robot_name") - 1,
                     tf_tree_->name_);
      }
      launch_ros << line << "\n";
    }
    example_launch_ros.close();
  }

  std::ifstream example_launch_ros2;
  example_launch_ros2.open(
      ament_index_cpp::get_package_share_directory("urdf_expansion") +
      "/config/example_ros2.launch.py");

  line = "";

  if (example_launch_ros2.is_open()) {
    while (getline(example_launch_ros2, line)) {
      std::ofstream launch_ros2(
          package_path_dir_ + "launch/" + new_package_name + ".launch.py",
          std::ios::app);
      if (line.find("package_name") != std::string::npos) {
        line.replace(line.find("package_name"), sizeof("package_name") - 1,
                     new_package_name);
      }
      if (line.find("robot_name") != std::string::npos) {
        line.replace(line.find("robot_name"), sizeof("robot_name") - 1,
                     tf_tree_->name_);
      }
      launch_ros2 << line << "\n";
    }
    example_launch_ros2.close();
  }
}

void URDFExpansion::copyMeshes() {
  int cur = old_path_dir_.find("urdf");
  old_path_dir_ = old_path_dir_.substr(0, cur);

  std::filesystem::path source_dir = old_path_dir_ + "/meshes/";
  std::filesystem::path dest_vis_dir = package_path_dir_ + "/meshes/visual/";
  std::filesystem::path dest_col_dir = package_path_dir_ + "/meshes/collision/";

  if (std::filesystem::exists(source_dir)) {
    for (const auto& entry : std::filesystem::directory_iterator(source_dir)) {
      if (entry.is_regular_file()) {
        std::filesystem::copy_file(entry.path(),
                                   dest_vis_dir / entry.path().filename());
        std::filesystem::copy_file(entry.path(),
                                   dest_col_dir / entry.path().filename());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("urdf_expansion"),
                           "Copied file: " << entry.path().filename());
      }
    }
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("urdf_expansion"),
                       "path_meshes_dir not found");
  }
}
void URDFExpansion::makeDirPackage() {
  new_package_name = tf_tree_->name_ + "_description";

  mkdir((package_path_dir_ + new_package_name).c_str(), 0777);
  package_path_dir_ += (new_package_name + "/");

  mkdir((package_path_dir_ + "urdf").c_str(), 0777);
  mkdir((package_path_dir_ + "launch").c_str(), 0777);
  mkdir((package_path_dir_ + "config").c_str(), 0777);
  mkdir((package_path_dir_ + "meshes").c_str(), 0777);
  mkdir((package_path_dir_ + "meshes/visual").c_str(), 0777);
  mkdir((package_path_dir_ + "meshes/collision").c_str(), 0777);
  mkdir((package_path_dir_ + "urdf/inc").c_str(), 0777);
}

void getCurrentPathAndFileName(std::string& path_dir, std::string& filename) {
  path_dir += "/";
  while (filename.find("/") != std::string::npos) {
    path_dir += filename.substr(0, filename.find("/") + 1);
    filename.erase(0, filename.find("/") + 1);
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2)
    throw std::runtime_error("Not enough arguments (need urdf file)");

  char current_path[FILENAME_MAX];
  getcwd(current_path, sizeof(current_path));

  std::string path = current_path;
  std::string filename = argv[1];

  getCurrentPathAndFileName(path, filename);

  auto urdf_expansion = std::make_shared<URDFExpansion>(path, filename);

  return 0;
}
