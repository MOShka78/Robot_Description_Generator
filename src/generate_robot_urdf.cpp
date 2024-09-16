
#include <generate_robot_urdf/generate_robot_urdf.hpp>
GenerateRobotURDF::GenerateRobotURDF(std::string path_dir, std::string filename)
    : path_dir_(path_dir), filename_(filename) {
  tf_tree_ = urdf::parseURDFFile(path_dir + filename);

  makeDirPackage();
  createCmakeLists();
  createPackageXML();
  createLaunch();
  setPathMesh();
  generateYAMLlimit();
  generateURDFInc();
  generateURDFmacro();
}

void GenerateRobotURDF::generateURDFInc() {
  std::ofstream xacro_macro_inc(path_dir_ + "urdf/inc/" + tf_tree_->name_ +
                                "_property.xacro");

  xacro_macro_inc << "<?xml version=\"1.0\"?>\n";
  xacro_macro_inc << "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\">\n";
  xacro_macro_inc << "  <xacro:macro name=\"" + tf_tree_->name_ +
                         "_property\">\n";

  setProperty(xacro_macro_inc);
  xacro_macro_inc << "  </xacro:macro>\n";
  xacro_macro_inc << "</robot>\n";
  xacro_macro_inc.close();
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

  std::ofstream fout(path_dir_ + "config/joint_limits.yaml");
  fout << emitter.c_str();
  fout.close();
}

void GenerateRobotURDF::setProperty(std::ofstream& file) {
  file << "     <xacro:property name=\"yaml_file\" value=\"${xacro.load_yaml(" +
              path_dir_ + tf_tree_->name_ + "_property.xacro" + ")}\"/>\n";

  file << "   <!-- TODO -->\n";
  file << "   <!-- <xacro:property name=\"yaml_path\" value=\"$(find "
          "generate_robot_urdf)/config/output/joint_limits.yaml\" /> -->\n";
  file << "   <!-- <xacro:property name=\"yaml_file\" "
          "value=\"${xacro.load_yaml(yaml_path)}\"/> -->\n\n";

  file << "   <!-- link mass [kg] -->\n";
  for (const auto& [link_name, link_value] : tf_tree_->links_) {
    file << "   <xacro:property name=\"" << link_name << "_mass"
         << "\" "
            "value=\"${yaml_file['link_mass']['"
         << link_name << "']['mass']}\"/>\n";
  }

  file << "\n   <!-- joint limits [rad] -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    file << "   <xacro:property name=\"" << joint_name << "_lower_limit"
         << "\" "
            "value=\"${yaml_file['joints_limits']['"
         << joint_name << "']['min_position']}\"/>\n";
    file << "   <xacro:property name=\"" << joint_name << "_upper_limit"
         << "\" "
            "value=\"${yaml_file['joints_limits']['"
         << joint_name << "']['max_position']}\"/>\n";
  }

  file << "\n   <!-- joint velocity limits [rad/s] -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    file << "   <xacro:property name=\"" << joint_name << "_velocity_limit"
         << "\" "
            "value=\"${yaml_file['joints_limits']['"
         << joint_name << "']['max_velocity']}\"/>\n";
  }

  file << "\n   <!-- joint effort limits -->\n";
  for (const auto& [joint_name, joint_value] : tf_tree_->joints_) {
    file << "   <xacro:property name=\"" << joint_name << "_effort_limit"
         << "\" "
            "value=\"${yaml_file['joints_limits']['"
         << joint_name << "']['max_effort']}\"/>\n";
  }
}

void GenerateRobotURDF::generateURDFmacro() {
  std::ofstream xacro_macro_robot(path_dir_ + "urdf/" + tf_tree_->name_ +
                                  "_macro.xacro");

  xacro_macro_robot << "<?xml version=\"1.0\"?>\n";
  xacro_macro_robot << "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\">\n";
  xacro_macro_robot << " <xacro:macro name=\"" << tf_tree_->name_
                    << "\" params=\"parent_link\">\n\n";

  xacro_macro_robot << "<xacro:include filename=\"$(find " + path_dir_ +
                           tf_tree_->name_ + "_macro.xacro" + "\" />\n";

  xacro_macro_robot << "  <!-- TODO -->\n";
  xacro_macro_robot << "  <!-- <xacro:include filename=\"$(find "
                       "ur_description)/urdf/inc/ur_common.xacro\" /> -->/n";

  addJointsLinks(xacro_macro_robot);

  xacro_macro_robot << " </xacro:macro>\n";
  xacro_macro_robot << "</robot>\n";
}

void GenerateRobotURDF::addJointsLinks(std::ofstream& file) {
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
    file << "         <mesh filename=\"" + link_path_mesh[link_name] + "\"/>\n";
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
    file << "         <mesh filename=\"" + link_path_mesh[link_name] + "\"/>\n";
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

void GenerateRobotURDF::setPathMesh() {
  auto doc_xml = urdf::exportURDF(tf_tree_);

  TiXmlPrinter printer;
  printer.SetIndent("  ");
  doc_xml->Accept(&printer);
  std::string urdf_str = printer.Str();

  while (urdf_str.find("<link name=\"") != std::string::npos) {
    std::string link_name = urdf_str.substr(urdf_str.find("<link name=\"") +
                                            sizeof("<link name=\"") - 1);

    link_name = link_name.substr(0, link_name.find("\""));

    urdf_str.erase(0, urdf_str.find("<link name=\"") + sizeof("<link name=\"") -
                          1 + link_name.length());

    std::string path_mesh = urdf_str.substr(urdf_str.find("<mesh filename=\"") +
                                            sizeof("<mesh filename=\"") - 1);

    path_mesh = path_mesh.substr(0, path_mesh.find("\""));

    link_path_mesh.insert({link_name, path_mesh});
  }
}

void GenerateRobotURDF::createCmakeLists() {
  std::ifstream myfile;
  myfile.open(
      ament_index_cpp::get_package_share_directory("generate_robot_urdf") +
      "/config/example_cmake.txt");

  std::string line;
  if (myfile.is_open()) {
    while (getline(myfile, line)) {
      std::ofstream myfile2(path_dir_ + "CMakeLists.txt", std::ios::app);
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
      myfile2 << line << "\n";
    }
    myfile.close();
  }
}

void GenerateRobotURDF::createPackageXML() {
  std::ifstream myfile;
  myfile.open(
      ament_index_cpp::get_package_share_directory("generate_robot_urdf") +
      "/config/example_package.xml");

  std::string line;

  if (myfile.is_open()) {
    while (getline(myfile, line)) {
      std::ofstream myfile2(path_dir_ + "package.xml", std::ios::app);
      if (line.find("<name></name>") != std::string::npos) {
        line.insert(line.find("</name>"), new_package_name);
      }
      myfile2 << line << "\n";
    }
    myfile.close();
  }
}

void GenerateRobotURDF::createLaunch() {
  std::ifstream myfile;
  myfile.open(
      ament_index_cpp::get_package_share_directory("generate_robot_urdf") +
      "/config/example_ros.launch");

  std::string line;

  if (myfile.is_open()) {
    while (getline(myfile, line)) {
      std::ofstream myfile2(
          path_dir_ + "launch/" + new_package_name + ".launch", std::ios::app);
      if (line.find("package_name") != std::string::npos) {
        line.replace(line.find("package_name"), sizeof("package_name") - 1,
                     new_package_name);
      }
      if (line.find("robot_name") != std::string::npos) {
        line.replace(line.find("robot_name"), sizeof("robot_name") - 1,
                     tf_tree_->name_);
      }
      myfile2 << line << "\n";
    }
    myfile.close();
  }

  std::ifstream myfile3;
  myfile3.open(
      ament_index_cpp::get_package_share_directory("generate_robot_urdf") +
      "/config/example_ros2.launch.py");

  line = "";

  if (myfile3.is_open()) {
    while (getline(myfile3, line)) {
      std::ofstream myfile4(
          path_dir_ + "launch/" + new_package_name + ".launch.py",
          std::ios::app);
      if (line.find("package_name") != std::string::npos) {
        line.replace(line.find("package_name"), sizeof("package_name") - 1,
                     new_package_name);
      }
      if (line.find("robot_name") != std::string::npos) {
        line.replace(line.find("robot_name"), sizeof("robot_name") - 1,
                     tf_tree_->name_);
      }
      myfile4 << line << "\n";
    }
    myfile3.close();
  }
}
void GenerateRobotURDF::makeDirPackage() {
  new_package_name = tf_tree_->name_ + "_description";

  mkdir((path_dir_ + new_package_name).c_str(), 0777);
  path_dir_ += ("/" + new_package_name + "/");

  mkdir((path_dir_ + "urdf").c_str(), 0777);
  mkdir((path_dir_ + "launch").c_str(), 0777);
  mkdir((path_dir_ + "config").c_str(), 0777);
  mkdir((path_dir_ + "meshes").c_str(), 0777);
  mkdir((path_dir_ + "meshes/visual").c_str(), 0777);
  mkdir((path_dir_ + "meshes/collision").c_str(), 0777);
  mkdir((path_dir_ + "urdf/inc").c_str(), 0777);
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

  auto generate_robot_urdf =
      std::make_shared<GenerateRobotURDF>(path, filename);
  return 0;
}
