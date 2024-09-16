#include <unistd.h>
#include <urdfdom/urdf_parser/urdf_parser.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "yaml-cpp/yaml.h"
using URDFPtr = urdf::ModelInterfaceSharedPtr;

class GenerateRobotURDF {
 public:
  GenerateRobotURDF(std::string path_dir, std::string filename);

 private:
  void generateYAMLlimit();
  void generateURDFInc();
  void generateURDFmacro();

  void setPathMesh();
  void setProperty(std::ofstream& file);
  void addJointsLinks(std::ofstream& file);

  std::string getTypeJoint(uint8_t type);

  std::string path_dir_;
  std::string filename_;

  std::map<std::string, std::string> link_path_mesh;

  URDFPtr tf_tree_;
};