#include <sys/stat.h>
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
  void makeDirPackage();
  void createCmakeLists();
  void createLaunch();
  void createPackageXML();

  std::string getTypeJoint(uint8_t type);

  std::string path_dir_;
  std::string filename_;
  std::string new_package_name;

  std::map<std::string, std::string> link_path_mesh;

  URDFPtr tf_tree_;
};