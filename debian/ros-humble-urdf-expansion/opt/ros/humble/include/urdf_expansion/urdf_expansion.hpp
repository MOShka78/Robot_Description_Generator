#include <sys/stat.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "yaml-cpp/yaml.h"

using URDFPtr = urdf::ModelInterfaceSharedPtr;

class URDFExpansion {
 public:
  URDFExpansion(std::string path_dir, std::string filename);
  ~URDFExpansion() {
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("urdf_expansion"),
        "Create package: " << package_path_dir_ << "/" << new_package_name);
  }

 private:
  void generateYAMLlimit();
  void generateURDFInc();
  void generateURDFmacro();
  void generateURDFcommon();

  void setProperty(std::ofstream& file);
  void addJointsLinks(std::ofstream& file);
  void makeDirPackage();
  void createCmakeLists();
  void createLaunch();
  void createPackageXML();
  void copyMeshes();

  std::string getTypeJoint(uint8_t type);

  std::string package_path_dir_;
  std::string old_path_dir_;
  std::string filename_;
  std::string new_package_name;

  // std::map<std::string, std::string> link_path_mesh;

  URDFPtr tf_tree_;
};