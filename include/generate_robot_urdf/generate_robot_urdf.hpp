#include <urdfdom/urdf_parser/urdf_parser.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>

#include "yaml-cpp/yaml.h"
using URDFPtr = urdf::ModelInterfaceSharedPtr;

class GenerateRobotURDF {
 public:
  GenerateRobotURDF();
  ~GenerateRobotURDF();

 private:
  void getURDF();
  /**
   * @brief Generate YAML file from URDF joint limits.
   *
   * Iterate over all joints in the URDF tree and write their limits to a YAML
   * file. The format of the YAML file is as follows:
   *
   * joint_limit:
   *   joint1:
   *     max_lower: -1.57
   *     max_upper: 1.57
   *     max_effort: 10.0
   *     max_velocity: 0.5
   *   joint2:
   *     ...
   */
  void generateYAMLlimit();
  void generateURDFmacro();
  void setProperty();
  std::string getTypeJoint(uint8_t type);

  std::ofstream xacro_macro_file;

  URDFPtr tf_tree_;
};