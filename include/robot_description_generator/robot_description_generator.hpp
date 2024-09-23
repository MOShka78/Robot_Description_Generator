#pragma once

#include <sys/stat.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "yaml-cpp/yaml.h"

using URDFPtr = urdf::ModelInterfaceSharedPtr;

class RobotDescriptionGenerator
{
public:
  RobotDescriptionGenerator(std::string package_path, std::string urdf_path);
  ~RobotDescriptionGenerator()
  {
    std::cout << "Create package: " << package_path_ << std::endl;
  }

  void generatePackage();

protected:
  std::shared_ptr<YAML::Emitter> createJointLimits();
  std::shared_ptr<YAML::Emitter> createLinkMass();
  void setProperty(std::ofstream& file);
  void addJointsLinks(std::ofstream& file);

private:
  void generateYAMLlimit();
  void generateURDFInc();
  void generateURDFmacro();
  void generateURDFcommon();

  void makeDirPackage();
  void createCmakeLists();
  void createLaunch();
  void createPackageXML();
  void copyMeshes();

  std::string getTypeJoint(uint8_t type);

  std::string package_path_;
  std::string urdf_path_;

  std::string new_package_name;

  URDFPtr tf_tree_;
};