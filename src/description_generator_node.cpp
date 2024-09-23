#include <robot_description_generator/robot_description_generator.hpp>

void getCurrentPathAndFilename(std::string& path_dir, std::string& filename)
{
  path_dir += "/";
  while (filename.find("/") != std::string::npos)
  {
    path_dir += filename.substr(0, filename.find("/") + 1);
    filename.erase(0, filename.find("/") + 1);
  }
}

int main(int argc, char* argv[])
{
  if (argc < 2)
    throw std::runtime_error("Not enough arguments");

  char current_path_char[FILENAME_MAX];
  getcwd(current_path_char, sizeof(current_path_char));
  std::string current_path = current_path_char;

  std::string filename = argv[1];
  std::string urdf_path = current_path + "/" + filename;

  std::string package_path;
  if (argc != 3)
  {
    package_path = current_path;
    getCurrentPathAndFilename(package_path, filename);
    std::cerr << "The path for the new package is not selected\n";
  }
  else
  {
    package_path = std::string(argv[2]) + "/";
  }

  auto urdf_expansion = std::make_shared<RobotDescriptionGenerator>(package_path, urdf_path);
  urdf_expansion->generatePackage();

  return 0;
}
