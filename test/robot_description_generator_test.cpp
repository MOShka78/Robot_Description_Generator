#include <robot_description_generator/robot_description_generator.hpp>
#include <gtest/gtest.h>

class RobotDescriptionGeneratorTest : public RobotDescriptionGenerator, public testing::Test
{
public:
  RobotDescriptionGeneratorTest()
    : RobotDescriptionGenerator("", ament_index_cpp::get_package_share_directory("robot_description_generator") +
                                        "/test/config/simple_test.urdf")
  {
  }
};

TEST_F(RobotDescriptionGeneratorTest, createJointLimits)
{
  auto emitter = createJointLimits();
  std::string emitter_want_str =
      "joint_limits:\n  joint1:\n    min_position: 1\n    max_position: 5\n    max_effort: 4\n    max_velocity: 1\n  "
      "joint2:\n    min_position: 1\n    max_position: 2\n    max_effort: 3\n    max_velocity: 4";
  EXPECT_EQ(emitter->c_str(), emitter_want_str);
}

TEST_F(RobotDescriptionGeneratorTest, createLinkMass)
{
  auto emitter = createLinkMass();
  std::string emitter_want_str = "link_mass:\n  base_link:\n    mass: 10\n  link1:\n    mass: 10\n "
                                 " link2:\n    mass: 20";
  EXPECT_EQ(emitter->c_str(), emitter_want_str);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}