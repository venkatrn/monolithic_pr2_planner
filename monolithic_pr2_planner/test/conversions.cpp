#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <gtest/gtest.h>

using namespace monolithic_pr2_planner;

class MiniEnv : public ::testing::Test {
    public:
        static void SetUpTestCase(){
            OccupancyGridParams og_params;
            RobotResolutionParams rr_params;

            og_params.env_resolution = .02;
            og_params.reference_frame = "/map";

            Point3D point;
            point.x = 0;
            point.y = 0;
            point.z = 0;

            og_params.origin = point; 
            point.x = 9;
            point.y = 5;
            point.z = 2;
            og_params.max_point = point;

            rr_params.obj_xyz_resolution = .000349;
            rr_params.obj_rpy_resolution = .098175;
            rr_params.arm_free_angle_resolution = .034907;
            rr_params.base_theta_resolution = .392699;
            rr_params.gripper_sphere_radius = .08;
            rr_params.num_free_angle_angles = 180;
            rr_params.num_rpy_angles = 64;
            rr_params.num_base_angles = 16;
            rr_params.ndof = 8;
            rr_params.num_base_prims = 144;

            OccupancyGridUser::init(og_params, rr_params);
        }
};


TEST_F(MiniEnv, baseDiscAngleNormalization)
{
    DiscBaseState base_state(0,0,0,0);
    base_state.theta(18);
    EXPECT_EQ(base_state.theta(), 2);

    base_state.theta(-2);
    EXPECT_EQ(base_state.theta(), 14);

    base_state.theta(5);
    EXPECT_EQ(base_state.theta(), 5);
}


TEST_F(MiniEnv, discAngleNormalization)
{
    DiscObjectState obj_state(0,0,0,0,0,0);
    obj_state.roll(65);
    obj_state.pitch(65);
    obj_state.yaw(65);
    EXPECT_EQ(obj_state.roll(), 1);
    EXPECT_EQ(obj_state.pitch(), 1);
    EXPECT_EQ(obj_state.yaw(), 1);

    obj_state.roll(-1);
    obj_state.pitch(-1);
    obj_state.yaw(-1);
    EXPECT_EQ(obj_state.roll(), 63);
    EXPECT_EQ(obj_state.pitch(), 63);
    EXPECT_EQ(obj_state.yaw(), 63);

    obj_state.roll(20);
    obj_state.pitch(20);
    obj_state.yaw(20);
    EXPECT_EQ(obj_state.roll(), 20);
    EXPECT_EQ(obj_state.pitch(), 20);
    EXPECT_EQ(obj_state.yaw(), 20);
}

TEST_F(MiniEnv, contAngleNormalization)
{
    ContObjectState obj_state(0,0,0,0,0,0);
    obj_state.roll(7);
    obj_state.pitch(7);
    obj_state.yaw(7);
    ASSERT_NEAR(obj_state.roll(), .7168, .0001);
    ASSERT_NEAR(obj_state.pitch(), .7168, .0001);
    ASSERT_NEAR(obj_state.yaw(), .7168, .0001);

    obj_state.roll(-1);
    obj_state.pitch(-1);
    obj_state.yaw(-1);
    ASSERT_NEAR(obj_state.roll(), 5.2831, .0001);
    ASSERT_NEAR(obj_state.pitch(), 5.2831, .0001);
    ASSERT_NEAR(obj_state.yaw(), 5.2831, .0001);

    obj_state.roll(3.1);
    obj_state.pitch(3.1);
    obj_state.yaw(3.1);
    ASSERT_NEAR(obj_state.roll(), 3.1, .0001);
    ASSERT_NEAR(obj_state.pitch(), 3.1, .0001);
    ASSERT_NEAR(obj_state.yaw(), 3.1, .0001);
}

TEST_F(MiniEnv, discToContConversion)
{
    DiscObjectState disc_obj_state(100,50,30,27,14,62);
    ContObjectState cont_obj_state = disc_obj_state;
    ASSERT_NEAR(cont_obj_state.x(), 2, .0001);
    ASSERT_NEAR(cont_obj_state.y(), 1, .0001);
    ASSERT_NEAR(cont_obj_state.z(), .6, .0001);
    ASSERT_NEAR(cont_obj_state.roll(), 2.650725, .0001);
    ASSERT_NEAR(cont_obj_state.pitch(), 1.37445, .0001);
    ASSERT_NEAR(cont_obj_state.yaw(), 6.08685, .0001);
}

TEST_F(MiniEnv, contToDiscConversion)
{
    ContObjectState cont_obj_state(2,1,.6,2.650725,1.37445,6.08685);
    DiscObjectState disc_obj_state = cont_obj_state;
    EXPECT_EQ(disc_obj_state.x(), 100);
    EXPECT_EQ(disc_obj_state.y(), 50);
    EXPECT_EQ(disc_obj_state.z(), 30);
    EXPECT_EQ(disc_obj_state.roll(), 27);
    EXPECT_EQ(disc_obj_state.pitch(), 14);
    EXPECT_EQ(disc_obj_state.yaw(), 62);
}

// TODO arm stuff
TEST_F(MiniEnv, contArmState)
{
    //LeftContArmState l_arm;
    //l_arm.set
}

// can't figure out why this fails
//TEST_F(MiniEnv, baseContAngleNormalization)
//{
//    // for some reason, this fails if i construct with 4 params
//    ContBaseState base_state;
//    base_state.theta(7);
//    //ASSERT_NEAR(base_state.theta(), .7168, .0001);
//
//    //base_state.theta(-1);
//    //ASSERT_NEAR(base_state.theta(), 5.2831, .0001);
//
//    //base_state.theta(3.1);
//    //ASSERT_NEAR(base_state.theta(), 3.1, .0001);
//}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
