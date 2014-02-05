#pragma once
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <string>

namespace monolithic_pr2_planner {

    struct Point3D {
        double x;
        double y;
        double z;
    };

    class StateSpaceParameters {
        // add functions to load in values
        private:
            std::map<std::string, std::pair<float, float> > bounds;
            std::map<std::string, float> resolutions;
    };

    // probably should be moved elsewhere
    typedef struct {
        float epsilon;
        float allocated_planning_time;
        std::string planning_joint;
    } PlannerSearchParams;

    typedef struct {
        std::string arm_motion_primitive_file;
        std::string base_motion_primitive_file;
        double nominal_vel; // m/s
        double angular_vel; // rad/sec
    } MotionPrimitiveParams;

    typedef struct {
        std::string arm_file;
        double env_resolution;
        std::string robot_description_string;
    } ArmDescriptionParams;

    typedef struct {
        double env_resolution;
        std::string reference_frame;
        Point3D origin; 
        Point3D max_point;
    } OccupancyGridParams;

    typedef struct {
        double obj_xyz_resolution;
        double obj_rpy_resolution;
        double arm_free_angle_resolution;
        double base_theta_resolution;
        double gripper_sphere_radius;
        int num_free_angle_angles;
        int num_rpy_angles;
        int num_base_angles;
        int ndof;
        int num_base_prims;
    } RobotResolutionParams;

    typedef struct {
        bool expansions;
        bool final_path;
    } VisualizationParams;

    /*! \brief Used to grab all parameters used by the planner and organize them
     * into various structs. 
     *
     * In an effort to keep things compartmentalized, each module has its own
     * struct defining what parameters it needs. A lot of the modules use the
     * same parameters, so there's going to be some duplicates. If you're
     * changing these structs, don't try and merge them all together - it's
     * easier to be very clear about what parameters each module takes.
     *
     * The highest level function is the fetch() function, which just calls all
     * parameter getter functions. This catalog is built when the Environment
     * object is first created. 
     */
    class ParameterCatalog {
        public:
            ParameterCatalog();
            void fetch(ros::NodeHandle nh);

            void setMotionPrimitiveParams(MotionPrimitiveParams& mprim);
            void setOccupancyGridParams(OccupancyGridParams& params);
            void setRobotResolutionParams(const MotionPrimitiveParams& mprims,
                                          RobotResolutionParams& params);
            void setLeftArmParams(ArmDescriptionParams& params);
            void setRightArmParams(ArmDescriptionParams& params);
            void setVisualizationParams(VisualizationParams& params);

            RobotResolutionParams m_robot_resolution_params;
            MotionPrimitiveParams m_motion_primitive_params;
            OccupancyGridParams m_occupancy_grid_params;
            ArmDescriptionParams m_left_arm_params;
            ArmDescriptionParams m_right_arm_params;
            VisualizationParams m_visualization_params;

        private:
            void parseArmMPrimFileHeader(const std::string& mprim_file, 
                                         RobotResolutionParams& params);
            bool parseBaseMPrimFileHeader(const std::string& mprim_file,
                                          RobotResolutionParams& params);
            ros::NodeHandle m_nodehandle;
            bool setFileNameFromParamServer(const std::string param_name, 
                                            std::string* parameter);
        
            void getNextLine(std::ifstream& file, std::stringstream& ss, 
                             std::string& line);
    };
}
