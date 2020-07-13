#ifndef BUZZ_CONN_CONTROLLER_KHEPERAIV_H
#define BUZZ_CONN_CONTROLLER_KHEPERAIV_H

#include <buzz/argos/buzz_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_ground_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_light_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_ultrasound_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_lidar_sensor.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "svg_image.h"
#include "path_existance_checking.h"

using namespace argos;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace pe = pathexsitance;
static const float MAX_ALT = 5;
static const float PLANE_RESOLUTION = 0.1;
static const float ROBOT_RANGE = 5; 
static float Required_path_segment_len = 0.5;
static int debug_steps=0;

class CConnectivityBuzzControllerKheperaIV : public CBuzzController {

public:

   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      SWheelTurningParams();
      void Init(TConfigurationNode& t_tree);
   };

public:

   CConnectivityBuzzControllerKheperaIV();
   virtual ~CConnectivityBuzzControllerKheperaIV();

   virtual void Init(TConfigurationNode& t_node);

   virtual void UpdateSensors();

   void SetWheels(Real f_left_speed, Real f_right_speed);
   void SetWheelSpeedsFromVector(const CVector2& c_heading);
   void SetLEDs(const CColor& c_color);
   std::vector<std::vector<double>> import_map(std::string m_map_file_name);
   std::string GetMapFName();   
   std::vector<std::vector<double>> OneShotPathPlanner(float* start_end_time);

private:

   virtual buzzvm_state RegisterFunctions();

protected:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the ground sensor */
   CCI_KheperaIVGroundSensor* m_pcGround;
   /* Pointer to the proximity sensor */
   CCI_KheperaIVProximitySensor* m_pcProximity;
   /* Pointer to the light sensor */
   CCI_KheperaIVLightSensor* m_pcLight;
   /* Pointer to the ultrasound sensor */
   CCI_KheperaIVUltrasoundSensor* m_pcUltrasound;
   /* Pointer to the lidar sensor */
   CCI_KheperaIVLIDARSensor* m_pcLIDAR;

   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;

   /* Path planner additional memeber functions declaration */
   /* returns a svg::point from state as double */
   svg::Point visualize_point(double* state, svg::Dimensions dims);

   svg::Point visualize_point(float* state, svg::Dimensions dims);

   svg::Point visualize_point(std::vector<double> state, svg::Dimensions dims);

   svg::Point visualize_point(ob::State* state, svg::Dimensions dims);

   svg::Point visualize_point_woScale(std::vector<double> state, svg::Dimensions dims);

   std::vector<std::vector<double>> import_3dmap(std::string m_map_file_name);

   int obtain_2d_path_end(std::vector<std::vector<double>> solution_nodes);
   /*Additional variables declared for the path planner */
   /* Map file name */
   std::string strmapFName;
   int map_option;
   float map_resolution;
   int MIN_X;
   int MAX_X;
   int MIN_Y;
   int MAX_Y;
   double half_map_height;
   double half_map_length; 
   std::vector<std::vector<std::vector<int>>> Grid_map;
   std::vector<std::vector<std::vector<int>>> Grid_map_GT;
   std::vector<std::vector<std::vector<int>>> Grid_map_visited;
   std::vector<std::pair <int,std::vector<float>>> m_restrictz;
   int save_solution_svg;
   int m_faulty;

   std::vector<std::vector<double>> start_end_pos;
   std::vector<std::vector<double>> debug_old_path;
   float exploration_bound_low[3];
   float exploration_bound_high[3];
   float statespacemax[3];
   int Deep_planning;

    // Collision checker for 2d Path planner.
    class ValidityChecker : public ob::StateValidityChecker
    {
    public:
        ValidityChecker(const ob::SpaceInformationPtr& si, float &res) :
            ob::StateValidityChecker(si),gridToCheck(5),map_resolution(res) {}
        ValidityChecker(const ob::SpaceInformationPtr& si, std::vector<std::vector<int>> *Grid_map, float &res) :
        ob::StateValidityChecker(si),gridToCheck(5),map_resolution(res) {
          Grid_obst = Grid_map;
        }
        ValidityChecker(const ob::SpaceInformationPtr& si, std::vector<std::vector<int>> *Grid_map,
          std::vector<std::pair <int,std::vector<float>>> *restictz, float &res) :
        ob::StateValidityChecker(si),gridToCheck(5),map_resolution(res) {
          Grid_obst = Grid_map;
          m_restrictz=restictz;
        }
        ValidityChecker(std::vector<std::vector<int>> *Grid_map,
                        std::vector<std::pair <int,std::vector<float>>> *restictz, float &res):ob::StateValidityChecker(nullptr),
                        gridToCheck(5),map_resolution(res){
          Grid_obst = Grid_map;
          m_restrictz=restictz;
        }
        bool isValid(double* state2D) const
        {
          std::vector<std::vector<int>>  ref_g_obst = *Grid_obst;
          for(int i=-3; i < 2; ++i){
            for(int j=-3; j < 2;++j){
              if((state2D[0])*(1/map_resolution)+i < ref_g_obst.size() &&
                   (state2D[0])*(1/map_resolution)+i >= 0 &&
                   (state2D[1])*(1/map_resolution)+j < ref_g_obst[state2D[0]+i].size() &&
                   (state2D[1])*(1/map_resolution)+j >=0 ){
                
                if(ref_g_obst[(state2D[0])*(1/map_resolution)+i][(state2D[1])*(1/map_resolution)+j]) return false;

              }
            }
          }
          for(size_t k=0;k<m_restrictz->size(); ++k){
            std::vector<float> c_zone = (*m_restrictz)[k].second;

            if( (state2D[0]) >= c_zone[0] && (state2D[0]) <= c_zone[1]){
              if( (!c_zone[4]) && (state2D[1]) < c_zone[3]){
                // printf(" Invalid \n");
                return false;
              }
              else if(c_zone[4] && (state2D[1]) > c_zone[2]){

                return false;
              } 

            }
          }
          return true;
        }
        // Returns whether the given state's position overlaps
        // any of the obstacles in grid map
        bool isValid(const ob::State* state) const
        {
          std::vector<std::vector<int>>  ref_g_obst = *Grid_obst;
          const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();

            if(state2D->values[0]*(1/map_resolution) < ref_g_obst.size()-1 &&
                state2D->values[0]*(1/map_resolution) >= 0 &&
                state2D->values[1]*(1/map_resolution) < ref_g_obst[state2D->values[0]].size()-1 &&
                state2D->values[1]*(1/map_resolution) >= 0){
              for(int i=-gridToCheck; i < gridToCheck; ++i){
                for(int j=-gridToCheck; j < gridToCheck;++j){
                  if((state2D->values[0])*(1/map_resolution)+i < ref_g_obst.size() &&
                       (state2D->values[0])*(1/map_resolution)+i >= 0 &&
                       (state2D->values[1])*(1/map_resolution)+j < ref_g_obst[state2D->values[0]+i].size() &&
                       (state2D->values[1])*(1/map_resolution)+j >=0 ){
                    if(ref_g_obst[(state2D->values[0])*(1/map_resolution)+i][(state2D->values[1])*(1/map_resolution)+j]) return false;

                    for(size_t k=0;k<m_restrictz->size(); ++k){
                      std::vector<float> c_zone = (*m_restrictz)[k].second;

                      if( (state2D->values[0]+i) >= c_zone[0] && (state2D->values[0]+i) <= c_zone[1]){
                        if( (!c_zone[4]) && (state2D->values[1]+j) < c_zone[3]){
                          // printf(" Invalid \n");
                          return false;
                        }
                        else if(c_zone[4] && (state2D->values[1]+j) > c_zone[2]){

                          return false;
                        } 

                      }
                    }
                  }
                }
              }
            }
            else{
              return false;
            }
          return true;
        }
    protected:
      int gridToCheck;
      float map_resolution;
      std::vector<std::vector<int>> *Grid_obst;
      std::vector<std::pair <int,std::vector<float>>> *m_restrictz;
    };
    //collision checker for 3d path planner
    class ValidityChecker3D : public ob::StateValidityChecker
    {
    public:
        ValidityChecker3D(const ob::SpaceInformationPtr& si, std::vector<std::vector<std::vector<int>>> 
          Grid_map) :ob::StateValidityChecker(si),gridToCheck(3) {
          Grid_obst= Grid_map;
        }
        // Returns whether the given state's position overlaps the
        // circular obstacle
        bool isValid(const ob::State* state) const
        {
          const ob::RealVectorStateSpace::StateType* state3D =
                state->as<ob::RealVectorStateSpace::StateType>();
          int x =round(state3D->values[0]);
          int y =round(state3D->values[1]);
              int comp_planning_plane = (state3D->values[2]/PLANE_RESOLUTION);
          if(Grid_obst[comp_planning_plane][x][y]) return false;
            int k=-2, check_z =gridToCheck;
          if(state3D->values[2] > MAX_ALT-1 || state3D->values[2] < 1 ){ k=0; check_z=1;}
          for(; k < check_z;++k){
            comp_planning_plane = (state3D->values[2]/PLANE_RESOLUTION)+k;
            if(Grid_obst[comp_planning_plane][x][y]) return false;
            else{

              for(int i=-2; i < gridToCheck; ++i){
                for(int j=-2; j < gridToCheck;++j){
                  if(x+i < Grid_obst[comp_planning_plane].size() &&
                     x+i >= 0 &&
                     y+j < Grid_obst[comp_planning_plane][x+i].size() &&
                     y+j >=0 ){
                    if(Grid_obst[comp_planning_plane][x+i][y+j]){
                      if(clearance(state,x+i,y+j,comp_planning_plane*PLANE_RESOLUTION)){
                        // std::cout<<"Clearence not satisfied"<<std::endl;
                        return false;
                      }
                    }
                  }
                }
              }
            }

        }
          return true;
            // return this->clearance(state) > 0.0;
        }
        // Returns the distance from the given state's position to
        // another state.
        bool clearance(const ob::State* state1, double x2, double y2, double z2) const
        {
            // We know we're working with a RealVectorStateSpace in this
            // example, so we downcast state into the specific type.
            const ob::RealVectorStateSpace::StateType* state_1 =
                state1->as<ob::RealVectorStateSpace::StateType>();
            // const ob::RealVectorStateSpace::StateType* state_2 =
            //     state2->as<ob::RealVectorStateSpace::StateType>();
            // Extract the robot's (x,y) position from its state
            double x1 = state_1->values[0];
            double y1 = state_1->values[1];
            double z1 = state_1->values[2];
            // Distance formula between two points, offset by a circle
            // radius for clearence from obst
            return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)) - 1.6 < 0.0;
        }

    protected:
      int gridToCheck;
      std::vector<std::vector<std::vector<int>>> Grid_obst;
    };
   // planner  Pointers 
   ob::StateSpacePtr space;
   ob::PlannerPtr optimizingPlanner;
   ob::ProblemDefinitionPtr pdef;
   // planners objects
   ob::RealVectorStateSpace* m_vec_space;
   ob::SpaceInformation* m_space_info;
   ValidityChecker* m_validity_checker;
   ob::ProblemDefinition* m_pdef_Real;
   // og::RecedingRRTstar* m_rrt_planner;

};

#endif
