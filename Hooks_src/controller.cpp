#include "controller.h"
#include "virtual_stigmergy2.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// path planning Constants
static const char free_space = '.';
static const char free_space_l = 'G';
static const char outofbound_space = '@';
static const char outofbound_space_l = 'O';
static const char Tree_space = 'T';
static const char Swap_space = 'S';
static const char Water_space = 'W';
static const char START_space = 'A';
static const char TARGET_space = 'X';
static const float ROBOT_BUFFER = 0.01; // Consider robot radius
static float OBSTACLE_SIDES1 = 0.7f;
static float OBSTACLE_SIDES2 = 0.7f;


/****************************************/
/****************************************/

CConnectivityBuzzControllerKheperaIV::SWheelTurningParams::SWheelTurningParams() :
   TurningMechanism(NO_TURN),
   HardTurnOnAngleThreshold(ToRadians(CDegrees(90.0))),
   SoftTurnOnAngleThreshold(ToRadians(CDegrees(70.0))),
   NoTurnAngleThreshold(ToRadians(CDegrees(10.0))),
   MaxSpeed(50.0)
{
}

/****************************************/
/****************************************/

void CConnectivityBuzzControllerKheperaIV::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

static int BuzzGoToC(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   /* Create a new vector with that */
   buzzobj_t tX = buzzvm_stack_at(vm, 2);
   buzzobj_t tY = buzzvm_stack_at(vm, 1);
   CVector2 cDir;
   if(tX->o.type == BUZZTYPE_INT) cDir.SetX(tX->i.value);
   else if(tX->o.type == BUZZTYPE_FLOAT) cDir.SetX(tX->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "gotoc(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tX->o.type]
         );
      return vm->state;
   }      
   if(tY->o.type == BUZZTYPE_INT) cDir.SetY(tY->i.value);
   else if(tY->o.type == BUZZTYPE_FLOAT) cDir.SetY(tY->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "gotoc(x,y): expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tY->o.type]
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CConnectivityBuzzControllerKheperaIV*>(buzzvm_stack_at(vm, 1)->u.value)->SetWheelSpeedsFromVector(cDir);
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzGoToP(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   /* Create a new vector with that */
   buzzobj_t tLinSpeed = buzzvm_stack_at(vm, 2);
   buzzobj_t tAngSpeed = buzzvm_stack_at(vm, 1);
   Real fLinSpeed = 0.0, fAngSpeed = 0.0;
   if(tLinSpeed->o.type == BUZZTYPE_INT) fLinSpeed = tLinSpeed->i.value;
   else if(tLinSpeed->o.type == BUZZTYPE_FLOAT) fLinSpeed = tLinSpeed->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "gotop(linspeed,angspeed): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tLinSpeed->o.type]
         );
      return vm->state;
   }      
   if(tAngSpeed->o.type == BUZZTYPE_INT) fAngSpeed = tAngSpeed->i.value;
   else if(tAngSpeed->o.type == BUZZTYPE_FLOAT) fAngSpeed = tAngSpeed->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "gotop(linspeed,angspeed): expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tAngSpeed->o.type]
         );
      return vm->state;
   }
   CVector2 cDir(fLinSpeed, CRadians(fAngSpeed));
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CConnectivityBuzzControllerKheperaIV*>(buzzvm_stack_at(vm, 1)->u.value)->SetWheelSpeedsFromVector(cDir);
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

int BuzzSetWheels(buzzvm_t vm) {
   buzzvm_lnum_assert(vm, 2);
   /* Push speeds */
   buzzvm_lload(vm, 1); /* Left speed */
   buzzvm_lload(vm, 2); /* Right speed */
   buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CConnectivityBuzzControllerKheperaIV*>(
      buzzvm_stack_at(vm, 1)->u.value)->
      SetWheels(buzzvm_stack_at(vm, 3)->f.value, /* Left speed */
                buzzvm_stack_at(vm, 2)->f.value  /* Right speed */
         );
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

int BuzzSetLEDs(buzzvm_t vm) {
   buzzvm_lnum_assert(vm, 3);
   /* Push the color components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
   buzzvm_type_assert(vm, 2, BUZZTYPE_INT);
   buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
   /* Create a new color with that */
   CColor cColor(buzzvm_stack_at(vm, 3)->i.value,
                 buzzvm_stack_at(vm, 2)->i.value,
                 buzzvm_stack_at(vm, 1)->i.value);
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CConnectivityBuzzControllerKheperaIV*>(buzzvm_stack_at(vm, 1)->u.value)->SetLEDs(cColor);
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

int C_OneShotPathPlanner(buzzvm_t vm) {
  if(buzzdarray_size(vm->lsyms->syms)< 6 || buzzdarray_size(vm->lsyms->syms) > 6){
    fprintf(stderr, "[ROBOT %u] expected 5 args for planner but received : %u \n\n",vm->robot, 
            buzzdarray_size(vm->lsyms->syms)-1);
    return buzzvm_ret0(vm);
  }
  float start_end_time[5];
  for(UInt32 i = 1; i < buzzdarray_size(vm->lsyms->syms); ++i) {
    buzzvm_lload(vm, i);
    buzzobj_t o = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    switch(o->o.type) {
      case BUZZTYPE_NIL:
        fprintf(stderr, "[ROBOT %u] expected %u arg to be int or float received nil \n\n",vm->robot, 
                i);
      break;
      case BUZZTYPE_INT:
        start_end_time[i-1]=o->i.value;
        // fprintf(stderr, "[ROBOT %u] arg %u is %f",vm->robot, 
        //         i,start_end_time[i]);
      break;
      case BUZZTYPE_FLOAT:
        start_end_time[i-1]= o->f.value;
        // fprintf(stderr, "[ROBOT %u] arg %u is %f",vm->robot, 
        //       i,start_end_time[i]);
      break;
    }
  }
  /* Get pointer to controller user data */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   buzzvm_type_assert(vm, 1, BUZZTYPE_USERDATA);

   CConnectivityBuzzControllerKheperaIV& cContr = *reinterpret_cast<CConnectivityBuzzControllerKheperaIV*>(buzzvm_stack_at(vm, 1)->u.value);

   /*Get controls*/
   std::vector<std::vector<double>> controls = cContr.OneShotPathPlanner(start_end_time);// (start,end,time)

   buzzvm_pusht(vm);
   buzzobj_t path_Pose = buzzvm_stack_at(vm, 1);
   for(uint32_t i=0; i< controls.size();++i){
     CVector3 cntrl(controls[i][0],controls[i][1],controls[i][2]); 
     /* Store position data */
     cContr.TablePut(path_Pose, i, cntrl);
   }
   /* Register positioning data table as global symbol */
   // cContr.Register("path_controls", path_Pose);
   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

int C_LoadGTMap(buzzvm_t vm){
   /* Get pointer to controller user data */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   buzzvm_type_assert(vm, 1, BUZZTYPE_USERDATA);
   CConnectivityBuzzControllerKheperaIV& cContr = *reinterpret_cast<CConnectivityBuzzControllerKheperaIV*>(buzzvm_stack_at(vm, 1)->u.value);
   std::vector<std::vector<double>> start_end_posaa = cContr.import_map(cContr.GetMapFName());
  return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

CConnectivityBuzzControllerKheperaIV::CConnectivityBuzzControllerKheperaIV() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcGround(NULL),
   m_pcProximity(NULL),
   m_pcLight(NULL),
   m_pcUltrasound(NULL),
   m_pcLIDAR(NULL),
   map_resolution(0.1) {
}

/****************************************/
/****************************************/

CConnectivityBuzzControllerKheperaIV::~CConnectivityBuzzControllerKheperaIV() {
}

/****************************************/
/****************************************/

void CConnectivityBuzzControllerKheperaIV::Init(TConfigurationNode& t_node) {
   try {
      /* Get pointers to devices */
      try {
         m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
         m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      }
      catch(CARGoSException& ex) {}
      try {
         m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
      }
      catch(CARGoSException& ex) {}
      try {
         m_pcGround = GetSensor<CCI_KheperaIVGroundSensor>("kheperaiv_ground");
      }
      catch(CARGoSException& ex) {}
      try {
         m_pcProximity = GetSensor<CCI_KheperaIVProximitySensor>("kheperaiv_proximity");
      }
      catch(CARGoSException& ex) {}
      try {
         m_pcLight = GetSensor<CCI_KheperaIVLightSensor>("kheperaiv_light");
      }
      catch(CARGoSException& ex) {}
      try {
         m_pcUltrasound = GetSensor<CCI_KheperaIVUltrasoundSensor>("kheperaiv_ultrasound");
      }
      catch(CARGoSException& ex) {}
      try {
         m_pcLIDAR = GetSensor<CCI_KheperaIVLIDARSensor>("kheperaiv_lidar");
      }
      catch(CARGoSException& ex) {}
      try{
         /* Get the map file name */
         GetNodeAttribute(t_node, "map_file", strmapFName);
      }
      catch(CARGoSException& ex){
         THROW_ARGOSEXCEPTION_NESTED("Error getting map file name", ex);
      }
      try{
        /* Get the map file name */
        GetNodeAttribute(t_node, "save_svg_image", save_solution_svg);
      }
      catch(CARGoSException& ex){
         THROW_ARGOSEXCEPTION_NESTED("Error getting save_solution_svg", ex);
      }
      /* Initialize the rest */
      CBuzzController::Init(t_node);
      /* Register the new vstig */
      buzzvstig2_register(m_tBuzzVM);
      /*call the init function again after pushing the new virtual stigmergy*/
      /* Call the Init() function */
      if(buzzvm_function_call(m_tBuzzVM, "init", 0) != BUZZVM_STATE_READY) {
         fprintf(stderr, "[ROBOT %u] %s: execution terminated abnormally: %s\n\n",
                 m_tBuzzVM->robot,
                 m_strBytecodeFName.c_str(),
                 ErrorInfo().c_str());
         for(UInt32 i = 1; i <= buzzdarray_size(m_tBuzzVM->stacks); ++i) {
            buzzdebug_stack_dump(m_tBuzzVM, i, stdout);
         }
         return;
      }

      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "V_TYPE", 1));
      buzzvm_pushi(m_tBuzzVM, 0);
      buzzvm_gstore(m_tBuzzVM);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the Buzz controller for the Khepera IV", ex);
   }
}

/****************************************/
/****************************************/

void CConnectivityBuzzControllerKheperaIV::UpdateSensors() {
   /*
    * Update generic sensors
    */
   CBuzzController::UpdateSensors();
   /*
    * Update ground sensor table
    */
   if(m_pcGround != NULL) {
      /* Create empty ground table */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "ground", 1));
      buzzvm_pusht(m_tBuzzVM);
      buzzobj_t tGrndTable = buzzvm_stack_at(m_tBuzzVM, 1);
      buzzvm_gstore(m_tBuzzVM);
      /* Get ground readings */
      const CCI_KheperaIVGroundSensor::TReadings& tGrndReads = m_pcGround->GetReadings();
      /* Fill into the ground table */
      buzzobj_t tGrndRead;
      for(size_t i = 0; i < tGrndReads.size(); ++i) {
         /* Create table for i-th read */
         tGrndRead = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);
         /* Fill in the read */
         TablePut(tGrndRead, "value", tGrndReads[i].Value);
         /* Create table for offset */
         buzzvm_push(m_tBuzzVM, tGrndRead);
         buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "offset", 1));
         buzzvm_pusht(m_tBuzzVM);
         /* Add x offset value */
         buzzvm_dup(m_tBuzzVM);
         buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "x", 1));
         buzzvm_pushf(m_tBuzzVM, tGrndReads[i].Offset.GetX());
         buzzvm_tput(m_tBuzzVM);
         /* Add y offset value */
         buzzvm_dup(m_tBuzzVM);
         buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "y", 1));
         buzzvm_pushf(m_tBuzzVM, tGrndReads[i].Offset.GetY());
         buzzvm_tput(m_tBuzzVM);
         /* Store read table in the read table */
         buzzvm_tput(m_tBuzzVM);
         /* Store read table in the ground table */
         TablePut(tGrndTable, i, tGrndRead);
      }
   }
   /*
    * Update proximity sensor table
    */
   if(m_pcProximity != NULL) {
      /* Create empty proximity table */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "proximity", 1));
      buzzvm_pusht(m_tBuzzVM);
      buzzobj_t tProxTable = buzzvm_stack_at(m_tBuzzVM, 1);
      buzzvm_gstore(m_tBuzzVM);
      /* Get proximity readings */
      const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
      /* Fill into the proximity table */
      buzzobj_t tProxRead;
      for(size_t i = 0; i < tProxReads.size(); ++i) {
         /* Create table for i-th read */
         tProxRead = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);
         /* Fill in the read */
         TablePut(tProxRead, "value", tProxReads[i].Value);
         TablePut(tProxRead, "angle", tProxReads[i].Angle);
         /* Store read table in the proximity table */
         TablePut(tProxTable, i, tProxRead);
      }
   }
   /*
    * Update light sensor table
    */
   if(m_pcLight != NULL) {
      /* Create empty proximity table */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "light", 1));
      buzzvm_pusht(m_tBuzzVM);
      buzzobj_t tLightTable = buzzvm_stack_at(m_tBuzzVM, 1);
      buzzvm_gstore(m_tBuzzVM);
      /* Get proximity readings */
      const CCI_KheperaIVLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
      /* Fill into the proximity table */
      buzzobj_t tLightRead;
      for(size_t i = 0; i < tLightReads.size(); ++i) {
         /* Create table for i-th read */
         tLightRead = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);
         /* Fill in the read */
         TablePut(tLightRead, "value", tLightReads[i].Value);
         TablePut(tLightRead, "angle", tLightReads[i].Angle);
         /* Store read table in the proximity table */
         TablePut(tLightTable, i, tLightRead);
      }
   }
   /*
    * Update ultrasound sensor table
    */
   if(m_pcUltrasound != NULL) {
      /* Create empty ultrasound table */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "ultrasound", 1));
      buzzvm_pusht(m_tBuzzVM);
      buzzobj_t tUSTable = buzzvm_stack_at(m_tBuzzVM, 1);
      buzzvm_gstore(m_tBuzzVM);
      /* Get ultrasound readings */
      const CCI_KheperaIVUltrasoundSensor::TReadings& tUSReads = m_pcUltrasound->GetReadings();
      /* Fill into the ultrasound table */
      buzzobj_t tUSRead;
      for(size_t i = 0; i < tUSReads.size(); ++i) {
         /* Create table for i-th read */
         tUSRead = buzzheap_newobj(m_tBuzzVM, BUZZTYPE_TABLE);
         /* Fill in the read */
         TablePut(tUSRead, "value", tUSReads[i].Value);
         TablePut(tUSRead, "angle", tUSReads[i].Angle);
         /* Store read table in the ultrasound table */
         TablePut(tUSTable, i, tUSRead);
      }
   }
   /*
    * Update lidar sensor table
    */
   if(m_pcLIDAR != NULL) {
      /* Create empty lidar table */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "lidar", 1));
      buzzvm_pusht(m_tBuzzVM);
      buzzobj_t tLTable = buzzvm_stack_at(m_tBuzzVM, 1);
      buzzvm_gstore(m_tBuzzVM);
      /* Fill into the lidar table */
      for(size_t i = 0; i < m_pcLIDAR->GetNumReadings(); ++i) {
         /* Store i-th read in the lidar table */
         TablePut(tLTable, i, static_cast<SInt32>(m_pcLIDAR->GetReading(i)));
      }
   }
}

/****************************************/
/****************************************/

void CConnectivityBuzzControllerKheperaIV::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseWheelSpeed;
         fSpeed2 = fBaseWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseWheelSpeed - fBaseWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseWheelSpeed + fBaseWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CConnectivityBuzzControllerKheperaIV::SetWheels(Real f_left_speed,
                                         Real f_right_speed) {
   m_pcWheels->SetLinearVelocity(f_left_speed,
                                 f_right_speed);
}

/****************************************/
/****************************************/

void CConnectivityBuzzControllerKheperaIV::SetLEDs(const CColor& c_color) {
   m_pcLEDs->SetAllColors(c_color);
}



std::vector<std::vector<double>> CConnectivityBuzzControllerKheperaIV::OneShotPathPlanner(float* start_end_time){
  // for(int i =0; i< 5; i++){
  //   fprintf(stderr, "pos args received in recur planner %i -> %f \n",i,start_end_time[i]);
  // }
  // /* If solved obtain the best solutions found */
  std::vector<std::vector<double>> solution_nodes;
  /* Initialize a 2d vector state space within the planner */
    ob::StateSpacePtr oneshot_space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));

    /* Set bounds to the state space */
    ob::RealVectorBounds m_bound(2);
    m_bound.setLow(0,0);
    m_bound.setLow(1,0);

    /* Should have been updated by import map call */
    m_bound.setHigh(0,half_map_height);
    m_bound.setHigh(1,half_map_length);
    statespacemax[0]=half_map_height;
    statespacemax[1]=half_map_length;

    /* Set the bounds of space */
    oneshot_space->as<ob::RealVectorStateSpace>()->setBounds(m_bound);
    /* Construct a space information instance for this state space */
    ob::SpaceInformationPtr si(new ob::SpaceInformation(oneshot_space));
    /* Set the object used to check which states in the space are valid */
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,&Grid_map[0], &m_restrictz, map_resolution)));
    si->setup();
    // printf("Half height %f length %f\n",half_map_height, half_map_length );
    // Set our robot's starting state to be the one obtained by import map
    ob::ScopedState<> start(oneshot_space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_end_time[0]+(half_map_height/2);//start_end_pos[0][0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_end_time[1]+(half_map_length/2);//start_end_pos[0][1];
    // Set our robot's goal state tto be the one obtained by import map
    ob::ScopedState<> goal(oneshot_space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_end_time[2]+(half_map_height/2); //start_end_pos[1][0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_end_time[3]+(half_map_length/2); //start_end_pos[1][1];
    // Create a problem instance
    ob::ProblemDefinitionPtr oneshot_pdef= ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    // Set the start and goal states
    oneshot_pdef->setStartAndGoalStates(start, goal);

    // Construct our optimizing planner using the RRTstar algorithm.
    og::RRTstar* oneshot_m_rrt_planner = new og::RRTstar(si);

    // UnComment for any other planner !!!
    // og::PRMstar* oneshot_m_rrt_planner = new og::PRMstar(si);

    // og::SST* oneshot_m_rrt_planner = new og::SST(si);

    // m_rrt_planner->printProperties(std::cout);
    ob::PlannerPtr oneshot_optimizingPlanner = ob::PlannerPtr(oneshot_m_rrt_planner);
    // Set the problem instance for our planner to solve
    oneshot_optimizingPlanner->setProblemDefinition(oneshot_pdef);
    oneshot_optimizingPlanner->setup();
    // attempt to solve the planning problem within time in second given by the user from bzz.
    ob::PlannerStatus solved = oneshot_optimizingPlanner->solve(start_end_time[4]);

    // Keep solving, if you did not reach the target yet. 
    // while(oneshot_pdef->hasApproximateSolution()) solved = oneshot_optimizingPlanner->solve(0.1);
    
    if (solved)
    {

      // Output the length of the path found
      // std::cout 
      //     << optimizingPlanner->getName()
      //     << " found a solution of length "
      //     << pdef->getSolutionPath()->length()
      //     << " with an optimization objective value of "
      //     << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective())<<"Best path : "
      //     << std::endl;
          // std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath())->print(std::cout);
      std::shared_ptr<ompl::geometric::PathGeometric> c_path = 
            std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(oneshot_pdef->getSolutionPath());
      
      // fprintf(stderr, "Current state count in path: %i , length: %f, Num of states req %f\n", 
      //                   (int)(c_path->getStateCount()), c_path->length(), c_path->length()/Required_path_segment_len);
      c_path->interpolate((unsigned int) c_path->length()/Required_path_segment_len);
      // fprintf(stderr, "After intepolation: %i \n", (int)(c_path->getStateCount())); 
      std::vector<ob::State *> solutionStates = c_path->getStates();                  
      for(auto state : solutionStates){
        std::vector<double> statePoint(3,0.0);
        // std::cout<<" SOl state  X "<< (state->as<ob::RealVectorStateSpace::StateType>()->values[0])-(half_map_height/2)
        //   <<" Y "<<(state->as<ob::RealVectorStateSpace::StateType>()->values[1])-(half_map_length/2)<<std::endl;
          statePoint[0]= state->as<ob::RealVectorStateSpace::StateType>()->values[0]-(half_map_height/2);
          statePoint[1]= state->as<ob::RealVectorStateSpace::StateType>()->values[1]-(half_map_length/2);
          solution_nodes.push_back(statePoint);
      }

      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "PATH_TYPE", 1));
      buzzvm_pushi(m_tBuzzVM, -1);
      buzzvm_gstore(m_tBuzzVM);

      
      // For boder padding
      MIN_X =-(half_map_height/2.0)-5;
      MIN_Y =-(half_map_length/2.0)-5;
      MAX_X =(half_map_height/2.0)+5;
      MAX_Y =(half_map_length/2.0)+5;
      // Save nodes to file
      if(save_solution_svg){
         std::stringstream s_name;
         s_name<<"nodes_"<<m_tBuzzVM->robot<<".svg";
         std::string dir(s_name.str());
         int image_width=500;
         int image_height=500;

         svg::Dimensions dimensions(image_width, image_height);
         svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));

         // Draw solution path

         std::vector<ob::State *> s_path = 
            std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(oneshot_pdef->getSolutionPath())->getStates();
         svg::Polyline traj_line(svg::Stroke(0.5, svg::Color::Black));
         for(auto state : s_path){
            traj_line<<visualize_point(state,dimensions);
         }
         doc<<traj_line;
         // if(! (debug_steps % 100) ) 
         //   m_rrt_planner->draw_tree(doc,dimensions,half_map_height,half_map_length);
         debug_steps++;
         double x_offset= 0.0,y_offset=0.0;
         int Planning_plane = 0;

        
         // Draw obstacles
         for(int i=0;i<Grid_map_GT[Planning_plane].size();++i){
           for(int j=0;j<Grid_map_GT[Planning_plane][i].size();++j){
             if(i==0){ // top case
               if(Grid_map_GT[Planning_plane][i][j] == 1 && (Grid_map_GT[Planning_plane][i][j-1] == 0 ||
                 Grid_map_GT[Planning_plane][i][j+1] == 0 || Grid_map_GT[Planning_plane][i+1][j] == 0 ||
                 Grid_map_GT[Planning_plane][i+1][j+1] == 0 || Grid_map_GT[Planning_plane][i+1][j-1] == 0) ){
                 double temp[2];
                 // Add a column for trees
                 temp[0] = (i*map_resolution) - (OBSTACLE_SIDES1/2)+x_offset;
                 temp[1] = (j*map_resolution) + (OBSTACLE_SIDES2/2)+y_offset;
                 doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                 (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                 (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                 svg::Color::Red);
               }
             }
             else if(i == Grid_map_GT[Planning_plane].size()-1){ // bottom case 
               if(Grid_map_GT[Planning_plane][i][j] == 1 && (Grid_map_GT[Planning_plane][i][j-1] == 0 || 
                 Grid_map_GT[Planning_plane][i][j+1] == 0 || Grid_map_GT[Planning_plane][i-1][j] == 0 ||
                 Grid_map_GT[Planning_plane][i-1][j-1] == 0 || Grid_map_GT[Planning_plane][i-1][j+1] == 0) ){
                 double temp[2];
                 // Add a column for trees
                 temp[0] = (i*map_resolution) - (OBSTACLE_SIDES1/2)+x_offset;
                 temp[1] = (j*map_resolution) + (OBSTACLE_SIDES2/2)+y_offset;
                 doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                 (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                 (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                 svg::Color::Red);
               }
             }
             else if(j == 0){ // Left case
               if(Grid_map_GT[Planning_plane][i][j] == 1 && (Grid_map_GT[Planning_plane][i][j+1] == 0 || 
                 Grid_map_GT[Planning_plane][i-1][j] == 0 || Grid_map_GT[Planning_plane][i+1][j] == 0 ||
                 Grid_map_GT[Planning_plane][i-1][j+1] == 0 || Grid_map_GT[Planning_plane][i+1][j+1] == 0) ){
                 double temp[2];
                 // Add a column for trees
                 temp[0] = (i*map_resolution) - (OBSTACLE_SIDES1/2)+x_offset;
                 temp[1] = (j*map_resolution) + (OBSTACLE_SIDES2/2)+y_offset;
                 doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                 (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                 (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                 svg::Color::Red);

               }
             }
             else if(j == Grid_map_GT[Planning_plane][i].size()-1){ // right case
               if(Grid_map_GT[Planning_plane][i][j] == 1 && (Grid_map_GT[Planning_plane][i][j-1] == 0 || 
                 Grid_map_GT[Planning_plane][i-1][j] == 0 || Grid_map_GT[Planning_plane][i+1][j] == 0 ||
                 Grid_map_GT[Planning_plane][i-1][j-1] == 0 || Grid_map_GT[Planning_plane][i+1][j-1] == 0) ){
                 double temp[2];
                 // Add a column for trees
                 temp[0] = (i*map_resolution) - (OBSTACLE_SIDES1/2)+x_offset;
                 temp[1] = (j*map_resolution) + (OBSTACLE_SIDES2/2)+y_offset;
                 doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                 (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                 (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                 svg::Color::Red);
               }
             }
             else{
               if(Grid_map_GT[Planning_plane][i][j] == 1 && (Grid_map_GT[Planning_plane][i][j-1] == 0 || 
                 Grid_map_GT[Planning_plane][i][j+1] == 0 || Grid_map_GT[Planning_plane][i-1][j] == 0 || Grid_map[Planning_plane][i+1][j] == 0 ||
                 Grid_map_GT[Planning_plane][i-1][j-1] == 0 || Grid_map_GT[Planning_plane][i-1][j+1] == 0 ||
                 Grid_map_GT[Planning_plane][i+1][j+1] == 0 || Grid_map_GT[Planning_plane][i+1][j-1] == 0) ){
                 double temp[2];
                 // Add a column for trees
                 temp[0] = (i*map_resolution) - (OBSTACLE_SIDES1/2)+x_offset;
                 temp[1] = (j*map_resolution) + (OBSTACLE_SIDES2/2)+y_offset;
                 doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                 (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                 (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                 svg::Color::Red);
               } 
             }
           }
         }

        double draw_c_pos[2];
        double m_point[2]={start->as<ob::RealVectorStateSpace::StateType>()->values[0], 
                   start->as<ob::RealVectorStateSpace::StateType>()->values[1]};
        draw_c_pos[0]= start_end_time[0]+(half_map_height/2);//start_end_pos[0][0];
        draw_c_pos[1]= start_end_time[1]+(half_map_length/2);
        svg::Circle circle(visualize_point(draw_c_pos,dimensions),
                        4,svg::Fill( svg::Color(255,0,0) ));
        doc<<circle;
        draw_c_pos[0]= start_end_time[2]+(half_map_height/2);//start_end_pos[0][0];
        draw_c_pos[1]= start_end_time[3]+(half_map_length/2);
        svg::Circle circle2(visualize_point(draw_c_pos,dimensions),
                        4,svg::Fill( svg::Color(0,255,0) ));
        doc<<circle2;

        doc.save();
      }
    }
    else{
     fprintf(stderr, "No solution found \n"); 
     buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "PATH_TYPE", 1));
     buzzvm_pushi(m_tBuzzVM, -2);
     buzzvm_gstore(m_tBuzzVM);
    }    
  return solution_nodes;
}

/************************************/
/************************************/

std::vector<std::vector<double>> CConnectivityBuzzControllerKheperaIV::import_map(std::string m_map_file_name)
{
  std::string line;
  std::ifstream m_map_file (m_map_file_name);
  int map_height = 0;
  int map_length = 0;
  // std::vector<std::vector<int>> Grid_map;
  std::vector<double> start_pos(2);
  std::vector<double> end_pos(2);
  if (m_map_file.is_open())
  {
    int line_num = 0;
    std::vector<std::vector<int>> Grid_map2d; // Actual map
    std::vector<std::vector<int>> Grid_GT; // Ground truth imported from map
    while ( getline (m_map_file,line) )
    { 
      if(line_num < 4){ // header
        if(line_num == 1){
           std::string str_height = line.substr (7,line.size());
           map_height = std::stoi(str_height) ;
        }
        if(line_num == 2){
           std::string str_length = line.substr (6,line.size());
           map_length = std::stoi(str_length) ;
        }
        if(line_num == 3){
          half_map_height = map_height;
          half_map_length = map_length;
          MIN_X = 0; // half_map_height;
          MIN_Y = 0; //half_map_length;
          MAX_X = map_height-1; // half_map_height*-1;
          MAX_Y = map_length-1; // half_map_length*-1;
        }
      }
      else{
        std::vector<int> v_row;
        for(uint32_t i=0;i<line.size();i++){
          if(line[i] == free_space || line[i] == free_space_l || line[i] == START_space ||
             line[i] == TARGET_space){
            for(int res =0; res < 1/map_resolution; res++)
              v_row.push_back(0);
          }
          else
            for(int res =0; res < 1/map_resolution; res++)
              v_row.push_back(1);
          if(line[i] == START_space){
            start_pos[0]=(line_num-4); //+half_map_height;
            start_pos[1]=i; // +half_map_length;
          }
          else if(line[i] == TARGET_space){
            end_pos[0]=(line_num-4); //+half_map_height;
            end_pos[1]=i; //+half_map_length;
          }
        }
        for(int res =0; res < 1/map_resolution; res++){
          Grid_GT.push_back(v_row);
          std::vector<int> empty_row(v_row.size(),0);
          Grid_map2d.push_back(empty_row);
        }
      }
      line_num +=1;
    }
    m_map_file.close();
    // Grid_map.push_back(Grid_map2d);
    // Grid_map_visited.push_back(Grid_map2d);
    Grid_map_visited.push_back(Grid_GT);
    Grid_map.push_back(Grid_GT);
    Grid_map_GT.push_back(Grid_GT);
    // half_map_length = half_map_length *-1;
    // half_map_height = half_map_height *-1;

    // for(int p=0; p<Grid_map_GT[0].size();p++){
    //   fprintf(stderr, "Index: %i",p );
    //   for(int q=0; q<Grid_map_GT[0][p].size();q++){
    //     fprintf(stderr, "%i", Grid_map_GT[0][p][q]);
    //   }
    //   fprintf(stderr, "\n" );
    // }


    //Temporary for testing update the portion of map from current pos. 
    // for(size_t i = 36; i <Grid_map_GT[0].size(); ++i ){
    //   for(size_t j = 0; j < Grid_map_GT[0][i].size(); ++j ){
    //     Grid_map[0][i][j] = Grid_map_GT[0][i][j];
    //     Grid_map_visited[0][i][j] = 1;
    //     if(i < exploration_bound_low[0]) exploration_bound_low[0] = i;
    //     if(i > exploration_bound_high[0]) exploration_bound_high[0] = i;
    //     if(j < exploration_bound_low[1]) exploration_bound_low[1] = j;
    //     if(i > exploration_bound_high[1]) exploration_bound_high[1] = j;
    //   }
    // }

  }
  else {printf("ERROR in Opening Map file\n");}
  std::vector<std::vector<double>> start_end_pos;
  start_end_pos.push_back(start_pos);
  start_end_pos.push_back(end_pos);
  return start_end_pos;
}


/************************************/
/************************************/


std::vector<std::vector<double>> CConnectivityBuzzControllerKheperaIV::import_3dmap(std::string m_map_file_name)
{
  std::string line;
  std::ifstream m_map_file (m_map_file_name);
  int map_height = 0;
  int map_length = 0;
  int map_planes = 0;
  // std::vector<std::vector<int>> Grid_map;
  std::vector<std::vector<int>> v_plane;
  std::vector<double> start_pos(2);
  std::vector<double> end_pos(2);
  int current_plane = 0;
  if (m_map_file.is_open())
  {
    int line_num = 0;
    while ( getline (m_map_file,line) )
    { 
      if(line_num < 6){ // header
        if(line_num == 2){
           std::string str_height = line.substr (7,line.size());
           map_height = std::stoi(str_height) ;
           // std::cout << "height "<< map_height << '\n';
        }
        if(line_num == 3){
           std::string str_length = line.substr (6,line.size());
           map_length = std::stoi(str_length) ;
           // std::cout << "length "<< map_length << '\n';
        }
        if(line_num == 4){
          half_map_height = map_height;
          half_map_length = map_length;
          MIN_X = 0; // half_map_height;
          MIN_Y = 0; //half_map_length;
          MAX_X = map_height-1; // half_map_height*-1;
          MAX_Y = map_length-1; // half_map_length*-1;
          std::string str_height = line.substr (7,line.size());
          map_planes = std::stoi(str_height);
          // std::cout << "half length "<< half_map_length << '\n';
          // std::cout << "half height "<< half_map_height << '\n';
        }
      }
      else{
        if(line_num == map_height+6) line_num = 5;
        // plane_pos = (current_plane + 1) * PLANE_RESOLUTION;
        std::string plane_str = line.substr(0,5);
        if(plane_str == "plane"){
          current_plane = std::stoi(line.substr (6,line.size()));
          Grid_map.push_back(v_plane);
          v_plane.clear();
        }
        else{
          std::vector<int> v_row;
          for(int i=0;i<line.size();i++){
            if(line[i] == free_space || line[i] == free_space_l || line[i] == START_space ||
               line[i] == TARGET_space)
              v_row.push_back(0);
            else
              v_row.push_back(1);
            if(line[i] == START_space && current_plane == 0){
              start_pos[0]=(line_num-6); //+half_map_height;
              start_pos[1]=i; // +half_map_length;
            }
            else if(line[i] == TARGET_space && current_plane == 0){
              end_pos[0]=(line_num-6); //+half_map_height;
              end_pos[1]=i; //+half_map_length;
            }
        }
        v_plane.push_back(v_row);
      }
    }
      line_num +=1;
    }
    if(v_plane.size() > 0){
            Grid_map.push_back(v_plane);
            v_plane.clear();
    }
    m_map_file.close();

  }
  else {printf("ERROR in Opening Map file\n");}
  std::vector<std::vector<double>> start_end_pos;
  start_end_pos.push_back(start_pos);
  start_end_pos.push_back(end_pos);
  // std::cout<<"start X "<<start_pos[0]<<" y "<<start_pos[1]<<std::endl;
  // std::cout<<"end x "<<end_pos[0]<<"  y "<<end_pos[1]<<std::endl; 
  return start_end_pos;
}



/****************************************/
/****************************************/

buzzvm_state CConnectivityBuzzControllerKheperaIV::RegisterFunctions() {
   /* Register base functions */
   CBuzzController::RegisterFunctions();
   /* Register path planning hooks */
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "OneShotPathPlanner", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, C_OneShotPathPlanner));
   buzzvm_gstore(m_tBuzzVM); 

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "LoadGTMap", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, C_LoadGTMap));
   buzzvm_gstore(m_tBuzzVM); 
   if(m_pcWheels) {
      /* BuzzSetWheels */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "set_wheels", 1));
      buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzSetWheels));
      buzzvm_gstore(m_tBuzzVM);
      /* BuzzGoTo with Cartesian coordinates */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "goto", 1));
      buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzGoToC));
      buzzvm_gstore(m_tBuzzVM);
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "gotoc", 1));
      buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzGoToC));
      buzzvm_gstore(m_tBuzzVM);
      /* BuzzGoTo with Polar coordinates */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "gotop", 1));
      buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzGoToP));
      buzzvm_gstore(m_tBuzzVM);
   }
   if(m_pcLEDs) {
      /* BuzzSetLEDs */
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "set_leds", 1));
      buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzSetLEDs));
      buzzvm_gstore(m_tBuzzVM);
   }
   return m_tBuzzVM->state;
}

std::string CConnectivityBuzzControllerKheperaIV::GetMapFName(){
  return strmapFName;
}

/****************************************/
/****************************************/
svg::Point CConnectivityBuzzControllerKheperaIV::visualize_point(float* state, svg::Dimensions dims)
{
  double x = ((state[0]-(half_map_height/2))-MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1]-(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point CConnectivityBuzzControllerKheperaIV::visualize_point(double* state, svg::Dimensions dims)
{
  double x = ((state[0]-(half_map_height/2))-MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1]-(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point CConnectivityBuzzControllerKheperaIV::visualize_point(std::vector<double> state, svg::Dimensions dims)
{
  double x = ((state[0]-(half_map_height/2)) -MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1]-(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point CConnectivityBuzzControllerKheperaIV::visualize_point_woScale(std::vector<double> state, svg::Dimensions dims)
{
  double x = ((state[0]+(half_map_height/2)) -MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1]+(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point CConnectivityBuzzControllerKheperaIV::visualize_point(ob::State* state, svg::Dimensions dims)
{
  double x = (((double)state->as<ob::RealVectorStateSpace::StateType>()->values[0]-(half_map_height/2)) -MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = (((double)state->as<ob::RealVectorStateSpace::StateType>()->values[1]-(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CConnectivityBuzzControllerKheperaIV, "buzz_connectivity_controller_kheperaiv");

#ifdef ARGOS_DYNAMIC_LIBRARY_LOADING
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
REGISTER_BUZZ_ROBOT(CKheperaIVEntity);
#endif

