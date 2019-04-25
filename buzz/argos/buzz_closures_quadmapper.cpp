#include "buzz_controller_quadmapper.h"

namespace buzz_quadmapper {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzInitOptimizer(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_period = buzzvm_stack_at(vm, 1);
   int period;

   if(buzz_period->o.type == BUZZTYPE_INT) period = buzz_period->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "srand(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_period->o.type]
         );
      return vm->state;
   } 

   // Initialize optimizer
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->InitOptimizer(period);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzOptimizerState(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   int state = reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->GetOptimizerState();
   buzzvm_pushi(vm, state);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzAddNeighborWithinCommunicationRange(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "AddNeighborWithinCommunicationRange: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->AddNeighborWithinCommunicationRange(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzComputeAndUpdateRotationEstimatesToSend(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzComputeAndUpdateRotationEstimatesToSend: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->ComputeAndUpdateRotationEstimatesToSend(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateNeighborRotationEstimates(buzzvm_t vm){

   buzzvm_lnum_assert(vm, 1);

   buzzvm_lload(vm, 1);

   buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);

   std::vector<std::vector<rotation_estimate_t>> received_rotation_estimates;

   buzzobj_t b_rotation_estimates_table = buzzvm_stack_at(vm, 1);

   for (int32_t i = 0; i < buzzdict_size(b_rotation_estimates_table->t.value); ++i) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, i);
      buzzvm_tget(vm);
      buzzobj_t b_rotation_estimates_from_robot_i = buzzvm_stack_at(vm, 1);

      received_rotation_estimates.emplace_back(std::vector<rotation_estimate_t>());
      for (int32_t j = 0; j < buzzdict_size(b_rotation_estimates_from_robot_i->t.value); ++j) {
         buzzvm_dup(vm);
         buzzvm_pushi(vm, j);
         buzzvm_tget(vm);
         buzzobj_t b_individual_rotation_estimate_j = buzzvm_stack_at(vm, 1);

         rotation_estimate_t rotation_estimate;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_id = buzzvm_stack_at(vm, 1);
         rotation_estimate.sender_robot_id = b_sender_robot_id->i.value;
         buzzvm_pop(vm);

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_pose_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_pose_id = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         rotation_estimate.sender_pose_id = b_sender_pose_id->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_is_initialized", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_is_initialized = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         rotation_estimate.sender_robot_is_initialized = (bool)b_sender_robot_is_initialized->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "rotation_estimate", 1));
         buzzvm_tget(vm);
         buzzobj_t b_rotation_estimate = buzzvm_stack_at(vm, 1);
         for (int32_t k = 0; k < buzzdict_size(b_rotation_estimate->t.value); ++k) {
            buzzvm_dup(vm);
            buzzvm_pushi(vm, k);
            buzzvm_tget(vm);
            buzzobj_t b_rotation_matrix_elem = buzzvm_stack_at(vm, 1);
            buzzvm_pop(vm);
            rotation_estimate.rotation_matrix[k] = b_rotation_matrix_elem->f.value;
         }
         buzzvm_pop(vm);
         buzzvm_pop(vm);
         received_rotation_estimates.at(i).emplace_back(rotation_estimate);
      }
      buzzvm_pop(vm);
   }

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdateNeighborRotationEstimates(received_rotation_estimates);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzEstimateRotationAndUpdateRotation(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->EstimateRotationAndUpdateRotation();
}

/****************************************/
/****************************************/

static int BuzzComputeAndUpdatePoseEstimatesToSend(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzComputeAndUpdatePoseEstimatesToSend: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->ComputeAndUpdatePoseEstimatesToSend(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzUpdateNeighborPoseEstimates(buzzvm_t vm){

   buzzvm_lnum_assert(vm, 1);

   buzzvm_lload(vm, 1);

   buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);

   std::vector<std::vector<pose_estimate_t>> received_pose_estimates;

   buzzobj_t b_pose_estimates_table = buzzvm_stack_at(vm, 1);

   for (int32_t i = 0; i < buzzdict_size(b_pose_estimates_table->t.value); ++i) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, i);
      buzzvm_tget(vm);
      buzzobj_t b_pose_estimates_from_robot_i = buzzvm_stack_at(vm, 1);

      received_pose_estimates.emplace_back(std::vector<pose_estimate_t>());
      for (int32_t j = 0; j < buzzdict_size(b_pose_estimates_from_robot_i->t.value); ++j) {
         buzzvm_dup(vm);
         buzzvm_pushi(vm, j);
         buzzvm_tget(vm);
         buzzobj_t b_individual_pose_estimate_j = buzzvm_stack_at(vm, 1);

         pose_estimate_t pose_estimate;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_id = buzzvm_stack_at(vm, 1);
         pose_estimate.sender_robot_id = b_sender_robot_id->i.value;
         buzzvm_pop(vm);

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_pose_id", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_pose_id = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         pose_estimate.sender_pose_id = b_sender_pose_id->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "sender_robot_is_initialized", 1));
         buzzvm_tget(vm);
         buzzobj_t b_sender_robot_is_initialized = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         pose_estimate.sender_robot_is_initialized = (bool)b_sender_robot_is_initialized->i.value;

         buzzvm_dup(vm);
         buzzvm_pushs(vm, buzzvm_string_register(vm, "pose_estimate", 1));
         buzzvm_tget(vm);
         buzzobj_t b_pose_estimate = buzzvm_stack_at(vm, 1);
         for (int32_t k = 0; k < buzzdict_size(b_pose_estimate->t.value); ++k) {
            buzzvm_dup(vm);
            buzzvm_pushi(vm, k);
            buzzvm_tget(vm);
            buzzobj_t b_pose_matrix_elem = buzzvm_stack_at(vm, 1);
            buzzvm_pop(vm);
            pose_estimate.pose_data[k] = b_pose_matrix_elem->f.value;
         }
         buzzvm_pop(vm);
         buzzvm_pop(vm);
         received_pose_estimates.at(i).emplace_back(pose_estimate);
      }
      buzzvm_pop(vm);
   }

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->UpdateNeighborPoseEstimates(received_pose_estimates);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzEstimatePoseAndUpdatePose(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->EstimatePoseAndUpdatePose();
}
/****************************************/
/****************************************/

static int BuzzSRand(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_seed = buzzvm_stack_at(vm, 1);
   int seed;

   if(buzz_seed->o.type == BUZZTYPE_INT) seed = buzz_seed->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "srand(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_seed->o.type]
         );
      return vm->state;
   } 

   srand(time(NULL)+seed);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzRandUniform(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzobj_t buzz_range_lowest = buzzvm_stack_at(vm, 2);
   buzzobj_t buzz_range_highest = buzzvm_stack_at(vm, 1);
   int range_lowest, range_highest;

   if(buzz_range_lowest->o.type == BUZZTYPE_INT) range_lowest = buzz_range_lowest->i.value;
   else if(buzz_range_lowest->o.type == BUZZTYPE_FLOAT) range_lowest = (int)buzz_range_lowest->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "rand_uniform(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_range_lowest->o.type]
         );
      return vm->state;
   } 
   if(buzz_range_highest->o.type == BUZZTYPE_INT) range_highest = buzz_range_highest->i.value;
   else if(buzz_range_highest->o.type == BUZZTYPE_FLOAT) range_highest = (int)buzz_range_highest->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "rand_uniform(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_range_highest->o.type]
         );
      return vm->state;
   } 

   float random_value = (rand()%(range_highest-range_lowest)+range_lowest);

   buzzvm_pushf(vm, random_value);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzGotoAbs(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   /* Create a new vector with that */
   CVector3 translation;
   buzzobj_t tX = buzzvm_stack_at(vm, 3);
   buzzobj_t tY = buzzvm_stack_at(vm, 2);
   buzzobj_t tZ = buzzvm_stack_at(vm, 1);
   if(tX->o.type == BUZZTYPE_INT) translation.SetX(tX->i.value);
   else if(tX->o.type == BUZZTYPE_FLOAT) translation.SetX(tX->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "goto_abs(x,y): expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tX->o.type]
         );
      return vm->state;
   }      
   if(tY->o.type == BUZZTYPE_INT) translation.SetY(tY->i.value);
   else if(tY->o.type == BUZZTYPE_FLOAT) translation.SetY(tY->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "goto_abs(x,y): expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tY->o.type]
         );
      return vm->state;
   }
   if(tZ->o.type == BUZZTYPE_INT) translation.SetZ(tZ->i.value);
   else if(tZ->o.type == BUZZTYPE_FLOAT) translation.SetZ(tZ->f.value);
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "goto_abs(x,y): expected %s, got %s in third argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[tZ->o.type]
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->SetNextPosition(translation);
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzAddSeparatorToLocalGraph(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   buzzvm_lload(vm, 4);
   buzzvm_lload(vm, 5);
   buzzvm_lload(vm, 6);
   buzzvm_lload(vm, 7);
   buzzvm_lload(vm, 8);
   buzzvm_lload(vm, 9);
   buzzvm_lload(vm, 10);
   buzzvm_lload(vm, 11);
   /* Retrieve parameters and check their types */
   buzzobj_t b_robot_1_id = buzzvm_stack_at(vm, 11);
   buzzobj_t b_robot_2_id = buzzvm_stack_at(vm, 10);
   buzzobj_t b_robot_1_pose_id = buzzvm_stack_at(vm, 9);
   buzzobj_t b_robot_2_pose_id = buzzvm_stack_at(vm, 8);
   buzzobj_t b_x = buzzvm_stack_at(vm, 7);
   buzzobj_t b_y = buzzvm_stack_at(vm, 6);
   buzzobj_t b_z = buzzvm_stack_at(vm, 5);
   buzzobj_t b_q_x = buzzvm_stack_at(vm, 4);
   buzzobj_t b_q_y = buzzvm_stack_at(vm, 3);
   buzzobj_t b_q_z = buzzvm_stack_at(vm, 2);
   buzzobj_t b_q_w = buzzvm_stack_at(vm, 1);
   int robot_1_id, robot_2_id, robot_1_pose_id, robot_2_pose_id;
   float x, y, z, q_x, q_y, q_z, q_w;
   if(b_robot_1_id->o.type == BUZZTYPE_INT &&
      b_robot_2_id->o.type == BUZZTYPE_INT &&
      b_robot_1_pose_id->o.type == BUZZTYPE_INT &&
      b_robot_2_pose_id->o.type == BUZZTYPE_INT &&
      b_x->o.type == BUZZTYPE_FLOAT &&
      b_y->o.type == BUZZTYPE_FLOAT &&
      b_z->o.type == BUZZTYPE_FLOAT &&
      b_q_x->o.type == BUZZTYPE_FLOAT &&
      b_q_y->o.type == BUZZTYPE_FLOAT &&
      b_q_z->o.type == BUZZTYPE_FLOAT &&
      b_q_w->o.type == BUZZTYPE_FLOAT) {

      // Fill in variables
      robot_1_id = b_robot_1_id->i.value;
      robot_2_id = b_robot_2_id->i.value;
      robot_1_pose_id = b_robot_1_pose_id->i.value;
      robot_2_pose_id = b_robot_2_pose_id->i.value;
      x = b_x->f.value;
      y = b_y->f.value;
      z = b_z->f.value;
      q_x = b_q_x->f.value;
      q_y = b_q_y->f.value;
      q_z = b_q_z->f.value;
      q_w = b_q_w->f.value;

   } else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "wrong parameter type for add_separator_to_local_graph."
         );
      return vm->state;
   }
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->AddSeparatorToLocalGraph(  robot_1_id, robot_2_id,
                                                                                                               robot_1_pose_id, robot_2_pose_id,
                                                                                                               x, y, z,
                                                                                                               q_x, q_y, q_z, q_w  );
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzRotationEstimationStoppingConditions(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   bool is_finished = reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->RotationEstimationStoppingConditions();
   buzzvm_pushi(vm, is_finished);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzPoseEstimationStoppingConditions(buzzvm_t vm){

   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   bool is_finished = reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->PoseEstimationStoppingConditions();
   buzzvm_pushi(vm, is_finished);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzNeighborRotationEstimationIsFinished(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzNeighborRotationEstimationIsFinished: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->NeighborRotationEstimationIsFinished(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

static int BuzzNeighborPoseEstimationIsFinished(buzzvm_t vm){

   buzzvm_lload(vm, 1);
   
   buzzobj_t buzz_rid = buzzvm_stack_at(vm, 1);
   int rid;

   if(buzz_rid->o.type == BUZZTYPE_INT) rid = buzz_rid->i.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "BuzzNeighborPoseEstimationIsFinished: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[buzz_rid->o.type]
         );
      return vm->state;
   } 

   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapper*>(buzzvm_stack_at(vm, 1)->u.value)->NeighborPoseEstimationIsFinished(rid);

   return buzzvm_ret0(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state CBuzzControllerQuadMapper::RegisterFunctions() {
   CBuzzControllerSpiri::RegisterFunctions();

   /* Register mapping specific functions */
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "srand", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzSRand));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "rand_uniform", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzRandUniform));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "add_separator_to_local_graph", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzAddSeparatorToLocalGraph));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "goto_abs", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzGotoAbs));
   buzzvm_gstore(m_tBuzzVM);
   
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "init_optimizer", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzInitOptimizer));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "optimizer_state", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzOptimizerState));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "add_neighbor_within_communication_range", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzAddNeighborWithinCommunicationRange));
   buzzvm_gstore(m_tBuzzVM);
   
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "compute_and_update_rotation_estimates_to_send", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzComputeAndUpdateRotationEstimatesToSend));
   buzzvm_gstore(m_tBuzzVM);
   
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "compute_and_update_pose_estimates_to_send", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzComputeAndUpdatePoseEstimatesToSend));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_neighbor_rotation_estimates", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdateNeighborRotationEstimates));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "update_neighbor_pose_estimates", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzUpdateNeighborPoseEstimates));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "estimate_rotation_and_update_rotation", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzEstimateRotationAndUpdateRotation));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "estimate_pose_and_update_pose", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzEstimatePoseAndUpdatePose));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "rotation_estimation_stopping_conditions", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzRotationEstimationStoppingConditions));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "pose_estimation_stopping_conditions", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzPoseEstimationStoppingConditions));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "neighbor_rotation_estimation_is_finished", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzNeighborRotationEstimationIsFinished));
   buzzvm_gstore(m_tBuzzVM);

   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "neighbor_pose_estimation_is_finished", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzNeighborPoseEstimationIsFinished));
   buzzvm_gstore(m_tBuzzVM);

   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapper, "buzz_controller_quadmapper");
}