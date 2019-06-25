#include "buzz_controller_quadmapper_with_dataset.h"

namespace buzz_quadmapper {

/****************************************/
/************ Buzz Closures *************/
/****************************************/

static int BuzzAddSeparator(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int is_outlier = reinterpret_cast<CBuzzControllerQuadMapperWithDataset*>(buzzvm_stack_at(vm, 1)->u.value)->AddSeparatorMeasurement();
   buzzvm_pushi(vm, is_outlier);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzMove(buzzvm_t vm) {
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   int current_pose_id = reinterpret_cast<CBuzzControllerQuadMapperWithDataset*>(buzzvm_stack_at(vm, 1)->u.value)->Move();
   
   buzzvm_pushi(vm, current_pose_id);

   return buzzvm_ret1(vm);
}

/****************************************/
/****************************************/

static int BuzzLoadDatasetParameters(buzzvm_t vm) {
   /* Push the vector components */
   buzzvm_lload(vm, 1);
   buzzvm_lload(vm, 2);
   buzzvm_lload(vm, 3);
   /* Create a new vector with that */
   float sensor_range;
   float outlier_probability;
   std::string dataset_name;
   buzzobj_t b_dataset_name = buzzvm_stack_at(vm, 3);
   buzzobj_t b_sensor_range = buzzvm_stack_at(vm, 2);
   buzzobj_t b_outlier_probability = buzzvm_stack_at(vm, 1);
   if(b_dataset_name->o.type == BUZZTYPE_STRING) dataset_name = b_dataset_name->s.value.str;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "load_no_sensing_parameters: expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_STRING],
                      buzztype_desc[b_dataset_name->o.type]
         );
      return vm->state;
   } 
   if(b_sensor_range->o.type == BUZZTYPE_FLOAT) sensor_range = b_sensor_range->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "load_no_sensing_parameters: expected %s, got %s in first argument",
                      buzztype_desc[BUZZTYPE_FLOAT],
                      buzztype_desc[b_sensor_range->o.type]
         );
      return vm->state;
   }   
   if(b_outlier_probability->o.type == BUZZTYPE_FLOAT) outlier_probability = b_outlier_probability->f.value;
   else {
      buzzvm_seterror(vm,
                      BUZZVM_ERROR_TYPE,
                      "load_no_sensing_parameters: expected %s, got %s in second argument",
                      buzztype_desc[BUZZTYPE_INT],
                      buzztype_desc[b_outlier_probability->o.type]
         );
      return vm->state;
   }    
   /* Get pointer to the controller */
   buzzvm_pushs(vm, buzzvm_string_register(vm, "controller", 1));
   buzzvm_gload(vm);
   /* Call function */
   reinterpret_cast<CBuzzControllerQuadMapperWithDataset*>(buzzvm_stack_at(vm, 1)->u.value)->LoadParameters(dataset_name, sensor_range, outlier_probability);
   
   return buzzvm_ret0(vm);
}

/****************************************/
/************ Registration **************/
/****************************************/

buzzvm_state CBuzzControllerQuadMapperWithDataset::RegisterFunctions() {
   CBuzzControllerQuadMapper::RegisterFunctions();

   /* Register mapping without sensing specific functions */
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "add_separator", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzAddSeparator));
   buzzvm_gstore(m_tBuzzVM);
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "move", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzMove));
   buzzvm_gstore(m_tBuzzVM);
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "load_dataset_parameters", 1));
   buzzvm_pushcc(m_tBuzzVM, buzzvm_function_register(m_tBuzzVM, BuzzLoadDatasetParameters));
   buzzvm_gstore(m_tBuzzVM);

   return m_tBuzzVM->state;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBuzzControllerQuadMapperWithDataset, "buzz_controller_quadmapper_with_dataset");

}