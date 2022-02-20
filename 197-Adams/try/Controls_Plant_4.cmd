!
!-------------------------- Default Units for Model ---------------------------!
!
!
defaults units  &
   length = mm  &
   angle = deg  &
   force = newton  &
   mass = kg  &
   time = sec
!
defaults units  &
   coordinate_system_type = cartesian  &
   orientation_type = body313
!
!------------------------ Default Attributes for Model ------------------------!
!
!
defaults attributes  &
   inheritance = bottom_up  &
   icon_visibility = on  &
   grid_visibility = off  &
   size_of_icons = 50.0  &
   spacing_for_grid = 1000.0
!
!--------------------------- Plugins used by Model ----------------------------!
!
!
plugin load  &
   plugin_name = .MDI.plugins.controls
!
!------------------------------ Adams View Model ------------------------------!
!
!
model create  &
   model_name = try
!
view erase
!
!-------------------------------- Data storage --------------------------------!
!
!
data_element create variable  &
   variable_name = .try.T  &
   adams_id = 1  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .try.pos_y  &
   adams_id = 2  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .try.pos_z  &
   adams_id = 3  &
   initial_condition = 0.0  &
   function = ""
!
data_element create plant input  &
   plant_input_name = .try.PINPUT_T  &
   adams_id = 3  &
   variable_name = .try.T
!
!--------------------------------- Materials ----------------------------------!
!
!
material create  &
   material_name = .try.steel  &
   adams_id = 1  &
   density = 7.801E-06  &
   youngs_modulus = 2.07E+05  &
   poissons_ratio = 0.29
!
!-------------------------------- Rigid Parts ---------------------------------!
!
! Create parts and their dependent markers and graphics
!
!----------------------------------- ground -----------------------------------!
!
!
! ****** Ground Part ******
!
defaults model  &
   part_name = ground
!
defaults coordinate_system  &
   default_coordinate_system = .try.ground
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .try.ground.MARKER_4  &
   adams_id = 4  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .try.ground  &
   material_type = .try.steel
!
part attributes  &
   part_name = .try.ground  &
   name_visibility = off
!
!----------------------------------- PART_2 -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .try.ground
!
part create rigid_body name_and_position  &
   part_name = .try.PART_2  &
   adams_id = 2  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .try.PART_2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .try.PART_2.MARKER_1  &
   adams_id = 1  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .try.PART_2.cm  &
   adams_id = 2  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .try.PART_2.MARKER_3  &
   adams_id = 3  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .try.PART_2  &
   material_type = .try.steel
!
! ****** Graphics for current part ******
!
geometry create shape ellipsoid  &
   ellipsoid_name = .try.PART_2.ELLIPSOID_1  &
   adams_id = 1  &
   center_marker = .try.PART_2.MARKER_1  &
   x_scale_factor = 223.60679775  &
   y_scale_factor = 223.60679775  &
   z_scale_factor = 223.60679775
!
part attributes  &
   part_name = .try.PART_2  &
   color = RED  &
   name_visibility = off
!
!----------------------------------- Forces -----------------------------------!
!
!
force create direct single_component_force  &
   single_component_force_name = .try.SFORCE_1  &
   adams_id = 1  &
   type_of_freedom = rotational  &
   i_marker_name = .try.PART_2.MARKER_3  &
   j_marker_name = .try.ground.MARKER_4  &
   action_only = on  &
   function = ""
!
!-------------------------- Adams View UDE Instances --------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .try.ground
!
undo begin_block suppress = yes
!
ude create instance  &
   instance_name = .try.Controls_Plant_1  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .try.Controls_Plant_2  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .try.Controls_Plant_3  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .try.Controls_Plant_4  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .try.Controls_Plant_1.input_channels  &
   object_value =   &
      .try.T,  &
      .try.pos_y
!
variable modify  &
   variable_name = .try.Controls_Plant_1.output_channels  &
   object_value = .try.pos_z
!
variable modify  &
   variable_name = .try.Controls_Plant_1.file_name  &
   string_value = "Controls_sphere"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_1.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_1.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .try.Controls_Plant_1.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_1.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .try.Controls_Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .try.Controls_Plant_2.input_channels  &
   object_value = .try.T
!
variable modify  &
   variable_name = .try.Controls_Plant_2.output_channels  &
   object_value =   &
      .try.pos_y,  &
      .try.pos_z
!
variable modify  &
   variable_name = .try.Controls_Plant_2.file_name  &
   string_value = "Controls_sphere"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_2.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_2.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .try.Controls_Plant_2.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_2.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .try.Controls_Plant_2
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .try.Controls_Plant_3.input_channels  &
   object_value = .try.T
!
variable modify  &
   variable_name = .try.Controls_Plant_3.output_channels  &
   object_value =   &
      .try.pos_y,  &
      .try.pos_z
!
variable modify  &
   variable_name = .try.Controls_Plant_3.file_name  &
   string_value = "Controls_Plant_3"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_3.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_3.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .try.Controls_Plant_3.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_3.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .try.Controls_Plant_3
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .try.Controls_Plant_4.input_channels  &
   object_value = .try.T
!
variable modify  &
   variable_name = .try.Controls_Plant_4.output_channels  &
   object_value =   &
      .try.pos_y,  &
      .try.pos_z
!
variable modify  &
   variable_name = .try.Controls_Plant_4.file_name  &
   string_value = "Controls_Plant_4"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_4.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .try.Controls_Plant_4.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .try.Controls_Plant_4.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .try.Controls_Plant_4.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .try.Controls_Plant_4
!
undo end_block
!
!------------------------------ Dynamic Graphics ------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .try.ground
!
geometry create shape force  &
   force_name = .try.SFORCE_1_force_graphic_1  &
   adams_id = 2  &
   force_element_name = .try.SFORCE_1  &
   applied_at_marker_name = .try.PART_2.MARKER_3
!
!---------------------------------- Accgrav -----------------------------------!
!
!
force create body gravitational  &
   gravity_field_name = gravity  &
   x_component_gravity = 0.0  &
   y_component_gravity = 0.0  &
   z_component_gravity = 0.0
!
force attributes  &
   force_name = .try.gravity  &
   visibility = off
!
!----------------------------- Analysis settings ------------------------------!
!
!
!---------------------------- Adams View Variables ----------------------------!
!
!
variable create  &
   variable_name = .try._model  &
   string_value = ".try"
!
!---------------------------- Function definitions ----------------------------!
!
!
data_element modify variable  &
   variable_name = .try.T  &
   function = "0"
!
data_element modify variable  &
   variable_name = .try.pos_y  &
   function = "DY( .try.PART_2.MARKER_1)"
!
data_element modify variable  &
   variable_name = .try.pos_z  &
   function = "DZ( .try.PART_2.MARKER_1)"
!
force modify direct single_component_force  &
   single_component_force_name = .try.SFORCE_1  &
   function = "VARVAL(.try.T)"
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .try.Controls_Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .try.Controls_Plant_2
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .try.Controls_Plant_3
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .try.Controls_Plant_4
!
!--------------------------- Expression definitions ---------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = ground
!
geometry modify shape ellipsoid  &
   ellipsoid_name = .try.PART_2.ELLIPSOID_1  &
   x_scale_factor = (2 * 111.803398875mm)  &
   y_scale_factor = (2 * 111.803398875mm)  &
   z_scale_factor = (2 * 111.803398875mm)
!
material modify  &
   material_name = .try.steel  &
   density = (7801.0(kg/meter**3))  &
   youngs_modulus = (2.07E+11(Newton/meter**2))
!
geometry modify shape force  &
   force_name = .try.SFORCE_1_force_graphic_1  &
   applied_at_marker_name = (.try.SFORCE_1.i)
!
model display  &
   model_name = try
