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
   model_name = pendulum
!
view erase
!
!-------------------------------- Data storage --------------------------------!
!
!
data_element create variable  &
   variable_name = .pendulum.ddisplace  &
   adams_id = 1  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .pendulum.displace  &
   adams_id = 2  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .pendulum.ddowndeg  &
   adams_id = 3  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .pendulum.downdeg  &
   adams_id = 4  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .pendulum.dupdeg  &
   adams_id = 5  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .pendulum.updeg  &
   adams_id = 6  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .pendulum.motivation  &
   adams_id = 7  &
   initial_condition = 0.0  &
   function = ""
!
data_element create plant input  &
   plant_input_name = .pendulum.PINPUT_mo  &
   adams_id = 5  &
   variable_name = .pendulum.motivation
!
!--------------------------------- Materials ----------------------------------!
!
!
material create  &
   material_name = .pendulum.steel  &
   adams_id = 1  &
   density = 7.801E-06  &
   youngs_modulus = 2.07E+05  &
   poissons_ratio = 0.29
!
!-------------------------------- Rigid Parts ---------------------------------!
!
! Create parts and their dependent markers and graphics
!
!----------------------------------- fixed ------------------------------------!
!
!
! ****** Ground Part ******
!
part modify rigid_body name_and_position  &
   part_name = ground  &
   new_part_name = fixed
!
part modify rigid_body name_and_position  &
   part_name = fixed  &
   adams_id = 5
!
defaults model  &
   part_name = fixed
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.fixed
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .pendulum.fixed.MARKER_19  &
   adams_id = 19  &
   location = 0.0, 30.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .pendulum.fixed.MARKER_13  &
   adams_id = 13  &
   location = -200.0, 0.0, 200.0  &
   orientation = 180.0d, 90.0d, 180.0d
!
marker create  &
   marker_name = .pendulum.fixed.cm  &
   adams_id = 16  &
   location = 0.0, -5.0, 0.0  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .pendulum.fixed.MARKER_14  &
   adams_id = 14  &
   location = 0.0, 0.0, 0.0  &
   orientation = 90.0d, 90.0d, 270.0d
!
part create rigid_body mass_properties  &
   part_name = .pendulum.fixed  &
   material_type = .pendulum.steel
!
! ****** Graphics for current part ******
!
geometry create shape block  &
   block_name = .pendulum.fixed.BOX_5  &
   adams_id = 5  &
   corner_marker = .pendulum.fixed.MARKER_13  &
   diag_corner_coords = 400.0, 400.0, -10.0
!
part attributes  &
   part_name = .pendulum.fixed  &
   color = MAGENTA  &
   name_visibility = off
!
!----------------------------------- ground -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.fixed
!
part create rigid_body name_and_position  &
   part_name = .pendulum.ground  &
   adams_id = 1  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.ground
!
part create rigid_body mass_properties  &
   part_name = .pendulum.ground  &
   material_type = .pendulum.steel
!
part attributes  &
   part_name = .pendulum.ground  &
   name_visibility = off
!
!---------------------------------- pedeatal ----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.fixed
!
part create rigid_body name_and_position  &
   part_name = .pendulum.pedeatal  &
   adams_id = 2  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
part create rigid_body initial_velocity  &
   part_name = .pendulum.pedeatal  &
   vx = 0.1
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.pedeatal
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .pendulum.pedeatal.MARKER_1  &
   adams_id = 1  &
   location = 0.0, 0.0, 0.0  &
   orientation = 180.0d, 90.0d, 180.0d
!
marker create  &
   marker_name = .pendulum.pedeatal.cm  &
   adams_id = 7  &
   location = 0.0, 15.0, 0.0  &
   orientation = 90.0d, 90.0d, 0.0d
!
marker create  &
   marker_name = .pendulum.pedeatal.MARKER_15  &
   adams_id = 15  &
   location = 0.0, 0.0, 0.0  &
   orientation = 90.0d, 90.0d, 270.0d
!
part create rigid_body mass_properties  &
   part_name = .pendulum.pedeatal  &
   material_type = .pendulum.steel
!
! ****** Graphics for current part ******
!
geometry create shape cylinder  &
   cylinder_name = .pendulum.pedeatal.CYLINDER_1  &
   adams_id = 1  &
   center_marker = .pendulum.pedeatal.MARKER_1  &
   angle_extent = 360.0  &
   length = 30.0  &
   radius = 50.0  &
   side_count_for_body = 20  &
   segment_count_for_ends = 20
!
part attributes  &
   part_name = .pendulum.pedeatal  &
   color = RED  &
   name_visibility = off
!
!---------------------------------- rod_down ----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.fixed
!
part create rigid_body name_and_position  &
   part_name = .pendulum.rod_down  &
   adams_id = 3  &
   location = -20.0, 10.0, -10.0  &
   orientation = 80.0d, 5.0d, 222.0d
!
part create rigid_body initial_velocity  &
   part_name = .pendulum.rod_down  &
   vx = 0.0  &
   vy = 0.0  &
   vz = 0.0
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.rod_down
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .pendulum.rod_down.MARKER_2  &
   adams_id = 2  &
   location = 0.0, 30.0, 0.0  &
   orientation = 135.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .pendulum.rod_down.MARKER_3  &
   adams_id = 3  &
   location = -70.7106781187, 90.7106781187, 0.0  &
   orientation = 135.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .pendulum.rod_down.cm  &
   adams_id = 8  &
   location = -35.3553390594, 60.3553390593, 0.0  &
   orientation = 49.3513159136d, 90.0000000109d, 359.9999950141d
!
marker create  &
   marker_name = .pendulum.rod_down.MARKER_20  &
   adams_id = 20  &
   location = -6.987070592, 26.8657764076, 11.3758912864  &
   orientation = 318.0d, 5.0d, 100.0d
!
marker create  &
   marker_name = .pendulum.rod_down.MARKER_17  &
   adams_id = 17  &
   location = -70.7106781187, 90.7106781187, 4.7264751686E-11  &
   orientation = 57.9627107873d, 0.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .pendulum.rod_down  &
   material_type = .pendulum.steel
!
! ****** Graphics for current part ******
!
geometry create shape link  &
   link_name = .pendulum.rod_down.LINK_2  &
   i_marker = .pendulum.rod_down.MARKER_2  &
   j_marker = .pendulum.rod_down.MARKER_3  &
   width = 10.0  &
   depth = 10.0
!
part attributes  &
   part_name = .pendulum.rod_down  &
   color = GREEN  &
   name_visibility = off
!
!----------------------------------- rod_up -----------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.fixed
!
part create rigid_body name_and_position  &
   part_name = .pendulum.rod_up  &
   adams_id = 4  &
   planar = yes  &
   planar_axes = xy  &
   location = -70.7106781187, 110.7106781187, 0.0  &
   orientation = 326.0d, 0.0d, 0.0d
!
part create rigid_body initial_velocity  &
   part_name = .pendulum.rod_up  &
   vx = 0.0  &
   vy = 0.0  &
   vz = 0.0
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.rod_up
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .pendulum.rod_up.MARKER_2  &
   adams_id = 4  &
   location = 70.5858301422, 56.4619625581, -11.7515246985  &
   orientation = 135.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .pendulum.rod_up.MARKER_3  &
   adams_id = 5  &
   location = -0.1248479765, 127.1726406768, -11.7515246985  &
   orientation = 135.0d, 0.0d, 0.0d
!
marker create  &
   marker_name = .pendulum.rod_up.cm  &
   adams_id = 6  &
   location = 35.2304910829, 91.8173016175, -11.7515246985  &
   orientation = 45.0d, 90.0000000125d, 359.9999959624d
!
marker create  &
   marker_name = .pendulum.rod_up.MARKER_18  &
   adams_id = 18  &
   location = 70.5858301422, 56.4619625581, -11.7515246985  &
   orientation = 114.0d, 5.0d, 279.9627107873d
!
part create rigid_body mass_properties  &
   part_name = .pendulum.rod_up  &
   material_type = .pendulum.steel
!
! ****** Graphics for current part ******
!
geometry create shape link  &
   link_name = .pendulum.rod_up.LINK_2  &
   i_marker = .pendulum.rod_up.MARKER_2  &
   j_marker = .pendulum.rod_up.MARKER_3  &
   width = 10.0  &
   depth = 10.0
!
part attributes  &
   part_name = .pendulum.rod_up  &
   color = CYAN  &
   name_visibility = off
!
!----------------------------------- Joints -----------------------------------!
!
!
constraint create joint revolute  &
   joint_name = .pendulum.JOINT_5  &
   adams_id = 5  &
   i_marker_name = .pendulum.fixed.MARKER_19  &
   j_marker_name = .pendulum.rod_down.MARKER_20
!
constraint attributes  &
   constraint_name = .pendulum.JOINT_5  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .pendulum.JOINT_4  &
   adams_id = 4  &
   i_marker_name = .pendulum.rod_down.MARKER_17  &
   j_marker_name = .pendulum.rod_up.MARKER_18
!
constraint attributes  &
   constraint_name = .pendulum.JOINT_4  &
   name_visibility = off
!
constraint create joint translational  &
   joint_name = .pendulum.JOINT_3  &
   adams_id = 3  &
   i_marker_name = .pendulum.fixed.MARKER_14  &
   j_marker_name = .pendulum.pedeatal.MARKER_15
!
constraint attributes  &
   constraint_name = .pendulum.JOINT_3  &
   visibility = on  &
   name_visibility = off
!
!----------------------------------- Forces -----------------------------------!
!
!
!----------------------------- Simulation Scripts -----------------------------!
!
!
simulation script create  &
   sim_script_name = .pendulum.Last_Sim  &
   commands =   &
              "simulation single_run transient type=auto_select initial_static=no end_time=2.0 number_of_steps=50 model_name=.pendulum"
!
!-------------------------- Adams View UDE Instances --------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.fixed
!
undo begin_block suppress = yes
!
ude create instance  &
   instance_name = .pendulum.control_1  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .pendulum.Controls_Plant_1  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .pendulum.Plant_1  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .pendulum.P  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .pendulum.Controls_Plant_2  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .pendulum.control_1.input_channels  &
   object_value = .pendulum.motivation
!
variable modify  &
   variable_name = .pendulum.control_1.output_channels  &
   object_value =   &
      .pendulum.ddisplace,  &
      .pendulum.ddowndeg,  &
      .pendulum.displace,  &
      .pendulum.downdeg,  &
      .pendulum.dupdeg,  &
      .pendulum.updeg
!
variable modify  &
   variable_name = .pendulum.control_1.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .pendulum.control_1.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .pendulum.control_1.target  &
   string_value = "EASY5_and_MATRIXX"
!
variable modify  &
   variable_name = .pendulum.control_1.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .pendulum.control_1.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .pendulum.control_1.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.control_1.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.control_1.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .pendulum.control_1.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .pendulum.control_1.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.control_1.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .pendulum.control_1.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.control_1.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .pendulum.control_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.input_channels  &
   object_value = .pendulum.motivation
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.output_channels  &
   object_value =   &
      .pendulum.ddisplace,  &
      .pendulum.ddowndeg,  &
      .pendulum.displace,  &
      .pendulum.downdeg,  &
      .pendulum.dupdeg,  &
      .pendulum.updeg
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_1.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .pendulum.Controls_Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .pendulum.Plant_1.input_channels  &
   object_value = .pendulum.motivation
!
variable modify  &
   variable_name = .pendulum.Plant_1.output_channels  &
   object_value =   &
      .pendulum.ddisplace,  &
      .pendulum.ddowndeg,  &
      .pendulum.displace,  &
      .pendulum.downdeg,  &
      .pendulum.dupdeg,  &
      .pendulum.updeg
!
variable modify  &
   variable_name = .pendulum.Plant_1.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .pendulum.Plant_1.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .pendulum.Plant_1.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .pendulum.Plant_1.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .pendulum.Plant_1.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .pendulum.Plant_1.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.Plant_1.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.Plant_1.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .pendulum.Plant_1.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .pendulum.Plant_1.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.Plant_1.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .pendulum.Plant_1.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.Plant_1.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .pendulum.Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .pendulum.P.input_channels  &
   object_value = .pendulum.motivation
!
variable modify  &
   variable_name = .pendulum.P.output_channels  &
   object_value =   &
      .pendulum.ddisplace,  &
      .pendulum.ddowndeg,  &
      .pendulum.displace,  &
      .pendulum.downdeg,  &
      .pendulum.dupdeg,  &
      .pendulum.updeg
!
variable modify  &
   variable_name = .pendulum.P.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .pendulum.P.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .pendulum.P.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .pendulum.P.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .pendulum.P.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .pendulum.P.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.P.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.P.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .pendulum.P.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .pendulum.P.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.P.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .pendulum.P.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.P.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .pendulum.P
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.input_channels  &
   object_value = .pendulum.motivation
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.output_channels  &
   object_value =   &
      .pendulum.ddisplace,  &
      .pendulum.ddowndeg,  &
      .pendulum.displace,  &
      .pendulum.downdeg,  &
      .pendulum.dupdeg,  &
      .pendulum.updeg
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .pendulum.Controls_Plant_2.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .pendulum.Controls_Plant_2
!
undo end_block
!
!---------------------------------- Motions -----------------------------------!
!
!
constraint create motion_generator  &
   motion_name = .pendulum.MOTION_1  &
   adams_id = 1  &
   type_of_freedom = translational  &
   joint_name = .pendulum.JOINT_3  &
   time_derivative = acceleration  &
   function = ""
!
constraint attributes  &
   constraint_name = .pendulum.MOTION_1  &
   name_visibility = off
!
!---------------------------------- Accgrav -----------------------------------!
!
!
force create body gravitational  &
   gravity_field_name = gravity  &
   x_component_gravity = 0.0  &
   y_component_gravity = -9806.65  &
   z_component_gravity = 0.0
!
!----------------------------- Analysis settings ------------------------------!
!
!
!---------------------------- Adams View Variables ----------------------------!
!
!
variable create  &
   variable_name = .pendulum._model  &
   string_value = ".pendulum"
!
!---------------------------- Function definitions ----------------------------!
!
!
constraint modify motion_generator  &
   motion_name = .pendulum.MOTION_1  &
   function = "VARVAL( .pendulum.motivation)"
!
data_element modify variable  &
   variable_name = .pendulum.ddisplace  &
   function = "VX(.pendulum.fixed.MARKER_13)"
!
data_element modify variable  &
   variable_name = .pendulum.displace  &
   function = "DX( .pendulum.fixed.MARKER_13)"
!
data_element modify variable  &
   variable_name = .pendulum.ddowndeg  &
   function = "WZ( .pendulum.rod_down.MARKER_2,.pendulum.pedeatal.cm,.pendulum.pedeatal.cm)"
!
data_element modify variable  &
   variable_name = .pendulum.downdeg  &
   function = "AZ( .pendulum.rod_down.MARKER_2,.pendulum.pedeatal.cm)"
!
data_element modify variable  &
   variable_name = .pendulum.dupdeg  &
   function = "WZ(.pendulum.rod_down.cm,.pendulum.rod_up.cm,.pendulum.fixed.cm)"
!
data_element modify variable  &
   variable_name = .pendulum.updeg  &
   function = "AZ(.pendulum.rod_down.cm,.pendulum.rod_up.cm)"
!
data_element modify variable  &
   variable_name = .pendulum.motivation  &
   function = "0"
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .pendulum.control_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .pendulum.Controls_Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .pendulum.Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .pendulum.P
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .pendulum.Controls_Plant_2
!
!--------------------------- Expression definitions ---------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = fixed
!
geometry modify shape cylinder  &
   cylinder_name = .pendulum.pedeatal.CYLINDER_1  &
   length = (3.0cm)  &
   radius = (5cm)
!
geometry modify shape link  &
   link_name = .pendulum.rod_down.LINK_2  &
   width = (1cm)  &
   depth = (1cm)
!
marker modify  &
   marker_name = .pendulum.rod_up.MARKER_3  &
   location =   &
      (LOC_RELATIVE_TO({10.0cm, 0.0, 0.0}, .pendulum.rod_up.MARKER_2))  &
   relative_to = .pendulum.rod_up
!
defaults coordinate_system  &
   default_coordinate_system = .pendulum.fixed
!
geometry modify shape link  &
   link_name = .pendulum.rod_up.LINK_2  &
   width = (1cm)  &
   depth = (1cm)
!
geometry modify shape block  &
   block_name = .pendulum.fixed.BOX_5  &
   diag_corner_coords =   &
      (40.0cm),  &
      (40.0cm),  &
      (-1.0cm)
!
material modify  &
   material_name = .pendulum.steel  &
   density = (7801.0(kg/meter**3))  &
   youngs_modulus = (2.07E+11(Newton/meter**2))
!
model display  &
   model_name = pendulum
