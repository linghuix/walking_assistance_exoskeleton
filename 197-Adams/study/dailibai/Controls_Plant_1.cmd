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
   model_name = MODEL_1
!
view erase
!
!-------------------------------- Data storage --------------------------------!
!
!
data_element create variable  &
   variable_name = .MODEL_1.theta_up  &
   adams_id = 1  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.theta_down  &
   adams_id = 2  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.x  &
   adams_id = 3  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.speedtheta_down  &
   adams_id = 4  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.speedtheta_up  &
   adams_id = 5  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.speed  &
   adams_id = 6  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .MODEL_1.a  &
   adams_id = 7  &
   initial_condition = 0.0  &
   function = ""
!
data_element create plant input  &
   plant_input_name = .MODEL_1.PINPUT_F  &
   adams_id = 1  &
   variable_name = .MODEL_1.a
!
data_element create plant output  &
   plant_output_name = .MODEL_1.POUTPUT_1  &
   adams_id = 1  &
   variable_name =  &
      .MODEL_1.theta_up,  &
      .MODEL_1.theta_down,  &
      .MODEL_1.x,  &
      .MODEL_1.speedtheta_down,  &
      .MODEL_1.speedtheta_up,  &
      .MODEL_1.speed
!
!--------------------------------- Materials ----------------------------------!
!
!
material create  &
   material_name = .MODEL_1.steel  &
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
   default_coordinate_system = .MODEL_1.ground
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.ground.MARKER_1  &
   adams_id = 1  &
   location = 49.7460435831, 7.499999508, 34.2951583555  &
   orientation = 0.0d, 180.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.ground  &
   material_type = .MODEL_1.steel
!
part attributes  &
   part_name = .MODEL_1.ground  &
   name_visibility = off
!
!-------------------------- dizuo_1_daolibai_1_body1 --------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.dizuo_1_daolibai_1_body1  &
   adams_id = 4  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.dizuo_1_daolibai_1_body1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.MARKER_2  &
   adams_id = 2  &
   location = 49.7460435831, 7.499999508, 34.2951583555  &
   orientation = 0.0d, 180.0d, 0.0d
!
marker create  &
   marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.MARKER_13  &
   adams_id = 13  &
   location = 49.7460435831, 7.499999508, 34.2951583555  &
   orientation = 270.0d, 90.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.PSMAR  &
   adams_id = 7  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker attributes  &
   marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.PSMAR  &
   visibility = off
!
marker create  &
   marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.cm  &
   adams_id = 8  &
   location = 49.7460782552, 7.5, 34.29515846  &
   orientation = 90.0d, 180.0d, 0.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.dizuo_1_daolibai_1_body1  &
   material_type = .MODEL_1.steel
!
! ****** Graphics for current part ******
!
part attributes  &
   part_name = .MODEL_1.dizuo_1_daolibai_1_body1  &
   color = WHITE
!
!------------------------ link_xia_1_daolibai_2_body1 -------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.link_xia_1_daolibai_2_body1  &
   adams_id = 5  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.link_xia_1_daolibai_2_body1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.link_xia_1_daolibai_2_body1.MARKER_14  &
   adams_id = 14  &
   location = 49.7460435831, 7.499999508, 34.2951583555  &
   orientation = 270.0d, 90.0d, 90.0d
!
marker create  &
   marker_name = .MODEL_1.link_xia_1_daolibai_2_body1.PSMAR  &
   adams_id = 9  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker attributes  &
   marker_name = .MODEL_1.link_xia_1_daolibai_2_body1.PSMAR  &
   visibility = off
!
marker create  &
   marker_name = .MODEL_1.link_xia_1_daolibai_2_body1.cm  &
   adams_id = 10  &
   location = 58.9894931409, 26.9786779483, 29.7596359112  &
   orientation = 0.0d, 76.892119548d, 359.9999851569d
!
marker create  &
   marker_name = .MODEL_1.link_xia_1_daolibai_2_body1.MARKER_15  &
   adams_id = 15  &
   location = 59.9894996972, 46.4578519325, 25.2240078821  &
   orientation = 270.0d, 90.0d, 90.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.link_xia_1_daolibai_2_body1  &
   mass = 6.8533970399E-03  &
   center_of_mass_marker = .MODEL_1.link_xia_1_daolibai_2_body1.cm  &
   ixx = 1.2465752205  &
   iyy = 1.1901502194  &
   izz = 6.0993945848E-02  &
   ixy = 0.0  &
   izx = 0.0  &
   iyz = 0.0
!
! ****** Graphics for current part ******
!
part attributes  &
   part_name = .MODEL_1.link_xia_1_daolibai_2_body1  &
   color = WHITE
!
!------------------------ link_xia_2_daolibai_2_body1 -------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
part create rigid_body name_and_position  &
   part_name = .MODEL_1.link_xia_2_daolibai_2_body1  &
   adams_id = 6  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d  &
   exact_coordinates = X, Y, Z, THETA
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.link_xia_2_daolibai_2_body1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .MODEL_1.link_xia_2_daolibai_2_body1.PSMAR  &
   adams_id = 11  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0d, 0.0d, 0.0d
!
marker attributes  &
   marker_name = .MODEL_1.link_xia_2_daolibai_2_body1.PSMAR  &
   visibility = off
!
marker create  &
   marker_name = .MODEL_1.link_xia_2_daolibai_2_body1.cm  &
   adams_id = 12  &
   location = 60.9895000125, 65.8490762238, 30.1215538645  &
   orientation = 359.9999995848d, 104.1742153228d, 359.9999996806d
!
marker create  &
   marker_name = .MODEL_1.link_xia_2_daolibai_2_body1.MARKER_16  &
   adams_id = 16  &
   location = 59.9894996972, 46.4578519325, 25.2240078821  &
   orientation = 270.0d, 90.0d, 90.0d
!
part create rigid_body mass_properties  &
   part_name = .MODEL_1.link_xia_2_daolibai_2_body1  &
   mass = 6.8530411483E-03  &
   center_of_mass_marker = .MODEL_1.link_xia_2_daolibai_2_body1.cm  &
   ixx = 1.2464004405  &
   iyy = 1.1899744148  &
   izz = 6.0994767725E-02  &
   ixy = 0.0  &
   izx = 0.0  &
   iyz = 0.0
!
! ****** Graphics for current part ******
!
part attributes  &
   part_name = .MODEL_1.link_xia_2_daolibai_2_body1  &
   color = WHITE
!
! ****** Graphics from Parasolid file ******
!
file parasolid read  &
   file_name = "Controls_Plant_1.xmt_txt"  &
   model_name = .MODEL_1
!
geometry attributes  &
   geometry_name = .MODEL_1.dizuo_1_daolibai_1_body1.SOLID4  &
   color = RED
!
geometry attributes  &
   geometry_name = .MODEL_1.link_xia_1_daolibai_2_body1.SOLID5  &
   color = GREEN
!
geometry attributes  &
   geometry_name = .MODEL_1.link_xia_2_daolibai_2_body1.SOLID6  &
   color = YELLOW
!
!----------------------------------- Joints -----------------------------------!
!
!
constraint create joint translational  &
   joint_name = .MODEL_1.move  &
   adams_id = 1  &
   i_marker_name = .MODEL_1.ground.MARKER_1  &
   j_marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.MARKER_2
!
constraint attributes  &
   constraint_name = .MODEL_1.move  &
   visibility = off  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.rotate1  &
   adams_id = 2  &
   i_marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.MARKER_13  &
   j_marker_name = .MODEL_1.link_xia_1_daolibai_2_body1.MARKER_14
!
constraint attributes  &
   constraint_name = .MODEL_1.rotate1  &
   visibility = off  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .MODEL_1.rotate2  &
   adams_id = 3  &
   i_marker_name = .MODEL_1.link_xia_1_daolibai_2_body1.MARKER_15  &
   j_marker_name = .MODEL_1.link_xia_2_daolibai_2_body1.MARKER_16
!
constraint attributes  &
   constraint_name = .MODEL_1.rotate2  &
   visibility = off  &
   name_visibility = off
!
!----------------------------------- Forces -----------------------------------!
!
!
!----------------------------- Simulation Scripts -----------------------------!
!
!
simulation script create  &
   sim_script_name = .MODEL_1.Last_Sim  &
   commands =   &
              "simulation single_run transient type=auto_select initial_static=no end_time=500.0 number_of_steps=800 model_name=.MODEL_1"
!
!-------------------------- Adams View UDE Instances --------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
undo begin_block suppress = yes
!
ude create instance  &
   instance_name = .MODEL_1.Controls_Plant_1  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.input_channels  &
   object_value = .MODEL_1.a
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.output_channels  &
   object_value =   &
      .MODEL_1.speed,  &
      .MODEL_1.speedtheta_down,  &
      .MODEL_1.speedtheta_up,  &
      .MODEL_1.theta_down,  &
      .MODEL_1.theta_up,  &
      .MODEL_1.x
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.host  &
   string_value = "Power-LJR"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .MODEL_1.Controls_Plant_1.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .MODEL_1.Controls_Plant_1
!
undo end_block
!
!------------------------------ Dynamic Graphics ------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .MODEL_1.ground
!
geometry create shape force  &
   force_name = .MODEL_1.JOINT_2_force_graphic_1  &
   adams_id = 8  &
   joint_name = .MODEL_1.rotate1  &
   applied_at_marker_name = .MODEL_1.dizuo_1_daolibai_1_body1.MARKER_13
!
!---------------------------------- Motions -----------------------------------!
!
!
constraint create motion_generator  &
   motion_name = .MODEL_1.MOTION_1  &
   adams_id = 1  &
   type_of_freedom = translational  &
   joint_name = .MODEL_1.move  &
   time_derivative = acceleration  &
   function = ""
!
constraint attributes  &
   constraint_name = .MODEL_1.MOTION_1  &
   visibility = off  &
   name_visibility = off
!
!---------------------------------- Accgrav -----------------------------------!
!
!
force create body gravitational  &
   gravity_field_name = gravity  &
   x_component_gravity = 0.0  &
   y_component_gravity = -9.8  &
   z_component_gravity = 0.0
!
!----------------------------- Analysis settings ------------------------------!
!
!
!---------------------------- Adams View Variables ----------------------------!
!
!
variable create  &
   variable_name = .MODEL_1._model  &
   string_value = ".MODEL_1"
!
!---------------------------- Function definitions ----------------------------!
!
!
constraint modify motion_generator  &
   motion_name = .MODEL_1.MOTION_1  &
   function = "VARVAL( .MODEL_1.a)"
!
data_element modify variable  &
   variable_name = .MODEL_1.theta_up  &
   function = "AX( .MODEL_1.link_xia_2_daolibai_2_body1.cm, .MODEL_1.link_xia_1_daolibai_2_body1.cm)"
!
data_element modify variable  &
   variable_name = .MODEL_1.theta_down  &
   function = "AX( .MODEL_1.link_xia_1_daolibai_2_body1.cm, .MODEL_1.dizuo_1_daolibai_1_body1.MARKER_2)"
!
data_element modify variable  &
   variable_name = .MODEL_1.x  &
   function = "DZ( .MODEL_1.dizuo_1_daolibai_1_body1.cm)"
!
data_element modify variable  &
   variable_name = .MODEL_1.speedtheta_down  &
   function = "WX( .MODEL_1.link_xia_1_daolibai_2_body1.cm, .MODEL_1.dizuo_1_daolibai_1_body1.MARKER_2)"
!
data_element modify variable  &
   variable_name = .MODEL_1.speedtheta_up  &
   function = "WX( .MODEL_1.link_xia_2_daolibai_2_body1.cm, .MODEL_1.link_xia_1_daolibai_2_body1.cm)"
!
data_element modify variable  &
   variable_name = .MODEL_1.speed  &
   function = "VZ( .MODEL_1.dizuo_1_daolibai_1_body1.cm)"
!
data_element modify variable  &
   variable_name = .MODEL_1.a  &
   function = "0"
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .MODEL_1.Controls_Plant_1
!
!--------------------------- Expression definitions ---------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = ground
!
material modify  &
   material_name = .MODEL_1.steel  &
   density = (7801.0(kg/meter**3))  &
   youngs_modulus = (2.07E+11(Newton/meter**2))
!
geometry modify shape force  &
   force_name = .MODEL_1.JOINT_2_force_graphic_1  &
   applied_at_marker_name = (.MODEL_1.rotate1.i)
!
model display  &
   model_name = MODEL_1
