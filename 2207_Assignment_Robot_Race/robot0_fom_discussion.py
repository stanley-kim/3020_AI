<?xml version="1.0"?>
<robot name="myfirst">

  <link name="body">
    <visual>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
        <inertia ixx="0.0003" iyy="0.0003" izz="0.0003" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="left_leg">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.25"/>
      <inertia ixx="0.0003" iyy="0.0003" izz="0.0003" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <cylinder length="0.7" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.7" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.25"/>
      <inertia ixx="0.0003" iyy="0.0003" izz="0.0003" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>



  <joint name="body_to_left" type="revolute">
    <parent link="body"/>
    <child link="left_leg"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" upper="0" lower="10" velocity="1"/>
    <origin rpy="0 0 0" xyz="0 0.5 -0.3"/>
  </joint>

  <joint name="body_to_right" type="revolute">
    <parent link="body"/>
    <child link="right_leg"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" upper="0" lower="10" velocity="1"/>
    <origin rpy="0 0 0" xyz="0 -0.5 -0.3"/>
  </joint>
</robot>