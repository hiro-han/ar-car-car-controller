# ar-car-car-controller

## Build
```
$ cd {catkin_workspace}/src
$ git clone https://github.com/hiro-han/ar-car-car-controller.git car_controller
$ cd {catkin_workspace}
$ catkin_make --pkg car_controller
$ source devel/setup.bash
```

## Run
```
<launch>
  <node name="car_controller" pkg="car_controller" type="car_controller_node">
    <param name="controller_type" value="ProCon" type="string" />
  </node>
</launch>
```
