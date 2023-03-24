## Solve spawn serivce failed

Modify `me5413_project.world` file, change `<sim_time>` to 0:

`<sim_time>0</sim_time>` 

## Solve No p gain specified for pid (If change this, cannot use twist keyboard, so please not change)

Modify `me5413_world/config/config.yaml` file, add these:

```yaml
/gazebo_ros_control:
  pid_gains:
    front_left_wheel:
      p: 100.0
      i: 0.01
      d: 10.0
    front_right_wheel:
      p: 100.0
      i: 0.01
      d: 10.0
    rear_left_wheel:
      p: 100.0
      i: 0.01
      d: 10.0
    rear_right_wheel:
      p: 100.0
      i: 0.01
      d: 10.0
```

## To accelerate computing and rendering in rivz

Don't use PointCloud2

Modify `gmapping.rivz`: comment these lines:
```rviz
        # - Alpha: 1
        #   Autocompute Intensity Bounds: true
        #   Autocompute Value Bounds:
        #     Max Value: 4.77785587310791
        #     Min Value: -0.07101771235466003
        #     Value: true
        #   Axis: Z
        #   Channel Name: intensity
        #   Class: rviz/PointCloud2
        #   Color: 255; 255; 255
        #   Color Transformer: AxisColor
        #   Decay Time: 0
        #   Enabled: true
        #   Invert Rainbow: false
        #   Max Color: 255; 255; 255
        #   Min Color: 0; 0; 0
        #   Name: PointCloud2
        #   Position Transformer: XYZ
        #   Queue Size: 10
        #   Selectable: true
        #   Size (Pixels): 3
        #   Size (m): 0.019999999552965164
        #   Style: Points
        #   Topic: /mid/points
        #   Unreliable: false
        #   Use Fixed Frame: true
        #   Use rainbow: true
        #   Value: true
```

## To use Cartographer do SLAM:
Change the original `mapping.launch` file to `my_mapping.launch` file.
