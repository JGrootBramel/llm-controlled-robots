```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}'
```    

## Turn right

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: -1.0}}'
```    

## Turn left

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 1.0}}'
```    

