sudo chmod 666 /dev/ttyS1
ros2 run serial_protocol serial_protocol_node --ros-args \
  -p port:=/dev/ttyS1 \
  -p baud:=115200 \
  -p send_rate_hz:=10.0 \
  -p vel_timeout:=0.2