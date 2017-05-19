INPUT ROS (OUTPUT controller)

10 byte message

T <32 bit signed> <32 bit signed> <CRC-8>

  left wheel        right wheel

ticks from previous message

OUTPUT ROS (INPUT controller)

6 byte message

[PID] <32 bit unsigned> <CRC-8>

		float * 100000

V <16 bit signed> <16 bit signed> <CRC-8>

   left wheel       right wheel
