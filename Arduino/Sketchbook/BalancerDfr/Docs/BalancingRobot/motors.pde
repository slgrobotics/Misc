int Drive_Motor(int torque)  {
  
  
  torque2 = map(torque, -255, 255, 0, 180);     // scale it to use it with the servo (value between 0 and 180) 
  servo1.write(torque1);
  torque1 = map(torque, -255, 255, 180, 0 );
  servo2.write(torque2);

 
  }
 
