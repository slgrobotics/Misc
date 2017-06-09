// **************************
// Remote control code
// **************************
void control()
{
  if(Serial.available())
  {
    int val;
    val=Serial.read();
    switch (val)
    {
    case 'a':   //forward
         if(error_a>-8)
         
          { error_a-=0.1;  }
           break;
    case 'b':   //back
          if(error_a<8)
          { error_a+=0.1;  }
           break;
    case 's':   //stop
      error_a=0;
      turn_flag=0;
      break;
    case 'r':  //right
      turn_flag=-1;
      break;
    case 'l':  //left
      turn_flag=1;
      break;
      
    default:
      break;
      
    }
  }
}











































































