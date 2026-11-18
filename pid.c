//10 раз в секунду
void UV_current_control(void)
{
   
#define PWR_MAX 100000
//Коэффициенты ПИД
  static float Kp=2;
  static float Ki=1;
  static float Kd=0;
  static double To=0.1;  
  
  //Память ПИД
  static double P=0;
  static double I=0;
  static double D=0;
  static float lastErr=0;
  
  static int fStopIntegration=0;
 
  
  //Промежуточные значения
  float value;
  float set_value;
  float Err=0;
  float Out=0;
  
  
  //Входные параметры
  value=Pow_lum;
  set_value=ct.PowRed_target;
  
  Err=set_value-value;
  
  P=Kp*Err;
  if(!fStopIntegration)I=I+Ki*To*Err;
  D=(Kd*(Err-lastErr))/To;
  
  
  if(I>PWR_MAX)I=PWR_MAX;
  if(I<(-PWR_MAX))I=-(PWR_MAX);
  
  Out=P+I+D;
  
  if(Out>PWR_MAX)Out=PWR_MAX;
  if(Out<800)
  {
    Out=800;
    if(I<0)I=0;
  }
  
  
  if((Out==0||Out==PWR_MAX)&&Kp!=0)fStopIntegration=1;
  else fStopIntegration=0;
  
  
  lastErr=Err;
  
  IUV_cur=Out/1000;
  

}