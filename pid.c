//20 раз в секунду
void UV_current_control(void)
{
   
#define PWR_MAX 100000
//Коэффициенты ПИД
  static float Kp=40;    // uA/nW, tuned 2026-11-18 from 10->25 mA step (ImpulsPower.csv), 20 Hz, no D
  static float Ki=2400;  // uA/(nW*s), integral term
  static float Kd=0;     // uA*s/nW, derivative on measurement (unused: see PID_tuning_report.pdf)
  static double To=0.05; // 20 Hz call rate 
  
  //Память ПИД
  static double P=0;
  static double I=0;
  static double D=0;
  static float lastValue=0;  // пред. значение сигнала (для D по измерению)
  static int fFirst=1;       // первый вызов: затравка lastValue
  
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
  //Производная по сигналу (а не по ошибке). При Kd=0 член выключен
  if(fFirst){ lastValue=value; fFirst=0; }
  D=Kd*(lastValue-value)/To;
  
  
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
  
  
  lastValue=value;
  
  IUV_cur=Out/1000;
  

}