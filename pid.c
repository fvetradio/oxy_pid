//20 раз в секунду
void UV_current_control(void)
{
   
#define PWR_MAX 100000
#define BLIND_NW   0.0f   // н¬т; "ниже порога" детектора (p-p прижимаетс€ к 0)
#define BLIND_RAMP 200    // мкј за вызов 20 √ц => ~+4 мј/с м€гкий подъЄм тока в слепой зоне
//--- «онирование по уровню свечени€ (gain scheduling) ---
// Ќа малом свечении планта усиливаетс€ в ~2.4 раза (8.7 н¬т/мј вместо 3.6): номинальные
// 40/2400 там недодемпфированы и дают затухающие волны ~4.6-5 √ц (см. "low curent generation.png").
// Ќиже LUM_LO действуют м€гкие коэффициенты, выше LUM_HI - номинальные, между ними -
// плавна€ линейна€ смесь по измерению. »нтегратор общий, переключений и бросков нет.
#define LUM_LO 60.0f   // н¬т; ниже - полна€ мала€ зона (примерно до ~8 мј тока)
#define LUM_HI 90.0f   // н¬т; выше - полна€ номинальна€ зона (от ~11 мј)
#define KP_LOW 25.0f   // мкј/н¬т; 0 перерегулировани€ во всех 18 угловых точках модели
#define KI_LOW 700.0f  // мкј/(н¬т*с); ts(5%) ~ 0.44 с
#define KD_LOW 0.0f    // мкј*с/н¬т; на стенде можно попробовать 2.0: быстрее (ts5 ~0.39 с),
                       // но ухудшает угловые точки до +0.3..+1.7 н¬т. ѕо умолчанию 0.
// оэффициенты ѕ»ƒ
  static float Kp=40;    // uA/nW, tuned 2026-11-18 from 10->25 mA step (ImpulsPower.csv), 20 Hz, no D
  static float Ki=2400;  // uA/(nW*s), integral term
  static float Kd=0;     // uA*s/nW, derivative on measurement (unused: see PID_tuning_report.pdf)
  static double To=0.05; // 20 Hz call rate 
  
  //ѕам€ть ѕ»ƒ
  static double P=0;
  static double I=0;
  static double D=0;
  static float lastValue=0;  // пред. значение сигнала (дл€ D по измерению)
  static int fFirst=1;       // первый вызов: затравка lastValue
  
  static int fStopIntegration=0;
 
  
  //ѕромежуточные значени€
  float value;
  float set_value;
  float Err=0;
  float Out=0;
  float wZone=0.0f;          // вес зоны: 0 - мала€, 1 - номинальна€
  float KpE=0.0f, KiE=0.0f, KdE=0.0f; // эффективные коэффициенты
  
  
  //¬ходные параметры
  value=Pow_lum;
  set_value=ct.PowRed_target;
  //=== —лепа€ зона детектора: измерение ниже порога (value==0) ===
  // ќшибку (уставка - 0) вслепую Ќ≈ интегрируем: ток м€гко поднимаем на BLIND_RAMP
  // за вызов, пока измерение не вернЄтс€. »наче интегратор накручивает
  // ток вслепую и раскачивает контур на малом свечении
  // (см. low curent generation.png). ¬не слепой зоны работает ѕ»ƒ с зонированием по свечению (см. ниже).
  if(value<=BLIND_NW)
  {
    I=I+BLIND_RAMP;            // м€гкий подъЄм тока вслепую, без накрутки ошибки
    if(I>PWR_MAX)I=PWR_MAX;
    Out=(float)I;
    if(Out>PWR_MAX)Out=PWR_MAX;
    if(Out<800)Out=800;        // нижний предел тока, как в основном пути
    fStopIntegration=0;         // интегратор разблокирован дл€ обычной работы после выхода из зоны
    fFirst=1;                   // до D: после возврата измерений lastValue инициализуетс€ заново, без броска
    IUV_cur=Out/1000;
    return;
  }
  
  //=== ¬ес зоны по измерению: 0 - мала€ зона, 1 - номинальна€ ===
  wZone=(value-LUM_LO)/(LUM_HI-LUM_LO);
  if(wZone<0.0f)wZone=0.0f;
  if(wZone>1.0f)wZone=1.0f;
  KpE=KP_LOW+wZone*(Kp-KP_LOW);
  KiE=KI_LOW+wZone*(Ki-KI_LOW);
  KdE=KD_LOW+wZone*(Kd-KD_LOW);
  Err=set_value-value;
  
  P=KpE*Err;
  if(!fStopIntegration)I=I+KiE*To*Err;
  //ѕроизводна€ по сигналу (а не по ошибке). ѕри Kd=0 член выключен
  if(fFirst){ lastValue=value; fFirst=0; }
  D=KdE*(lastValue-value)/To;
  
  
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