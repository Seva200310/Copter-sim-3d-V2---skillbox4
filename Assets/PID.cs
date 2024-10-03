using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PID 
{
    public float Kp,Ki,Kd;
    public float Dt;
    private float Max_U;
    public float Min_U; 
    private float Error;
    private float ErrorPast = 0;
    private float ErrorIntegral;
    private float U;

    public void Setup(Vector3 PID_Setup,Vector2 rangeU,float dt)
    {
        this.Kp = PID_Setup[0];
        this.Ki = PID_Setup[1];
        this.Kd = PID_Setup[2];
        this.Min_U = rangeU[0];
        this.Max_U = rangeU[1]; 
        this.Dt = dt;

    }
    public float GetU(float desiredValue,float value)
    {   
        Error =  desiredValue - value;//Находим ошибку
        ErrorIntegral += Error*Dt;//Находим интеграл ошибки
        U = Kp*Error + Ki*ErrorIntegral+Kd*(Error-ErrorPast)/Dt;//Вычисляем управляющее воздействие
        ErrorPast = Error;//Запомним текущее значение ошибки для вычисления дифференциала ошибки в  следующей итерации
        Saturation();
        return Saturation();//Возвращаем результат
    }
    private float Saturation()
    {   
        if (U > Max_U)
        {
            U = Max_U;
            return U;
        }
        else if(U<Min_U)
        {
            U = Min_U;
            return U;
        }
        else
        {   
            return U;
        }
    }

}
