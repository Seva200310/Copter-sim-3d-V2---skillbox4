using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlSystem : MonoBehaviour
{   
    /*
    //создаем ПИД регуляторы для управления положением
    controllerPositionX = new PID();
    controllerPositionY = new PID();
    controllerPositionZ = new PID();
    //создаем ПИД регуляторы для линейных скоростей
    controllerVelX = new PID();
    controllerVelY = new PID();
    controllerVelZ = new PID();
    //создаем ПИД регуляторы для углового положения
    controllerRotationX = new PID();
    controllerRotationY = new PID();
    controllerRotationZ = new PID();
    */
    
    
    //создаем ПИД регуляторы для угловых скоростей
    

    private PID ControllerAngVelX = new PID();
    private PID ControllerAngVelY = new PID();
    private PID ControllerAngVelZ = new PID();
    

    
    private PID ControllerRotX = new PID();
    private PID ControllerRotZ = new PID();
    


    
    private PID ControllerVelX = new PID();
    private PID ControllerVelZ = new PID();
    private PID ControllerThrust = new PID();
    
    
    
    private PID ControllerPositionX = new PID();
    private PID ControllerPositionY = new PID();
    private PID ControllerPositionZ = new PID();

    private float MaxMotorSpeed = 2652;
    private float MinMotorSpeed = 0;
    






    public void ControlSystemStart(float dt)
    {   
        //Настраиваем ПИД регуляторы для управления угловой скоростью
        Vector2 angAccRange = new Vector2(-650,650);//ограничение максимального управляющего воздействия(в данном случае оборотов моторов)
        Vector3 angVelXPID_Setup = new Vector3(2,10,0);
        ControllerAngVelX.Setup(angVelXPID_Setup,angAccRange,dt);
        Vector3 angVelYPID_Setup = new Vector3(2,10,0);
        ControllerAngVelY.Setup(angVelYPID_Setup,angAccRange,dt);
        Vector3 angVelZPID_Setup = new Vector3(2,10,0);
        ControllerAngVelZ.Setup(angVelZPID_Setup,angAccRange,dt);
        

        //Настраиваем ПИД регуляторы для управления угловым положением
        Vector2 angVelRange = new Vector2(-720,720);//ограничение максимального управляющего воздействия(в данном случае угловой скорости)
        Vector3 rotXPID_Setup = new Vector3(5,20,0);
        ControllerRotX.Setup(rotXPID_Setup,angVelRange,dt);
        Vector3 rotZPID_Setup = new Vector3(5,20,0);
        ControllerRotZ.Setup(rotZPID_Setup,angVelRange,dt);

        //Настраиваем ПИД регуляторы для управления линейной скоростью
        Vector2 thrustRange = new Vector2(0,2650);//ограничение максимального управляющего воздействия(в данном случае тяги)
        Vector2 angRange = new Vector2(-50,50);//ограничение максимального управляющего воздействия(в данном случае линейной скорости)

        Vector3 thrustPID_Setup = new Vector3(1200,2690,0);
        ControllerThrust.Setup(thrustPID_Setup,thrustRange,dt);

        Vector3 velXPID_Setup = new Vector3(10,5,20);
        ControllerVelX.Setup(velXPID_Setup,angRange,dt);

        Vector3 velZPID_Setup = new Vector3(10,5,20);
        ControllerVelZ.Setup(velZPID_Setup,angRange,dt);

        //Настраиваем ПИД регуляторы управления Линейным положением
        Vector2 velRange = new Vector2(-50,50);

        Vector3 positionXPID_Setup = new Vector3(0.6f,0.1f,0.05f);
        ControllerPositionX.Setup(positionXPID_Setup,velRange,dt); 
        Vector3 positionYPID_Setup = new Vector3(0.6f,0.1f,0.05f);//(0.5f,0.005f,3f);
        ControllerPositionY.Setup(positionYPID_Setup,velRange,dt); 
        Vector3 positionZPID_Setup = new Vector3(0.6f,0.1f,0.05f);
        ControllerPositionZ.Setup(positionYPID_Setup,velRange,dt);
        

    }
    public Vector4 GetMotorSpeed_cmd(short mode,Vector4 cmd,Vector3 position,Vector3 rotation,Vector3 velocity,Vector3 angularVelocity)
    {   
        Vector3 position_cmd = new Vector3(cmd[0],cmd[1],cmd[2]);
        Vector3 vel_cmd = PositionController(position_cmd,position);
        Vector3 velocityController_cmd = VelocityController(vel_cmd,velocity,rotation[1]);
        Vector2 rotation_cmd = new Vector2(velocityController_cmd.z,-velocityController_cmd.x);
        float thrust = velocityController_cmd.y;
        Vector2 rotationController_cmd = RotationController(rotation_cmd,rotation);
        Vector3 angularVelocity_cmd = new Vector3(rotationController_cmd[0],cmd[3],rotationController_cmd[1]);
        Vector3 angularAccelerations_cmd = AngularVelocityController(angularVelocity_cmd,angularVelocity);
        Vector4 motorsSpeed = Mixer(thrust,angularAccelerations_cmd);
        //Debug.Log("Текущая линейная скорость: "+ velocity +" Целевая линейная скорость: "+vel_cmd);
        return  motorsSpeed;//Возвращаем значения оборотов
    }
    Vector3 PositionController (Vector3 position_cmd,Vector3 position)
    {   
       //Debug.Log("ошибка по высоте: "+(position_cmd.y-position.y)+" Управляющее воздействие по высоте: "+ControllerPositionY.GetU(position_cmd.y,position.y));
       return new Vector3(ControllerPositionX.GetU(position_cmd.x,position.x),ControllerPositionY.GetU(position_cmd.y,position.y),ControllerPositionZ.GetU(position_cmd.z,position.z));
    }

    Vector3 VelocityController(Vector3 velocity_cmd,Vector3 velocity,float yaw)
    {
        float thrust_cmd = ControllerThrust.GetU(velocity_cmd.y,velocity.y);
        //Debug.Log("Ошибка по вертикальной скорости: " + " velocity_cmd: "+velocity_cmd.y+" velocity: "+velocity.y+"Команда по тяге: "+thrust_cmd);
        float rotX_cmd = ControllerVelX.GetU(velocity_cmd.x,velocity.x);
        float rotZ_cmd = ControllerVelZ.GetU(velocity_cmd.z,velocity.z);
        //Debug.Log("Ошибка по X"+(velocity_cmd.x-velocity.x)+" Ошибка по Z "+ (velocity_cmd.z-velocity.z));
        float rotX_cmd_res = rotX_cmd*Mathf.Cos(yaw*Mathf.Deg2Rad)-rotZ_cmd*Mathf.Sin(yaw*Mathf.Deg2Rad);
        float rotZ_cmd_res = rotX_cmd*Mathf.Sin(yaw*Mathf.Deg2Rad)+rotZ_cmd*Mathf.Cos(yaw*Mathf.Deg2Rad);/// Если управляем линейным положением то скорости в связанной системе координат надо перевести в скорости в стартовой тк из-за угла рысканья оси x и z будут отличаться
        return new Vector3(rotX_cmd_res,thrust_cmd,rotZ_cmd_res);
    }

    Vector2 RotationController (Vector2 rotation_cmd,Vector3 rotation)
    {   
        return new Vector2(ControllerRotX.GetU(CalculateAngleError(rotation_cmd[0],rotation.x),0),ControllerRotZ.GetU(CalculateAngleError(rotation_cmd[1],rotation.z),0));
    }

    Vector3 AngularVelocityController(Vector3 angularVelocity_cmd,Vector3 angularVelocity)
    {
        return new Vector3(ControllerAngVelX.GetU(angularVelocity_cmd[0],angularVelocity.x),ControllerAngVelY.GetU(angularVelocity_cmd[1],angularVelocity.y),ControllerAngVelZ.GetU(angularVelocity_cmd[2],angularVelocity.z));
    }

    Vector4 Mixer(float thrust,Vector3 angularAccelerations_cmd)
    {
       //Вычисляем значения команд для каждого мотора
       float w1 = thrust+angularAccelerations_cmd.x-angularAccelerations_cmd.y;//Требуемые обороты для первого мотора
       Vector2 overLoad_w1 = CalcMotorOverload(w1,angularAccelerations_cmd.x,-angularAccelerations_cmd.y);//Перегрузка первого мотора

       float w2 = thrust-angularAccelerations_cmd.z+angularAccelerations_cmd.y;//Требуемые обороты для второго мотора
       Vector2 overLoad_w2 = CalcMotorOverload(w2,-angularAccelerations_cmd.z,angularAccelerations_cmd.y);//Перегрузка второго мотора

       float w3 = thrust-angularAccelerations_cmd.x-angularAccelerations_cmd.y;//Требуемые обороты для третьего мотора
       Vector2 overLoad_w3 = CalcMotorOverload(w3,-angularAccelerations_cmd.x,-angularAccelerations_cmd.y);//Перегрузка третьего мотора

       float w4 = thrust+angularAccelerations_cmd.z+angularAccelerations_cmd.y;//Требуемые обороты для четвертого мотора
       Vector2 overLoad_w4 = CalcMotorOverload(w4,angularAccelerations_cmd.z,angularAccelerations_cmd.y);//Перегрузка четвертого мотора
       
       //перераспределяем тягу вышедшую за допустимые пределы
       w1-=(overLoad_w3[0]+overLoad_w2[1]);
       w2-=(overLoad_w4[0]+overLoad_w1[1]);
       w3-=(overLoad_w1[0]+overLoad_w4[1]);
       w4-=(overLoad_w2[0]+overLoad_w3[1]);

       
       return new Vector4(w1,w2,w3,w4);//Возвращаем вектор из 4х целевых значений угловых скоростей моторов
    }
    Vector2 CalcMotorOverload(float w,float axisCommand,float yawCommand)
    {   
        float totalOverload;
        if (w>MaxMotorSpeed)
       {
         totalOverload = w-MaxMotorSpeed;//Находим общую перегрузку двигателя
       }
       else if (w<MinMotorSpeed)
       {
         totalOverload = w-MinMotorSpeed;//Находим общую перегрузку двигателя
       }
       else 
       {
        totalOverload = 0;
       }

           float overloadX = totalOverload*axisCommand/(axisCommand+yawCommand);//находим вклад команды по X в итоговую перегрузку
           float overloadY = totalOverload*yawCommand/(axisCommand+yawCommand);//находим вклад команды по Y в итоговую перегрузку
        return new Vector2 (overloadX,overloadY);
    }

    private float CalculateAngleError(float targetAngle, float currentAngle)
    {
        // Нормализуем углы к диапазону [-180, 180]
        targetAngle = NormalizeAngle(targetAngle);
        currentAngle = NormalizeAngle(currentAngle);

        // Вычисляем ошибку с учетом цикличности
        float error = targetAngle - currentAngle;

        // Если ошибка больше 180 градусов, то уменьшаем ее на 360 градусов, чтобы получить минимальное значение ошибки
        if (Mathf.Abs(error) > 180) 
        {
            error -= Mathf.Sign(error) * 360;
        }

        return error;
    }

    // Функция для нормализации угла к диапазону [-180, 180]
    private float NormalizeAngle(float angle)
    {
        angle %= 360;
        if (angle > 180) 
        {
           angle -= 360;
        }
        return angle;
    }
}
