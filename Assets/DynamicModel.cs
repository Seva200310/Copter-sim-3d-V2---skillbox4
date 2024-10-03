using UnityEngine;
using System;

public class DynamicModel : MonoBehaviour
{   
    public Rigidbody Drone;//3D модель дрона

    //ограничения
    private float MaxAngularVelocity = 150f;
    private float minAcceleration = 0.001f;
    private float minAngularAcceleration = 0.001f;

    //массо-габаритные характеристики ЛА
    public float M;//масса бпла
    public float L;//длина лучей бпла
    public float Ix;//момент инерции вокруг оси x
    public float Iy;//момент инерции вокруг оси y
    public float Iz;//момент инерции вокруг оси z




    //конфигурация ВМГ
    private int Motor_count = 4;//количество моторов БПЛА
    public float ThrustCoef;//коэффициент тяги ВМГ
    public float DrugCoef;//коэффициент момента пропеллера

    private Vector3 ThrustForce;//вектор проекций сил зависящих от ориентации БПЛА,[0,P,0] в данном случае учитывается только сила тяги направленная вверх
    private Vector3 ExternalAccelerations = new Vector3(0,9.81f,0);// Вектор проекций ускорений, не зависящих от положения беспилотника (ускорение свободного падения)

    //управляющее воздействие
    public Vector4 MotorSpeedArray;//угловые скорости моторов

    //Состояние БПЛА 
    private Vector3 Acceleration;// Вектор проекций ускорений беспилотника
    private Vector3 AngularAcceleration;//Вектор проекций угловых ускорений беспилотника
    private Vector3 Velocity;//скорость беспилотника по осям
    private Vector3 AngularVelocity;//Угловые скорости беспилотника
    private Vector3 Position;//Положение БПЛА в глобальной системе координат
    private Vector3 Rotation;//Угловое положение БПЛА в глобальной системе координат

    
    private float[,] RotationMatrix;// Матрица поворота (вычисляется динамически)

    void Start()
    {
        Rotation = Drone.transform.eulerAngles;
        Position = Drone.transform.position;      
    }
    private void CalculateAccelerations()//функция для вычисления проекций ускорения беспилотнка
    {   
        CalculateTrust();//вычисляем суммарную тягу
        CalculateRotationMatrix();// Вычисляем матрицу поворота из углов Эйлера
        
        //externalAccelerations = new Vector3(0, -10, 0);
        // Вычисляем поворот вектора тяги 
        Vector3 rotatedThrust = Drone.transform.TransformDirection(ThrustForce);//LocalToGlobal(ThrustForce);
        //Debug.Log("Угол поворота "+Rotation+" Тяга " + ThrustForce+" Вектор тяги "+rotatedThrust);

        // Вычисляем ускорения
        Acceleration = rotatedThrust/M - ExternalAccelerations;
        //Debug.Log(externalAccelerations);
    }
    public void CalculateAngularAcceleration()//Метод для вычисления угловых ускорений
    {   
        Vector3 Wb = AngularVelocity; // Вектор угловой скорости дрона

        // Вычисляемое угловое ускорение
        float w0 = MotorSpeedArray[0];
        float w1 = MotorSpeedArray[1];
        float w2 = MotorSpeedArray[2];       
        float w3 = MotorSpeedArray[3];

        // Создаем матрицу 3x3 для момента инерции
        Matrix4x4 inertiaMatrix = new Matrix4x4(
            new Vector4(Ix, 0, 0, 0),
            new Vector4(0, Iy, 0, 0),
            new Vector4(0, 0, Iz, 0),
            new Vector4(0, 0, 0, 1)
        );
        //создаем оббратную матрицу для момента инерции
        Matrix4x4 inertiaMatrixReversed = new Matrix4x4(
            new Vector4(1/Ix, 0, 0, 0),
            new Vector4(0, 1/Iy, 0, 0),
            new Vector4(0, 0, 1/Iz, 0),
            new Vector4(0, 0, 0, 1)
        );
        // Вычисляем вектор момента сил
        Vector3 momentOfForces = new Vector3(
            L * ThrustCoef * (w0 * w0 - w2 * w2),
            DrugCoef * (w3 * w3 + w1 * w1 - w0 * w0 - w2 * w2),
            L * ThrustCoef * (w3 * w3 - w1 * w1)
        );

        // Вычисляем векторное произведение Wb и момента инерции
        Vector3 crossProduct = Vector3.Cross(Wb, inertiaMatrix.MultiplyVector(Wb));
        // Вычисляем вектор углового ускорения
        AngularAcceleration = inertiaMatrixReversed.MultiplyVector(momentOfForces - crossProduct);

    }
    
    private void CalculateTrust()//функция для вычисления суммарной тяги моторов
    {      
        float total_power = 0f;
        for (int i =0;i<Motor_count;i++)
        {
            total_power += ThrustCoef*MotorSpeedArray[i]*MotorSpeedArray[i];
        }
        ThrustForce = new Vector3(0f,  total_power, 0f);
    }

    public void SimulateIteration(float dt)//Метод для расчета симуляции отдельной итерации
    {   
        CalculateAccelerations();
        CalculateAngularAcceleration();
        FilterAccVector();
        Velocity += Integrate(Acceleration, dt);
        //Debug.Log("Ускорения "+Acceleration+" Скорости "+Velocity+" обороты "+MotorSpeedArray);
        AngularVelocity += Integrate(AngularAcceleration, dt);
        LimitAngularVelocity();
        Position += Integrate(Velocity, dt); 
        Rotation += Integrate(AngularVelocity, dt)*Mathf.Rad2Deg;
        Rotation = FilterAngles(Rotation);
    }

    public void Update_position(float dt)//обновление положения беспилотника
    {   
        Drone.transform.eulerAngles = Rotation;
        CheckGround();
        Drone.transform.position = Position;
    }

    //Методы для установки ограничений
    //---------------------------------------------------------------------
    private void CheckGround()//метод проверки минимальной высоты БПЛА
    {
        if (Position.y<1)
        {
            Position.y = 1f;
            
        }
    }

        
    private void LimitAngularVelocity()// Метод для ограничения угловой скорости
    {   
        // Ограничиваем каждую компоненту скорости
        for (int i = 0; i < 3; i++)
        {
            if (Mathf.Abs(AngularVelocity[i]) > MaxAngularVelocity)
            {
                AngularVelocity[i] = Mathf.Sign(AngularVelocity[i]) * MaxAngularVelocity;
            }
        }

        // Устанавливаем ограниченную угловую скорость 
    }

    //Вспомогательные методы
    //---------------------------------------------------------------------
        private void CalculateRotationMatrix()// Функция для вычисления матрицы поворота из углов Эйлера

    {   
        
        // Вычисление синусов и косинусов углов Эйлера
        float cr = Mathf.Cos(Rotation[0] * Mathf.Deg2Rad);
        float sr = Mathf.Sin(Rotation[0] * Mathf.Deg2Rad);
        float cp = Mathf.Cos(Rotation[1] * Mathf.Deg2Rad);
        float sp = Mathf.Sin(Rotation[1] * Mathf.Deg2Rad);
        float cy = Mathf.Cos(Rotation[2] * Mathf.Deg2Rad);
        float sy = Mathf.Sin(Rotation[2] * Mathf.Deg2Rad);
        
        

        // Вычисление матрицы поворота
        // Вычисление матрицы поворота (с изменением порядка)
        RotationMatrix = new float[,] {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr}
    };
        /*RotationMatrix = new float[,] {
            {cp*cy,-sy*cp,sp},
            {sr*sp*cy+sy*cr,-sr*sp*sy+cr*cy,-sy*cp},
            {sr*sy-sp*cr*cy,sr*cy+sp*sy*cr,cr*cp}
        };*/
    }

    private Vector3 Integrate(Vector3 values, float dt)//Метод для интегрирования
    {
        return values * dt; 
    }
    private Vector3 LocalToGlobal(Vector3 values)//метод для перепроектирования системы координат с связанной в глобальную
    {
       return new Vector3(
            RotationMatrix[0, 0] * values.x + RotationMatrix[0, 1] * values.y + RotationMatrix[0, 2] * values.z,
            RotationMatrix[1, 0] * values.x + RotationMatrix[1, 1] * values.y + RotationMatrix[1, 2] * values.z,
            RotationMatrix[2, 0] * values.x + RotationMatrix[2, 1] * values.y + RotationMatrix[2, 2] * values.z
        );
    }

    public static Vector3 FilterAngles(Vector3 angles)//метод для приведения углов к диапазону 0-360
    {
        return new Vector3(
            FilterAngle(angles.x),
            FilterAngle(angles.y),
            FilterAngle(angles.z)
        );
    }
    private static float FilterAngle(float angle)//метод для приведения угла к диапазону 0-360
    {
        // Используем остаток от деления на 360, чтобы получить угол в диапазоне [0, 360)
        angle %= 360f;
        // Если угол меньше 0, добавляем 360, чтобы получить значение в диапазоне [0, 360)
        if (angle < 0)
        {
            angle += 360f;
        }
        return angle;
    }
    private void FilterAccVector()//метод для фильтрации угловых и линейных ускорений
    {
        Acceleration = new Vector3(
            FilterAcc(Acceleration.x,minAcceleration),
            FilterAcc(Acceleration.y,minAcceleration),
            FilterAcc(Acceleration.z,minAcceleration)
        );
        AngularAcceleration = new Vector3(
            FilterAcc(AngularAcceleration.x,minAngularAcceleration),
            FilterAcc(AngularAcceleration.y,minAngularAcceleration),
            FilterAcc(AngularAcceleration.z,minAngularAcceleration)
        );    
    }
    private float FilterAcc(float acc,float minValue)//метод для фильтрации одного ускорения
    {
        if (Mathf.Abs(acc)<minValue)
        {
            return 0;
        }
        else
        {
            return acc;
        }
    }



    

    //Методы для получения состояния беспилотника
    //---------------------------------------------------------------------
    public Vector3 GetRotation()//получение углов ориентации БПЛА
    {
        return Rotation;
    }
    public Vector3 GetPosition()//Получение координат БПЛА
    {
        return Position;
    }
    public Vector3 GetAngularVelocity()//Получение угловых скоростей БПЛА
    {   
        return AngularVelocity;
    }

    public Vector3 GetVelocity()//Получение линейных скоростей БПЛА
    {
        return Velocity;
    }
}