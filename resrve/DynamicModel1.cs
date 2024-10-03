using UnityEngine;
using System;

public class DynamicModel : MonoBehaviour
{   
    public Rigidbody Drone;//3D модель дрона

    //ограничения
    private float maxRotationVelocity = 2f;
    const float g = 9.81f;//ускорение свободного падения,константа

    //массо-габаритные характеристики ЛА
    public float m;//масса бпла
    public float l;//длина лучей бпла
    public float Ix;//момент инерции вокруг оси x
    public float Iy;//момент инерции вокруг оси y
    public float Iz;//момент инерции вокруг оси z




    //конфигурация ВМГ
    private int motor_count = 4;//количество моторов БПЛА
    public float b;//коэффициент тяги ВМГ
    public float d;//коэффициент момента пропеллера

    private Vector3 thrustForce;//вектор проекций сил зависящих от ориентации БПЛА,[0,P,0] в данном случае учитывается только сила тяги направленная вверх
    private Vector3 externalAccelerations = new Vector3(0,g,0);// Вектор проекций ускорений, не зависящих от положения беспилотника (ускорение свободного падения)

    //управляющее воздействие
    public float[] motorSpeedArray;//угловые скорости моторов

    //Состояние БПЛА 
    
    private Vector3 acceleration;// Вектор проекций ускорений беспилотника
    private Vector3 rotationAcceleration;//Вектор проекций угловых ускорений беспилотника
    private Vector3 velocity;//скорость беспилотника по осям
    private Vector3 rotationVelocity;//Угловые скорости беспилотника
    private Vector3 position;//Положение БПЛА в глобальной системе координат
    private Vector3 rotation;//Угловое положение БПЛА в глобальной системе координат

    
    private float[,] rotationMatrix;// Матрица поворота (вычисляется динамически)

    void Start()
    {
        rotation = Drone.transform.eulerAngles;
        position = Drone.transform.position;      
    }
    private void calculateAccelerations()//функция для вычисления проекций ускорения беспилотнка
    {   
        CalculateTrust();//вычисляем суммарную тягу
        CalculateRotationMatrix();// Вычисляем матрицу поворота из углов Эйлера
        
        //externalAccelerations = new Vector3(0, -10, 0);
        // Вычисляем поворот вектора тяги 
        Vector3 rotatedThrust = new Vector3(
            rotationMatrix[0, 0] * thrustForce.x + rotationMatrix[0, 1] * thrustForce.y + rotationMatrix[0, 2] * thrustForce.z,
            rotationMatrix[1, 0] * thrustForce.x + rotationMatrix[1, 1] * thrustForce.y + rotationMatrix[1, 2] * thrustForce.z,
            rotationMatrix[2, 0] * thrustForce.x + rotationMatrix[2, 1] * thrustForce.y + rotationMatrix[2, 2] * thrustForce.z
        );

        // Вычисляем ускорения
        acceleration = rotatedThrust/m - externalAccelerations;
        //Debug.Log(externalAccelerations);
    }
    public void CalculateAngularAcceleration()//gpt
    {   
        Vector3 Wb = rotationVelocity; // Вектор угловой скорости дрона
        // Вычисляемое угловое ускорение
        Vector3 Waz;
        float w0 = motorSpeedArray[0];
        float w1 = motorSpeedArray[1];
        float w2 = motorSpeedArray[2];       
        float w3 = motorSpeedArray[3];
        Debug.Log(w0+","+w1+","+w2+","+w3);

        // Создаем матрицу 3x3 для момента инерции
        Matrix4x4 inertiaMatrix = new Matrix4x4(
            new Vector4(Ix, 0, 0, 0),
            new Vector4(0, Iy, 0, 0),
            new Vector4(0, 0, Iz, 0),
            new Vector4(0, 0, 0, 1)
        );
        Matrix4x4 inertiaMatrixReversed = new Matrix4x4(
            new Vector4(1/Ix, 0, 0, 0),
            new Vector4(0, 1/Iy, 0, 0),
            new Vector4(0, 0, 1/Iz, 0),
            new Vector4(0, 0, 0, 1)
        );
        Debug.Log("Тензор инерции:"+inertiaMatrix.ToString("G"));
        // Вычисляем вектор момента сил
        Vector3 momentOfForces = new Vector3(
            l * b * (w0 * w0 - w2 * w2),
            l * b * (w3 * w3 - w1 * w1),
            d * (w3 * w3 + w1 * w1 - w0 * w0 - w2 * w2)
        );
        Debug.Log("моменты от двигателей:"+momentOfForces.ToString("G"));

        // Вычисляем векторное произведение Wb и момента инерции
        Vector3 crossProduct = Vector3.Cross(Wb, inertiaMatrix.MultiplyVector(Wb));
        // Вычисляем вектор углового ускорения
        rotationAcceleration = inertiaMatrixReversed.MultiplyVector(momentOfForces - crossProduct);
    }
    public void CalculateRotationAccelerations()//функция для вычисления вектора угловых ускорений
    {   
        
        rotationAcceleration = new Vector3(
            (l*b*(motorSpeedArray[0]*motorSpeedArray[0]-motorSpeedArray[2]*motorSpeedArray[2])-rotationVelocity[1]*rotationVelocity[2]* Mathf.Deg2Rad*(Iy-Iz))/Ix,
            (l*b*(motorSpeedArray[3]*motorSpeedArray[3]-motorSpeedArray[1]*motorSpeedArray[1])+rotationVelocity[0]*rotationVelocity[2]* Mathf.Deg2Rad*(Ix-Iz))/Iy,
            (l*d*(motorSpeedArray[3]*motorSpeedArray[3]+motorSpeedArray[1]*motorSpeedArray[1]-motorSpeedArray[0]*motorSpeedArray[0]-motorSpeedArray[2]*motorSpeedArray[2])-rotationVelocity[0]*rotationVelocity[1]*(Ix-Iy))/Iz);
        Debug.Log("rot acc: "+rotationAcceleration);
        rotationAcceleration*=Mathf.Rad2Deg;

    }

    
    private void CalculateRotationMatrix()// Функция для вычисления матрицы поворота из углов Эйлера

    {
        // Вычисление синусов и косинусов углов Эйлера
        float cr = Mathf.Cos(rotation[0] * Mathf.Deg2Rad);
        float sr = Mathf.Sin(rotation[0] * Mathf.Deg2Rad);
        float cy = Mathf.Cos(rotation[1] * Mathf.Deg2Rad);
        float sy = Mathf.Sin(rotation[1] * Mathf.Deg2Rad);
        float cp = Mathf.Cos(rotation[2] * Mathf.Deg2Rad);
        float sp = Mathf.Sin(rotation[2] * Mathf.Deg2Rad);
        

        // Вычисление матрицы поворота
        rotationMatrix = new float[,] {
            {cp*cy,-sy*cp,sp},
            {sr*sp*cy+sy*cr,-sr*sp*sy+cr*cy,-sy*cp},
            {sr*sy-sp*cr*cy,sr*cy+sp*sy*cr,cr*cp}
        };
    }
    private void CalculateTrust()//функция для вычисления суммарной тяги моторов
    {      
        //Debug.Log(motorSpeedArray[0]);
        //Debug.Log(motorSpeedArray[1]);
        //Debug.Log(motorSpeedArray[2]);
        //Debug.Log(motorSpeedArray[3]);
        float total_power = 0f;
        for (int i =0;i<motor_count;i++)
        {
            total_power += b*motorSpeedArray[i]*motorSpeedArray[i];
        }
        thrustForce = new Vector3(0f,  total_power, 0f);
    }

    private Vector3 integrate(Vector3 values, float dt)
    {
        return values * dt; 
    }

    public void simulateIteration(float dt)
    {   
        float realTimecoef = 1000f;//dt/Time.deltaTime;
        calculateAccelerations();
        //CalculateRotationAccelerations();
        CalculateAngularAcceleration();
        //acceleration = new Vector3(0, -10, 0);
        //rotationAcceleration ={0,0,0};
        velocity += integrate(acceleration, dt);  // Прямое присвоение 
        rotationVelocity += integrate(rotationAcceleration, dt);
        LimitAngularSpeed();
        position += integrate(velocity, dt); 
        rotation += integrate(rotationVelocity, dt)*Mathf.Rad2Deg;
        //rotation += rotation_delta*Mathf.Rad2Deg;
        rotation = FilterAngles(rotation);
        

        
    }
    public static Vector3 FilterAngles(Vector3 angles)
    {
        return new Vector3(
            FilterAngle(angles.x),
            FilterAngle(angles.y),
            FilterAngle(angles.z)
        );
    }

    // Метод для ограничения угловой скорости
    private void LimitAngularSpeed()
    {   
        // Ограничиваем каждую компоненту скорости
        for (int i = 0; i < 3; i++)
        {
            if (Mathf.Abs(rotationVelocity[i]) > maxRotationVelocity)
            {
                rotationVelocity[i] = Mathf.Sign(rotationVelocity[i]) * maxRotationVelocity;
            }
        }

        // Устанавливаем ограниченную угловую скорость 
    }
    private static float FilterAngle(float angle)
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
    private void checkGround()
    {
        if (position.y<1)
        {
            position.y = 1f;
            
        }
    }
    public void update_position(float dt)
    {   
        //float realTimecoef = Time.deltaTime/dt;
        Drone.transform.eulerAngles = rotation;
        checkGround();
        Drone.transform.position = position;
    }
    
}