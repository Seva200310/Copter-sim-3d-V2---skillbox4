using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimulationManger : MonoBehaviour
{   
    private const float dt = 0.02f;
    private float SimulationTime = 0;
    public DynamicModel Dm;
    public ControlSystem Cs;
    public RoutePlanner Rp;
    private float [] targetMotorsRotation;
    private Vector3 LastPosition;
    private Vector3 CurrentPosition;

    void Start()
    {   
        Vector3 startPoint = new Vector3(0,200,0);
        Vector3 endPoint = new Vector3(15,220,20);
        Cs.ControlSystemStart(dt);
        Rp.Setup(startPoint,endPoint,10);
        LastPosition = startPoint;
        
    }
    void FixedUpdate()
    {   
        SimulationTime+=dt;
        //Debug.Log(SimulationTime);
        Vector4 newWayPoint = Rp.GetWayPoint(SimulationTime);
        Debug.Log("Время: "+SimulationTime+" Текущее положение: "+Dm.GetPosition()+" Точка маршрута: "+ newWayPoint);
        //newWayPoint = new Vector4(10,200,0,0);
        Vector4 motorSpeedArray = Cs.GetMotorSpeed_cmd(0,newWayPoint,Dm.GetPosition(),Dm.GetRotation(),Dm.GetVelocity(),Dm.GetAngularVelocity()*Mathf.Rad2Deg);
        
        //Debug.Log(motorSpeedArray);
        //Debug.Log("Обороты моторов: " + motorSpeedArray);
        //Debug.Log("Угловое положение БПЛА :" + Dm.GetRotation()+"Время: "+SimulationTime);
        //motorSpeedArray = new Vector4(2510,2510,2490,2490);
        Dm.MotorSpeedArray = motorSpeedArray;
        Dm.SimulateIteration(dt);
        //targetMotorsRotation = cs.get_target_Rotation()//передаем данные о состоянии беспилотника в систему управления,она определяет управляющее воздействие
        //dm.motorSpeedArray = {2000, 1800, 2000, 2000};//передаем управляющее воздействие в динамическую модель
        Dm.Update_position(dt);
        CurrentPosition = Dm.GetPosition();
        Debug.Log("Прошлое положение: "+ LastPosition + " Актуальное положение " + CurrentPosition);
        Debug.DrawLine(LastPosition, CurrentPosition, Color.red,1000);
        LastPosition = CurrentPosition;
        
    }
    
}
