using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoutePlanner : MonoBehaviour
{
    
    private float[] RouteCoefX;
    private float[] RouteCoefY;
    private float[] RouteCoefZ;
    private Vector3 StartPosition;
    private Vector3 EndPosition;
    private float TotalSimulationTime;
    

    public void Setup(Vector3 startPosition,Vector3 endPosition,float totalSimulationTime)//Настраиваем планировщик маршрута
    {   
        //Начальная и конеечные точки
        this.StartPosition = startPosition;
        this.EndPosition = endPosition;
        //Максимальное время сиимуляции
        this.TotalSimulationTime = totalSimulationTime;
        //Расчет коэфициентов
        CalcCoef(TotalSimulationTime);

    }
    
    private void CalcCoef(float simulationTime)//Функция для вычисления траекторных коэффициентов
    {   
        RouteCoefX = new float[] {(-6*StartPosition.x+6*EndPosition.x)/Mathf.Pow(simulationTime,5),(15*StartPosition.x-15*EndPosition.x)/Mathf.Pow(simulationTime,4),(-10*StartPosition.x+10*EndPosition.x)/Mathf.Pow(simulationTime,3),0,0,StartPosition.x};
        RouteCoefY = new float[] {(-6*StartPosition.y+6*EndPosition.y)/Mathf.Pow(simulationTime,5),(15*StartPosition.y-15*EndPosition.y)/Mathf.Pow(simulationTime,4),(-10*StartPosition.y+10*EndPosition.y)/Mathf.Pow(simulationTime,3),0,0,StartPosition.y};
        RouteCoefZ = new float[] {(-6*StartPosition.z+6*EndPosition.z)/Mathf.Pow(simulationTime,5),(15*StartPosition.z-15*EndPosition.z)/Mathf.Pow(simulationTime,4),(-10*StartPosition.z+10*EndPosition.z)/Mathf.Pow(simulationTime,3),0,0,StartPosition.z};
    }
    public Vector3 GetWayPoint(float simulationTime)//Функция для получения точки маршрута в момент времени
    {   
        float way_point_x = 0; 
        float way_point_y = 0; 
        float way_point_z = 0;
        
        if (TotalSimulationTime<simulationTime)//Если время симуляции превышает прогнозируемое то выбираем последнюю точку маршрута
        {
            simulationTime = TotalSimulationTime;
        }
        for (int i=0;i<6;i++)//Вычисляем актальную точку траектории
        {
            way_point_x += Mathf.Pow(simulationTime,i)*RouteCoefX[5-i];
            way_point_y += Mathf.Pow(simulationTime,i)*RouteCoefY[5-i];
            way_point_z += Mathf.Pow(simulationTime,i)*RouteCoefZ[5-i];
        }

        return new Vector4(way_point_x,way_point_y,way_point_z,0);

    }
}
