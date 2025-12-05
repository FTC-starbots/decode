package org.firstinspires.ftc.teamcode.Core.Algorithms;


import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PIDFOController {

    public double integral_CorAng = 0, lastheading_CorAng = 0;
    public double integral_Vel = 0, lastheading_Vel = 0;


    public enum pidSystem{
     CorAng,
     Vel;

    }

    public double PIDF(pidSystem sistema,double erro, double kp, double ki, double kd, double kf, double min, double max, double integral, double lasterror, double cicle_time, double voltagem){

        double BatVoltage = voltagem;
        double feedfoward = 1 + (kf * (12.5-BatVoltage));
        double integrativo = ki * (erro * cicle_time);
        double derivativo = kd * ((erro - lasterror) / cicle_time);
        double proporcional = kp * erro;

        integral += integrativo;
        lasterror = erro;


        switch (sistema) {

            case CorAng:
                integral_CorAng = integral;
                lastheading_CorAng = lasterror;
                break;
            case Vel:
                integral_Vel = integral;
                lastheading_Vel = lasterror;
                break;
        }

        if (erro<0 && integral >  0)integral = 0;
        if (erro>0 && integral <  0)integral = 0;
        if (integral>0.15) integral=0.05;
        else if (integral<-0.15) integral=-0.05;

        double output = (proporcional + integral + derivativo ) * feedfoward;

        if (Math.abs(output) < 0.065)output = 0;

        if ( output > max) output = max;
        else if (output < min) output = min;


        return output;
    }





}
