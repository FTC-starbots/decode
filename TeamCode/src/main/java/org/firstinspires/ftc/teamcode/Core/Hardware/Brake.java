package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;

public class Brake implements InterfaceSimple {
   public Servo Freio;

    public enum Mode{
        BRAKE,
        NEUTRAL
    }

    public void start(HardwareMap hardware){
        Freio = hardware.get(Servo.class, "Brake");
    }
    public void setMode(Mode mode){
        switch (mode){
            case BRAKE:
                Freio.setPosition(0.65);
                break;
            case NEUTRAL:
                Freio.setPosition(0.5);
                break;
        
        }
    }

    @Override
    public void update() {


    }
}
