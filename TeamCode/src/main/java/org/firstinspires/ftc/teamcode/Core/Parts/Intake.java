package org.firstinspires.ftc.teamcode.Core.Parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.InterfaceSimple;

public class Intake implements InterfaceSimple {
   public DcMotorEx Intake;


    public enum Mode{
        INTAKE,
        EJECT,
        STOP
    }

    public void start(HardwareMap hardware){
        Intake = hardware.get(DcMotorEx .class, "Intake");

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(Mode mode){
        switch (mode){
            case INTAKE:
                Intake.setPower(1);
                break;
            case EJECT:
                Intake.setPower(-1);
                break;
            case STOP:
                Intake.setPower(0);
                break;
        }
    }

public void teleop(Gamepad gamepad){
        if (gamepad.cross) setMode(Mode.INTAKE);
        else if (gamepad.circle) setMode(Mode.EJECT);
        else setMode(Mode.STOP);
}

    @Override
    public void update() {


    }
}
