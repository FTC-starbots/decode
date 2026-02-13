package org.firstinspires.ftc.teamcode.Core.Parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;

public class Intake implements InterfaceSimple {
   public DcMotor Intake_1, Intake_2;


    public enum Mode{
        INTAKE,
        EJECT,
        STOP
    }

    public void start(HardwareMap hardware){
        Intake_1 = hardware.get(DcMotor.class, "Intake_1");
        Intake_2 = hardware.get(DcMotor.class, "Intake_2");

        Intake_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake_1.setDirection(DcMotor.Direction.REVERSE);
        Intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake_2.setDirection(DcMotor.Direction.FORWARD);
        Intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(Mode mode){
        switch (mode){
            case INTAKE:
                Intake_1.setPower(-1);
                Intake_2.setPower(-1);
                break;
            case EJECT:
                Intake_1.setPower(1);
                Intake_2.setPower(1);
                break;
            case STOP:
                Intake_1.setPower(0);
                Intake_2.setPower(0);
                break;
        }
    }

public void teleop(Gamepad gamepad){
        if (gamepad.cross || gamepad.a) setMode(Mode.INTAKE);
        else if (gamepad.circle || gamepad.b) setMode(Mode.EJECT);
        else setMode(Mode.STOP);
}

    @Override
    public void update() {


    }
}
