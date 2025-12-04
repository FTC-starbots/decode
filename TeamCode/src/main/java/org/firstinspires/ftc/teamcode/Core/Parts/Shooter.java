package org.firstinspires.ftc.teamcode.Core.Parts;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Hardware.InterfaceSimple;

public class Shooter implements InterfaceSimple {
    public int velocidade_desejada;
    DcMotorEx Atirador_1, Atirador_2;
    public double erro;

    PIDFController controllerHeading = new PIDFController(new PIDFCoefficients(0, 0, 0, 0));

    public void setVelDesejada(int vel){
        velocidade_desejada = vel;
    }

    public void teleop(Gamepad gamepad){
        if (gamepad.triangle) setVelDesejada(1100);
        else setVelDesejada(0);
    }

    public void setCoefficients(double kp, double ki, double kd, double kf){
        controllerHeading.setCoefficients(new PIDFCoefficients(kp, ki, kd, kf));
    }

    public void start(HardwareMap hardware){
        Atirador_1 = hardware.get(DcMotorEx.class, "Atirador_1");
        Atirador_2 = hardware.get(DcMotorEx.class, "Atirador_2");

        Atirador_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Atirador_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Atirador_1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Atirador_2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Atirador_1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Atirador_2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }
    @Override
    public void update() {
        erro = velocidade_desejada + Atirador_1.getVelocity();
        controllerHeading.updateError(erro);
        double power = controllerHeading.run();

        if (power <= 0) power = 0;

        Atirador_1.setPower(power);
        Atirador_2.setPower(-Atirador_1.getPower());


    }
}
