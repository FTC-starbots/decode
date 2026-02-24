package org.firstinspires.ftc.teamcode.Core.Parts;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;

public class Shooter implements InterfaceSimple {
    public int velocidade_desejada;
   public  DcMotorEx Atirador_1, Atirador_2;
    public double erro;
    public boolean shooting = false, right_vel = false;
    boolean servo_lift;

    public boolean pre_shoot = false;

    int shoottype;


    PIDFOController controllerHeading = new PIDFOController(new PIDFCoefficients(0, 0, 0, 0));

    public void setVelDesejada(int vel){
        velocidade_desejada = vel;
    }

    public void isShooting(boolean isshooting){shooting = isshooting;}

    public void isPreShoot(boolean ispre){pre_shoot = ispre;}


    public void teleop(Gamepad gamepad, int vel, boolean gamepad1, int minvel){

        if (gamepad.left_bumper || gamepad.right_bumper ||gamepad.dpad_left || gamepad.dpad_up || gamepad.dpad_right || gamepad.y || gamepad.triangle){
            pre_shoot = false;
            shooting = true;
            setVelDesejada(vel);
        }
        else{
            pre_shoot = true;
            shooting = false;
            setVelDesejada(vel-60);
        }
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

        Atirador_2.setDirection(DcMotorEx.Direction.REVERSE);


        Atirador_1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Atirador_2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    public void isServoLifted(boolean lift){servo_lift = lift;}

    public void ShootType(int shoot){shoottype = shoot;}

    @Override
    public void update() {

        erro = velocidade_desejada - Atirador_1.getVelocity();

        controllerHeading.updateError(erro);
        double power = controllerHeading.run();

        if((erro > -60 && erro < 40) && velocidade_desejada != 0 && !pre_shoot) right_vel = true;
        else right_vel = false;

        Atirador_1.setPower(0);
        Atirador_2.setPower(0);

//        if (velocidade_desejada == 0){
//            Atirador_1.setPower(0);
//            Atirador_2.setPower(0);
//        }else if(pre_shoot){
//            Atirador_1.setPower(power);
//            Atirador_2.setPower(power);
//        } else{
//            /*if (shoottype == 1 && right_vel){
//                Atirador_1.setPower(1);
//                Atirador_2.setPower(1);
//            } else*/ if(erro>60) {
//                Atirador_1.setPower(1);
//                Atirador_2.setPower(1);
//            }
//            else{
//                Atirador_1.setPower(power);
//                Atirador_2.setPower(power);
//            }
//        }


    }
}
