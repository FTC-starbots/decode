package org.firstinspires.ftc.teamcode.Core.Parts;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;
import org.firstinspires.ftc.teamcode.Core.Hardware.Brake;

public class Shooter implements InterfaceSimple {
    public int velocidade_desejada;
   public  DcMotorEx Atirador_1, Atirador_2, rightFront, rightBack, leftFront,leftBack;
    public double erro;
    public boolean shooting = false, right_vel = false;
    boolean servo_lift;

    public boolean pre_shoot = false;

    int shoottype;
    double corr;
    public double compensation;
boolean isblue;
    int Upp, Low,pathstate, auton;
    double txLime;
    double idDetecgted;

    boolean isAuto, validation;
    Brake freio = new Brake();

    PIDFOController controllerHeading = new PIDFOController(new PIDFCoefficients(0, 0, 0, 0));

    public void setVelDesejada(int vel){
        velocidade_desejada = vel;
    }

    public void isShooting(boolean isshooting){shooting = isshooting;}

    public void isPreShoot(boolean ispre){pre_shoot = ispre;}


    public void teleop(Gamepad gamepad, int vel, boolean gamepad1, int minvel){
        isAuto = false;
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
        leftBack =hardware.get(DcMotorEx.class,"leftBack");
        leftFront =hardware.get(DcMotorEx.class,"leftFront");
        rightBack =hardware.get(DcMotorEx.class,"rightBack");
        rightFront =hardware.get(DcMotorEx.class,"rightFront");
        freio.start(hardware);


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

    public void setMargin(int upper, int lower){
        Upp = upper;
        Low = lower;
    }
    public void isAuto(boolean auto ,int autono){isAuto = auto; auton = autono;}

    public void pathstate(int paths){pathstate = paths;}

    public void setlimeInfo(double TX, double id ,boolean val){txLime = TX; idDetecgted = id; validation = val;}
    public void isBlue(boolean isblue){this.isblue = isblue;}

    @Override
    public void update() {
        if (isAuto /*&& (auton == 3 || auton == 4)*/) {
            if (shooting) {
                if (validation) {
                if (isblue) {

                        if (idDetecgted == 20) {
                            //if (auton == 3 || auton == 4) corr = (txLime - 845) / 680;
                             corr = 0;

                            // else if (auton == 1 || auton == 2) corr = (txLime - 900) / 680;

//                            leftBack.setPower(corr);
//                            leftFront.setPower(corr);
//                            rightBack.setPower(-corr);
//                            rightFront.setPower(-corr);

                        }


                }else{

                    if (idDetecgted == 24) {
                        //if (auton == 3 || auton == 4) corr = (txLime - 640) / 680;
                         corr = 0;
                        //else if (auton == 1 || auton == 2) corr = (txLime - 380) / 680;

//                        leftBack.setPower(corr);
//                        leftFront.setPower(corr);
//                        rightBack.setPower(-corr);
//                        rightFront.setPower(-corr);
                    }

                }
                } else corr = 0;

            }
        }
        erro = velocidade_desejada - Atirador_1.getVelocity();

        controllerHeading.updateError(erro);
        double power = controllerHeading.run();

        if((erro > -Low && erro < Upp) && velocidade_desejada != 0 && !pre_shoot) right_vel = true;
        else right_vel = false;

//        Atirador_1.setPower(0);
//        Atirador_2.setPower(0);

         compensation = 0;


       if (velocidade_desejada == 0){
           Atirador_1.setPower(0);
           Atirador_2.setPower(0);
       }else if(pre_shoot){
           Atirador_1.setPower(power);
           Atirador_2.setPower(power);
       } else{
            if(erro<0) {
               Atirador_1.setPower(0);
               Atirador_2.setPower(0);
                //freio.setMode(Brake.Mode.BRAKE);
           }
           if(erro>40) {
               Atirador_1.setPower(1);
               Atirador_2.setPower(1);
           }
           else{
               Atirador_1.setPower(power);
               Atirador_2.setPower(power);
           }
       }


    }
}
