package org.firstinspires.ftc.teamcode.Core.Parts;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PIDFOController;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain implements InterfaceSimple {
    double rotation;
    public double lastheading = 0,atualheading = 0;
    public double headingError ;

    Lime Limelight = new Lime();

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime rightpos = new ElapsedTime();

     public double error_hor;
    public double error_tag;
    public double error_final;

    public double widht_divididopor2menospixel;
    PIDFOController controllerHeading = new PIDFOController(new PIDFCoefficients(2, 0, 0.01, 0));

   public double targetHeading;
   public boolean headingLock, aptag;

    double kpa, kia, kda, kfa;

    double kpf, kif, kdf, kff;

    double kps, kis, kds, kfs;

    double vectorx, vectory;

    double lime_tag_ang;
public boolean right_pos;


    public void setCoefficientsAptag(double kp, double ki, double kd, double kf)  {
      kpa = kp;
      kia = ki;
      kda = kd;
      kfa = kf;
  }

    public void setCoefficientsfirt(double kp, double ki, double kd, double kf)  {
        kpf = kp;
        kif = ki;
        kdf = kd;
        kff = kf;
    }

    public void setCoefficientssecond(double kp, double ki, double kd, double kf)  {
        kps = kp;
        kis = ki;
        kds = kd;
        kfs = kf;
    }

    public void setTargetHeading(double target){targetHeading = target;}

    public void setVectorX(double x){vectorx = x;}
    public void setVectorY(double y){vectory = y;}

    public double angleWrap(double angle) {
        return Math.atan2(
                Math.sin(angle),
                Math.cos(angle)
        );
    }



  public void Drive(boolean gamepad1,double id, boolean isBlue, Gamepad gamepad, Follower followerdrive, boolean isShooting, double pixel, double width, boolean validation, double IdealAngle, boolean shoot_tag){
double rotate_tag = 0;
        if (gamepad1) {
    rotation = (gamepad.left_trigger - gamepad.right_trigger);
}else{
    rotation = 0;
            rotate_tag = (gamepad.left_trigger - gamepad.right_trigger);
}


     if (isShooting || !gamepad1){

         if (shoot_tag) {
             if (validation && (id == 20 || id ==24)) {
                 headingLock = true;
                 lime_tag_ang =Math.toRadians( -pixel +(rotate_tag*5));
                 error_final = lime_tag_ang;
             }else{
                 targetHeading = IdealAngle;

                 error_final = angleWrap(targetHeading - followerdrive.getHeading());
             }
         }else{
             targetHeading = IdealAngle;

             error_final = angleWrap(targetHeading - followerdrive.getHeading());

         }
     } else{

         error_final = angleWrap(targetHeading - followerdrive.getHeading());

     }

        if (!gamepad1)controllerHeading.setCoefficients(new PIDFCoefficients(kpf, kif, kdf, kff));
         else controllerHeading.setCoefficients(new PIDFCoefficients(kps, kis, kds, kfs));



      if(Math.abs(Math.toDegrees(error_final)) > 8 || Math.abs(Math.toDegrees(lime_tag_ang)) > 30 ) {
          controllerHeading.errorIntegral = 0;
          rightpos.reset();
      }



    if((Math.abs(Math.toDegrees(error_final)) < 8 || Math.abs(Math.toDegrees(lime_tag_ang)) < 30) && rightpos.seconds()>0.3) {
        right_pos = true;
    }else {
        right_pos = false;
    }



        controllerHeading.updateError(error_final);

      if(gamepad.left_trigger > 0 || gamepad.right_trigger > 0 ){
          headingLock = false;
          targetHeading = followerdrive.getHeading();
          timer.reset();
      }else {
          if (timer.seconds()> 0.1) headingLock = true;
          else targetHeading = followerdrive.getHeading();
      }


      if (isBlue) {
          if (headingLock) followerdrive.setTeleOpDrive(gamepad.left_stick_y , gamepad.left_stick_x, controllerHeading.run(), false);
          else followerdrive.setTeleOpDrive(gamepad.left_stick_y * 1.2, gamepad.left_stick_x * 1.2, rotation * 0.7, false);
      }else{
          if (headingLock) followerdrive.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, controllerHeading.run(), false);
          else followerdrive.setTeleOpDrive(-gamepad.left_stick_y * 1.2, -gamepad.left_stick_x * 1.2, rotation * 0.7, false);
      }



      followerdrive.update();

  }



    @Override
    public void update(){



    }
}
