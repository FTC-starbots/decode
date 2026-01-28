package org.firstinspires.ftc.teamcode.Core.Parts;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
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

     public double error_hor;
    public double error_anti_hor;
    public double error_final;

    public double widht_divididopor2menospixel;
    PIDFController controllerHeading = new PIDFController(new PIDFCoefficients(2, 0, 0.01, 0));

   public double targetHeading = Math.toRadians(90);
    boolean headingLock, aptag;

    double kpa, kia, kda, kfa;

    double kpf, kif, kdf, kff;

    double kps, kis, kds, kfs;


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


  public void Drive(Gamepad gamepad, Follower followerdrive, boolean AptagLock, double pixel, double width, boolean validation, double offsetpercent, double IdealAngle,double corrreserv){

      rotation = (gamepad.left_trigger - gamepad.right_trigger);



     if (AptagLock){

         if (validation) {
             error_final = (((width / 2) - (width * offsetpercent)) - pixel) / (((width / 2) - (width * offsetpercent)) * 0.03);
             aptag = true;
         }else{
             targetHeading = IdealAngle + Math.toRadians(corrreserv) * -6;
             error_hor = targetHeading - followerdrive.getHeading();
             error_anti_hor = followerdrive.getHeading() - targetHeading;
             aptag = false;

             if (Math.abs(error_hor) < Math.abs(error_anti_hor)) error_final = error_hor;
             else error_final = -error_anti_hor;
         }
     } else{
         error_hor = targetHeading - followerdrive.getHeading();
         error_anti_hor = followerdrive.getHeading() - targetHeading;
         aptag = false;

         if (Math.abs(error_hor) < Math.abs(error_anti_hor)) error_final = error_hor;
         else error_final = -error_anti_hor;
     }



     if(AptagLock && validation){
         controllerHeading.setCoefficients(new PIDFCoefficients(kpa, kia, kda, kfa));
         timer.reset();

     }else{
         if(Math.abs(error_final) > 15) controllerHeading.setCoefficients(new PIDFCoefficients(kpf, kif, kdf, kff));
         else controllerHeading.setCoefficients(new PIDFCoefficients(kps, kis, kds, kfs));



     }

    if(Math.abs(error_final) < 5) {
        right_pos = true;
    }

      controllerHeading.updateError(error_final);

      if(gamepad.left_trigger > 0 || gamepad.right_trigger > 0 ){
          headingLock = false;
          targetHeading = followerdrive.getHeading();
          timer.reset();
      }else {
          if (timer.seconds()> 0.2) headingLock = true;
          else targetHeading = followerdrive.getHeading();
      }

      if (aptag){
          headingLock = true;
          targetHeading = followerdrive.getHeading();
      }


      if (headingLock) followerdrive.setTeleOpDrive(gamepad.left_stick_y, gamepad.left_stick_x, controllerHeading.run(), false);
      else followerdrive.setTeleOpDrive(gamepad.left_stick_y * 1.2, gamepad.left_stick_x * 1.2, rotation * 0.7, false);




      followerdrive.update();

  }



    @Override
    public void update(){



    }
}
