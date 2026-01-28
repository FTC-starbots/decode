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


  public void setCoeficcients(double kp, double ki, double kd, double kf)  {
      kpa = kp;
      kia = ki;
      kda = kd;
      kfa = kf;
  }

  public void Drive(Gamepad gamepad, Follower followerdrive, boolean AptagLock, double pixel, double width, boolean validation, double offsetpercent){

      rotation = (gamepad.left_trigger - gamepad.right_trigger);



     if (AptagLock && validation){
         error_final = (((width/2)- (width* offsetpercent)) - pixel )/(((width/2)-(width* offsetpercent))* 0.03);
         aptag = true;

     } else{
         error_hor = targetHeading - followerdrive.getHeading();
         error_anti_hor = followerdrive.getHeading() - targetHeading;
         aptag = false;

         if (Math.abs(error_hor) < Math.abs(error_anti_hor)) error_final = error_hor;
         else error_final = -error_anti_hor;
     }



     if(AptagLock && validation){
         controllerHeading.setCoefficients(new PIDFCoefficients(0.02, 0.00005, 0, 0));


     }else{
         if(Math.abs(error_final) > 15) controllerHeading.setCoefficients(new PIDFCoefficients(2, 0, 0.01, 0));
         else controllerHeading.setCoefficients(new PIDFCoefficients(1, 0, 0.01, 0));



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
