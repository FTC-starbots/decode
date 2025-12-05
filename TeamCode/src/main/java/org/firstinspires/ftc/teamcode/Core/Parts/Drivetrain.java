package org.firstinspires.ftc.teamcode.Core.Parts;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain implements InterfaceSimple {
    double rotation;
    public double lastheading = 0,atualheading = 0;
    public double headingError ;

    ElapsedTime timer = new ElapsedTime();


    PIDFController controllerHeading = new PIDFController(new PIDFCoefficients(2, 0, 0.01, 0));

    double targetHeading = Math.toRadians(90);
    boolean headingLock;

  public void Drive(Gamepad gamepad, Follower followerdrive, boolean ManualAngle){

      rotation = (gamepad.left_trigger - gamepad.right_trigger);

      double error = targetHeading - followerdrive.getHeading();
      if(error > Math.toRadians(15)) controllerHeading.setCoefficients(new PIDFCoefficients(2, 0, 0.01, 0));
      else controllerHeading.setCoefficients(new PIDFCoefficients(1, 0, 0.01, 0));

      controllerHeading.updateError(error);

      if(gamepad.left_trigger > 0 || gamepad.right_trigger > 0){
          headingLock = false;
          targetHeading = followerdrive.getHeading();
          timer.reset();
      }else {
          if (timer.seconds()> 0.6) headingLock = true;
          else targetHeading = followerdrive.getHeading();
      }


      if (headingLock) followerdrive.setTeleOpDrive(gamepad.left_stick_y, gamepad.left_stick_x, controllerHeading.run(), false);
      else followerdrive.setTeleOpDrive(gamepad.left_stick_y * 1.2, gamepad.left_stick_x * 1.2, rotation * 0.7, false);




      followerdrive.update();

  }



    @Override
    public void update(){



    }
}
