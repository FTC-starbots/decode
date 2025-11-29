package org.firstinspires.ftc.teamcode.Core.Parts;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Core.Hardware.InterfaceSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
public class Drivetrain implements InterfaceSimple {
    double rotation;
    double lastheading,atualheading;

    public void StartFollower(Follower follower, Pose startingPose, HardwareMap hardwareMap){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower.startTeleopDrive();

    }

  public void Drive(Gamepad gamepad, Follower follower, boolean ManualAngle, double angulo_desejado){

      rotation = (gamepad.left_trigger - gamepad.right_trigger);
      follower.setTeleOpDrive(gamepad.left_stick_y, gamepad.left_stick_x, rotation*0.5, false);

      if(rotation != 0) lastheading = Math.toDegrees(follower.getHeading());
      atualheading = Math.toDegrees(follower.getHeading()) % 360;
      if (atualheading < 0) atualheading += 360;

      int rotate;

      if (atualheading >= 0) {
          rotate = 360;
      } else {
          rotate = -360;
      }

      if (!ManualAngle) {
          if (gamepad.left_trigger > 0 || gamepad.right_trigger > 0) angulo_desejado = atualheading;
      }

      double corr_1 = atualheading - angulo_desejado;
      double corr_2 = angulo_desejado + rotate - atualheading;
      double correcaoP;
      if (Math.abs(corr_1) > Math.abs(corr_2)) correcaoP = -corr_2;
      else correcaoP = corr_1;


      follower.update();

  }



    @Override
    public void update(){


        telemetryM.update();


    }
}
