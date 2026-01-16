package org.firstinspires.ftc.teamcode.OpModes.TeleOp;





import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.TagAlign;
import org.firstinspires.ftc.teamcode.Core.Hardware.AprilTagCam;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

@Configurable

@TeleOp
public class TeleOperado extends OpMode{

    Pose startingPose = new Pose(105.19626168224299,32.971962616822424, Math.toRadians(90));
    Follower followerteleop;


    Robot robot = new Robot();

    Drivetrain drivetrain = robot.drivetrain;
    TelemetryManager telemetryM;


    Launching launching = new Launching();
    public static double kp = 0.003, ki = 0.0000001, kd = 0.003, kf = 0.05;
     ElapsedTime Ciclo;

     double tempoCiclo;

    double CorrAng;

    double volts;


    public  VoltageSensor myControlHubVoltageSensor;

     Shooter Shooter = robot.shooter;

     Intake Intake = new Intake();

     Lime Limelight = robot.limelight;

   // AprilTagCam aptag = robot.aptag;

    TagAlign tagalign = new TagAlign();



    @Override
    public void init() {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        followerteleop = Constants.createFollower(hardwareMap);
        followerteleop.setStartingPose(startingPose);
        followerteleop.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        followerteleop.startTeleopDrive();
       // Limelight.setup(hardwareMap);
        //aptag.setup(hardwareMap);
        //Intake.start(hardwareMap);
        //Shooter.start(hardwareMap);
        //launching.start(hardwareMap);
       // Limelight.setPipeline(5);
Drawing.init();



    }


    public void start(){

        Ciclo = new ElapsedTime();
        Ciclo.reset();
        //Shooter.setCoefficients(kp,ki,kd,kf);

    }

    @Override
    public void loop() {
        Ciclo.reset();
       // tagalign.setValues(Limelight.tpixel, aptag.LeituraPixels, Limelight.tagdetected, aptag.tagdetected, 2592, 720);


        //launching.teleop(gamepad1);
        //Shooter.teleop(gamepad1);
        //Intake.teleop(gamepad1);
        drivetrain.Drive(gamepad1, followerteleop, Shooter.shooting, tagalign.pixel, tagalign.width, tagalign.validation);

        robot.update();
        telemetry();
        tempoCiclo = Ciclo.milliseconds();
        Drawing.draw(followerteleop);
    }


    public void stop(){
       // aptag.stop();
    }
public void telemetry(){


    //telemetryM.addData("Velocidade desejada" , Shooter.velocidade_desejada);
    //telemetryM.addData("Velocidade atual" , Shooter.Atirador_1.getVelocity());
    //telemetryM.addData("erro " , Shooter.erro);

    telemetry.addLine();
    telemetry.addData("Target Heading", drivetrain.targetHeading);
    telemetry.addData("Heading", followerteleop.getHeading());
    telemetry.addLine();


    //telemetry.addData("TagPixels LIME", Limelight.tpixel);
    //telemetry.addData("TagPixels CAM", aptag.LeituraDegrees);
    telemetry.addLine();

    telemetry.addData("Heading error", followerteleop.getHeadingError());
    telemetry.addData("Heading Vector", followerteleop.getTeleopHeadingVector());
    telemetry.addLine();

    telemetryM.update(telemetry);
    telemetry.update();

}
}
