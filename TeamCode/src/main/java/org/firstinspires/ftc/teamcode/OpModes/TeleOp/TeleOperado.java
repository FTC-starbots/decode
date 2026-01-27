package org.firstinspires.ftc.teamcode.OpModes.TeleOp;





import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
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

import org.firstinspires.ftc.teamcode.Core.Parts.AimAssisters;
import org.firstinspires.ftc.teamcode.Core.Parts.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.Parts.LimeServo;
import org.firstinspires.ftc.teamcode.Core.Parts.Reservoirs;
import org.firstinspires.ftc.teamcode.Core.Parts.Shooter;
import org.firstinspires.ftc.teamcode.Core.Parts.Intake;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Drawing;


@Configurable

@TeleOp
public class TeleOperado extends OpMode{

    Pose startingPose = new Pose(105.19626168224299,32.971962616822424, Math.toRadians(90));
    Follower followerteleop;

    boolean isGamepad1 = false, isManual = false;
    Robot robot = new Robot();

    Drivetrain drivetrain = robot.drivetrain;
    TelemetryManager telemetryM;


    Launching launching = new Launching();
    public static double kp = 0.000125, ki = 0.00009, kd = 0, kf = 0.05;
     ElapsedTime Ciclo;

     double tempoCiclo;



    public  VoltageSensor myControlHubVoltageSensor;

     Shooter Shooter = robot.shooter;

     Intake Intake = new Intake();

     Lime Limelight = robot.limelight;

    AprilTagCam aptag = robot.aptag;

    TagAlign tagalign = new TagAlign();
    AimAssisters Capota = launching.Capota;
    public Reservoirs reservatorio = new Reservoirs();
    public static double position1 = 0.82, position2 = 0.88, position3 = 0.85, limepos = 0;

    public static int vel = 1440;

    PoseIdentifier poseidentifier = new PoseIdentifier();

    LimeServo servolime = new LimeServo();






    @Override
    public void init() {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        followerteleop = Constants.createFollower(hardwareMap);
        followerteleop.setStartingPose(startingPose);
        followerteleop.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        followerteleop.startTeleopDrive();
       Limelight.setup(hardwareMap);
        reservatorio.start(hardwareMap);
        aptag.setup(hardwareMap);
        Intake.start(hardwareMap);
        Shooter.start(hardwareMap);
        launching.start(hardwareMap);
        Capota.start(hardwareMap);
        servolime.start(hardwareMap);
       Limelight.setPipeline(5);
        Capota.setMode(AimAssisters.Mode.SET_POSITION);


        Shooter.setCoefficients(kp,ki,kd,kf);

        Drawing.init();



    }


    public void start(){
        Capota.setValue(0.65);
        Ciclo = new ElapsedTime();
        Ciclo.reset();


    }

    @Override
    public void loop() {
        Ciclo.reset();
       tagalign.setValues(Limelight.tpixel, aptag.LeituraPixels, Limelight.tagdetected, aptag.tagdetected, 2592, 720);

       servolime.setPosition(limepos);

//===FUNÇÕES VITAIS==============================================================================
        {
            launching.teleop(gamepad2);

            Shooter.teleop(gamepad2, vel);

            Intake.teleop(gamepad1);
        }

//===LANÇAMENTO==================================================================================
        {
            if (Shooter.shooting && !launching.shoot_requested) {
                reservatorio.setPosition(reservatorio.Reservoir_1, position1);
                reservatorio.setPosition(reservatorio.Reservoir_2, position2);
                reservatorio.setPosition(reservatorio.Reservoir_3, position3);
            } else if (!Shooter.shooting && !launching.shoot_requested) {
                reservatorio.setPosition(reservatorio.Reservoir_1, 0.48);
                reservatorio.setPosition(reservatorio.Reservoir_2, 0.5);
                reservatorio.setPosition(reservatorio.Reservoir_3, 0.5);
            }

            Shooter.isServoLifted(launching.servo_lifted);
            launching.isAtRightVel(Shooter.right_vel);
        }

//===FUNÇÕES DE MOVIMENTO========================================================================
        {
            if (isGamepad1) {
                if (isManual) drivetrain.Drive(gamepad1, followerteleop, Shooter.shooting, 0, 100, false);
                else drivetrain.Drive(gamepad1, followerteleop, Shooter.shooting, tagalign.pixel, tagalign.width, tagalign.validation);


            } else {
                if (isManual) drivetrain.Drive(gamepad2, followerteleop, Shooter.shooting, 0, 100, false);
                else drivetrain.Drive(gamepad2, followerteleop, Shooter.shooting, tagalign.pixel, tagalign.width, tagalign.validation);

            }
        }

//===FUNÇÕES DE CONTROLE=========================================================================
        {
            if (gamepad2.back) isManual = !isManual;


            if (gamepad1.square || gamepad1.x) {
                isGamepad1 = true;
            }

            else if (gamepad2.square || gamepad2.x){
                isGamepad1 = false;
            }



        }

//===UPDATE DE POSIÇÃO===========================================================================
        {
            poseidentifier.infoLIME(Limelight.x, Limelight.y, Limelight.angle, Limelight.distance);
            poseidentifier.infoCAM(aptag.LeituraX, aptag.LeituraY, aptag.LeituraYaw);
            poseidentifier.infoPINPOINT(followerteleop.getPose().getX(), followerteleop.getPose().getY(), followerteleop.getPose().getHeading());

        }

        robot.update();
        telemetry();
        tempoCiclo = Ciclo.milliseconds();
        Drawing.draw(followerteleop);
    }


    public void stop(){
        aptag.stop();
    }

    public void telemetry(){

        telemetry.addData("Capota " , Capota.getPosition());
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("POSIÇÕES");
        telemetry.addData("CamX", poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.X));
        telemetry.addData("CamY",  poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.Y));
        telemetry.addData("CamYaw",  poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.YAW));
        telemetry.addData("CamDist",  poseidentifier.getInfo(PoseIdentifier.Source.LIMELIGHT, PoseIdentifier.DataInfo.DISTANCE));
        telemetry.addLine();
        telemetry.addData("CamX", poseidentifier.getInfo(PoseIdentifier.Source.CAMERA, PoseIdentifier.DataInfo.X));
        telemetry.addData("CamY",  poseidentifier.getInfo(PoseIdentifier.Source.CAMERA, PoseIdentifier.DataInfo.Y));
        telemetry.addData("CamYaw",  poseidentifier.getInfo(PoseIdentifier.Source.CAMERA, PoseIdentifier.DataInfo.YAW));
        telemetry.addData("CamDist",  poseidentifier.getInfo(PoseIdentifier.Source.CAMERA, PoseIdentifier.DataInfo.DISTANCE));
        telemetry.addLine();
        telemetry.addData("PinX", poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X));
        telemetry.addData("PinY",  poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y));
        telemetry.addData("PinYaw",  poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.YAW));
        telemetry.addData("PinDist",  poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE));
        telemetry.addLine();

        telemetry.addLine("ATIRADOR");


        telemetryM.addData("Velocidade desejada" , Shooter.velocidade_desejada);
        telemetryM.addData("Velocidade atual" , Shooter.Atirador_1.getVelocity());
        telemetryM.addData("erro shooter " , Shooter.erro);
        telemetryM.addData("power " , Shooter.Atirador_1.getPower());

        telemetry.addData("Velocidade desejada" , Shooter.velocidade_desejada);
        telemetry.addData("Velocidade atual" , Shooter.Atirador_1.getVelocity());
        telemetry.addData("shooter erro" , Shooter.erro);
        telemetry.addData("power " , Shooter.Atirador_1.getPower());

        telemetry.addLine();
        telemetry.addData("Target Heading", followerteleop.getPose());
        telemetry.addData("Target Heading", drivetrain.targetHeading);
        telemetry.addData("Heading", followerteleop.getHeading());
        telemetry.addLine();




    //telemetry.addData("TagPixels LIME", Limelight.tpixel);
        telemetry.addData("TagPixels CAM", aptag.LeituraPixels);
        telemetry.addData("CAM ERROR", drivetrain.error_final);
        telemetry.addData("calculo de erro", drivetrain.widht_divididopor2menospixel);

        telemetry.addLine();

        telemetry.addData("Heading error", followerteleop.getHeadingError());
        telemetry.addData("Heading Vector", followerteleop.getTeleopHeadingVector());
        telemetry.addLine();

        telemetry.addLine();
        telemetry.addData("RobotPoseX", aptag.LeituraX);
        telemetry.addData("RobotPoseY", aptag.LeituraY);
        telemetry.addData("RobotPoseYAW", aptag.LeituraYaw);


        telemetryM.update(telemetry);
        telemetry.update();

}
}
