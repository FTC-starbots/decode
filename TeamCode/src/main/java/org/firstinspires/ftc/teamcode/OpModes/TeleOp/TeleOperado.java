package org.firstinspires.ftc.teamcode.OpModes.TeleOp;





import org.firstinspires.ftc.teamcode.Core.Algorithms.Launching;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
import org.firstinspires.ftc.teamcode.Core.Algorithms.Selectors;
import org.firstinspires.ftc.teamcode.Core.Algorithms.TagAlign;
import org.firstinspires.ftc.teamcode.Core.Hardware.AprilTagCam;
import org.firstinspires.ftc.teamcode.Core.Hardware.Leds;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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


    Follower followerteleop;

    boolean isGamepad1 = false, isManual = false, isValidArea = false;
    Robot robot = new Robot();

    Drivetrain drivetrain = robot.drivetrain;
    TelemetryManager telemetryM;


    Launching launching = new Launching();
    public static double kpShooter = 0.0002, kiShooter = 0.00009, kdShooter = 0, kfShooter = 0.05;
    public static double kpAlign = 0.02, kiAlign = 0.00005, kdAlign = 0, kfAlign = 0;


    public static double Firstkp = 2, Firstki = 0, Firstkd = 0.01, Firstkf = 0;
    public static double Secondkp = 0.5, Secondki= 0.00005, Secondkd = 0.01, Secondkf = 0;

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
    public static double  limepos = 0;


    Selectors seletores = new Selectors();
    PoseIdentifier poseidentifier = new PoseIdentifier();

    LimeServo servolime = new LimeServo();

    Leds LED = new Leds();



    public static double servoA = 0.000022, servoB = 0.0079, servoC = 1.55;

    public static double velA = 2.1;
    public static int velB = 940;

    public static double offset = 0.0000001;


 public double servoIdeal;

 public int  velIdeal;

 public static double timer = 0.2;

 public static double  corr;

 public int auto;

 public boolean isBlue, manualSelectAlliance, manualSelectAuto;




    @Override
    public void init() {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        followerteleop = Constants.createFollower(hardwareMap);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        LED.start(hardwareMap);

        reservatorio.start(hardwareMap);

        aptag.setup(hardwareMap);

        Intake.start(hardwareMap);

        Shooter.start(hardwareMap);
        Shooter.setCoefficients(kpShooter,kiShooter, kdShooter, kfShooter);

        launching.start(hardwareMap);

        Capota.start(hardwareMap);
        Capota.setMode(AimAssisters.Mode.SET_POSITION);

        servolime.start(hardwareMap);

        seletores.start(hardwareMap);


        Limelight.setup(hardwareMap);
        Limelight.setPipeline(5);





        Drawing.init();



    }

    @Override
    public void init_loop(){



        if (!manualSelectAuto){
            auto = seletores.getAuto();
        }

        if (gamepad2.aWasPressed() || gamepad2.crossWasPressed()) {
            manualSelectAuto = true;
            auto =1;
        }
        else if (gamepad2.bWasPressed() || gamepad2.circleWasPressed()){
            manualSelectAuto = true;
            auto =2;
        }
        else if (gamepad2.yWasPressed() || gamepad2.triangleWasPressed()){
            manualSelectAuto = true;
            auto =3;
        }
        else if (gamepad2.xWasPressed() || gamepad2.squareWasPressed()) {
            manualSelectAuto = true;
            auto = 4;
        } else if (gamepad2.dpadUpWasPressed()) {
            manualSelectAuto = true;
            auto = 5;
        }



        if (!manualSelectAlliance){
            isBlue = seletores.isBlueAlliance();
        }

        if (gamepad2.dpadLeftWasPressed()) {
            manualSelectAlliance = true;
            isBlue = true;
        }
        else if (gamepad2.dpadRightWasPressed()){
            manualSelectAlliance = true;
            isBlue = false;
        }

        telemetry.addData("auto", auto);
        telemetry.addData("isBlue", isBlue);
        telemetry.update();

    }


@Override
    public void start(){
        Ciclo = new ElapsedTime();
       Ciclo.reset();


    if (auto == 1 || auto == 2){
        if (isBlue) {
            followerteleop.setStartingPose(new Pose(57.421, 100.037, Math.toRadians(323)));
        }else {
            followerteleop.setStartingPose(new Pose(86.579, 100.037, Math.toRadians(-143)));
        }
    }
    else if (auto == 3 || auto == 4){
        if (isBlue) {
            followerteleop.setStartingPose(new Pose(35.888, 7.981, Math.toRadians(180)));
        }else {
            followerteleop.setStartingPose(new Pose(108.112, 7.981, Math.toRadians(0)));
        }
    }
    if (auto == 5){
        followerteleop.setStartingPose(new Pose(105.19626168224299,32.971962616822424, Math.toRadians(90)));
    }

    followerteleop.update();
    followerteleop.startTeleopDrive();
    }

    @Override
    public void loop() {
        Ciclo.reset();


       tagalign.setValues(Limelight.tpixel, aptag.LeituraPixels, Limelight.tagdetected, aptag.tagdetected, 2592, 720);


       drivetrain.setCoefficientsAptag(kpAlign,kiAlign,kdAlign,kfAlign);
       drivetrain.setCoefficientsfirt(Firstkp,Firstki,Firstkd,Firstkf);
       drivetrain.setCoefficientssecond(Secondkp,Secondki,Secondkd,Secondkf);

       servolime.updateDesiredReference(LimeServo.Mode.OBELISK);
       servolime.setCurrentPosition(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X), poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y), poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.YAW));



        double d = poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.DISTANCE);

        servoIdeal = servoA * d * d - servoB * d + servoC;
        if (d > 100){
            velIdeal = (int) (Math.round((velA * d + velB + 40) / 20.0) * 20);
        }else{
            velIdeal = (int) (Math.round((velA * d + velB) / 20.0) * 20);
        }


        double angleIdeal = Math.atan2(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y) - 138, poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) - 12);

        if ( poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y) < Math.abs(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) - 72) + 68.5
                ||
                poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.Y) < - Math.abs(poseidentifier.getInfo(PoseIdentifier.Source.PINPOINT, PoseIdentifier.DataInfo.X) - 72) + 35.5){
            isValidArea = true;
        }else{
            isValidArea = false;
        }


//===FUNÇÕES VITAIS==============================================================================
        {
            launching.teleop(gamepad2,timer);

            Shooter.teleop(gamepad2, velIdeal, isGamepad1, velIdeal-60);

            Intake.teleop(gamepad1);

            servolime.update();
        }

//===LANÇAMENTO==================================================================================
        {
            if (!Shooter.shooting) {
               reservatorio.setPosition(reservatorio.Reservoir_1, 0.48);
               reservatorio.setPosition(reservatorio.Reservoir_2, 0.5);
               reservatorio.setPosition(reservatorio.Reservoir_3, 0.5);
            }



            Shooter.isServoLifted(launching.servo_lifted);
            launching.isAtRightVel(Shooter.right_vel);
            launching.isShooting(Shooter.shooting);
            Capota.setValue(servoIdeal);
            launching.isAtRightPos(drivetrain.right_pos);
        }

//===FUNÇÕES DE MOVIMENTO========================================================================
        {
            if (isGamepad1) {
                if (isManual) drivetrain.Drive(gamepad1, followerteleop, Shooter.shooting, 0, 100, false, offset, angleIdeal, launching.corretor);
                else drivetrain.Drive(gamepad1, followerteleop, Shooter.shooting, tagalign.pixel, tagalign.width, tagalign.validation, offset, angleIdeal,launching.corretor);


            } else {
                if (isManual) drivetrain.Drive(gamepad2, followerteleop, Shooter.shooting, 0, 100, false, offset, angleIdeal, launching.corretor);
                else drivetrain.Drive(gamepad2, followerteleop, Shooter.shooting, tagalign.pixel, tagalign.width, tagalign.validation, offset, angleIdeal,launching.corretor);

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

        if (gamepad1.startWasPressed()){
            if (isBlue){
            followerteleop.setPose(new Pose(134,8, Math.toRadians(180)));
            }else {
                followerteleop.setPose(new Pose(9,8, Math.toRadians(0)));
            }
        }


            poseidentifier.infoLIME(Limelight.x, Limelight.y, Limelight.angle, Limelight.distance);
            poseidentifier.infoCAM(aptag.LeituraX, aptag.LeituraY, aptag.LeituraYaw);
            poseidentifier.infoPINPOINT(followerteleop.getPose().getX(), followerteleop.getPose().getY(), followerteleop.getPose().getHeading());

        }


//===LEDS========================================================================================
        {
            if(isValidArea) LED.Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            else  LED.Led_Strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
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

        telemetry.addLine();
        telemetry.addData("ang ideal", servolime.IdealYaw);
        telemetry.addData("current yaw", servolime.currentYaw);
        telemetry.addData("current yaw", servolime.currentYaw);

        telemetry.addData("limeservo", servolime.ServoLime.getPosition());
        telemetry.addLine();
        telemetry.addData("Alliace_blue", seletores.Alliance_B.getState());
        telemetry.addData("Alliace_red", seletores.Alliance_R.getState());
        telemetry.addData("Auto1", seletores.Auto1.getState());
        telemetry.addData("Auto2", seletores.Auto2.getState());
        telemetry.addData("Auto3", seletores.Auto3.getState());
        telemetry.addData("Auto4", seletores.Auto4.getState());

        telemetry.addLine();
        telemetry.addData("Capota " , Capota.getPosition());
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
        telemetryM.addData("Target Heading", drivetrain.targetHeading);
        telemetryM.addData("Heading", followerteleop.getHeading());
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
