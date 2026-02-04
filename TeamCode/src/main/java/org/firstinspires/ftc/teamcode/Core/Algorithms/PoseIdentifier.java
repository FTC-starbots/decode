package org.firstinspires.ftc.teamcode.Core.Algorithms;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

public class PoseIdentifier implements  InterfaceSimple{
    public enum Source{
        LIMELIGHT,
        CAMERA,
        PINPOINT
    }

    public enum DataInfo{
        X,
        Y,
        YAW,
        DISTANCE
    }

    public double poseX, poseY, poseYaw, distance, returninfo;
     double LIMEposeX, LIMEposeY, LIMEposeYaw, LIMEdistance;

    double CAMposeX, CAMposeY, CAMposeYaw, CAMdistance;

    double PINPposeX, PINPposeY, PINPposeYaw, PINPdistance;

    Pose LimePose, CamPose;
    boolean isBlue;

    public double getInfo(Source source, DataInfo dataInfo) {
            switch (source) {
                case CAMERA:
                    poseX = CAMposeX;
                    poseY = CAMposeY;
                    poseYaw = CAMposeYaw;
                    distance = CAMdistance;

                    break;

                case LIMELIGHT:
                    poseX = LIMEposeX;
                    poseY = LIMEposeY;
                    poseYaw = LIMEposeYaw;
                    distance = LIMEdistance;
                    break;
                case PINPOINT:

                    poseX = PINPposeX;
                    poseY = PINPposeY;
                    poseYaw = PINPposeYaw;
                    distance = PINPdistance;
                    break;

            }

                    switch (dataInfo){
                        case X:
                            returninfo = poseX;
                            break;
                        case Y:
                            returninfo = poseY;
                            break;
                        case YAW:
                            returninfo = poseYaw;
                            break;
                        case DISTANCE:
                            returninfo = distance;
                            break;

                    }
                    return returninfo;
            }

            public void isblue(boolean isblue){isBlue = isblue;}

    public void infoLIME(double limex,double limey, double limeyaw,double limedist){

        LimePose = new Pose(limex, limey, limeyaw, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);


        LIMEposeX = LimePose.getX();
        LIMEposeY = LimePose.getY();
        LIMEposeYaw = LimePose.getHeading();
        LIMEdistance = limedist;
    }

    public void infoCAM(double camx, double camy, double camyaw){

        CamPose = new Pose(camx, camy, camyaw, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);


            CAMposeX = CamPose.getX();
            CAMposeY = CamPose.getY();
            CAMposeYaw = CamPose.getHeading();

            CAMdistance = Math.sqrt((CAMposeX - 10)*(CAMposeX - 10) + (CAMposeY - 138)*(CAMposeY - 138));
    }

    public void infoPINPOINT(double pinx, double piny, double pinyaw){
        PINPposeX = pinx;
        PINPposeY = piny;
        PINPposeYaw =  Math.toDegrees(pinyaw);
if (isBlue) {
    PINPdistance = Math.sqrt((PINPposeX - 10) * (PINPposeX - 10) + (PINPposeY - 138) * (PINPposeY - 138));
}else {
    PINPdistance = Math.sqrt((PINPposeX - 134) * (PINPposeX - 134) + (PINPposeY - 138) * (PINPposeY - 138));
}
    }

    @Override
    public void update() {

    }
}
