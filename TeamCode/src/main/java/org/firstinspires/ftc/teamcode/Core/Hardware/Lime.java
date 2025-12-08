package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;

import java.util.List;

public class Lime implements InterfaceSimple {

    public Limelight3A limelight;
    int pipeline;
     double id, x, y, angle, distance, returnedData, tx;


    public enum Data{
        BOTPOSE_X,
        BOTPOSE_Y,
        ANGLE,
        ID,
        VALID,
        DISTANCETO_TARGET,
        TagTx
    }
    public void setup(HardwareMap hardware) {
        limelight = hardware.get(Limelight3A.class, "limelight");
        limelight.start();
        setPipeline(5);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void restart(){
        stop();
        start();
        limelight.pipelineSwitch(pipeline);
    }

    public void setPipeline(int pipe){
        pipeline = pipe;
        limelight.pipelineSwitch(pipeline);

    }

    public double getFilteredData(double ID,Data info){

        if (id == ID){
        switch (info) {
            case BOTPOSE_X:
                returnedData = x;
                break;
            case BOTPOSE_Y:
                returnedData = y;
                break;
            case ANGLE:
                returnedData = angle;
                break;
            case DISTANCETO_TARGET:
                returnedData = distance;
                break;
                case TagTx:
                returnedData = tx;
                break;
        }
        } else {
            returnedData = 0;
        }

        return returnedData;
    }

    public double getData(Data info){
        switch (info){
            case BOTPOSE_X:
                returnedData = x;
                break;
            case BOTPOSE_Y:
                returnedData = y;
                break;
            case ANGLE:
                returnedData = angle;
                break;
            case ID:
                returnedData = id;
                break;
            case DISTANCETO_TARGET:
                returnedData = distance;
                break;

        }

        return returnedData;
    }
@Override
    public void update(){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult id_verifier : fiducialResults) {

            if (limelight.getLatestResult().isValid()){
               id = id_verifier.getFiducialId();
               x = result.getBotpose().getPosition().x;
               y = result.getBotpose().getPosition().y;
               angle = result.getBotpose().getOrientation().getYaw();
               distance = result.getBotposeAvgDist();

            }
        }
    }

}
