package org.firstinspires.ftc.teamcode.Core.Hardware;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;

import java.util.List;

public class Lime implements InterfaceSimple {

    public Limelight3A limelight;
    public int pipeline;
    public double id, x, y, angle, distance, returnedData, tx, tdegrees, tpixel;

    public double preKalmanGreenBalls, preKalmanPurpleBalls, preKalmanTotalBalls;
    public double GreenBalls, PurpleBalls, TotalBalls;

    public boolean tagdetected;


    KalmanFilter GreenFilter = new KalmanFilter(new KalmanFilterParameters(0.05,1), 0, 100,1.2);
    KalmanFilter PurpleFilter = new KalmanFilter(new KalmanFilterParameters(0.05,1), 0, 100,1.2);


    public Mode OperationMode;

public enum Mode{
    TAG,
    GATE
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

    public void setOperationMode(Mode mode){
        OperationMode = mode;
        if (OperationMode == Mode.GATE) setPipeline(2);
        else setPipeline(5);
    }

@Override
    public void update() {

    LLResult result = limelight.getLatestResult();
    if (OperationMode == Mode.TAG) {


        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {

            if (limelight.getLatestResult().isValid()) {
                id = fr.getFiducialId();
                x = result.getBotpose().getPosition().x;
                y = result.getBotpose().getPosition().y;
                angle = result.getBotpose().getOrientation().getYaw();
                distance = result.getBotposeAvgDist();
                tx = result.getTx();
                tdegrees = fr.getTargetXDegrees();
                tpixel = fr.getTargetXPixels();
                tagdetected = true;

            } else {
                id = 0;
                x = 0;
                y = 0;
                angle = 0;
                distance = 0;
                tx = 0;
                tdegrees = 0;
                tpixel = 0;
                tagdetected = false;
            }
        }
    } else if (OperationMode == Mode.GATE) {

        double[] pythonOutputs = result.getPythonOutput();
//
        preKalmanTotalBalls = pythonOutputs[3];
        preKalmanPurpleBalls = pythonOutputs[4];
        preKalmanGreenBalls = pythonOutputs[5];


        GreenFilter.update(0, preKalmanGreenBalls);
        PurpleFilter.update(0, preKalmanPurpleBalls);

        GreenBalls = (int) Math.round(GreenFilter.getState());
        PurpleBalls = (int) Math.round(PurpleFilter.getState());

        TotalBalls = GreenBalls + PurpleBalls;


    }
}
}
