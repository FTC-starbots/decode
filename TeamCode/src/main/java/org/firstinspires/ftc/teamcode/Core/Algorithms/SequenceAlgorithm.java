package org.firstinspires.ftc.teamcode.Core.Algorithms;

import org.firstinspires.ftc.teamcode.Core.Hardware.ColorSensor;

public class SequenceAlgorithm implements InterfaceSimple{
    ColorSensor sensor = new ColorSensor();
    double id;

    public void setMOTIF(double motif){id = motif;}

     ColorSensor.BALL[] gateOrder = new ColorSensor.BALL[9];

    public ColorSensor.BALL[] gateOrder(){
        if(id == 21){
            gateOrder[0] = ColorSensor.BALL.GREEN;
            gateOrder[1] = ColorSensor.BALL.PURPLE;
            gateOrder[2] = ColorSensor.BALL.PURPLE;
            gateOrder[3] = ColorSensor.BALL.GREEN;
            gateOrder[4] = ColorSensor.BALL.PURPLE;
            gateOrder[5] = ColorSensor.BALL.PURPLE;
            gateOrder[6] = ColorSensor.BALL.GREEN;
            gateOrder[7] = ColorSensor.BALL.PURPLE;
            gateOrder[8] = ColorSensor.BALL.PURPLE;
        } else if (id == 22) {
            gateOrder[0] = ColorSensor.BALL.PURPLE;
            gateOrder[1] = ColorSensor.BALL.GREEN;
            gateOrder[2] = ColorSensor.BALL.PURPLE;
            gateOrder[3] = ColorSensor.BALL.PURPLE;
            gateOrder[4] = ColorSensor.BALL.GREEN;
            gateOrder[5] = ColorSensor.BALL.PURPLE;
            gateOrder[6] = ColorSensor.BALL.PURPLE;
            gateOrder[7] = ColorSensor.BALL.GREEN;
            gateOrder[8] = ColorSensor.BALL.PURPLE;
        }else if (id == 23) {
            gateOrder[0] = ColorSensor.BALL.PURPLE;
            gateOrder[1] = ColorSensor.BALL.PURPLE;
            gateOrder[2] = ColorSensor.BALL.GREEN;
            gateOrder[3] = ColorSensor.BALL.PURPLE;
            gateOrder[4] = ColorSensor.BALL.PURPLE;
            gateOrder[5] = ColorSensor.BALL.GREEN;
            gateOrder[6] = ColorSensor.BALL.PURPLE;
            gateOrder[7] = ColorSensor.BALL.PURPLE;
            gateOrder[8] = ColorSensor.BALL.GREEN;
        }
        return gateOrder;
    }



    @Override
    public void update() {
    }
}
