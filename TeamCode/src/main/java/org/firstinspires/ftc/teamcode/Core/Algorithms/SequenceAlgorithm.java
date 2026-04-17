package org.firstinspires.ftc.teamcode.Core.Algorithms;

import org.firstinspires.ftc.teamcode.Core.Hardware.ColorSensor;

public class SequenceAlgorithm {

    private ColorSensor.BALL[] sequence;
    private int index = 0;

    public void setId(int id) {

        if (id == 21) {
            sequence = new ColorSensor.BALL[]{
                    ColorSensor.BALL.GREEN, ColorSensor.BALL.PURPLE, ColorSensor.BALL.PURPLE,
                    ColorSensor.BALL.GREEN, ColorSensor.BALL.PURPLE, ColorSensor.BALL.PURPLE,
                    ColorSensor.BALL.GREEN, ColorSensor.BALL.PURPLE, ColorSensor.BALL.PURPLE
            };
        }
        else if (id == 22) {
            sequence = new ColorSensor.BALL[]{
                    ColorSensor.BALL.PURPLE, ColorSensor.BALL.GREEN, ColorSensor.BALL.PURPLE,
                    ColorSensor.BALL.PURPLE, ColorSensor.BALL.GREEN, ColorSensor.BALL.PURPLE,
                    ColorSensor.BALL.PURPLE, ColorSensor.BALL.GREEN, ColorSensor.BALL.PURPLE
            };
        }
        else { // 23
            sequence = new ColorSensor.BALL[]{
                    ColorSensor.BALL.PURPLE, ColorSensor.BALL.PURPLE, ColorSensor.BALL.GREEN,
                    ColorSensor.BALL.PURPLE, ColorSensor.BALL.PURPLE, ColorSensor.BALL.GREEN,
                    ColorSensor.BALL.PURPLE, ColorSensor.BALL.PURPLE, ColorSensor.BALL.GREEN
            };
        }

        index = 0;
    }

    // Atualiza com valor da limelight (0 a 9)
    public void updateIndex(int ballsInGate) {

        if (ballsInGate < 0) ballsInGate = 0;
        if (ballsInGate > 9) ballsInGate = 9;

        index = ballsInGate;
    }

    // Retorna sempre 3 bolas (sem null)
    public ColorSensor.BALL[] getNext3() {

        ColorSensor.BALL[] result = new ColorSensor.BALL[3];

        for (int i = 0; i < 3; i++) {

            int pos = index + i;

            if (pos < sequence.length) {
                result[i] = sequence[pos];
            } else {
                result[i] = sequence[8]; // fallback fixo
            }
        }

        return result;
    }
}