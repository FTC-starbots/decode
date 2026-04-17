package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Algorithms.SequenceAlgorithm;
import org.firstinspires.ftc.teamcode.Core.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Core.Hardware.Leds;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;

@TeleOp(name = "Sequence Test", group = "Test")
public class SequenceTest extends OpMode {

    SequenceAlgorithm alg = new SequenceAlgorithm();
    Leds leds = new Leds();
    Lime lime = new Lime();

    int id = 21;

    int stableBalls = 0;
    int lastReading = 0;
    int counter = 0;
    final int THRESHOLD = 5;

    @Override
    public void init() {

        leds.start(hardwareMap);

        lime.setup(hardwareMap);
        lime.setOperationMode(Lime.Mode.GATE);

        alg.setId(id);
    }

    @Override
    public void loop() {

        lime.update();

        int currentReading = (int) lime.TotalBalls;

        if (currentReading == lastReading) {
            counter++;
        } else {
            counter = 0;
        }

        if (counter >= THRESHOLD) {
            stableBalls = currentReading;
        }

        lastReading = currentReading;

        if (gamepad1.a) {
            id = 21;
            alg.setId(id);
        }
        if (gamepad1.b) {
            id = 22;
            alg.setId(id);
        }
        if (gamepad1.y) {
            id = 23;
            alg.setId(id);
        }


        alg.updateIndex(stableBalls);

        ColorSensor.BALL[] next3 = alg.getNext3();

        leds.setBallsState(
                safe(next3[0]),
                safe(next3[1]),
                safe(next3[2])
        );

        telemetry.addData("ID", id);
        telemetry.addData("Raw Balls", currentReading);
        telemetry.addData("Stable Balls", stableBalls);
        telemetry.addData("Next1", next3[0]);
        telemetry.addData("Next2", next3[1]);
        telemetry.addData("Next3", next3[2]);
        telemetry.update();
    }

    private ColorSensor.BALL safe(ColorSensor.BALL b) {
        return b == null ? ColorSensor.BALL.NONE : b;
    }
}