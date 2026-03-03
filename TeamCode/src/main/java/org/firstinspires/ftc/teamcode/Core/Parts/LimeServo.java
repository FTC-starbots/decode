package org.firstinspires.ftc.teamcode.Core.Parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;
import org.firstinspires.ftc.teamcode.Core.Hardware.Lime;

public class LimeServo implements InterfaceSimple {
public Servo ServoLime;
public Lime limelight = new Lime();

public double cordX, cordY, IdealYaw, distance;

public double currentX, currentY, currentYaw;

Mode current_mode;

public enum Mode{
    BLUE_TAG,
    RED_TAG,
    BLUE_GATE,
    RED_GATE,
    OBELISK

}

    public void start(HardwareMap hardware){
        ServoLime = hardware.get(Servo.class, "ServoLime");

    }

    public void setPosition(double position) {
        ServoLime.setPosition(position);
    }


    public void setCurrentPosition(double cordX, double cordY, double cordYaw){
        currentX = cordX;
        currentY = cordY;
        currentYaw = cordYaw;
    }

    public void updateDesiredReference(Mode mode){
        current_mode = mode;
    switch (mode){
        case BLUE_TAG:
            cordX = 12;
            cordY = 135;
            break;
        case RED_TAG:
            cordX = 132;
            cordY = 135;
            break;
        case BLUE_GATE:
            cordX = 6;
            cordY = 91;
            break;
        case RED_GATE:
            cordX = 138;
            cordY = 91;
            break;
        case OBELISK:
            cordX = 72;
            cordY = 143;
            break;
    }
    }

    private double normalize(double ang) {
        return Math.atan2(Math.sin(Math.toRadians(ang)),
                Math.cos(Math.toRadians(ang))) * 180 / Math.PI;
    }
    @Override
    public void update() {
        if (current_mode == Mode.BLUE_TAG || current_mode == Mode.RED_TAG || current_mode == Mode.OBELISK) limelight.setOperationMode(Lime.Mode.TAG);
        else limelight.setOperationMode(Lime.Mode.GATE);


        // Ângulo do robô até o alvo
        IdealYaw = Math.toDegrees(
                Math.atan2(cordY - currentY, cordX - currentX)
        );

        // Erro angular normalizado (-180 a 180)
        double erro = normalize(IdealYaw - currentYaw + 180);

        // Mapeamento linear: -180 → 0 | 0 → 0.5 | 180 → 1
        double servo = (erro + 180.0) / 360.0;

        // Limite físico do servo
        servo = Math.max(0.0, Math.min(1.0, servo));

        ServoLime.setPosition(servo);

    }
}
