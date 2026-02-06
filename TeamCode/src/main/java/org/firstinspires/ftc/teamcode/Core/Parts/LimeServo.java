package org.firstinspires.ftc.teamcode.Core.Parts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Algorithms.InterfaceSimple;
import org.firstinspires.ftc.teamcode.Core.Algorithms.PoseIdentifier;

public class LimeServo implements InterfaceSimple {
public Servo ServoLime;

public double cordX, cordY, IdealYaw, distance;

public double currentX, currentY, currentYaw;

public enum Mode{
    BLUE_TAG,
    RED_TAG,
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
    switch (mode){
        case BLUE_TAG:
            cordX = 12;
            cordY = 135;
            break;
        case RED_TAG:
            cordX = 132;
            cordY = 135;
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
        // Ângulo do robô até o alvo
        IdealYaw = Math.toDegrees(
                Math.atan2(cordY - currentY, cordX - currentX)
        );

        // Erro angular normalizado (-180 a 180)
        double erro = normalize(IdealYaw - currentYaw);

        // Converte erro em posição do servo
        double servo = Math.max(0.0, Math.min(1.0,
                0.00163 * erro + 0.20
        ));

        ServoLime.setPosition(servo);

    }
}
