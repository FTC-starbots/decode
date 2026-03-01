package org.firstinspires.ftc.teamcode.Core.Algorithms;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorRPMReading implements InterfaceSimple{
double lastPosition = 0;
double lastTime = 0;
double rpm = 0;
ElapsedTime timerRPM = new ElapsedTime();
double ticksPerRev = 560; // ajuste depois

public void init() {
    lastPosition = motor.getCurrentPosition();
    timerRPM.reset();
    lastTime = timerRPM.seconds();
}

  @Override
  public void update(double pos) {
    double currentTime = timerRPM.seconds();

    // janela de 0.1s (100ms)
    if (currentTime - lastTime >= 0.1) {

        double currentPosition = pos;

        double deltaTicks = currentPosition - lastPosition;
        double deltaTime = currentTime - lastTime;

        double rps = deltaTicks / ticksPerRev / deltaTime;
        rpm = rps * 60.0;

        lastPosition = currentPosition;
        lastTime = currentTime;
  }
}
