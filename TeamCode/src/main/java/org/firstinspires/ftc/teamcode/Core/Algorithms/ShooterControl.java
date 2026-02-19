package org.firstinspires.ftc.teamcode.Core.Algorithms;

public class ShooterControl implements InterfaceSimple {

     double servoIdeal;

    public double servoA, servoB;

    public double velA, velB;
     int velIdeal;
    public boolean isBlue;

    public int corretor;

    public double d;



    public void setDistance(double dist){d = dist;}

    public void isBlue(boolean blue){isBlue = blue;}


    public void setServo(double A,double B){servoA = A; servoB = B;}

    public void setVel(double A,double B){velA = A; velB = B;}



    public double getServo(){return servoIdeal;}

    public int getVel(){return velIdeal;}

    @Override
    public void update() {

        if (d > 100) {
            servoIdeal = -(servoA - 0.0011) * d + servoB;
            velIdeal = (int) (Math.round((velA * d + velB ) / 20.0) * 20);
        } else {
            servoIdeal = -(servoA) * d + servoB;
            velIdeal = (int) (Math.round((velA * d + velB) / 20.0) * 20);
        }



    }
}


