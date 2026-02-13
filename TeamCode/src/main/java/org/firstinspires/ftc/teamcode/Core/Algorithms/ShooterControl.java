package org.firstinspires.ftc.teamcode.Core.Algorithms;

public class ShooterControl implements InterfaceSimple {

     double servoIdeal;

    public double servoA, servoB,servoC;

    public double velA, velB;
     int velIdeal;
    public boolean isBlue;

    public int corretor;

    public double d;



    public void setDistance(double dist){d = dist;}

    public void isBlue(boolean blue){isBlue = blue;}


    public void setServo(double A,double B,double C){servoA = A; servoB = B; servoC = C;}

    public void setVel(double A,double B){velA = A; velB = B;}



    public double getServo(){return servoIdeal;}

    public int getVel(){return velIdeal;}

    @Override
    public void update() {
        servoIdeal = servoA * d * d - servoB * d + servoC;



        if (d > 100) {
            velIdeal = (int) (Math.round((velA * d + velB + 40) / 20.0) * 20);
        } else {
            velIdeal = (int) (Math.round((velA * d + velB-20) / 20.0) * 20);
        }



    }
}


