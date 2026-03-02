package org.firstinspires.ftc.teamcode.Core.Algorithms;

public class ShooterControl implements InterfaceSimple {

     double servoIdeal;

    public double servoA, servoB;

    public double velA, velB, AllBallsConstant;
     int velIdeal;
    public boolean isBlue;

    public int corretor;

    public double d;

    int ShootType;


    public void setDistance(double dist){d = dist;}

    public void isBlue(boolean blue){isBlue = blue;}


    public void setServo(double A,double B){servoA = A; servoB = B;}

    public void setVel(double A,double B, double AllBalls){velA = A; velB = B;AllBallsConstant = AllBalls;}

    public void setShootType(int shoot){ShootType = shoot;}

    public double getServo(){return servoIdeal;}

    public int getVel(){return velIdeal;}

    @Override
    public void update() {

        if (ShootType == 1){
            velIdeal = (int) (Math.round((velA * d + velB + AllBallsConstant) / 20.0) * 20);
        }else {
            velIdeal = (int) (Math.round((velA * d + velB) / 20.0) * 20);
        }
        if (d > 100) {
            servoIdeal = -(servoA - 0.0011) * d + servoB;
        } else {
            servoIdeal = -(servoA) * d + servoB;

        }

        servoIdeal = Math.min(1, servoIdeal);



    }
}


