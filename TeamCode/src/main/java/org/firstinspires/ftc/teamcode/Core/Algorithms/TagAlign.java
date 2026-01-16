package org.firstinspires.ftc.teamcode.Core.Algorithms;

public class TagAlign {
   public boolean validation;
    public double width;
    public double pixel;

    public void setValues(double LIMEpix,double CAMpix, boolean LIMEstate, boolean CAMstate, double LIMEwidth, double CAMwidth){
            if (CAMstate){
                validation = true;
                width = CAMwidth;;
                pixel = CAMpix;
            } else if (LIMEstate) {
                validation = true;
                width = LIMEwidth;
                pixel = LIMEpix;
            }else {
                validation = false;
                width = 0;
                pixel = 0;
            }

    }

}
