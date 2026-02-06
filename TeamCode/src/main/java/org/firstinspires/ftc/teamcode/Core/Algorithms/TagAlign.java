package org.firstinspires.ftc.teamcode.Core.Algorithms;

import org.firstinspires.ftc.teamcode.Core.Parts.LimeServo;

public class TagAlign {
   public boolean validation = false;
    public double width = 0;
    public double pixel = 0;
    LimeServo servo = new LimeServo();




    public void setValues(double LIMEpix,double CAMpix, boolean LIMEstate, boolean CAMstate, double LIMEwidth, double CAMwidth){
            if (CAMstate){
                validation = true;
                width = CAMwidth;
                pixel = CAMpix;
               // servo.isLimeAllianceTag(false);
            } else if (LIMEstate) {
                validation = true;
                width = LIMEwidth;
                pixel = LIMEpix;
             //   servo.isLimeAllianceTag(true);
            }else {
                validation = false;
                width = 0;
                pixel = 0;
               // servo.isLimeAllianceTag(true);

            }

    }

}
