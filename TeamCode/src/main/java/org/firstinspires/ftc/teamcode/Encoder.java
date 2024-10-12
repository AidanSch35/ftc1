package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder extends DcMotorExComposition{
    private DcMotorEx encoder;
    private int pos = 0;
    public Encoder(DcMotorEx encoder) {
        super();
        this.encoder = encoder;
    }


    public void encoder(DcMotorEx encoder){
        this.encoder = encoder;
        return;
    }
    public void updateEncoder(){
        pos = encoder.getCurrentPosition();
    }


    public void resetEncoder(){
        pos = 0;
    }
    public int getCurrentPosition(){
        return pos;
    }
    public double getVelocity(){
        return encoder.getVelocity();
    }



}
