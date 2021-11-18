package com.careri.APLdrive;

public class Joystick {
    double acc_And_dcc; // -1~1
    double angle_offset;

    public double getAngle_offset() {
        return angle_offset;
    }

    public void setAngle_offset(double angle_offset) {
        this.angle_offset = angle_offset;
    }

    public double getAcc_And_dcc() {
        return acc_And_dcc;
    }

    public void setAcc_And_dcc(double acc_And_dcc) {
        this.acc_And_dcc = acc_And_dcc;
    }
}
