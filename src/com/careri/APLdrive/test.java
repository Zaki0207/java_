package com.careri.APLdrive;

import java.io.IOException;

public class test {
    public static void main(String[] args) throws IOException, InterruptedException {
        APLposition new_posi = new APLposition();
        new_posi.lat = 30.22;
        new_posi.longti = 133.5;
        new_posi.speed = 4;
        new_posi.heading = 30;
        GuidSignal gs = new GuidSignal();
        Joystick js = new Joystick();
        gs.target_angle = 100;
        gs.target_speed = 0;
        js.acc_And_dcc = 0;
        js.angle_offset = 0;
        Main.getProperty();
        while(true){
            new_posi = Main.getPOSI(new_posi, gs, js);
            Thread.sleep(20);
            System.out.println(new_posi.longti + " " + new_posi.lat + " " +new_posi.height + " " +
                    new_posi.speed + " " + new_posi.heading + " ");
        }
    }
}

