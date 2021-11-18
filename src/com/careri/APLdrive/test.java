package com.careri.APLdrive;

public class test {
    public static void main(String[] args) {
        double x = 155.399;
        double y;
        y = calc(x);
        System.out.println(x);
        System.out.println(y);

    }
    public static double calc(double x){
        x = x + 1;
        return x;
    }
}
