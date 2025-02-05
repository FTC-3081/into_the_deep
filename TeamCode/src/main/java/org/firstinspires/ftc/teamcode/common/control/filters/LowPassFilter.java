package org.firstinspires.ftc.teamcode.common.control.filters;

public class LowPassFilter{

    double prevFilter = 0;
    double a;

    public LowPassFilter(double a){
        set(a);
    }

    public double loop(double currentFilter){
        currentFilter = (prevFilter * a) + (currentFilter * (1 - a));
        prevFilter = currentFilter;
        return currentFilter;
    }

    public void set(double a){
        if(a >= 0 && a <= 1) this.a = a;
        else this.a = 0;
    }

    public void reset(){
        prevFilter = 0;
    }
}
