package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Hive {
    public double velocity;
    public String color;
    public boolean isUp;
    public List<Fiducial> fiducials;
    public String name;

    public Hive(){
        velocity = 0;
        color = "";
        isUp = false;  //bc dad never looks on the bright side
        fiducials = new ArrayList<Fiducial>();
        name = "";
    }

    public Hive(String color, String name, List<Integer> fiducialIds) {
        this.color = color;
        this.name = name;
        this.fiducials = new ArrayList<Fiducial>();
        for (Integer id : fiducialIds) {
            this.fiducials.add(new Fiducial(id));
        }
    }
}
