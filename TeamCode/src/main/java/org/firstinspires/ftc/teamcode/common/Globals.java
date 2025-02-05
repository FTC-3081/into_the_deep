package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

//    Servo range per radian
    public static final double AXON_MINI_SPR = 0.27 / (Math.PI / 2); // verified
    public static final double AXON_MAX_SPR = 0.27 / (Math.PI / 2); // unverified didnt use any positional maxes
    public static final double AXON_MICRO_SPR = 0.27 / (Math.PI / 2); // unverified didnt use any positional micros

//    Operating Voltage
    public static final double OPERATING_VOLTAGE = 13.0;

//    Swerve
    public static double MAX_DRIVE_SPEED = 0.75, MAX_TURN_SPEED = 0.8;
    public static double MIN_DRIVE_SPEED = 0.3, MIN_TURN_SPEED = 0.35;

    public static boolean USE_POSITION_REMEMBRANCE = true;
    public static double REMEMBRANCE_ZONE = 0.025;
    public static boolean USE_OPTIMIZED_MODULE_ROTATION = true;
    public static boolean USE_FIELD_CENTRIC_DRIVE = true;

    public static double LF_OFFSET = -1.96, RF_OFFSET = -2.57, LB_OFFSET = -3.06, RB_OFFSET = 3.07;

//    Differential
    public static double DIFFY_OFFSET_1 = 78, DIFFY_OFFSET_2 = 16;

    public static double DIFFY_INTAKE_SAMPLE_1 = 70;
    public static double DIFFY_INTAKE_SIDE_1 = 70, DIFFY_INTAKE_SIDE_2 = 45; // Assume Left
    public static double DIFFY_HIGH_BAR_1 = 0, DIFFY_HIGH_BAR_2 = -45;
    public static double DIFFY_HIGH_BASKET_1 = 105;

//    Level
    public static double LEVEL_OFFSET = 168;

    public static double LEVEL_INTAKE_SIDE = 30; // Assume Left
    public static double LEVEL_INTAKE_SPECIMEN = -90;
    public static double LEVEL_HIGH_BAR = -90;

//    Roller
    public static double ROLLER_BARF_TIME = 0.25;

//    Arm
    public static double ARM_X_HIGH_BAR = 200, ARM_Y_HIGH_BAR = 525;
    public static double ARM_X_HIGH_BAR_SNAP = 200, ARM_Y_HIGH_BAR_SNAP = 650;
    public static double ARM_T_HIGH_BASKET = 100, ARM_M_HIGH_BASKET = 875;
    public static double ARM_X_INTAKE_SAMPLE = 500, ARM_Y_INTAKE_SAMPLE = -94;
    public static double ARM_T_INTAKE_SPECIMEN = 149;
//    public static double SNAP_SPECIMEN_TIME = 0.3;
    public static double SNAP_OVERRIDE_POWER = 0.75;
    public static double SNAP_THRESHOLD_VELOCITY = 0.25;
}
