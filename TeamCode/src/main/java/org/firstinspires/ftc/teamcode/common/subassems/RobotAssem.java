package org.firstinspires.ftc.teamcode.common.subassems;

import static org.firstinspires.ftc.teamcode.common.Globals.AXON_MINI_SPR;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_OFFSET_1;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_OFFSET_2;
import static org.firstinspires.ftc.teamcode.common.Globals.LB_OFFSET;
import static org.firstinspires.ftc.teamcode.common.Globals.LF_OFFSET;
import static org.firstinspires.ftc.teamcode.common.Globals.LEVEL_OFFSET;
import static org.firstinspires.ftc.teamcode.common.Globals.MIN_VOLTAGE;
import static org.firstinspires.ftc.teamcode.common.Globals.RB_OFFSET;
import static org.firstinspires.ftc.teamcode.common.Globals.RF_OFFSET;
import static java.lang.Math.PI;
import static java.lang.Math.max;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.DirectionalFF;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.ExtensionFF;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.PivotFF;
import org.firstinspires.ftc.teamcode.common.control.geometry.Constraints;
import org.firstinspires.ftc.teamcode.common.control.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.control.geometry.Vector;
import org.firstinspires.ftc.teamcode.common.control.motionProfiles.TrapezoidMP;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.LinearPID;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.WraparoundPID;
import org.firstinspires.ftc.teamcode.common.customHardware.AnalogEncoder;
import org.firstinspires.ftc.teamcode.common.customHardware.QuadratureEncoder;
import org.firstinspires.ftc.teamcode.common.customHardware.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.subassems.drive.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.common.subassems.drive.SwerveModule;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.DifferentialAssem;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.ExtensionAssem;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.MiniRevoluteAssem;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.MiniRollerAssem;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.RevoluteAssem;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.ArmAssem;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem;

import java.util.List;

public class RobotAssem implements Subassem{

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

//    Top level subassemblies
    public SwerveDrivetrain swerve;
    public ScoringAssem scorer;

//    Low level subassemblies (not local to reference each other, private to still only be accessed through top level)
    private ArmAssem arm;
    private ExtensionAssem telescope;
    private RevoluteAssem pivot;
    private DifferentialAssem diffy;
    private MiniRevoluteAssem level;
    private MiniRollerAssem roller;

//    Voltage
    private VoltageSensor voltageSensor;
    private static double voltage;

//    Bulk Read Stuff
    private List<LynxModule> hubs;

//    Constructor and stuff
    public RobotAssem(LinearOpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
    }

    public void init(){
//        Motors
        DcMotorEx lfDrive, rfDrive, lbDrive, rbDrive, lTelescope, rTelescope, fPivot, bPivot;

        lfDrive = hardwareMap.get(DcMotorEx.class, "lfDrive");
        rfDrive = hardwareMap.get(DcMotorEx.class, "rfDrive");
        lbDrive = hardwareMap.get(DcMotorEx.class, "lbDrive");
        rbDrive = hardwareMap.get(DcMotorEx.class, "rbDrive");
        lTelescope = hardwareMap.get(DcMotorEx.class, "lTelescope");
        rTelescope = hardwareMap.get(DcMotorEx.class, "rTelescope");
        fPivot = hardwareMap.get(DcMotorEx.class, "fPivot");
        bPivot = hardwareMap.get(DcMotorEx.class, "bPivot");

        lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rfDrive.setDirection(DcMotorEx.Direction.REVERSE);
        lbDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rbDrive.setDirection(DcMotorEx.Direction.REVERSE);
        lTelescope.setDirection(DcMotorEx.Direction.REVERSE);
        rTelescope.setDirection(DcMotorEx.Direction.FORWARD);
        fPivot.setDirection(DcMotorEx.Direction.REVERSE);
        bPivot.setDirection(DcMotorEx.Direction.FORWARD);

        lfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lTelescope.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rTelescope.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fPivot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bPivot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lfDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lTelescope.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rTelescope.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        Continuous Rotation Servos
        CRServo lfSwerve, rfSwerve, lbSwerve, rbSwerve, rollerServo;

        lfSwerve = hardwareMap.get(CRServo.class, "lfSwerve");
        rfSwerve = hardwareMap.get(CRServo.class, "rfSwerve");
        lbSwerve = hardwareMap.get(CRServo.class, "lbSwerve");
        rbSwerve = hardwareMap.get(CRServo.class, "rbSwerve");
        rollerServo = hardwareMap.get(CRServo.class, "roller");

        lfSwerve.setDirection(DcMotorSimple.Direction.REVERSE);
        rfSwerve.setDirection(DcMotorSimple.Direction.REVERSE);
        lbSwerve.setDirection(DcMotorSimple.Direction.REVERSE);
        rbSwerve.setDirection(DcMotorSimple.Direction.FORWARD);
        rollerServo.setDirection(DcMotorSimple.Direction.FORWARD);

//        Positional Servos
        Servo lDifferential, rDifferential, levelServo;

        lDifferential = hardwareMap.get(Servo.class, "lDifferential");
        rDifferential = hardwareMap.get(Servo.class, "rDifferential");
        levelServo = hardwareMap.get(Servo.class, "level");

        lfSwerve.setDirection(DcMotorSimple.Direction.FORWARD);
        rfSwerve.setDirection(DcMotorSimple.Direction.FORWARD);
        lbSwerve.setDirection(DcMotorSimple.Direction.FORWARD);

//        Analog Encoders
        AnalogInput lfEncoder, rfEncoder, lbEncoder, rbEncoder, pivotEncoder;

        lfEncoder = hardwareMap.get(AnalogInput.class, "lfEncoder");
        rfEncoder = hardwareMap.get(AnalogInput.class, "rfEncoder");
        lbEncoder = hardwareMap.get(AnalogInput.class, "lbEncoder");
        rbEncoder = hardwareMap.get(AnalogInput.class, "rbEncoder");
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");

        TwoWheelLocalizer localizer = hardwareMap.get(TwoWheelLocalizer.class,"localizer");
        localizer.setOffsets(-97.5, 94.5);
        localizer.setEncoderDirections(TwoWheelLocalizer.EncoderDirection.FORWARD, TwoWheelLocalizer.EncoderDirection.REVERSE);
        localizer.setEncoderResolution(8192 / (35 * PI));
        localizer.resetPosAndIMU();

//        Voltage Sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

//        Subassems
        SwerveModule lfModule = new SwerveModule(
                "lf",
                lfDrive, lfSwerve, lfEncoder,
                new WraparoundPID(0.3, 0, 0.02, new DirectionalFF(0.07, 0.04, new Range(toRadians(3.5))), 0.9, 0),
                new Range(toRadians(45)),
                LF_OFFSET
        );
        SwerveModule rfModule = new SwerveModule(
                "rf",
                rfDrive, rfSwerve, rfEncoder,
                new WraparoundPID(0.3, 0, 0.02, new DirectionalFF(0.09, 0.06, new Range(toRadians(3.5))), 0.9, 0),
                new Range(toRadians(45)),
                RF_OFFSET
        );
        SwerveModule lbModule = new SwerveModule(
                "lb",
                lbDrive, lbSwerve, lbEncoder,
                new WraparoundPID(0.3, 0, 0.02, new DirectionalFF(0.08, 0.05, new Range(toRadians(3.5))), 0.9, 0),
                new Range(toRadians(45)),
                LB_OFFSET
        );
        SwerveModule rbModule = new SwerveModule(
                "rb", rbDrive, rbSwerve, rbEncoder,
                new WraparoundPID(0.3, 0, 0.02, new DirectionalFF(0.08, 0.06, new Range(toRadians(3.5))), 0.9, 0),
                new Range(toRadians(45)),
                RB_OFFSET
        );
        swerve = new SwerveDrivetrain(
                lfModule, rfModule, lbModule, rbModule,
                localizer,
                new LinearPID(0.002, 0, 0.0002, 0.9, 0),
                new LinearPID(0.002, 0, 0.0002, 0.9, 0),
                new WraparoundPID(0.35, 0, 0.004, 0.95, 0),
                new DirectionalFF(0.11, 0.03, new Range(0.008)),
                new Range(7.5), new Range(toRadians(2.5)),
                228, 228
        );
        diffy = new DifferentialAssem(
                lDifferential, rDifferential,
                toRadians(DIFFY_OFFSET_1), toRadians(DIFFY_OFFSET_2), AXON_MINI_SPR,
                new Range(0, toRadians(180)), new Range(toRadians(-90), toRadians(90))
        );
        level = new MiniRevoluteAssem(
                toRadians(LEVEL_OFFSET), AXON_MINI_SPR,
                new Range(toRadians(-90), toRadians(90)),
                levelServo
        );
        roller = new MiniRollerAssem(
                rollerServo
        );
        telescope = new ExtensionAssem(
                new LinearPID(
                        0.0018, 0, 0.00013,
                        new ExtensionFF(0.18, 0.2, 0.25, -0.02, () -> pivot.getPosition(), () -> telescope.getPercentExtension(), 0.12, new Range(3)),
                        0.95, 0), // ff (-0.16, 0.16), (-0.18, 0.18), (-0.03, 0.25)
                new TrapezoidMP(new Constraints(2400, 30000), new Constraints(2400, 30000, 15000)),
                new Range(0, 295 * 2), new Range(5), new Range(2.5), 48 * 2 * 2,
                () -> pivot.getPosition(),
                new QuadratureEncoder(lfDrive, 28, (double)85/12),
                lTelescope, rTelescope
        );
        pivot = new RevoluteAssem(
                new LinearPID(
                        0.35, 0, 0.03,
                        new PivotFF(0.15, 0.33, 0.08, () -> pivot.getPosition(), () -> telescope.getPercentExtension(), 0.06, new Range(toRadians(1))),
                        0.9, 0), // ff (-0.09, 0.09), (-0.15, x), (-0.34, y)
                new TrapezoidMP(new Constraints(toRadians(350), toRadians(5000))),
                new Range(-2.1, 2.1), new Range(toRadians(3)), new Range(0.01),
                new AnalogEncoder(pivotEncoder, 2.16, 1, toRadians(181), 0),
                fPivot, bPivot
        );
        arm = new ArmAssem(
                telescope, pivot,
                new Vector(Math.PI/2, 260.5)
        );
        arm.zero();
        scorer = new ScoringAssem(
                arm, diffy, level, roller
        );

//        Bulk Read Stuff
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();
    }

//    Periodic methods

    public void read() {
        voltage = voltageSensor.getVoltage();
        swerve.read();
        scorer.read();
    }

    public void loop() {
        swerve.loop();
        scorer.loop();
    }

    public void loop(Pose drivePose) {
        swerve.loop(drivePose);
        scorer.loop();
    }

    public void write() {
        swerve.write();
        scorer.write();
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public static double getVoltage(){
        return max(voltage, MIN_VOLTAGE);
    }
}