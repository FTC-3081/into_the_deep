package org.firstinspires.ftc.teamcode.opMode.auto;

import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.HIGH_BAR;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.IDLE;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.INTAKE_SAMPLE;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.INTAKE_SPECIMEN;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.CommandSet;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.CommandMap;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.CommandToggler;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.InstantCommand;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.PauseUntilCommand;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.SequenceCommand;
import org.firstinspires.ftc.teamcode.common.command.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.common.control.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.subassems.RobotAssem;


@Config
@Autonomous(name="Auto_v4.0", group="Robot")
public class Auto_v4_0 extends LinearOpMode {

    private RobotAssem robot;

    private double prevLoop = 0;

    public static double Kp = 0, Kd = 0, a = 0, Kf = 0.11, KfDead = 0.06, deadRange = 0.007;
    public static double xSetpoint, ySetpoint, tSetpoint;

    @Override
    public void runOpMode() {

//        Globals.USE_POSITION_REMEMBRANCE = false;
        Globals.USE_FIELD_CENTRIC_DRIVE = true;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotAssem(this);
        robot.init();

        CommandSet commandSet = new CommandSet(robot);

        CommandMap.getInstance().mapCommand(
                () -> false,
                new SequenceCommand( true,
                        commandSet.driveToPosition(-300, 600, 0),
                        commandSet.scoreSpecimen,
                        commandSet.driveToPosition(0, 0, 0),
                        commandSet.scoreSpecimen,
                        commandSet.driveToPosition(0, 0, 0),
                        commandSet.scoreSpecimen
                )
        );

        waitForStart();

        robot.scorer.setPosition(IDLE);

        while(opModeIsActive()){

            robot.clearBulkCache();

            robot.read();

//            CommandMap.getInstance().loop();

            robot.swerve.setPosition(new Pose(xSetpoint, ySetpoint, toRadians(tSetpoint)));

            robot.swerve.feedforward.set(Kf);
            robot.swerve.feedforward.set(KfDead, new Range(deadRange));

//            robot.swerve.tController.setGains(Kp, 0, Kd);
//            robot.swerve.tController.filter.set(a);

            robot.swerve.xController.setGains(Kp, 0, Kd);
            robot.swerve.xController.filter.set(a);
            robot.swerve.yController.setGains(Kp, 0, Kd);
            robot.swerve.yController.filter.set(a);

            robot.loop();

            robot.write();

            telemetry.addData("arm dm", robot.scorer.arm.getTelemetryAsVector());
            telemetry.addData("arm xy", robot.scorer.arm.getTelemetryAsPoint());
            telemetry.addData("xState", robot.swerve.localizer.getPosition().x);
            telemetry.addData("yState", robot.swerve.localizer.getPosition().y);
            telemetry.addData("tState", toDegrees(robot.swerve.localizer.getPosition().t));
            telemetry.addData("xSetpoint", xSetpoint);
            telemetry.addData("ySetpoint", ySetpoint);
            telemetry.addData("tSetpoint", tSetpoint);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - prevLoop));
            prevLoop = loop;
            telemetry.update();
        }
    }
}