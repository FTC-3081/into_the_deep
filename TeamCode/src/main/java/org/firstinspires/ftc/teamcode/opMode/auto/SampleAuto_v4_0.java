package org.firstinspires.ftc.teamcode.opMode.auto;

import static org.firstinspires.ftc.teamcode.common.Globals.SAMPLE_AUTO_T_1;
import static org.firstinspires.ftc.teamcode.common.Globals.SAMPLE_AUTO_X_1;
import static org.firstinspires.ftc.teamcode.common.Globals.SAMPLE_AUTO_Y_1;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.IDLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.CommandSet;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.CommandMap;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.SequenceCommand;
import org.firstinspires.ftc.teamcode.common.subassems.RobotAssem;

@Config
@Autonomous(name="Sample Auto v4.0", group="Robot")
public class SampleAuto_v4_0 extends LinearOpMode {

    private RobotAssem robot;

    private double prevLoop = 0;

//    public static double Kp = 0.002, Kd = 0.00015, a = 0.9, Kf = 0.11, KfDead = 0.06, deadRange = 0.007;
//    public static double xSetpoint, ySetpoint, tSetpoint;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotAssem(this);
        robot.init();
        robot.swerve.usePowerCuts(true);
        robot.swerve.useFieldCentricDrive(true);

        CommandSet commandSet = new CommandSet(robot);

        CommandMap.getInstance().mapCommand(
                () -> false,
                new SequenceCommand( true,
                        commandSet.driveToPosition(SAMPLE_AUTO_X_1, SAMPLE_AUTO_Y_1, SAMPLE_AUTO_T_1),
                        commandSet.scoreSample
                )
        );

        waitForStart();

        robot.scorer.setPosition(IDLE);

        while(opModeIsActive()){

            robot.clearBulkCache();

            robot.read();

            CommandMap.getInstance().loop();

//            robot.swerve.setPosition(new Pose(xSetpoint, ySetpoint, toRadians(tSetpoint)));
//
//            robot.swerve.feedforward.set(Kf);
//            robot.swerve.feedforward.set(KfDead, new Range(deadRange));

//            robot.swerve.tController.setGains(Kp, 0, Kd);
//            robot.swerve.tController.filter.set(a);

//            robot.swerve.xController.setGains(Kp, 0, Kd);
//            robot.swerve.xController.filter.set(a);
//            robot.swerve.yController.setGains(Kp, 0, Kd);
//            robot.swerve.yController.filter.set(a);

            robot.loop();

            robot.write();

            telemetry.addData("arm dm", robot.scorer.arm.getTelemetryAsVector());
            telemetry.addData("arm xy", robot.scorer.arm.getTelemetryAsPoint());
            telemetry.addData("pose", robot.swerve.localizer.getTelemetry());
            telemetry.addData("Drivetrain@Position", robot.swerve.isAtPosition());
            telemetry.addData("Modules@Position", robot.swerve.modulesAreAtPosition());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - prevLoop));
            prevLoop = loop;

            telemetry.update();
        }
    }
}