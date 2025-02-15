package org.firstinspires.ftc.teamcode.opMode.teleop;

import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.HIGH_BAR;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.IDLE;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.INTAKE_SAMPLE;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.INTAKE_SPECIMEN;

import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.CommandSet;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.CommandMap;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.CommandToggler;

import org.firstinspires.ftc.teamcode.common.command.commandStructure.InstantCommand;
import org.firstinspires.ftc.teamcode.common.command.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff;
import org.firstinspires.ftc.teamcode.common.control.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.subassems.RobotAssem;
import org.firstinspires.ftc.teamcode.common.subassems.drive.SwerveModule;

@Config
@TeleOp(name="Teleop v4.0", group="Robot")
public class Teleop_v4_0 extends LinearOpMode {

    private RobotAssem robot;

    private double prevLoop = 0;

//    public static double Kp = 0, Kd = 0, a = 0.95, Kf = 0, KfDead = 0, deadRange = 0;
//    public static double tSetpoint = 0, pSetpoint = 0;
//    double tPrev, pPrev;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        robot = new RobotAssem(this);
        robot.init();
        robot.swerve.usePowerCuts(false);

        CommandSet commandSet = new CommandSet(robot);

        CommandMap.getInstance().mapCommand(
                gamepadEx1.Back::wasJustPressed,
                new InstantCommand(() -> robot.swerve.localizer.resetIMU())
        );

//        High Bar
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.A.wasJustPressed() && (robot.scorer.isAt(IDLE) || robot.scorer.isAt(HIGH_BAR))),
                new CommandToggler(commandSet.lowSpeed(commandSet.extendToHighBar), commandSet.highSpeed(commandSet.retractFromHighBar))
        );

//        Intake Sample
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.B.wasJustPressed() && (robot.scorer.isAt(IDLE) || robot.scorer.isAt(INTAKE_SAMPLE))),
                new CommandToggler(commandSet.lowSpeed(commandSet.extendSubmersible), commandSet.highSpeed(commandSet.retractSubmersible))
        );

//        Intake Specimen
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.X.wasJustPressed() && (robot.scorer.isAt(IDLE) || robot.scorer.isAt(INTAKE_SPECIMEN))),
                new CommandToggler(commandSet.lowSpeed(commandSet.intakeSpecimen), commandSet.highSpeed(commandSet.setArmIdle))
        );

//        High Basket
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.Y.wasJustPressed() && (robot.scorer.isAt(IDLE) || robot.scorer.isAt(HIGH_BASKET))),
                new CommandToggler(commandSet.lowSpeed(commandSet.extendToHighBasket), commandSet.highSpeed(commandSet.retractFromHighBasket))
        );

//        Intake Sample Left
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.LeftBumper.wasJustPressed() && robot.scorer.isAt(INTAKE_SAMPLE)),
                new CommandToggler(commandSet.intakeLeft, commandSet.intakeCenter)
        );

//        Intake Sample Right
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.RightBumper.wasJustPressed() && robot.scorer.isAt(INTAKE_SAMPLE)),
                new CommandToggler(commandSet.intakeRight, commandSet.intakeCenter)
        );

//        Roller Out
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.A.isPressed() && robot.scorer.isIntaking()),
                commandSet.rollerScarf
        );

//        Roller In
        CommandMap.getInstance().mapCommand(
                () -> (gamepadEx1.Y.isPressed() && robot.scorer.isIntaking()),
                commandSet.rollerBarf
        );

//        Roller Stop
        CommandMap.getInstance().mapCommand(
                () -> ((!gamepadEx1.Y.isPressed() && !gamepadEx1.A.isPressed() && robot.scorer.isIntaking())),
                commandSet.rollerStop
        );

//        Manual Control
        CommandMap.getInstance().mapCommand(
                gamepadEx1.Start::wasJustPressed,
                new CommandToggler(commandSet.enableManualArmControl, commandSet.disableManualArmControl)
        );

        CommandMap.getInstance().mapCommand(
                () -> (abs(gamepadEx1.RightJoystick.getX()) > abs(gamepadEx1.RightJoystick.getY()) || gamepadEx1.RightJoystick.getX() == 0),
                new InstantCommand(() -> robot.scorer.arm.pivot.setPower(gamepadEx1.RightJoystick.getX() * 0.5))
        );

        CommandMap.getInstance().mapCommand(
                () -> (abs(gamepadEx1.RightJoystick.getY()) > abs(gamepadEx1.RightJoystick.getX()) || gamepadEx1.RightJoystick.getY() == 0),
                new InstantCommand(() -> robot.scorer.arm.telescope.setPower(gamepadEx1.RightJoystick.getY()))
        );

        waitForStart();

        robot.scorer.setPosition(IDLE);

        while(opModeIsActive()){

            robot.clearBulkCache();

            robot.read();

            Pose drivePose = new Pose(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    (gamepad1.right_trigger - gamepad1.left_trigger)
            );

            gamepadEx1.loop();
            CommandMap.getInstance().loop();

            robot.loop(drivePose);

            robot.write();

            telemetry.addData("arm dm", robot.scorer.arm.getTelemetryAsVector());
            telemetry.addData("arm xy", robot.scorer.arm.getTelemetryAsPoint());
            telemetry.addData("Pose", robot.swerve.localizer.getTelemetry());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - prevLoop));
            prevLoop = loop;

            telemetry.update();
        }
    }
}