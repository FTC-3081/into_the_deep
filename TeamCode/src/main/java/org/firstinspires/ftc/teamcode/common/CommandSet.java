package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Globals.ARM_Y_HIGH_BAR_SNAP;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_INTAKE_SAMPLE_1;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_INTAKE_SIDE_1;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_INTAKE_SIDE_2;
import static org.firstinspires.ftc.teamcode.common.Globals.LEVEL_INTAKE_SIDE;
import static org.firstinspires.ftc.teamcode.common.Globals.MAX_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.MAX_TURN_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.MIN_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.MIN_TURN_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.SNAP_OVERRIDE_POWER;
import static org.firstinspires.ftc.teamcode.common.Globals.ROLLER_BARF_TIME;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.HIGH_BAR;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.HIGH_BAR_SNAP;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.IDLE;
import static org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem.Setpoint.INTAKE_SPECIMEN;

import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.common.command.commandStructure.Command;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.InstantCommand;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.ParallelCommand;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.ParallelRaceCommand;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.PauseUntilCommand;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.SequenceCommand;
import org.firstinspires.ftc.teamcode.common.command.commandStructure.TimedPauseCommand;
import org.firstinspires.ftc.teamcode.common.control.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.subassems.RobotAssem;
import org.firstinspires.ftc.teamcode.common.subassems.scoring.ScoringAssem;

public class CommandSet {

    RobotAssem robot;

    public CommandSet(RobotAssem robot){
        this.robot = robot;
    }


    public final SequenceCommand example = new SequenceCommand(
            new InstantCommand(() -> robot.scorer.setPosition(HIGH_BASKET)),
            new PauseUntilCommand(() -> robot.scorer.arm.telescope.isAtPosition()),
            new InstantCommand(() -> robot.scorer.roller.barf()),
            new TimedPauseCommand(ROLLER_BARF_TIME),
            new InstantCommand(() -> robot.scorer.setPosition(IDLE))
    );


//    Roller
    public final InstantCommand rollerBarf = new InstantCommand(
            () -> robot.scorer.roller.barf()
    );

    public final InstantCommand rollerScarf = new InstantCommand(
            () -> robot.scorer.roller.scarf()
    );

    public final InstantCommand rollerStop = new InstantCommand(
            () -> robot.scorer.roller.stop()
    );



//    Arm / Scoring
    public final InstantCommand retractTelescope = new InstantCommand(
        () -> robot.scorer.arm.telescope.retract()
    );

    public final InstantCommand setArmIdle = new InstantCommand(
            () -> robot.scorer.setPosition(ScoringAssem.Setpoint.IDLE)
    );

    public final InstantCommand extendToHighBasket = new InstantCommand(
            () -> robot.scorer.setPosition(HIGH_BASKET)
    );

    public final SequenceCommand retractFromHighBasket = new SequenceCommand(
            rollerBarf,
            new TimedPauseCommand(ROLLER_BARF_TIME),
            new ParallelCommand(
                    rollerStop,
                    setArmIdle,
                    retractTelescope
            )

    );

    public final InstantCommand extendToHighBar = new InstantCommand(
            () -> robot.scorer.setPosition(HIGH_BAR)
    );

    public final SequenceCommand retractFromHighBar = new SequenceCommand(
            new InstantCommand(() -> robot.scorer.arm.telescope.enableMP(false)),
            new InstantCommand(() -> robot.scorer.arm.telescope.setPowerRange(new Range(SNAP_OVERRIDE_POWER, 1))),
            new InstantCommand(() -> robot.scorer.setPosition(HIGH_BAR_SNAP)),
            new ParallelRaceCommand(
                    new PauseUntilCommand(() -> robot.scorer.arm.isStopped()), //(robot.scorer.arm.isStopped() && robot.scorer.arm.getPositionAsPoint().y > ARM_Y_HIGH_BAR + 10)),
                    new PauseUntilCommand(() -> (robot.scorer.arm.getPositionAsPoint().y > ARM_Y_HIGH_BAR_SNAP))
                    ),
            new InstantCommand(() -> robot.scorer.arm.telescope.resetPowerRange()),
            new InstantCommand(() -> robot.scorer.arm.telescope.enableMP(true)),
            rollerBarf,
            new ParallelCommand(
                    new SequenceCommand(
                            new TimedPauseCommand(ROLLER_BARF_TIME),
                            rollerStop,
                            setArmIdle,
                            retractTelescope
                    ),
                    new InstantCommand(() -> robot.scorer.arm.setPosition(robot.scorer.arm.offset))
            )
    );

    public final ParallelCommand intakeLeft = new ParallelCommand(
            new InstantCommand(() -> robot.scorer.diffy.setPosition(toRadians(DIFFY_INTAKE_SIDE_1), toRadians(DIFFY_INTAKE_SIDE_2))),
            new InstantCommand(() -> robot.scorer.level.setPosition(toRadians(LEVEL_INTAKE_SIDE)))
    );

    public final ParallelCommand intakeRight = new ParallelCommand(
            new InstantCommand(() -> robot.scorer.diffy.setPosition(toRadians(DIFFY_INTAKE_SIDE_1), toRadians(-DIFFY_INTAKE_SIDE_2))),
            new InstantCommand(() -> robot.scorer.level.setPosition(toRadians(-LEVEL_INTAKE_SIDE)))
    );

    public final ParallelCommand intakeCenter = new ParallelCommand(
            new InstantCommand(() -> robot.scorer.diffy.setPosition(toRadians(DIFFY_INTAKE_SAMPLE_1), 0)),
            new InstantCommand(() -> robot.scorer.level.setPosition(0))
    );

    public final SequenceCommand extendSubmersible = new SequenceCommand(
            new InstantCommand(() -> robot.scorer.setDiffyLevel(true)),
            new InstantCommand(() -> robot.scorer.setPosition(ScoringAssem.Setpoint.PRE_INTAKE)),
            new PauseUntilCommand(() -> robot.scorer.arm.pivot.isAtPosition()),
            new TimedPauseCommand(0.5),
            new InstantCommand(() -> robot.scorer.setDiffyLevel(false)),
            new InstantCommand(() -> robot.scorer.setPosition(ScoringAssem.Setpoint.INTAKE_SAMPLE))
    );

    public final SequenceCommand retractSubmersible = new SequenceCommand(
            new InstantCommand(() -> robot.scorer.setDiffyLevel(true)),
            setArmIdle,
            retractTelescope,
            new PauseUntilCommand(() -> robot.scorer.arm.isAtPosition()),
            new InstantCommand(() -> robot.scorer.setDiffyLevel(false)),
            setArmIdle
    );

    public final InstantCommand intakeSpecimen = new InstantCommand(
            () -> robot.scorer.setPosition(INTAKE_SPECIMEN)
    );

//    Drive Speed
    public final InstantCommand setDriveSpeedHigh = new InstantCommand(
            () -> robot.swerve.setDriveSpeed(MAX_DRIVE_SPEED, MAX_TURN_SPEED)
    );

    public final InstantCommand setDriveSpeedLow = new InstantCommand(
            () -> robot.swerve.setDriveSpeed(MIN_DRIVE_SPEED, MIN_TURN_SPEED)
    );

    public ParallelCommand lowSpeed(Command command){
        return new ParallelCommand(setDriveSpeedLow, command);
    }

    public ParallelCommand highSpeed(Command command){
        return new ParallelCommand(setDriveSpeedHigh, command);
    }

    public SequenceCommand driveToPosition(double x, double y, double t){
        return new SequenceCommand(
                new InstantCommand(() -> robot.swerve.setPosition(new Pose(x, y, toRadians(t)))),
                new PauseUntilCommand(() -> robot.swerve.isAtPosition())
        );
    }

    public SequenceCommand scoreSpecimen = new SequenceCommand(
            extendToHighBar,
            new PauseUntilCommand(() -> robot.scorer.arm.isAtPosition()),
            retractFromHighBar,
            new PauseUntilCommand(() -> robot.scorer.arm.isAtPosition())
    );

    public SequenceCommand scoreSample = new SequenceCommand(
            extendToHighBasket,
            new PauseUntilCommand(() -> robot.scorer.arm.isAtPosition()),
            retractFromHighBasket,
            new PauseUntilCommand(() -> robot.scorer.arm.isAtPosition())
    );
}
