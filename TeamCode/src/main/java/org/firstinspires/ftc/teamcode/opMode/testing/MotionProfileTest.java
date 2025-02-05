package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.control.geometry.Constraints;
import org.firstinspires.ftc.teamcode.common.control.motionProfiles.TrapezoidMP;


@Disabled
//@Config
@TeleOp(name="Motion Profile Test", group="Robot")
public class MotionProfileTest extends LinearOpMode {

    Constraints extend = new Constraints(200, 150);
    Constraints retract = new Constraints(200, 100, 50);
    private final TrapezoidMP motionProfile = new TrapezoidMP(extend, retract);
    public static double initState = 0;
    public static double finalSetpoint = 500;
    double prevInitState = 0;
    double prevFinalSetpoint = 500;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        motionProfile.setProfile(initState, finalSetpoint);

        while(opModeIsActive()){

            double position = motionProfile.getInstantSetpoint();
            if(position == initState || initState != prevInitState || finalSetpoint != prevFinalSetpoint){
                motionProfile.setProfile(initState, finalSetpoint);
            } else if(position == finalSetpoint){
                motionProfile.setProfile(finalSetpoint, initState);
            }
            prevInitState = initState;
            prevFinalSetpoint = finalSetpoint;

            telemetry.addData("times", motionProfile.getTimes());
            telemetry.addData("position", position);
            telemetry.addData("upper", finalSetpoint + 100);
            telemetry.addData("lower", initState - 100);
            telemetry.update();
        }
    }
}