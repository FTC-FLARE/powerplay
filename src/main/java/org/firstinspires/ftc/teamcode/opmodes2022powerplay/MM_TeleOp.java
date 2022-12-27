package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;

@TeleOp(name="TeleOp", group="MM")
public class MM_TeleOp extends MM_OpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Please wait for Initialization");
        telemetry.update();
        robot.init();
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateController();
            robot.drivetrain.driveWithSticks();
            robot.slide.driverControl();
            robot.collector.runCollector();
            telemetry.update();
        }
    }

    private void updateController() {
        try{
            gamepad1Prior.copy(gamepad1Current);
            gamepad1Current.copy(gamepad1);
            gamepad2Prior.copy(gamepad2Current);
            gamepad2Current.copy(gamepad2);
        } catch(RobotCoreException e){}
    }
}
