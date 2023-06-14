package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.drivetrain;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);

        double rotation = 0;

        waitForStart();

        while (!isStopRequested()){

            rotation = gamepad1.right_trigger - gamepad1.left_trigger;
            float factoredY = (float) -0.5 * gamepad1.left_stick_y;
            float factoredX = (float) -0.5 * gamepad1.left_stick_x;

            drivetrain.fieldCentric(factoredY, factoredX, rotation);
            while(gamepad1.a) drivetrain.test();
            telemetry.addData("y", gamepad1.left_stick_y);
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.update();

        }
    }
}
