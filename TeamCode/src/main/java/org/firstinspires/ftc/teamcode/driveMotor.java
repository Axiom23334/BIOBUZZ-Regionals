package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "driveMotor")
public class driveMotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Starting...");
        telemetry.update();

        DcMotorEx leftMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx leftMotor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        telemetry.addLine("Motors found");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        while (opModeIsActive()) {

            double y = gamepad1.right_stick_y;
            double x = gamepad1.left_stick_y;

            leftMotor1.setPower(y);
            leftMotor2.setPower(x);

            telemetry.addData("motor1 power", y);
            telemetry.addData("motor2 power", x);
            telemetry.update();
        }
    }
}