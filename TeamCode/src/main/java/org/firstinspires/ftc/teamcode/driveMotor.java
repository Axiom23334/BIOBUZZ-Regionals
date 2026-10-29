package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "driveMotor")
public class driveMotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting...");
        telemetry.update();

        DcMotorEx leftMotor1 = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx leftMotor2 = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx rightMotor1 = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx rightMotor2 = hardwareMap.get(DcMotorEx.class, "backRightMotor");


        telemetry.addLine("Motors found");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        while (opModeIsActive()) {

            double y = gamepad1.right_stick_y;
            double x = gamepad1.left_stick_y;

            leftMotor1.setPower(1);
            leftMotor2.setPower(1);
            rightMotor1.setPower(0.2);
            rightMotor2.setPower(1);


            telemetry.addData("motor1 power", y);
            telemetry.addData("motor2 power", x);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("tick1", leftMotor1.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}