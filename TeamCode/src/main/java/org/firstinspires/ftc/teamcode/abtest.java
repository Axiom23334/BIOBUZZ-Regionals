package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/*
 * This OpMode drives two continuous rotation servos using the gamepad joysticks.
 * Left stick (Y-axis)  -> controls servo1 ("left")
 * Right stick (Y-axis) -> controls servo2 ("right")
 *
 * NOTE: The names "left" and "right" below must exactly match the names
 * you gave these servos in your robot configuration file on the Driver Station.
 */
@TeleOp(name = "Concept: Scan Servo", group = "Concept")
public class abtest extends LinearOpMode {

    // Define class members
    CRServo servo1;
    CRServo servo2;

    @Override
    public void runOpMode() {

        // Connect to servos (make sure these names match your configuration exactly)
        servo1 = hardwareMap.get(CRServo.class, "left");
        servo2 = hardwareMap.get(CRServo.class, "right");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();

        // Scan servos till stop pressed.
        while (opModeIsActive()) {

            // Gamepad Y-axis is inverted by default (up = negative), so flip the sign
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            servo1.setPower(leftPower);
            servo2.setPower(rightPower);

            telemetry.addData("Left Stick Power", leftPower);
            telemetry.addData("Right Stick Power", rightPower);
            telemetry.update();
        }

        // Signal done
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
