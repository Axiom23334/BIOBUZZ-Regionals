package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Utility: Test Diffy Encoders", group = "Utility")
public class DiffyEncoderTest extends LinearOpMode {

    public static double TICKS_PER_REV = 6260.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Telemetry for both Phone Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Diffy Drive Motors from hardwareMap
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        telemetry.addLine("Diffy Encoder Test Initialized.");
        telemetry.addLine("Press START to read encoder ticks and calculated pod angles.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Read raw encoder ticks from all four motors
            double frTicks = frontRightMotor.getCurrentPosition();
            double brTicks = backRightMotor.getCurrentPosition();
            double flTicks = frontLeftMotor.getCurrentPosition();
            double blTicks = backLeftMotor.getCurrentPosition();

            // Read motor velocities in ticks/second
            double frVel = frontRightMotor.getVelocity();
            double brVel = backRightMotor.getVelocity();
            double flVel = frontLeftMotor.getVelocity();
            double blVel = backLeftMotor.getVelocity();

            // Calculate Diffy Pod angles matching diffyCode math:
            // Pod 1 angle derived from backRightMotor (leftMotor2 in diffyCode)
            double pod1Rad = ((brTicks / TICKS_PER_REV) * Math.PI + Math.PI / 2.0);
            double pod1Deg = Math.toDegrees(pod1Rad);

            // Pod 2 angle derived from frontRightMotor (leftMotor1 in diffyCode)
            double pod2Rad = ((frTicks / TICKS_PER_REV) * Math.PI + Math.PI / 2.0);
            double pod2Deg = Math.toDegrees(pod2Rad);

            // Output to Driver Station Telemetry
            telemetry.addLine("===== RAW ENCODER TICKS =====");
            telemetry.addData("Front Right Ticks (fr)", frTicks);
            telemetry.addData("Back Right Ticks  (br)", brTicks);
            telemetry.addData("Front Left Ticks  (fl)", flTicks);
            telemetry.addData("Back Left Ticks   (bl)", blTicks);

            telemetry.addLine("\n===== MOTOR VELOCITIES (Ticks/s) =====");
            telemetry.addData("Front Right Vel", frVel);
            telemetry.addData("Back Right Vel",  brVel);
            telemetry.addData("Front Left Vel",  flVel);
            telemetry.addData("Back Left Vel",   blVel);

            telemetry.addLine("\n===== CALCULATED DIFFY POD ANGLES =====");
            telemetry.addData("Pod 1 Angle (Deg)", "%.2f deg", pod1Deg);
            telemetry.addData("Pod 1 Angle (Rad)", "%.3f rad", pod1Rad);
            telemetry.addData("Pod 2 Angle (Deg)", "%.2f deg", pod2Deg);
            telemetry.addData("Pod 2 Angle (Rad)", "%.3f rad", pod2Rad);

            // Output packet to FTC Dashboard
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Front Right Ticks", frTicks);
            packet.put("Back Right Ticks", brTicks);
            packet.put("Front Left Ticks", flTicks);
            packet.put("Back Left Ticks", blTicks);
            packet.put("Pod 1 Deg", pod1Deg);
            packet.put("Pod 2 Deg", pod2Deg);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
