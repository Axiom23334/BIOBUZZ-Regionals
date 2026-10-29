package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class DiffyDriveSubsystem extends SubsystemBase {
    private final DcMotorEx frontRightMotor; // leftMotor1 in diffyCode
    private final DcMotorEx backRightMotor;  // leftMotor2 in diffyCode
    private final DcMotorEx frontLeftMotor;  // rightMotor1 in diffyCode
    private final DcMotorEx backLeftMotor;   // rightMotor2 in diffyCode

    public static double kP1 = 0.7;
    public static double kP2 = 0.7;
    public boolean diffy1Ready = false;
    public boolean diffy2Ready = false;

    public DiffyDriveSubsystem(HardwareMap hardwareMap) {
        frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("backLeftMotor");
    }

    public void drive(double x, double y) {
        double currentTick1 = backRightMotor.getCurrentPosition();
        double currentAngle1 = ((currentTick1 / 6260.5) * Math.PI + Math.PI / 2);
        driveDiffy(x, y, frontLeftMotor, backLeftMotor, currentAngle1, kP1, 1);

        double currentTick2 = frontRightMotor.getCurrentPosition();
        double currentAngle2 = ((currentTick2 / 6260.5) * Math.PI + Math.PI / 2);
        driveDiffy(x, y, frontRightMotor, backRightMotor, currentAngle2, kP2, 2);
    }

    public void driveDiffy(double x, double y, DcMotorEx motor1, DcMotorEx motor2, double currentAngle, double kP, double diffy) {
        double targetAngle = Math.atan2(x, y);
        double error = targetAngle - currentAngle;

        double speed = 0;
        double power = 0;
        if (Math.abs(error) > 0.2) {
            motor1.setPower(-Range.clip(error * kP, -1, 1));
            motor2.setPower(-Range.clip(error * kP, -1, 1));
            if (diffy == 1) {
                diffy1Ready = false;
            }
            if (diffy == 2) {
                diffy2Ready = false;
            }
        } else {
            if (diffy == 1) {
                diffy1Ready = true;
            }
            if (diffy == 2) {
                diffy2Ready = true;
            }

            if (diffy1Ready && diffy2Ready) {
                power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
                speed = power * ((1 + ((double) 46 / 11)) * 28);
                motor1.setPower(-0);
                motor2.setPower(0);
            }
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Angle", targetAngle);
        packet.put("Current Angle", currentAngle);
        packet.put("Error", error);
        packet.put("speed", power);
        packet.put("y", y);
        dashboard.sendTelemetryPacket(packet);
    }

    public void stop() {
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
}
