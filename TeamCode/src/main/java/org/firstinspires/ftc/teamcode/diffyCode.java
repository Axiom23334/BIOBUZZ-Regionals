package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class diffyCode extends LinearOpMode{
    public void driveDiffy(double x, double y, DcMotorEx motor1, DcMotorEx motor2, double currentAngle) {
        double targetAngle = Math.atan(-y / x);
        if (x == 0) {
            if (y > 0) {
                targetAngle = -Math.PI / 2;
            } else {
                targetAngle = Math.PI / 2;
            }
        }
        targetAngle = targetAngle + Math.PI / 2;

        double error = targetAngle - currentAngle;

        double speed = 0;
        double power;
        if (Math.abs(error) > 0.1) {
            motor1.setPower(Range.clip(-error * 0.5, -1, 1));
            motor2.setPower(Range.clip(error * 0.5, -1, 1));
        } else {
            power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            speed = power * ((1 + ((double) 46 / 11)) * 28);
            if (Math.atan(-y / x) == Math.atan2(-y, x)) {
                motor1.setPower(-power);
                motor2.setPower(-power);
            } else {
                motor1.setPower(power);
                motor2.setPower(power);
            }


        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Angle", targetAngle);
        packet.put("Current Angle", currentAngle);
        packet.put("Error", error);
        packet.put("speed", speed);
        packet.put("y", y);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotorEx leftMotor1 = (DcMotorEx) hardwareMap.dcMotor.get("frontRightMotor");
        DcMotorEx leftMotor2 = (DcMotorEx) hardwareMap.dcMotor.get("backRightMotor");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double currentTick = leftMotor1.getCurrentPosition();
            double currentAngle = ((currentTick / 4214) * Math.PI);
            driveDiffy(x, y, leftMotor1, leftMotor2, currentAngle);


        }

    }
}
