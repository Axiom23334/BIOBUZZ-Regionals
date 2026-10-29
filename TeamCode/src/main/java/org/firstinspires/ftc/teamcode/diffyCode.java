package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp(name="Utility: Diffy Code", group = "Utility")
public class diffyCode extends LinearOpMode{
    public static double kP1 = 0.7;
    public static double kP2 = 0.7;
    public boolean diffy1Ready = false;
    public boolean diffy2Ready = false;


    public void driveDiffy(double x, double y, DcMotorEx motor1, DcMotorEx motor2, double currentAngle, double kP, double diffy) {
        double targetAngle = Math.atan2(x, y);
        double error = targetAngle - currentAngle;

        double speed = 0;
        double power = 0;
        if (Math.abs(error) > 0.2) {
            motor1.setPower(-Range.clip(error * kP, -1, 1));
            motor2.setPower(-Range.clip(error * kP, -1, 1));
            if (diffy == 1){
                diffy1Ready = false;
            }
            if (diffy == 2){
                diffy2Ready = false;
            }
        } else {
            if (diffy == 1){
                diffy1Ready = true;
            }
            if (diffy == 2){
                diffy2Ready = true;
            }

            if ((diffy1Ready)&&(diffy2Ready)) {
                power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
                speed = power * ((1 + ((double) 46 / 11)) * 28);
                motor1.setPower(-0);
                motor2.setPower(0);
            } else {
                //motor1.setPower(0);
                //motor2.setPower(0);
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

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotorEx leftMotor1 = (DcMotorEx) hardwareMap.dcMotor.get("frontRightMotor");
        DcMotorEx leftMotor2 = (DcMotorEx) hardwareMap.dcMotor.get("backRightMotor");
        DcMotorEx rightMotor1 = (DcMotorEx) hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotorEx rightMotor2 = (DcMotorEx) hardwareMap.dcMotor.get("backLeftMotor");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            double y = gamepad1.left_stick_x*1;
            double x = gamepad1.left_stick_y*1;
            double currentTick = leftMotor2.getCurrentPosition();
            double currentAngle = ((currentTick / 6260.5) * Math.PI + Math.PI/2);
            driveDiffy(x, y, rightMotor1, rightMotor2, currentAngle, kP1, 1);


            double currentTick2 = leftMotor1.getCurrentPosition();
            double currentAngle2 = ((currentTick2 / 6260.5) * Math.PI + Math.PI/2);
            driveDiffy(x, y, leftMotor1, leftMotor2, currentAngle2, kP2, 2);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("tick1", currentTick);
            packet.put("tick2", currentTick2);
            dashboard.sendTelemetryPacket(packet);


        }

    }
}
