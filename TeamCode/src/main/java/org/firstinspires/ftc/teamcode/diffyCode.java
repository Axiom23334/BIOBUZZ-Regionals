package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class diffyCode extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotorEx leftMotor1 = (DcMotorEx) hardwareMap.dcMotor.get("frontRightMotor");
        DcMotorEx leftMotor2 = (DcMotorEx) hardwareMap.dcMotor.get("backRightMotor");

        double speed = 0;
        double power = 0;
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double targetAngle = Math.atan(-y/x);
            if (x==0){
                if (y>0){
                    targetAngle = -Math.PI / 2;
                } else{
                    targetAngle = Math.PI / 2;
                }
            }
            targetAngle = targetAngle+Math.PI/2;
            double currentTick = leftMotor1.getCurrentPosition();
            double currentAngle = ((currentTick/4214)*Math.PI);
            double error = targetAngle - currentAngle;

            if (Math.abs(error) > 0.1){
                leftMotor1.setPower(Range.clip(-error*0.5, -1, 1));
                leftMotor2.setPower(Range.clip(error*0.5, -1, 1));
            } else{
                power = Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
                speed = speed*((1+((double) 46 /11))*28);
                if (Math.atan(-y/x) == Math.atan2(-y,x)){
                    leftMotor1.setPower(-power);
                    leftMotor2.setPower(-power);
                } else{
                    leftMotor1.setPower(power);
                    leftMotor2.setPower(power);
                }


            }
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Angle", targetAngle);
            packet.put("Current Tick", currentTick);
            packet.put("Current Angle", currentAngle);
            packet.put("Error", error);
            packet.put("speed", speed);
            packet.put("y", y);
            dashboard.sendTelemetryPacket(packet);
        }

    }
}
