package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class clear extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor leftMotor1 = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor leftMotor2 = hardwareMap.dcMotor.get("backRightMotor");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

    }
}
