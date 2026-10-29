package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DiffyDriveSubsystem;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setDrivetrain(new DiffyPedroDrivetrain(hardwareMap))
                .build();
    }

    public static Follower createFollower(HardwareMap hardwareMap, DiffyDriveSubsystem diffyDrive) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setDrivetrain(new DiffyPedroDrivetrain(diffyDrive))
                .build();
    }
}
