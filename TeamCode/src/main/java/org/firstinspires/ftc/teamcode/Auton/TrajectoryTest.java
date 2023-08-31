package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class TrajectoryTest extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60, 36, Math.toRadians(0)); //tell the robot where it starts
        drivetrain.setPoseEstimate(startPose);

        //Roadrunner Guide to Trajectory Options:
        //https://learnroadrunner.com/trajectorybuilder-functions.html#linetolinearheading-endpose-pose2d

        //Trajectory Sequence example:
        TrajectorySequence trajSeq = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(48,36))
                .splineTo(new Vector2d(40,-30), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(20, 20), () -> {
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate

                    // Run your action in here!
                })
                .build();

        TrajectorySequence trajSeq2 = drivetrain.trajectorySequenceBuilder(trajSeq.end())
                .lineTo(new Vector2d(48,36))
                .splineTo(new Vector2d(40,-30), Math.toRadians(0))
                .addDisplacementMarker(20, () -> {
                    // This marker runs 20 inches into the trajectory

                    // Run your action in here!
                })
                .build();

        waitForStart();

        if (!isStopRequested())
            drivetrain.followTrajectorySequence(trajSeq);
        drivetrain.followTrajectorySequence(trajSeq2);

    }
}
