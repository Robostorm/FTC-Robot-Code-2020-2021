package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class Testing extends LinearOpMode {
    //Gamepad 1 Timers
    public static long uptimer=0;
    public static long downtimer=0;

    public static float servopos=0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            //Drive Controls
            float ly = gamepad1.left_stick_y;
            float lx = gamepad1.left_stick_x;
            float ry = gamepad1.right_stick_y;
            float rx = gamepad1.right_stick_x;

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if(up && System.currentTimeMillis()-uptimer>200){
                if(servopos<1) servopos+=0.05;
                uptimer=System.currentTimeMillis();
            }
            if(down && System.currentTimeMillis()-downtimer>200){
                if(servopos>0) servopos-=0.05;
                downtimer=System.currentTimeMillis();
            }

            drive.ringPusher.setPosition(servopos);
            telemetry.addData("Servo Pos: ",servopos);
            telemetry.update();
        }
    }
}