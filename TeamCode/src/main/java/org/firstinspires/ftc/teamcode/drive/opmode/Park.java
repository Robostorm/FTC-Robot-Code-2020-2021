package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.grabberArmRetracted;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.grabberArmExtended;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class Park extends OpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AVVIoiT/////AAABmX81rTPW60hcgKTP12YL9sFIHAXL7WyR1JI578v+YFJG/JSwjny6iEWiHEZ+twbt7HQ61pyg3A4/CCjpG1/u6VC6N2uK5bnWgFzeIHRESoUVX0pbphXVmkJ8NQmi9ZdKeNKV2ZgnM++ZT3cwvksRhXaA5LfVH0oB3XGNhrOzteP66UquAJUaNRKnMRjH4VjBiw9EWD1YGImGzeFPpA0p2xTKXQZAfLalNGnDRXM+3BlUfJsFbaSR+Uu/C3MIb8PMyA6h1nQGxMaIZLnl/Py2LPFgo5prafgdcD+9tV/BqE9F89AJC5LvHwOSKTfvsF9qe0fsZHFjg/+h10hZdeF8b1bQBhVO2OZf/T/e94I85MOh";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final int powershotAngle=7;

    long timer=0;

    private static int numRings=0;
    private static final float shooterSpeed=1500f;
    private static final float powershotSpeed=1300f;
    private static long updateTimer=0;
    public SampleMecanumDrive drive;

    //Set A Trajectories
    Trajectory t_A_0, t_A_1, t_A_2, t_A_3, t_A_4, t_A_5;
    //Set B Trajectories
    Trajectory t_B_0, t_B_1, t_B_2, t_B_3, t_B_4, t_B_5;
    //Set C Trajectories
    Trajectory t_C_0, t_C_1, t_C_2, t_C_3, t_C_4, t_C_5;

    enum State {
        t0,
        t1,
        t2,
        t3,
        t4,
        t5,
        turn1,
        turn2,
        shootright,
        shootmid,
        shootleft,
        IDLE
    }
    enum Set {
        A,
        B,
        C,
        UNKNOWN
    }

    State currentState = State.IDLE;
    Set set = Set.A;
    Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        initTrajectories();

        drive.wobbleGrabber.setPosition(SampleMecanumDrive.wobblegrabber_retracted);

        currentState=State.t0;
    }

    @Override
    public void init_loop(){

    }
    @Override
    public void loop() {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Shooter Speed: ",drive.shooter.getVelocity()/103.6f);
        telemetry.addData("Wobble Arm Pos: ",drive.wobbleGrabberArm.getCurrentPosition());
        telemetry.update();
        switch(currentState){
            case t0://Drive to zone A
                drive.followTrajectoryAsync(t_A_0);
                currentState=State.IDLE;
                break;
            case IDLE:
                break;
        }
        //if((currentState==State.t5 && set==Set.A) || (currentState==State.t1 && set==Set.B)) shootRings();
        if(currentState==State.turn1) {
            shootRingKeepActive(powershotSpeed);//shootright
            telemetry.addData("Shooting right shot","");
        }
        if(currentState==State.turn2) {
            shootRingActive();//shootmid
            telemetry.addData("Shooting middle shot","");
        }
        if(currentState==State.t5) {
            shootRingActive();//shootleft
            telemetry.addData("Shooting left shot","");
        }
        telemetry.update();
        drive.update();
    }

    public void initTrajectories() {
        //------------------------ 0 Rings -> Zone A -> Set A ------------------------
        //Move from starting position to zone A with preloaded Wobble Goal
        t_A_0 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(40, 0, Math.toRadians(0)))
                .build();
    }

    public void shootRingKeepActive(float speed){//takes 3 seconds
        if (System.currentTimeMillis() - timer < 1800 && drive.shooter.getPower() == 0)
            drive.setShooterSpeed(speed);
        if (System.currentTimeMillis() - timer > 1800 && System.currentTimeMillis() - timer < 2200 /*&& drive.ringPusher.getPosition()==SampleMecanumDrive.ringPusher_retracted*/)
            drive.ringPusher.setPosition(SampleMecanumDrive.ringPusher_extended);
        if (System.currentTimeMillis() - timer > 2200 /*&& drive.ringPusher.getPosition()==SampleMecanumDrive.ringPusher_extended*/)
            drive.ringPusher.setPosition(SampleMecanumDrive.ringPusher_retracted);
    }
    public void shootRingActive(){//takes 1.25 second
        if (System.currentTimeMillis() - timer < 400 /*&& drive.ringPusher.getPosition()==SampleMecanumDrive.ringPusher_retracted*/)
            drive.ringPusher.setPosition(SampleMecanumDrive.ringPusher_extended);
        if (System.currentTimeMillis() - timer > 400 /*&& drive.ringPusher.getPosition()==SampleMecanumDrive.ringPusher_extended*/)
            drive.ringPusher.setPosition(SampleMecanumDrive.ringPusher_retracted);
    }
    public void shootRings(float speed) {//takes abt 5 seconds
        if (System.currentTimeMillis() - timer < 1600 && drive.shooter.getPower() == 0)
            drive.setShooterSpeed(shooterSpeed);
        if (System.currentTimeMillis() - timer > 1600 && System.currentTimeMillis() - timer < 2400)
            drive.ringPusher.setPosition(1);
        if (System.currentTimeMillis() - timer > 2400 && System.currentTimeMillis() - timer < 3000)
            drive.ringPusher.setPosition(0);
        if (System.currentTimeMillis() - timer > 3000 && System.currentTimeMillis() - timer < 3800)
            drive.ringPusher.setPosition(1);
        if (System.currentTimeMillis() - timer > 3800 && System.currentTimeMillis() - timer < 4400)
            drive.ringPusher.setPosition(0);
        if (System.currentTimeMillis() - timer > 4400 && System.currentTimeMillis() - timer < 5200)
            drive.ringPusher.setPosition(1);
        if (System.currentTimeMillis() - timer > 5200) {
            drive.ringPusher.setPosition(0);
            drive.setShooterSpeed(0);
        }
    }
    public void shootRing(float speed) {//takes about 2.5 seconds
        if (System.currentTimeMillis() - timer < 800 && drive.shooter.getPower() == 0)
            drive.setShooterSpeed(shooterSpeed);
        if (System.currentTimeMillis() - timer > 800 && System.currentTimeMillis() - timer < 1600)
            drive.ringPusher.setPosition(1);
        if (System.currentTimeMillis() - timer > 1600 && System.currentTimeMillis() - timer < 2000) {
            drive.ringPusher.setPosition(0);
            drive.setShooterSpeed(0);
        }
    }

    public void releaseWobble() {
        drive.wobbleGrabber.setPosition(SampleMecanumDrive.wobblegrabber_extended);
    }
    public void grabWobble() {
        drive.wobbleGrabber.setPosition(SampleMecanumDrive.wobblegrabber_retracted);
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}