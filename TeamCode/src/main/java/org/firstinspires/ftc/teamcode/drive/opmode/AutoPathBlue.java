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
public class AutoPathBlue extends OpMode {

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
    Pose2d startPose = new Pose2d(-57, -48, Math.toRadians(90));//-57 is for competition tiles without border. -58 for tiles with border

    @Override
    public void init() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        initTrajectories();

        drive.wobbleGrabber.setPosition(SampleMecanumDrive.wobblegrabber_retracted);

        currentState=State.t0;
    }

    @Override
    public void init_loop(){
        if(System.currentTimeMillis()-updateTimer>5000){
            numRings=0;
            updateTimer=System.currentTimeMillis();
        }
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size()>0) {
                for(Recognition recognition : updatedRecognitions){
                    if(recognition.getLabel().equals("Quad")) {
                        numRings=4;
                        break;
                    }
                    if(recognition.getLabel().equals("Single")) numRings=1;
                }
            }
        }
        telemetry.addData("Number of Rings",numRings);
        telemetry.update();
        if(numRings==0) set = Set.A;
        else if(numRings==1) set = Set.B;
        else if(numRings==4) set = Set.C;
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
        switch(set){
            case A:
                switch(currentState){
                    case t0://Drive to zone A
                        drive.followTrajectoryAsync(t_A_0);
                        currentState=State.t1;
                        break;
                    case t1:
                        if(!drive.isBusy()){
                            currentState = State.t2;
                            drive.followTrajectoryAsync(t_A_1);
                        }
                        break;
                    case t2:
                        if(!drive.isBusy()){
                            currentState = State.t3;
                            drive.followTrajectoryAsync(t_A_2);
                        }
                        break;
                    case t3:
                        if(!drive.isBusy()){
                            currentState = State.t4;
                            drive.followTrajectoryAsync(t_A_3);
                        }
                        break;
                    case t4:
                        if(!drive.isBusy()){
                            currentState = State.shootright;
                            drive.followTrajectoryAsync(t_A_4);
                        }
                        break;
                    case shootright:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.turn1;
                        }break;
                    case turn1:
                        if(System.currentTimeMillis()-timer>2500){
                            currentState = State.shootmid;
                            drive.turnAsync(Math.toRadians(2*powershotAngle));
                        }break;
                    case shootmid:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.turn2;
                        }break;
                    case turn2:
                        if(System.currentTimeMillis()-timer>1000){
                            currentState = State.shootleft;
                            drive.turnAsync(Math.toRadians(-powershotAngle));
                        }break;
                    case shootleft:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.t5;
                        }break;
                    case t5:
                        if(System.currentTimeMillis()-timer>1000){
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(t_A_5);
                        }break;
                    case IDLE:
                        break;
                }
                break;
            case B:
                switch(currentState){
                    case t0://Drive to zone B
                        drive.followTrajectoryAsync(t_B_0);
                        currentState=State.t1;
                        break;
                    case t1:
                        if(!drive.isBusy()){
                            currentState = State.t2;
                            drive.followTrajectoryAsync(t_B_1);
                        }
                        break;
                    case t2:
                        if(!drive.isBusy()){
                            currentState = State.t3;
                            drive.followTrajectoryAsync(t_B_2);
                        }
                        break;
                    case t3:
                        if(!drive.isBusy()){
                            currentState = State.t4;
                            drive.followTrajectoryAsync(t_B_3);
                        }
                        break;
                    case t4:
                        if(!drive.isBusy()){
                            currentState = State.shootright;
                            drive.followTrajectoryAsync(t_B_4);
                        }
                        break;
                    case shootright:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.turn1;
                        }break;
                    case turn1:
                        if(System.currentTimeMillis()-timer>2500){
                            currentState = State.shootmid;
                            drive.turnAsync(Math.toRadians(2*powershotAngle));
                        }break;
                    case shootmid:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.turn2;
                        }break;
                    case turn2:
                        if(System.currentTimeMillis()-timer>1000){
                            currentState = State.shootleft;
                            drive.turnAsync(Math.toRadians(-powershotAngle));
                        }break;
                    case shootleft:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.t5;
                        }break;
                    case t5:
                        if(System.currentTimeMillis()-timer>1000){
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(t_B_5);
                        }break;
                    case IDLE:
                        break;
                }
                break;
            case C:
                switch(currentState){
                    case t0://Drive to zone C
                        drive.followTrajectoryAsync(t_C_0);
                        currentState=State.t1;
                        break;
                    case t1:
                        if(!drive.isBusy()){
                            currentState = State.t2;
                            drive.followTrajectoryAsync(t_C_1);
                        }
                        break;
                    case t2:
                        if(!drive.isBusy()){
                            currentState = State.t3;
                            drive.followTrajectoryAsync(t_C_2);
                        }
                        break;
                    case t3:
                        if(!drive.isBusy()){
                            currentState = State.t4;
                            drive.followTrajectoryAsync(t_C_3);
                        }
                        break;
                    case t4:
                        if(!drive.isBusy()){
                            currentState = State.shootright;
                            drive.followTrajectoryAsync(t_C_4);
                        }
                        break;
                    case shootright:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.turn1;
                        }break;
                    case turn1:
                        if(System.currentTimeMillis()-timer>2500){
                            currentState = State.shootmid;
                            drive.turnAsync(Math.toRadians(2*powershotAngle));
                        }break;
                    case shootmid:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.turn2;
                        }break;
                    case turn2:
                        if(System.currentTimeMillis()-timer>1000){
                            currentState = State.shootleft;
                            drive.turnAsync(Math.toRadians(-powershotAngle));
                        }break;
                    case shootleft:
                        if(!drive.isBusy()){
                            timer=System.currentTimeMillis();
                            currentState=State.t5;
                        }break;
                    case t5:
                        if(System.currentTimeMillis()-timer>1000){
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(t_C_5);
                        }break;
                    case IDLE:
                        break;
                }
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
                .addDisplacementMarker(0.8, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                })
                .lineToLinearHeading(new Pose2d(6, 60, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    releaseWobble();
                })
                .build();
        //Drop Wobble Goal
        //Move back to other wobble goal
        t_A_1 = drive.trajectoryBuilder(t_A_0.end())
                .addDisplacementMarker(0.1, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .splineToLinearHeading(new Pose2d(-42, 50, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                    releaseWobble();
                })
                .build();
        //Lower Wobble Goal Arm
        //Drive into Second Wobble Goal
        t_A_2 = drive.trajectoryBuilder(t_A_1.end())
                .lineTo(new Vector2d(-42, 34))
                .addDisplacementMarker(() -> {
                    grabWobble();
                })
                .build();
        //Pick Up Wobble Goal
        //Move back to zone A with grabbed Wobble Goal
        t_A_3 = drive.trajectoryBuilder(t_A_2.end(), true)
                .addDisplacementMarker(0.1, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .lineToLinearHeading(new Pose2d(0, 60, Math.toRadians(180)))
                .addDisplacementMarker(0.9, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                })
                .addDisplacementMarker(0.98, 0, () -> {
                    releaseWobble();
                })
                .build();
        //Drop second wobble goal
        //Move to shooting line, lined up with goal
        t_A_4 = drive.trajectoryBuilder(t_A_3.end())
                .addDisplacementMarker(()->{
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .splineToLinearHeading(new Pose2d(4, 8, Math.toRadians(-powershotAngle)), Math.toRadians(90))
                .build();
        //Shoot preloaded rings
        //Move to parking line
        t_A_5 = drive.trajectoryBuilder(t_A_4.end())
                .addDisplacementMarker(()->{
                    drive.setShooterSpeed(0);
                })
                .lineTo(new Vector2d(18, 8))
                .build();

        //------------------------ 1 Ring -> Zone B -> Set B ------------------------
        //Strafe to the left and shoot three preloaded rings
        //Move from starting position to zone A with preloaded Wobble Goal
        t_B_0 = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(0.8, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                })
                .lineToLinearHeading(new Pose2d(30, 36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    releaseWobble();
                })
                .build();
        //Drop Wobble Goal
        //Move back to other wobble goal
        t_B_1 = drive.trajectoryBuilder(t_B_0.end())
                .addDisplacementMarker(0.1, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .splineToLinearHeading(new Pose2d(-42, 50, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                    releaseWobble();
                })
                .build();
        //Lower Wobble Goal Arm
        //Drive into Second Wobble Goal
        t_B_2 = drive.trajectoryBuilder(t_B_1.end())
                .lineTo(new Vector2d(-42, 34))
                .addDisplacementMarker(() -> {
                    grabWobble();
                })
                .build();
        //Pick Up Wobble Goal
        //Move back to zone A with grabbed Wobble Goal
        t_B_3 = drive.trajectoryBuilder(t_B_2.end(), true)
                .addDisplacementMarker(0.1, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .lineToLinearHeading(new Pose2d(24, 36, Math.toRadians(180)))
                .addDisplacementMarker(0.9, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                })
                .addDisplacementMarker(0.98, 0, () -> {
                    releaseWobble();
                })
                .build();
        //Drop second wobble goal
        //Move to shooting line, lined up with goal
        t_B_4 = drive.trajectoryBuilder(t_B_3.end())
                .addDisplacementMarker(()->{
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .splineToLinearHeading(new Pose2d(4, 8, Math.toRadians(-powershotAngle)), Math.toRadians(90))
                .build();
        //Shoot preloaded rings
        //Move to parking line
        t_B_5 = drive.trajectoryBuilder(t_B_4.end())
                .addDisplacementMarker(()->{
                    drive.setShooterSpeed(0);
                })
                .lineTo(new Vector2d(18, 8))
                .build();

        //------------------------ 4 Rings -> Zone C -> Set C ------------------------
        //Drive to Zone C, avoiding the pile of rings
        //Strafe to the left and shoot three preloaded rings
        //Move from starting position to zone A with preloaded Wobble Goal
        t_C_0 = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(0.8, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                })
                .lineToLinearHeading(new Pose2d(30, 60, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    releaseWobble();
                })
                .build();
        //Drop Wobble Goal
        //Move back to other wobble goal
        t_C_1 = drive.trajectoryBuilder(t_C_0.end())
                .addDisplacementMarker(0.1, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .splineToLinearHeading(new Pose2d(-42, 50, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                    releaseWobble();
                })
                .build();
        //Lower Wobble Goal Arm
        //Drive into Second Wobble Goal
        t_C_2 = drive.trajectoryBuilder(t_C_1.end())
                .lineTo(new Vector2d(-42, 34))
                .addDisplacementMarker(() -> {
                    grabWobble();
                })
                .build();
        //Pick Up Wobble Goal
        //Move back to zone A with grabbed Wobble Goal
        t_C_3 = drive.trajectoryBuilder(t_C_2.end(), true)
                .addDisplacementMarker(0.1, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .lineToLinearHeading(new Pose2d(24, 60, Math.toRadians(180)))
                .addDisplacementMarker(0.9, 0, () -> {
                    drive.setWobbleArmPosition(grabberArmExtended);
                })
                .addDisplacementMarker(0.98, 0, () -> {
                    releaseWobble();
                })
                .build();
        //Drop second wobble goal
        //Move to shooting line, lined up with goal
        t_C_4 = drive.trajectoryBuilder(t_C_3.end())
                .addDisplacementMarker(() -> {
                    drive.setWobbleArmPosition(grabberArmRetracted);
                })
                .lineToLinearHeading(new Pose2d(4, 8, Math.toRadians(-powershotAngle)))
                .build();
        //Shoot preloaded rings
        //Move to parking line
        t_C_5 = drive.trajectoryBuilder(t_C_4.end())
                .addDisplacementMarker(()->{
                    drive.setShooterSpeed(0);
                })
                .lineTo(new Vector2d(18, 8))
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