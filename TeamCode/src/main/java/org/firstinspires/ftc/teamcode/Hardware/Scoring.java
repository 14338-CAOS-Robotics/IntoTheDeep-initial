package org.firstinspires.ftc.teamcode.Hardware;

//import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.F;
//import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.SlidesD;
//import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.SlidesI;
//import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.SlidesP;
//import static org.firstinspires.ftc.teamcode.Utilities.Constants.grabberOffset;
//import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
//import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffsetAuto;
//import static org.firstinspires.ftc.teamcode.Utilities.Constants.v4bOffset;
//import static org.firstinspires.ftc.teamcode.Utilities.Constants.v4bOffsetAuto;
//import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.Vinno;
//import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.VkA;
//import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.VkS;
//import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.VkV;
//import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.vP;
//import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.inRange;
//import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.interpolateRanges;
//import static org.firstinspires.ftc.teamcode.Utilities.NonConstants.fullyDown;
//import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
//import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.linearOpMode;
//import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
//import static java.lang.Math.abs;
//
//import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Control.Localization.TwoWheelOdometry;
//import org.firstinspires.ftc.teamcode.Control.Movement;
//import org.firstinspires.ftc.teamcode.Utilities.AlphaNoiseFilter;
//import org.firstinspires.ftc.teamcode.Utilities.Files.BlackBox.LoopTimer;
//import org.firstinspires.ftc.teamcode.Utilities.PID;
//
//public class Scoring {
//
//    public DcMotor spool;
//    public DcMotor spool2;
//    public Servo squeezer;
//    public DcMotor v4b;
//    public PID v4bPID;
//    public Servo grabberSpin;
//    public TouchSensor beam1;
//    public Intake intake;
//    public Servo brace;
//    public static int idfk = 0;
//    public boolean previousPress = false;
//    public boolean clawToggleOpen = false;
//    public static boolean beaconScore;
//
//    private PID slidesPID = new PID(SlidesP, SlidesI, SlidesD);
//
//    public boolean wentDown = false;
//
//    public ElapsedTime time = new ElapsedTime();
//    public ElapsedTime sleep = new ElapsedTime();
//    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(VkS, VkV, VkA);
//    public double prevV4BVelocity = 0;
//    public LoopTimer loopTimer = new LoopTimer();
//    public AlphaNoiseFilter armAccelFilter = new AlphaNoiseFilter(0, Vinno);
//    public int v4BTarget = -60;
//    public int slidesTarget = 0;
//
//
//    public Scoring() {
//
//        intake = new Intake();
//
//
//        squeezer = hardwareMap.get(Servo.class, "squeeze");
//        v4b = hardwareMap.get(DcMotor.class, "v4b");
//        v4b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        //v4b.setTargetPosition(0);
//        //v4b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        v4b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        v4b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        //v4bPID = new PID(vP, vI, vD);
//
//        brace = hardwareMap.get(Servo.class, "brace");
//
//        grabberSpin = hardwareMap.get(Servo.class, "spin");
//
//
//        spool = hardwareMap.get(DcMotor.class, "spool");
//        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spool.setTargetPosition(0);
//        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        spool.setPower(.6);
//
//        spool2 = hardwareMap.get(DcMotor.class, "spool2");
//        spool2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spool2.setTargetPosition(0);
//        spool2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        spool2.setPower(.6);
//
//        beam1 = hardwareMap.get(TouchSensor.class, "beam1");
//        loopTimer.reset();
//    }
//
//    //AUTO
//    public void autoHigh() {
//        v4b(v4bScoreBack);
//    }
//
//    public void resetManual(){
////        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        spool2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        spool.setPower(0);
////        spool2.setPower(0);
////        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        spool2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void resetAutomatic(){
////        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        spool2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        spool2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//    }
//
//
//
//    public void autoMid() {
//        braceOut();
//        slides(.5, slidesMidAuto);
//        grabber(grabberScoreAuto);
//        v4b(v4bScoreBackAuto);
//
//    }
//
//
//    public void autoLow() {
//        braceOut();
//        sleep(0.4);
//        slides(1, (int)slidesOffset);
//        v4b(v4bScoreFrontLow);
//        grabber(grabberScore);
//
//    }
//
//    public void autoDeposit(Movement drive, double x, double y, double heading) {
//        openScore();
//        sleep(0.2,drive,x,y,heading);
//        close();
//        v4b(v4bHide);
//        sleep(0.1,drive,x,y,heading);
//        slides(1, (int)slidesOffset);
////        braceOut();
//        sleep(0.25,drive,x,y,heading);
//        v4b(v4bDown);
//        open(false);
//    }
//
//    public void autoDeposit() {
//        openScore();
//        sleep(0.4);
//        close();
//        v4b(v4bHide);
//        sleep(0.4);
//        slides(1, (int)slidesOffset);
//        braceOut();
//        v4b(v4bDown);
//    }
//
//    public void stackPickup(int height) {
//        v4b(v4bDown);
//        grabber(grabberDown);
//        slides(1, (height - 1) * slidesStackIncrease);
//    }
//
//    public void stackEscape(int height) {
//        slides(1, (height + 4) * slidesStackIncrease);
//    }
//
//    public boolean beamBroken() {
//        return beam1.isPressed();
//    }
//
//
//    //returns if beam just triggered
//    public boolean updateBeam() {
//        boolean retVal = beam1.isPressed() && !previousPress;
//        previousPress = beam1.isPressed();
//        return retVal;
//    }
//
//
//    //Base Movement Methods
//    public void open(boolean tipped) {
//        if (tipped) {
//            squeezer.setPosition(clawTipped);
//        } else {
//            squeezer.setPosition(clawOpen);
//        }
//    }
//
//    public void openScore() {
//        if (beaconScore) {
//            squeezer.setPosition(clawOpenBeacon);
//        } else {
//            squeezer.setPosition(clawOpenScore);
//        }
//    }
//
//    public void close(boolean down) {
//        if (down) {
//            slides(1, 100);
//        }
//        squeezer.setPosition(clawClose);
//    }
//
//    public void close() {
//        squeezer.setPosition(clawClose);
//    }
//
//    public int interpolate(int target) {
//        return (int) interpolateRanges(target, 0, 90, v4b0, v4b90);
//    }
//
//    public int unterpolate(int target) {
//        return (int) interpolateRanges(target, v4b0, v4b90, 0, 90);
//    }
//
////    public void v4b(int target) {
////        int undershoot;
////        v4b.setPower(v4bSpeed);
////        //target becomes new target with auto start offset
////        int newTarget = target - (int)v4bOffset;
////        //if current position is not close to new target (close means closer than the undershoot)
////        if(!inRange(interpolate(newTarget)-(v4bUndershoot+10),v4b.getCurrentPosition(),interpolate(newTarget)+(v4bUndershoot-10))) {
////            //if target is greater than the current position
////            if (target > uninterpolate(v4b.getCurrentPosition())) {
////                //add the undershoot
////                undershoot = newTarget + v4bUndershoot;
////            } else {
////                //subtract the undershoot
////                undershoot = newTarget - v4bUndershoot;
////            }
////            //go to the undershoot position
////            v4b.setTargetPosition(interpolate(undershoot));
////        }else{
////            //if it is close then go to actual position
////            v4b.setTargetPosition(interpolate(newTarget));
////        }
////
////
////        multTelemetry.addData("target", newTarget);
////        multTelemetry.addData("current", v4b.getCurrentPosition());
////    }
//
////    public void v4b(int target) {
////        int newTarget = target - (int)v4bOffset;
////        v4b.setPower(v4bSpeed);
////        v4b.setTargetPosition(interpolate(newTarget));
////        multTelemetry.addData("target", target);
////        multTelemetry.addData("current", v4b.getCurrentPosition());
////    }
//
//    //PID V4B
////    public void v4b(int target) {
////        int newTarget = target - (int)v4bOffset;
////        v4b.setPower((v4bPID.update((double)(interpolate(newTarget) - v4b.getCurrentPosition()),false)));
////        multTelemetry.addData("power", v4b.getPower());
////        multTelemetry.addData("target", interpolate(newTarget));
////        multTelemetry.addData("current", v4b.getCurrentPosition());
////    }
//
//    public void braceIn(){
//        brace.setPosition(braceIn);
//    }
//
//    public void braceOut(){
//        brace.setPosition(braceOut);
//    }
//    //FeedForward V4B
//    public void v4b(int target) {
//        v4BTarget = target;
//        loopTimer.update();
//        armAccelFilter.setInnovationGain(Vinno);
//        //v4bPID.setWeights(vP, vI, vD);
//        feedforward = new SimpleMotorFeedforward(VkS, VkV, VkA);
//        int newTarget = interpolate(target + (int)v4bOffsetAuto - (int)v4bOffset);
//        double error = newTarget - v4b.getCurrentPosition();
//        double velocity = error * vP;
//        double accel = (velocity - prevV4BVelocity) / loopTimer.getSeconds();
//        accel = armAccelFilter.update(accel);
//        prevV4BVelocity = velocity;//((DcMotorEx) v4b).getVelocity();
//        //How does this know what to do if it doesn't get current position
//        double power = feedforward.calculate(velocity,accel);
//        v4b.setPower(power);
//        //abs(power) > 0.08 ? power : 0
////        multTelemetry.addData("power", v4b.getPower());
////        multTelemetry.addData("veloc", ((DcMotorEx) v4b).getVelocity());
////        multTelemetry.addData("calcVeloc", velocity);
////        multTelemetry.addData("calcAccel", accel);
//        multTelemetry.addData("target", newTarget);
////        multTelemetry.addData("looptime", loopTimer.getHertz());
//        multTelemetry.addData("current", v4b.getCurrentPosition());
//    }
//
//    public void v4bManual(double power){
//        v4b.setPower(power);
//    }
//
//    public void v4bHold(){
//        v4b(v4BTarget);
//    }
//
//    public void slidesHold(){
//        slides(1, slidesTarget);
//    }
//
//    public void slidesHoldWTF(){
//        slides(1, slidesTarget);
//    }
//
//    public void v4bNeutral(){
//        v4b.setPower(0);
//    }
//
//
//
//    //No controller V4B
//    @Deprecated
//    public void v4b(int target, boolean feedForward) {
//        int newTarget = target - (int)v4bOffset;
//        v4b.setPower(0.6);
//        v4b.setTargetPosition(interpolate(newTarget));
//        multTelemetry.addData("target", interpolate(newTarget));
//        multTelemetry.addData("current", v4b.getCurrentPosition());
//    }
//
//
//    public void grabber(double target){
//        grabberSpin.setPosition(target -  grabberOffset);
//        //grabberSpin.setPosition(interpolateRanges(target, 0, 90, grabber0, grabber90));
//    }
//
//    public void grabberManual(double power){
//        grabberSpin.setPosition(grabberSpin.getPosition() + power);
//    }
//
//    public void slides(double power, int target){
//        slidesTarget = target;
//        double newTarget = target - slidesOffset + slidesOffsetAuto;
//        double current = (-spool.getCurrentPosition() + spool2.getCurrentPosition()) * .5;
//        double dist = newTarget - current;
//
//        if(!(newTarget < 30 && dist < 30)) {
//            slidesPID.setWeights(SlidesP, SlidesI, SlidesD);
//            idfk ++;
//            double pow = slidesPID.update(dist, false) + ((dist > 10) ? F : 0);
////            multTelemetry.addData("Spool1", spool.getCurrentPosition());
////            multTelemetry.addData("Spool2", spool2.getCurrentPosition());
////            multTelemetry.addData("Current", current);
////            multTelemetry.addData("Target", newTarget);
////            multTelemetry.addData("Power", pow);
////            multTelemetry.addData("Dist", dist);
//            multTelemetry.addData("idfk", idfk);
//            multTelemetry.update();
//            spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            spool2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            spool.setDirection(DcMotorSimple.Direction.FORWARD);
//            spool2.setDirection(DcMotorSimple.Direction.FORWARD);
//            spool.setPower(-pow);
//            spool2.setPower(pow);
//        }else{
//            spool.setPower(-0);
//            spool2.setPower(0);
//        }
//
////
////        spool.setTargetPosition(-newTarget);
////        spool2.setTargetPosition(newTarget);
//    }
//
//    public void slidesManual(double power){
////        spool.setPower(-power);
////        spool2.setPower(power);
//
//    }
//
//    //Score Methods
//
//    //Front Score
//    public void highFront(boolean funny) {
//        braceOut();
//        if (true) {
//            close();
//            slides(1, slidesHighFront);
//            v4b(v4bScoreFront);
//            if (time.seconds() > .2) {
//                grabber(grabberScoreFront);
//            }
//        }else{
//            close();
//            slides(1, slidesHighFront);
//            v4b(v4bScoreFront);
//            if (time.seconds() > .2) {
//                grabber(grabberScoreFunnyFront);
//            }
//        }
//    }
//
//    public void midFront(boolean funny){
//        braceOut();
//        if (true) {
//            close();
//            slides(1, slidesMidFront);
//            v4b(v4bScoreFront);
//            if(time.seconds() > .2){
//                grabber(grabberScoreFront);
//            }
//        }else{
//            close();
//            slides(1, slidesMidFront);
//            if(time.seconds() > .2){
//                v4b(v4bScoreFront);
//                grabber(grabberScoreFunnyFront);
//            }
//        }
//    }
//
//    public void lowFront(boolean funny){
//        braceOut();
//        if (true) {
//            close();
//            slides(1, (int)slidesOffset);
//            v4b(v4bScoreFrontLow);
//            if(time.seconds() > .2){
//                grabber(grabberScoreFront);
//            }
//        }else{
//            close();
//            slides(1, (int)slidesOffset);
//            if(time.seconds() > .2){
//                v4b(v4bScoreFrontLow);
//                grabber(grabberScoreFunnyFront);
//            }
//        }
//    }
//
//    //Back Score
//    public void highBack(boolean funny){
//        braceOut();
//        close();
//        v4b(v4bScoreBack);
//        if(time.seconds() > .2){
//            if(funny) {
//                slides(1, slidesHighFunny);
//                grabber(grabberScoreFunny);
//            }else{
//                grabber(grabberScore);
//                slides(1, slidesHighJunction);
//            }
//        }
//    }
//
//    public void midBack(boolean funny){
//        braceOut();
//        close();
//        v4b(v4bScoreBack);
//        if(time.seconds() > .2){
//            if(funny) {
//                slides(1, slidesMidFunny);
//                grabber(grabberScoreFunny);
//            }else{
//                grabber(grabberScore);
//                slides(1, slidesMidJunction);
//            }
//        }
//    }
//
//    public void lowBack(boolean funny){
//        braceIn();
//        close();
//        slides(1, (int)slidesOffset);
//        v4b(v4bScoreBackLow);
//        if(time.seconds() > .2){
//            if(funny) {
//                grabber(grabberScoreFunny);
//            }else{
//                grabber(grabberScore);
//            }
//        }
//    }
//
//    //Deposit
//    public void deposit(String coneAngle){
//        beaconScore = false;
//        if(inRange(0,time.seconds(), .3)){
//            openScore();
//            fullyDown = false;
//        }
//        if(inRange(.3,time.seconds(), .5)){
//            close();
//        }
//        if(inRange(.7,time.seconds(),.8)){
//            grabber(grabberHide);
//        }
//        if(inRange(0.7,time.seconds(),1.8)) {
//            v4b(v4bHide);
//        }
//        if(inRange(1.5,time.seconds(),1.7)){
//            slides(1, (int)slidesOffset);
//            if(coneAngle.equals("Straight")) {
//                open(false);
//            } else if(coneAngle.equals("Forwards")) {
//                open(true);
//            }
//        }
//        if(2<time.seconds()) {
//            if(coneAngle.equals("Straight")) {
//                grabber(grabberDown);
//                slides(1, (int)slidesOffset);
//            } else if(coneAngle.equals("Forwards")) {
//                grabber(grabberTip);
//                slides(.5, slidesTippedHeight);
//            }
//        }
//        if(time.seconds()>1.8){
//            fullyDown = true;
//            braceOut();
//            if(coneAngle.equals("Forwards")){
//                v4b(v4bDownFunny);
//            }else {
//                v4b(v4bDown);
//            }
//        }
//    }
//
//    public void startTeleop(String coneAngle) {
//        braceIn();
//        open(false);
//        fullyDown = true;
//        grabber(grabberDown);
//        v4b(v4bDown);
//        if (time.seconds() > 1) {
//            slides(1, (int)slidesOffset);
//        }
//    }
//
//
//
//    //Immediately Drop Slides
//    public void crashSlides(){
//        close();
//        slides(1, (int)slidesOffset);
//        v4b(v4bDown);
//    }
//
//    public void autoStart(){
//        braceIn();
//        close();
//        grabber(grabberStartAuto);
//        v4bNeutral();
//        v4BTarget = v4bStartAuto;
//        slides(1, (int)slidesOffset);
//    }
//
//    public void sleep(double sleepTime, TwoWheelOdometry odo){
//        sleep.reset();
//        while(sleep.seconds()<sleepTime && linearOpMode.opModeIsActive()){
//            v4b(v4BTarget);
//            slidesHold();
//            odo.localize();
//        }
//    }
//
//    public void sleep(double sleepTime){
//        sleep.reset();
//        while(sleep.seconds()<sleepTime  && linearOpMode.opModeIsActive()){
//            v4b(v4BTarget);
//            slidesHold();
//        }
//    }
//
//    public void sleep(double sleepTime, Movement drive, double x, double y, double heading){
//        sleep.reset();
//        while(sleep.seconds()<sleepTime && linearOpMode.opModeIsActive()){
//            v4b(v4BTarget);
//            slidesHold();
//            drive.holdPosition(x, y, heading);
//        }
//    }
//
//    public int getHeight(){
//        return (abs(spool.getCurrentPosition()) + abs(spool2.getCurrentPosition()))/2;
//    }
//}
