package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.ColesSplitPipeline;
import org.firstinspires.ftc.teamcode.Vision.PipelineProcessor;

@Config
@TeleOp
public class RedNearAuto extends LinearOpMode {
    public static double power = 0.3;
    public static boolean vision = true;
    public static int spike = 1;
    PathPoint start, beforeRotate, afterRotate, spikeOne, spikeTwo, spikeThree;
    Movement movement;
    CAOSHardware hardware;
    ColesSplitPipeline pipeline;
    PipelineProcessor processor;
    Robot robot;
    boolean isRed = true;
    int m = -1;

    boolean lineTo(PathPoint pp, boolean stop) {
        Movement.Function[] line = movement.getLine(new double[][] {movement.odo.getLocation(), pp.getPos()});
        while (movement.followPath(line, power, pp.getHeading(), 1, 1, 1, power, stop, false)) {
            telemetry.addData("current target", pp.toString());
            telemetry.update();
        }
        //this.position = pp;
        return true;
    }



    boolean lineTo(PathPoint pp) {
        return lineTo(pp, false);
    }
    public int doTheVision() {
        while (!pipeline.hasProcessed()) {} //let the vision thread cook
        return pipeline.getTargetPosition();

    }
    public void autoInit() {

        if (isRed) m *= -1;

        start = new PathPoint(12, 65 * m, m * Math.PI/2);
        beforeRotate = new PathPoint(12, 53 * m, m * Math.PI / 2);
        afterRotate = new PathPoint(12, 36 * m, 0);
        spikeOne = new PathPoint(10, 36 * m, 0);
        spikeTwo = new PathPoint(20, 28 * m, 0);
        spikeThree = new PathPoint(36, 36 * m, 0);

        hardware = new CAOSHardware(hardwareMap);
        movement = new Movement(start.getX(),start.getY(),start.getHeading(),hardware);

        pipeline = new ColesSplitPipeline(isRed);
        processor = new PipelineProcessor(hardwareMap);
        processor.setPipeline(pipeline);
        processor.startStream();

        robot = new Robot();
    }

    public void dropPurple() {
        robot.intake.goToArmZero();
        while(robot.intake.getCorrectedTicks() > 11) {
            //chill
        }
        robot.intake.openClaw();
        double time = getRuntime();
        while (getRuntime() < time + 2000) {}
    }


    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.setOpMode(this);
        this.autoInit();
        waitForStart();
        if (vision) spike = doTheVision();

        while (opModeIsActive()) {

            lineTo(beforeRotate);
            //don't start rotating until it's a safe distance away
            lineTo(afterRotate);
            switch (spike) {
                case 1:
                    lineTo(spikeOne, true);
                    break;
                case 2:
                    lineTo(spikeTwo, true);
                    break;
                default:
                    lineTo(spikeThree, true);
                    break;
            }
            dropPurple();
            lineTo(new PathPoint(movement.odo.getLocation()[0] + 5, movement.odo.getLocation()[1], 0));
            sleep(30000);
        }

    }
}
