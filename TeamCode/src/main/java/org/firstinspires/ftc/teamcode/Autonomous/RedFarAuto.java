package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.ColesSplitPipeline;
import org.firstinspires.ftc.teamcode.Vision.PipelineProcessor;

@TeleOp
@Config
public class RedFarAuto extends LinearOpMode {

    public static double POWER;
    public static boolean useVision = false;

    public static int spike = 1;
    final double PI = Math.PI;
    boolean isRed = true;
    double startHeading = -PI / 2;
    double m = -1;
    Movement movement;
    CAOSHardware hardware;
    Robot robot;

    public static double power = 0.5;
    PathPoint position;

    //double[] startPos, firstPoint, spikeOne, spikeTwo, spikeThree, pointTwo;
    PathPoint startPoint, pointOne, spikeOne, spikeTwo, spikeThree, secondPoint;

    ColesSplitPipeline pipeline;
    PipelineProcessor proc;

    boolean lineTo(PathPoint pp, boolean stop) {
        Movement.Function[] line = movement.getLine(new double[][] {movement.odo.getLocation(), pp.getPos()});
        while (movement.followPath(line, POWER, pp.getHeading(), 1, 1, 1, POWER, stop, false)) {
            telemetry.addData("current target", pp.toString());
            telemetry.update();
        }
        this.position = pp;
        return true;
    }

    public void autoInit() {
        OpModeUtils.setOpMode(this);
        //Instantiate vision stuff here
        if (isRed) m *= -1;

//        startPos = new double[]{-36, 65 * m};   //heading -m*pi/2
//        firstPoint = new double[]{-36, 36 * m}; //heading 0
//        spikeOne = new double[]{-38, 36 * m}; //heading 0
//        spikeTwo = new double[]{-26, 34 * m}; //heading m*pi/4
//        spikeThree = new double[]{-10, 36 * m}; //heading 0
//        pointTwo = new double[]{-8, 12 * m};

        startPoint = new PathPoint(-36, 65 * m, -m * Math.PI / 2);
        pointOne = new PathPoint(-36, 36 * m, 0);
        spikeOne = new PathPoint(-38, 36 * m, 0);
        spikeTwo = new PathPoint(-26, 34 * m, m * Math.PI / 4);
        spikeThree = new PathPoint(-10, 36 * m, 0);
        secondPoint = new PathPoint(-8, 12 * m, 0);

        pipeline = new ColesSplitPipeline(this.isRed);

        proc = new PipelineProcessor(hardwareMap);
        proc.setPipeline(pipeline);
        proc.startStream();

        robot = new Robot();

    }

    public int doTheVision() {
        while (!pipeline.hasProcessed()) {} //let the vision thread cook
        return pipeline.getTargetPosition();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(telemetry);
        this.autoInit();
        hardware = new CAOSHardware(hardwareMap);
        movement = new Movement(startPoint.getX(), startPoint.getY(), startPoint.getHeading(), hardware);

        movement.setMovementPID();
        movement.setHeadingPID();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("path", "path 1");
            telemetry.update();

            if (useVision) spike = doTheVision();

            waitForStart();

            while (opModeIsActive()) {

                lineTo(pointOne, false);
                //sleep(30000);

                switch (spike) {
                    case 1:
                        lineTo(spikeOne, true);
                        //sleep(30000);
                        break;
                    case 2:
                        lineTo(spikeTwo, true);
                        break;
                    default:
                        lineTo(spikeThree, true);
                        break;
                }

                sleep(10000);
                telemetry.addData("path", "to the waitpoint");
                //drop purple pixel from intake claw
                robot.intake.setArmState(Intake.ArmStates.ZERO);
                while (!robot.intake.armIsAtState(Intake.ArmStates.ZERO, 5) && !isStopRequested()) {
                    robot.intake.update();
                }
                robot.intake.openClaw();
                sleep(1000);

                lineTo(secondPoint, true);
                sleep(30000);
            }
        }
    }
}