//package org.firstinspires.ftc.teamcode.Testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.GamepadControls.ButtonControls;
//import org.firstinspires.ftc.teamcode.GamepadControls.Controller;
//import org.firstinspires.ftc.teamcode.Hardware.Robot;
//import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
//
//@TeleOp (name = "Testing Intake FANCY")
//public class IntakeTestFancy extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        OpModeUtils.setOpMode(this);
//        Robot robot = new Robot();
//        Controller gp1 = new Controller(gamepad1);
//        robot.setController1(gp1);
//        robot.intake.testing();
//        waitForStart();
//        while (opModeIsActive()&& !isStopRequested()) {
//            robot.intake.update();
//            robot.intake.testingUpdate();
//            robot.intake.testingTestingData();
//            Controller.update();
//            robot.intake.updateCoefficients();
//        }
//    }
//}
