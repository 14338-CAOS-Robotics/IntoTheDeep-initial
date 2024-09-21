package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;

import org.firstinspires.ftc.teamcode.Utilities.Controller;

import org.firstinspires.ftc.teamcode.Utilities.PID;

public class SlidesTestingOpMode extends OpMode {

    CAOSHardware hardware;
    PID pid;
    SlideControl slideController;
    Controller controller;
    ElapsedTime runtime;

    ArmControl armController;
    @Override
    public void init() {
        setOpMode(this);
        hardware = new CAOSHardware(hardwareMap);
        slideController = new SlideControl(hardware);
        armController = new ArmControl(hardware);
        controller = new Controller(gamepad1);
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        System.out.println(hardware.outtake4Bar.getCurrentPosition());

    }

    @Override
    public void loop() {
        if (controller.dpad_down.isTapped()) {
            slideController.setHeight(SlideControl.PoleHeight.Short);
            armController.setHeight(SlideControl.PoleHeight.Short);
        }
        else if (controller.dpad_left.isTapped()) {
            slideController.setHeight(SlideControl.PoleHeight.Med);
            armController.setHeight(SlideControl.PoleHeight.Med);
        }
        else if (controller.dpad_up.isTapped()) {
            slideController.setHeight(SlideControl.PoleHeight.Tall);
            armController.setHeight(SlideControl.PoleHeight.Tall);
        }
        else if (controller.dpad_right.isTapped()) {
            slideController.setHeight(SlideControl.PoleHeight.Home);
            armController.setHeight(SlideControl.PoleHeight.Home);
        }
        update();

    }

    private void update(){
        slideController.update();
        armController.update();
        controller.update();
    }

}
