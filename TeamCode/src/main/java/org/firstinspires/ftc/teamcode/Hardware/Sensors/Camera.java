package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.TeamPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {

    public TeamPipeline colorPicker = new TeamPipeline();
    private OpenCvCamera webcam;
    private String id;
    private boolean display2Phone;


    public Camera(String id, boolean display2Phone){
        this.id = id;

        // If we enabled display, add the cameraMonitorViewId to the creation of our webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, id), cameraMonitorViewId);


        //start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // Set the pipeline depending on id

    }

    public Camera(String id){
        this.id = id;

        //no display to phone
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, id));

        //start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        // Set the pipeline depending on id


    }


}