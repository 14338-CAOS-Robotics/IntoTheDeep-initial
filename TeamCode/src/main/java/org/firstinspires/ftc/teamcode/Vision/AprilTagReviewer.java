package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagReviewer {

    public enum TagFamily{
        TAG_1(583),
        TAG_2(584),
        TAG_3(585),
        TAG_4(586);

        public final int tagNum;

        TagFamily(int tagNum)
        {
            this.tagNum = tagNum;
        }

        public static boolean contains(int tagNum) {
            for (TagFamily tag : TagFamily.values()) {
                if (tag.tagNum == tagNum) {
                    return true;
                }
            }
            return false;
        }
    }

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    HardwareMap hardwareMap;

    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;

    List<AprilTagDetection> currentDetections;

    public AprilTagReviewer(HardwareMap hwp) {
        hardwareMap = hwp;
        initAprilTag();

    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal the easy way.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        builder.setCameraResolution(new Size(640,480));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        visionPortal = builder.build();

    }
//too lazy to find out why this was erroring ngl
//    private void getCameraSetting() {
//        // Ensure Vision Portal has been setup.
//        if (visionPortal == null) {
//            return;
//        }
//
//        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
//        maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
//
//        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//        minGain = gainControl.getMinGain();
//        maxGain = gainControl.getMaxGain();
//    }

    public List<AprilTagDetection> getCameraDetections() {
        currentDetections = aprilTag.getDetections();
        return currentDetections;
    }

    public AprilTagDetection getDetection(int id) {
        currentDetections = getCameraDetections();
        for (AprilTagDetection det : currentDetections) {
            if (det.id == id) {
                return det;
            }
        }
        return null;
    }
}
