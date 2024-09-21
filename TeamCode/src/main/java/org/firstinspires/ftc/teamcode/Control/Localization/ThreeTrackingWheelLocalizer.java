package org.firstinspires.ftc.teamcode.Control.Localization;
//Samson Started messing around

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.collections4.ListUtils;
import org.apache.commons.math3.linear.*;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Hardware.Physical.CAOSHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * Localizer based on three unpowered tracking omni wheels.
 */
public class ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.69; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    DcMotor verticalLeftEncoder, verticalRightEncoder;
    DcMotor horizontalEncoder;
    private Pose2d _poseEstimate = new Pose2d();
    private List<Double> lastWheelPositions = new ArrayList<>();
    private List<Double> wheelPositions = new ArrayList<>();
    private DecompositionSolver forwardSolver;

    double x,y,h = 0;
    double hPrevDist, vlPrevDist, vrPrevDist;

    //new readings
    double newVerticalLeft, newVerticalRight, newHorizontal, newHeading;
    //change in readings / calculated change
    double dVerticalLeft, dVerticalRight, dHorizontal, dHeading, dX, dY;
    double encoderMultiplier = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    Pose2d robotPose = new Pose2d(0,0,0);
    public ThreeTrackingWheelLocalizer(List<Pose2d> wheelPoses, HardwareMap hwMap) {
        CAOSHardware caosHardware = new CAOSHardware(hwMap);
        verticalLeftEncoder = caosHardware.verticalEncoderLeft;
        verticalRightEncoder = caosHardware.verticalEncoderRight;
        horizontalEncoder = caosHardware.horizontalEncoder;
        verticalLeftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalRightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        RealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i < 3; i++) {
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            Vector2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(i, 2,
                    positionVector.getX() * orientationVector.getY() - positionVector.getY() * orientationVector.getX());
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
        lastWheelPositions = new ArrayList<>();
    }

    Pose2d robotPoseDelta;
    public void update() {
        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
        wheelPositions = Arrays.asList(verticalLeftEncoder.getCurrentPosition() * encoderMultiplier,
                                       verticalRightEncoder.getCurrentPosition() * encoderMultiplier,
                                       horizontalEncoder.getCurrentPosition() * encoderMultiplier);
        List<Double> wheelDeltas;
        double sineTerm;
        double cosTerm;
        //get change in heading, Vertical encoder, and horizontal encoder
        if (!lastWheelPositions.isEmpty()) {
            wheelDeltas = IntStream.range(0, wheelPositions.size())
                    .mapToObj(i -> wheelPositions.get(i) - lastWheelPositions.get(i))
                    .collect(Collectors.toList());
        } else {
            lastWheelPositions = Arrays.asList(0.0,0.0,0.0);
            wheelDeltas = IntStream.range(0, wheelPositions.size())
                    .mapToObj(i -> wheelPositions.get(i) - lastWheelPositions.get(i))
                    .collect(Collectors.toList());
        }

        double[] robotPoseDelta = calculatePoseDelta(wheelDeltas);

        if(wheelPositions.get(2) < 0.0000001) {
            sineTerm = 1.0 - wheelPositions.get(2)*wheelPositions.get(2) / 6.0;
            cosTerm = wheelPositions.get(2)/2.0;
        } else {
            sineTerm = Math.sin(wheelPositions.get(2))/wheelPositions.get(2);
            cosTerm = (1 - Math.cos(wheelPositions.get(2)))/wheelPositions.get(2);
        }
        Vector2d fieldPositionOffset = new Vector2d(
                (sineTerm * robotPoseDelta[0] - cosTerm * robotPoseDelta[1]),
                cosTerm * robotPoseDelta[0] + sineTerm * robotPoseDelta[1]);

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionOffset.rotated(h), robotPoseDelta[2]);
        lastWheelPositions = wheelPositions;
        robotPose = new Pose2d(
                robotPose.getX() + fieldPoseDelta.getX(),
                robotPose.getY() + fieldPoseDelta.getY(),
                Angle.norm(robotPose.getHeading() + fieldPoseDelta.getHeading())
        );
        x = robotPose.getX();
        y = robotPose.getY();
        h = robotPose.getHeading();
    }
    public double[] getPoseEstimate() {
        return new double[]{robotPose.getX(), robotPose.getY(), robotPose.getHeading()};
    }

    public void setPoseEstimate(Pose2d value) {
        lastWheelPositions = new ArrayList<>();
        lastWheelPositions.add(0.0);
        lastWheelPositions.add(0.0);
        lastWheelPositions.add(0.0);

        robotPose = value;
    }

    //RADIANS
    public void setPoseEstimate(double x, double y, double h) {
        lastWheelPositions = new ArrayList<>();
        lastWheelPositions.add(0.0);
        lastWheelPositions.add(0.0);
        lastWheelPositions.add(0.0);

        robotPose = new Pose2d(x,y,h);
    }

    private double[] calculatePoseDelta(List<Double> wheelDeltas) {
        RealMatrix rawPoseDelta = forwardSolver.solve(MatrixUtils.createColumnRealMatrix(
                wheelDeltas.stream().mapToDouble(d -> d).toArray()
        ));
        return new double[]{
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        };
    }

    public double[] getRawValues(){
        return new double[]{wheelPositions.get(0), wheelPositions.get(1), wheelPositions.get(2)};
    }
}