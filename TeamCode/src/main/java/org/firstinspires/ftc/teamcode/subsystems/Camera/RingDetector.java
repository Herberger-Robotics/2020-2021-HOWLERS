package org.firstinspires.ftc.teamcode.subsystems.Camera;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RingDetector {

    private OpenCvCamera phoneCamera;
    private UGContourRingPipeline pipeline;

    public RingDetector(OpenCvCamera cam) {
        phoneCamera = cam;
        pipeline = new UGContourRingPipeline();

        phoneCamera.openCameraDevice();
        phoneCamera.setPipeline(pipeline);

        phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }



    public UGContourRingPipeline.Height getHeight() {
        return pipeline.getHeight();
    }
}
