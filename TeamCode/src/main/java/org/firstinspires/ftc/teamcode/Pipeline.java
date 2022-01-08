package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.qual.C;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    // Color Evaluation
    public int lowX = 235;
    public int lowY = 120;
    public int lowZ = 120;
    public int highX = 255;
    public int highY = 255;
    public int highZ = 255;

    // Image Evaluation
    public double pixelCount;

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2RGBA);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);

        Scalar lowColorValue = new Scalar(lowX, lowY, lowZ);
        Scalar highColorValue = new Scalar(highX, highY, highZ);
        Core.inRange(mat, lowColorValue, highColorValue, mat);

        int whiteCount = 0;
        int hitCount = 0;
        int middleRow = mat.height()/2;
        for (int i = 0; i < mat.width(); i++) {
            if (mat.get(middleRow, i)[0] == 255d) {
                whiteCount =+ i;
                hitCount += 1;
            }
        }
        if (hitCount != 0) {
            setPixelCount(whiteCount/hitCount);
        } else {
            setPixelCount(0);
        }


//        setPixelCount(mat.get(0,0)[0]);

        return mat;
    }

    public double getPixelCount() {
        return pixelCount;
    }

    private void setPixelCount(double newPC) {
        pixelCount = newPC;
    }

    public int getFrameWidth() {
        return mat.width();
    }

    public int getFrameHeight() {
        return mat.height();
    }
}
