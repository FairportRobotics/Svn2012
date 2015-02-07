package com.fairportfirst.frc2012.vision;

import com.sun.cldc.jna.BlockingFunction;
import com.sun.cldc.jna.NativeLibrary;
import com.sun.cldc.jna.Pointer;
import com.sun.cldc.jna.TaskExecutor;
import com.sun.cldc.jna.ptr.IntByReference;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Tyler the awsome sauce
 */
//extends WPILaptopCameraExtension
public class Camera {

    CriteriaCollection cc;      // the criteria for doing the particle filter operation
    ParticleAnalysisReport r;
    public int pos;
    int eqRecHeight;
    public double distance;
    ColorImage imageR;
    double scottysConstant = (2 * Math.tan(Math.toRadians(23.5)));
    double realHeight = 0.0;
    double tempVarTwo = 0.0;
    double tempVarOne = 0.0;
    public boolean hasImage = false;
    ParticleAnalysisReport[] targets = new ParticleAnalysisReport[4];

    public Camera() {
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 10, 400, false);
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 20, 400, false);
    }

    public int calImage(ColorImage imag, int placement) throws NIVisionException {
        /**
         * Do the image capture with the camera and apply the algorithm described above. This
         * sample will either get images from the camera or from an image file stored in the top
         * level directory in the flash memory on the cRIO. The file name in this case is "10ft2.jpg"
         *
         */
        try
        {
            imageR = imag;
            //ColorImage image = camera.getImage();     // comment if using stored images
            BinaryImage thresholdImage = imag.thresholdHSL(100, 138, 50, 255, 100, 255);   // keep only red objects
            BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 1);  // remove small artifacts
            BinaryImage convexHullImage = bigObjectsImage.convexHull(false);          // fill in occluded rectangles
            BinaryImage filteredImage = convexHullImage.particleFilter(cc);           // find filled in rectangles
            ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();  // get list of results
            for (int i = 0; i < reports.length; i++)
            {
                hasImage = true;
                r = reports[i];
                targets[i] = r;
                //System.out.println("CUPCAKE YOU AND ME BACK A DA PARK: " +NIVision.countParticles(image.image));
    //            eqRecHeight = (int) NIVision.MeasureParticle(image.image,  NIVision.countParticles(image.image), false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                pos = r.center_mass_x;
                //System.out.println("Particle: " + placement + ":  Center of mass x: " + r.center_mass_y);
                //System.out.println(filteredImage.getNumberParticles());
            }

            /**
             * all images in Java must be freed after they are used since they are allocated out
             * of C data structures. Not calling free() will cause the memory to accumulate over
             * each pass of this loop.
             */
            filteredImage.free();
            convexHullImage.free();
            bigObjectsImage.free();
            thresholdImage.free();
            imag.free();
            try
            {
                if(targets[placement].center_mass_x > 0)
                {
                    hasImage = true;
                }
                else
                {
                    hasImage = false;
                }
//                System.out.println("marsX: " + targets[placement].center_mass_x );
                return targets[placement].center_mass_x;
            }
            catch (NullPointerException e)
            {
                //System.out.print("---ERROR---[ Cannot find target ]");\
            }
        }
        catch(ArrayIndexOutOfBoundsException mad)
        {

        }
        hasImage = false;
        pos = 0;
        return 0;
    }

    public double getDistance(int target) {
        try {
            tempVarOne = targets[target].boundingRectHeight * targets[target].boundingRectWidth;
            tempVarTwo = (tempVarOne - targets[target].particleArea) / 2;
            tempVarOne = ((2 * tempVarTwo) / targets[target].boundingRectWidth);
            realHeight = (targets[target].boundingRectHeight - (tempVarOne));
            System.out.println(">>>>>>>>>REALH: " + realHeight);
            //double thag = ((heigth * heigth) + (targets[target].boundingRectWidth * targets[target].boundingRectWidth));
            //realHeight = Math.sqrt(pthag);
            //tempVarTwo = ((targets[target].boundingRectHeight * targets[target].boundingRectWidth) - targets[target].particleArea)/targets[target].boundingRectWidth;
            //realHeight = Math.sqrt(((targets[target].boundingRectWidth * targets[target].boundingRectWidth) + (realHeight * realHeight)));
            //realHeight = (Math.sqrt(((targets[target].boundingRectWidth * targets[target].boundingRectWidth) + (realHeight * realHeight))));
            //distance = (7680/realHeight )/scottysConstant;
            distance = (18 * 320) / (realHeight * scottysConstant);
            //System.out.println("pArea: " + targets[target].particleArea);
            //System.out.println("Width: " + realHeight);
            System.out.println(">>>>>>>>>>DISTANCE: " + distance + " POS: " + target + " get: " + getPreciseRecPos(0));
            return Math.min(distance, 324);
        } catch (ArrayIndexOutOfBoundsException e) 
        {
            //e.printStackTrace();
            //System.out.println("---ERROR---[ target specified out of range ]");
        } 
        catch (NullPointerException e)
        {
            //e.printStackTrace();
           // System.out.println("---ERROR--- [ no you are a null pointer exeption ]");
        }
        distance = 0;
        return 0.0;
    }

    public int getMotorDirection(int target) {
        try {
            if (targets[target].center_mass_x > 162) {
                return -1;
            } else if (targets[target].center_mass_x < 158) {
                return 1;
            }
            else 
            {
                return 0;
            }
        } 
        catch (NullPointerException e)
        {
            //System.out.println("---ERROR--- [ no you are a null pointer exeption ]");
        }
        catch(IndexOutOfBoundsException e)
        {
            //System.out.println("---ERROR---[ target specified out of range ]");
        }
        return 0;
    }

    //0 = top most rec
    // 1 = middle left rec
    // 2 = middle right rec
    //3 = bottom most rec
    /**
     * Returns the position of a requested
     * @param wantedPos the position of the rectangle to request
     * @return the position of the requested rectangle
     */
    public int getPreciseRecPos(int wantedPos)
    {                                                                                                                                                                                                                                                                       // I'm in space.
        try
        {
            int mostTop = 240;
            int block = 0;
            int mostSide = 320;
            if (wantedPos == 3)
            {
                mostTop = 0;
            }
            if (wantedPos == 2)
            {
                mostSide = 0;
            }
            for (int q = 0; q < targets.length; q++)
            {
                if (targets[q].center_mass_y < mostTop && wantedPos == 0)
                {
                    mostTop = targets[q].center_mass_y;
                    block = q;
                }
                else if (targets[q].center_mass_y > mostTop && wantedPos == 3)
                {
                    mostTop = targets[q].center_mass_y;
                    block = q;
                }
                else if (targets[q].center_mass_x < mostSide && wantedPos == 1)
                {
                    mostSide = targets[q].center_mass_x;
                    block = q;
                }
                else if (targets[q].center_mass_x > mostSide && wantedPos == 2)
                {
                    mostSide = targets[q].center_mass_x;
                    block = q;
                }
            }
            //System.out.print("Actual thing: " + block);
            return block;
        }
        catch (NullPointerException e)
        {
           // System.out.println("---ERROR--- [ no you are a null pointer exeption ]");
        }
        return 0;
    }
    public int getTargetCenter(int q)
    {
        try
        {
            return targets[q].center_mass_x;
        }
        catch(NullPointerException e)
        {
            //oh well
        }
        catch(ArrayIndexOutOfBoundsException e)
        {
            //ya
        }
        return 0;
    }
    public double getRPMFromDistance(double distance)
    {
        //EQ derived form data points!
//        return (2.0*(distance*distance)) - (576.72*distance) + 42446;
        return (0.74*(distance*distance)) - (199.82*distance) + 15719.17;
    }
    public double getRPMFromPer(double per)
    {
        //YeS
        return (5100/100.0) * per;
    }
}
