/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.vision;

import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
/**
 *
 * @author Tomr
 */
public class OpticalInterface
{
    CriteriaCollection cc;

    public OpticalInterface()
    {
        cc = new CriteriaCollection();
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false);
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false);
    }

    public void updateImageAlgo(ColorImage image)
    {
        if(image != null)
        {
            try {
                BinaryImage thresholdImage = image.thresholdHSL(224, 255, 0, 255, 0, 101);
                BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 2);
                BinaryImage convexHullImage = bigObjectsImage.convexHull(false);
                BinaryImage filteredImage = convexHullImage.particleFilter(cc);

                ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();
                for(int q = 0; q < reports.length; q++)
                {
                    ParticleAnalysisReport r = reports[q];
                    System.out.println("X " +r.center_mass_x + " : Y  " + r.center_mass_y);
                }
                System.out.print("NUM PARTS " +filteredImage.getNumberParticles());
                filteredImage.free();
                convexHullImage.free();
                bigObjectsImage.free();
                thresholdImage.free();
                image.free();
            }
            catch (NIVisionException ex)
            {
                ex.printStackTrace();
            }
        }
        else
        {
            System.out.println("NO IMAGE AVALABE!");
        }
    }
}
