#include "counterUtils.h"
#include "interactive_counter.h"

double counter::countObjects()
        {
            bool splane = false, wplane = false;
            double *p, len, shelfLength = 420, cnt = 0;
            std::vector<cv::Point3d> normals(3); //  normals[1] = shelf normal (y), normals[2] = wall's (z), normals[0] = their cross product (x). 
            cv::Mat trans, nnz, srcInBox;
            std::vector<EPV::DepthImageCoords> nnzCoords; // real world coordinates

            if (_src.empty())
            {
                std::cout << "original grayscale image wasn`t loaded" << std::endl;
                return -1;
            }

        //#ifdef FITTING    
            // Plane fitting - need to find the planes in order to know the normals
            // EPV::CameraIntrinsics intrinsics = { 671.062439,671.062439,679.713806,369.511169,_src.cols,_src.rows };
            EPV::CameraIntrinsics intrinsics = { 402.6374816894531,402.6374816894531,343.8282775878906,245.70669555664062,_src.cols,_src.rows };
            // EPV::CameraIntrinsics intrinsics = { 390.748,390.748,314.01,243.3996,_src.cols,_src.rows };
            cv::Rect roi = { 0,0,_src.cols,_src.rows };
            std::vector<EPV::FittedPlane> results; 
            EPV::RansacParams params; 

            // initial parameters for UI taskbar
            int nnz2MinInliersUI = 30; 
            int inlierThreshFactorUI = 1000; 
            int numIterationsUI = 200; 
            params.refinePlane = false; // for Ransac - uses SVD to refine the vector
            params.nnz2MinInliers = nnz2MinInliersUI /1e3;
            params.inlierThreshFactor = inlierThreshFactorUI / 1e3;
            params.numIterations = (unsigned int)numIterationsUI;
            nnzCoords.clear();

            // Plane fitting
            iterativePlaneFitting(_src, intrinsics, roi, results, params, nnzCoords);
            
            // Extract the shelf and wall's normals
            for (auto result : results)
            {
                // save the normal of the surface		
                if (result.planeEquation[3] < 600) // the shelf's normal
                {
                    if (fabs(result.planeEquation[1]) > fabs(result.planeEquation[2]) && fabs(result.planeEquation[1]) > fabs(result.planeEquation[0]) && !splane)
                    {
                        normals[1].x = -result.planeEquation[0];
                        normals[1].y = -result.planeEquation[1];
                        normals[1].z = -result.planeEquation[2];
                        splane = true;
                    }
                }
                else // the wall's normal 
                {
                    if (!wplane)
                    {
                        normals[2].x = -result.planeEquation[0];
                        normals[2].y = -result.planeEquation[1];
                        normals[2].z = -result.planeEquation[2];
                        wplane = true;
                    }
                }
            }
            if(!splane || !wplane)
            {
                std::cout << "sorry, couldn't find one of the planes... :(";
                return -1;
            }
            
            // produce cross normal
            normals[0] = normals[1].cross(normals[2]);
            
            // 3X3 matrix for rotating the coordinate system to suit the shelf's system
            for (size_t i = 0; i < 3; i++)
            {
                _rotation.at<double>(0, i) = normals[i].x;
                _rotation.at<double>(1, i) = normals[i].y;
                _rotation.at<double>(2, i) = normals[i].z;
            }
        //#endif

            // 3X1 matrix for translating the coordinate system so the separator is in (0,0,0)
            cv::Mat translation(_separator);

            // NX3 matrix (point cloud) with real world coordinates for the src pixels
            nnz = cv::Mat::zeros(nnzCoords.size(), 3, CV_64FC1);
            double* nnzData = (double *)nnz.data;
            for (size_t i = 0; i < nnzCoords.size(); i++, nnzData+=3) {
                nnzData[0] = nnzCoords[i].x;
                nnzData[1] = nnzCoords[i].y;
                nnzData[2] = nnzCoords[i].z;
            }
            
            // real world coordinates after rotation and translation
            trans = TransformUtils::doInverseTransform(nnz, _rotation, translation);

            cv::Point offset(0,0);
            int _u = (_separator.x*intrinsics.fx)/_separator.z + intrinsics.px -offset.x;
            int _v = (_separator.y*intrinsics.fy)/_separator.z + intrinsics.py -offset.y;
            // is in box
            srcInBox = cv::Mat::zeros(_src.rows, _src.cols, CV_64FC1);
            objectType object = getObject();
            for (size_t i = 0; i < trans.rows; i++)
            {
                p = trans.ptr<double>(i);
                if (p[0] >= 0 && p[0] <= object._width && p[1] <= -10 && p[1] >= object._height && p[2] >= 0 && p[2] <= shelfLength)
                    srcInBox.at<double>(nnzCoords[i].v, nnzCoords[i].u) = p[2];
            }
            cv::Mat srcInBoxF;
            srcInBox.convertTo(srcInBoxF, CV_32FC1);
            // cv::imshow("srcInBox",srcInBox);
            // cv::waitKey(0);
            float range[] = { 1, 1025 };
            double per95 = EPV::computePercentilePoint(srcInBoxF, 0.999, range);
            double per5 = EPV::computePercentilePoint(srcInBoxF, 0.05, range);
            len = per95 - per5;
            cnt += len / object._depth;
            
            return cnt;
        }