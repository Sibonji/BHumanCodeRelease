#include "WhiteLinesZedDetector.h"

WhiteLinesZedDetector::WhiteLinesZedDetector(LineSizePixProviderZed& line_size_provider) 
    : line_size_provider(line_size_provider) {
    // Initialize any necessary resources
}

WhiteLinesZedDetector::~WhiteLinesZedDetector() {
    // Clean up any resources
}

cv::Mat WhiteLinesZedDetector::processFrame(const cv::Mat& input_frame, std::vector<CornerResultInSelf>& valid_corners_out, std::vector<LineResultInSelf>& valid_lines_out) {
    // Store the input frame
    last_processed_frame = input_frame.clone();
    
    // Convert to grayscale for integral image computation
    cv::Mat gray;
    cv::cvtColor(input_frame, gray, cv::COLOR_BGR2GRAY);
    
    // Compute integral image
    cv::integral(gray, integral_image, CV_32S);


    int row_nb = input_frame.rows;
    int col_nb = input_frame.cols;


    //Defining verbose image
    // cv::Mat im(row_nb, col_nb, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat im = last_processed_frame.clone();


    // Lines detection processing
    Mat isum = integral_image;

    Mat iresx = Mat(im.rows, im.cols, CV_8UC1);
    Mat iresy = Mat(im.rows, im.cols, CV_8UC1);
    Mat iresrgb = Mat(im.rows, im.cols, CV_8UC3);
    iresx.setTo(0);
    iresy.setTo(0);
    iresrgb.setTo(0);

    int win_height=10; 

    //Divider to check only Nth scanline. Greatly improves speed with almost no quality degradation. 4 seems to be enoughth for everybody (c)
    int scanline_divider = 4;

    //x traverse/scalnine
    for(int yn=0; yn < (im.rows-win_height)/scanline_divider; yn++) {
        int y = yn*scanline_divider;
            for(int x=0; x < im.cols-1; x++) {
            int win_line_width = -(planex_a*x + planex_b*y + planex_d)/planex_c;
            // int win_line_width = 50; // HARDCODED
            if(win_line_width<=0) continue;
            if(win_line_width<3) win_line_width = 3;
            if (x>=im.cols-win_line_width*3-1) break; //not to hit the border
            int sleft   = getRegionSum(isum, x,y, win_line_width, win_height);
            int smiddle = getRegionSum(isum, x+win_line_width, y, win_line_width, win_height);
            int sright  = getRegionSum(isum, x+win_line_width*2, y, win_line_width, win_height);
            int diff = (smiddle-sleft)*(smiddle-sright)/(win_line_width*win_height)/(win_line_width*win_height)/20;
            if((smiddle-sright)<0) diff=0;
            if((smiddle-sleft)<0) diff=0;
            if(diff<0) diff = 0;
            if(diff>255) diff = 255; // This forms a plateus in case of several adjacent 255's. TODO: fix this - switch to float image etc
            iresx.at<uchar>(y+win_height/2,x+(win_line_width*3)/2) = diff;
            iresx.at<uchar>(y+win_height/2,x+(win_line_width*3)/2+1) = diff; //Dirty hack to remove empty pixels when linewidth changes a lot, TODO: refactor - start the window from meddle, not from left edge
        }
    }

    //y traverse/scanline
    for(int xn=0; xn < (im.cols-win_height)/scanline_divider; xn++) {
        int x = xn*scanline_divider;
            for(int y=0; y < im.rows-1; y++) {
            int win_line_width = -(planey_a*x + planey_b*y + planey_d)/planey_c;
            // int win_line_width = 50; // HARDCODED
            if(win_line_width<=0) continue;
            if(win_line_width<3) win_line_width = 3;
            if (y>=im.rows-win_line_width*3-1) break; //not to hit the border          
            int sleft   = getRegionSum(isum, x,y, win_height, win_line_width);
            int smiddle = getRegionSum(isum, x,y+win_line_width, win_height, win_line_width);
            int sright  = getRegionSum(isum, x,y+win_line_width*2, win_height, win_line_width);
            int diff = (smiddle-sleft)*(smiddle-sright)/(win_line_width*win_height)/(win_line_width*win_height)/20;
            if((smiddle-sright)<0) diff=0;
            if((smiddle-sleft)<0) diff=0;          
            if(diff<0) diff = 0;
            if(diff>255) diff = 255;
            iresy.at<uchar>(y+(win_line_width*3)/2,x+win_height/2) = diff;
            iresy.at<uchar>(y+(win_line_width*3)/2+1,x+win_height/2) = diff; //Dirty hack to remove empty pixels when linewidth changes a lot, TODO: refactor - start the window from meddle, not from left edge
        }
    }


    //Now let's do non-maxima suppression on traverse results - this will thin results to single pixel width in the middle of detected line (if line quality is not too bad)
    Mat nmsx_pt = cv::Mat(row_nb, col_nb, CV_8UC1); // NMS result with plateus
    Mat nmsy_pt = cv::Mat(row_nb, col_nb, CV_8UC1);

    Mat nmsx = cv::Mat(row_nb, col_nb, CV_8UC1);
    Mat nmsy = cv::Mat(row_nb, col_nb, CV_8UC1);
    nmsx.setTo(0);
    nmsy.setTo(0);

    //Suppress local maximas in 30 pixel window with intensity thresold
    int line_intencity_treshold = 50; //with treshold=3 even wery subtile lines are detected, but too many false positives (edges of a ball, etc)
    //int line_intencity_treshold = 100;

    //TODO: remove hardcode
    non_maxima_suppression(iresx, nmsx_pt, 30, 1, line_intencity_treshold);
    non_maxima_suppression(iresy, nmsy_pt, 1, 30, line_intencity_treshold);
    plateuThinningX(nmsx_pt, nmsx);
    plateuThinningY(nmsy_pt, nmsy);

    //Resulting mat for both x and y results in single channel
    Mat nms = cv::Mat(row_nb, col_nb, CV_8UC1);
    nms.setTo(0);

    //Checking for proper green color at left and right sides of a line
    for(int y=0; y < im.rows-win_height; y++) {
        for(int x=0; x < im.cols; x++) {
            if(nmsx.at<uchar>(y,x)>0) {        
                int val = 255;
                // im.at<Vec3b>(y,x) = Vec3b(255, 0, 0);
                // int win_line_width = -(planex_a*x + planex_b*y + planex_d)/planex_c;
                // if(win_line_width<=0) continue;
                // if(win_line_width<3) win_line_width = 3;
                // if (x>=im.cols-win_line_width*3-1) break; //not to hit the border
                // int gleft = countNonZero(green(Rect(x,y, win_line_width,win_height)));
                // int gright = countNonZero(green(Rect(x+win_line_width*2,y, win_line_width,win_height)));
                // if(gleft < 0.05*(float)win_line_width*(float)win_height) val = 0;  //5% green coverage, TODO: remove hardcode 
                // if(gright < 0.05*(float)win_line_width*(float)win_height) val = 0;  //5% green coverage, TODO: remove hardcode         
                // if(val>0) nms.at<uchar>(y,x) = val;
                nms.at<uchar>(y,x) = val;
            }
        }
    }
    for(int x=0; x < im.cols-win_height; x++) {
        for(int y=0; y < im.rows; y++) {
            if(nmsy.at<uchar>(y,x)>0) {
                int val = 255;
                // im.at<Vec3b>(y,x) = Vec3b(255, 0, 0);
                // int win_line_width = -(planey_a*x + planey_b*y + planey_d)/planey_c;
                // if(win_line_width<=0) continue;
                // if(win_line_width<3) win_line_width = 3;
                // if (y>=im.rows-win_line_width*3-1) break; //not to hit the border 
                // int gleft = countNonZero(green(Rect(x,y, win_height,win_line_width)));
                // int gright = countNonZero(green(Rect(x,y+win_line_width*2, win_height,win_line_width)));       
                // if(gleft < 0.05*(float)win_line_width*(float)win_height) val = 0;  //5% green coverage, TODO: remove hardcode
                // if(gright < 0.05*(float)win_line_width*(float)win_height) val = 0;  //5% green coverage, TODO: remove hardcode        
                // if(val>0) nms.at<uchar>(y,x) = val;
                nms.at<uchar>(y,x) = val;
            }

        }
    }

    //Probabilistic Line Transform after NMS and green check
    dilate(nms, nms, Mat(), Point(-1, -1), 1, 1, 1);
    vector<Vec4i> linesP_all;
    int pointsThreshold = 100/scanline_divider;
    int lengthThreshold = 50;
    int gapThreshold = 15; //25 from 2018 version gives too musk false positives;
    HoughLinesP(nms, linesP_all, 2, 2*CV_PI/180, pointsThreshold, lengthThreshold, gapThreshold );

    //Extending lines length with a half line width because inter-corss square is usually not detected (line width on it is incorrect)
    // double extend_in_pix = 5.0;
    for( size_t i = 0; i < linesP_all.size(); i++ )
    {
        Vec4i l = linesP_all[i];
        double len = sqrt(pow(l[2]-l[0], 2) + pow(l[3]-l[1], 2));
        double ldx = (l[2]-l[0])/len;
        double ldy = (l[3]-l[1])/len;
        double win_line_width_p0_x = -(planex_a*l[0] + planex_b*l[1] + planex_d)/planex_c;
        double win_line_width_p0_y = -(planey_a*l[0] + planey_b*l[1] + planey_d)/planey_c;
        double win_line_width_p1_x = -(planex_a*l[2] + planex_b*l[3] + planex_d)/planex_c;
        double win_line_width_p1_y = -(planey_a*l[2] + planey_b*l[3] + planey_d)/planey_c;        

        linesP_all[i][2] += ldx * win_line_width_p1_x*0.5;
        linesP_all[i][0] -= ldx * win_line_width_p0_x*0.5;
        linesP_all[i][3] += ldy * win_line_width_p1_y*0.5;
        linesP_all[i][1] -= ldy * win_line_width_p0_y*0.5;
    }

    //Merging similar segments
    //TODO: Not effective and leave small lines as artifacts, need to be rewritten (!)
    bool changes_done_line = true;
    while(changes_done_line) {
        changes_done_line = false;
        for( size_t i = 0; i < linesP_all.size(); i++ ) {
            for( size_t j = 0; j < linesP_all.size(); j++ ) {        
                if( (i!=j) 
                && (get_line_magnitude(linesP_all[i][0], linesP_all[i][1], linesP_all[i][2], linesP_all[i][3])>1.0) 
                && (get_line_magnitude(linesP_all[j][0], linesP_all[j][1], linesP_all[j][2], linesP_all[j][3])>1.0) ) {

                if( (get_segnent2segment_distance(linesP_all[i][0], linesP_all[i][1], linesP_all[i][2], linesP_all[i][3],
                                                linesP_all[j][0], linesP_all[j][1], linesP_all[j][2], linesP_all[j][3])<10.0)

                    &&(fabs(get_line2line_angle(linesP_all[i][0], linesP_all[i][1], linesP_all[i][2], linesP_all[i][3],
                                        linesP_all[j][0], linesP_all[j][1], linesP_all[j][2], linesP_all[j][3]))*180.0/M_PI<5.0) ) {
                        //This segments should be merged
                        float mx1, my1, mx2, my2;
                        merge_two_segments( linesP_all[i][0], linesP_all[i][1], linesP_all[i][2], linesP_all[i][3],
                                            linesP_all[j][0], linesP_all[j][1], linesP_all[j][2], linesP_all[j][3],
                                            &mx1, &my1, &mx2, &my2);
                        linesP_all[j][0] = mx1;
                        linesP_all[j][1] = my1;
                        linesP_all[j][2] = mx2;
                        linesP_all[j][3] = my2;
                        linesP_all[i][0] = 0;
                        linesP_all[i][1] = 0;
                        linesP_all[i][2] = 0;
                        linesP_all[i][3] = 0;
                        changes_done_line = true;          
                    }
                }
            }
        }
    }

    vector<Vec4i> linesP;
    for( size_t i = 0; i < linesP_all.size(); i++ )
    {
        if(get_line_magnitude(linesP_all[i][0], linesP_all[i][1], linesP_all[i][2], linesP_all[i][3])>1.0) 
            linesP.push_back(linesP_all[i]);  
    }
    //std::cout << "Segments merge results: before=" << linesP_all.size() <<", after=" << linesP.size() << std::endl;


    // Draw merged segments on input image
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        //Scalar color = Scalar(rand()%255,rand()%255,rand()%255 );
        Vec4i l = linesP[i];
    
        line( im, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,175,100), 1);
    }    
    
    //Check for intersections and fill an array of corner candidates
    std::vector<CornerCandiateInSelf> cornerCandidates;  
    // float maxDist = std::max(Constants::field.field_length + Constants::field.border_strip_width_x*2, Constants::field.field_width + Constants::field.border_strip_width_x*2);
    float maxDist = 9.0 + 1.0;
    for( size_t i = 0; i < linesP.size(); i++ ) {
        for( size_t j = i+1; j < linesP.size(); j++ ) {
            float i_x, i_y;
            Vec4i la = linesP[i];
            Vec4i lb = linesP[j];
            char intersect = get_line_intersection(la[0], la[1], la[2], la[3], 
                                                            lb[0], lb[1], lb[2], lb[3],
                                                            &i_x, &i_y);
            if(intersect) {
                cv::Point2f ps0, ps1, ps2, ps3, pi;
                try
                {
                    // ps0 = cs->robotPosFromImg(la[0], la[1]);
                    // ps1 = cs->robotPosFromImg(la[2], la[3]);
                    // ps2 = cs->robotPosFromImg(lb[0], lb[1]);
                    // ps3 = cs->robotPosFromImg(lb[2], lb[3]);
                    // pi = cs->robotPosFromImg(i_x, i_y);
                    ps0 = line_size_provider.robotCentricFlatPointOnPlaneFromPixInMetersCv(la[0], la[1]);
                    ps1 = line_size_provider.robotCentricFlatPointOnPlaneFromPixInMetersCv(la[2], la[3]);
                    ps2 = line_size_provider.robotCentricFlatPointOnPlaneFromPixInMetersCv(lb[0], lb[1]);
                    ps3 = line_size_provider.robotCentricFlatPointOnPlaneFromPixInMetersCv(lb[2], lb[3]);
                    pi = line_size_provider.robotCentricFlatPointOnPlaneFromPixInMetersCv(i_x, i_y);
                    // std::cout << "ps0=" << ps0 << std::endl;
                    // std::cout << "ps1=" << ps1 << std::endl;
                    // std::cout << "ps2=" << ps2 << std::endl;
                    // std::cout << "ps3=" << ps3 << std::endl;
                    // std::cout << "pi=" << pi << std::endl;

                }
                catch (const std::runtime_error& exc)
                {
                    continue;
                }
                float length_a = my_norm(ps0,ps1);
                float length_b = my_norm(ps2,ps3);
                if(    
                       (ps0.x < maxDist)&&(ps0.y < maxDist)&&(ps1.x < maxDist)&&(ps1.y < maxDist) //intersection point is farther than field size
                    && (ps2.x < maxDist)&&(ps2.y < maxDist)&&(ps3.x < maxDist)&&(ps3.y < maxDist) //intersection point is farther than field size
                    // && ( sqrt(pow(pi.x,2)+ pow(pi.y,2)) >0.2 )  //Also skip here 0.2 meters radius zone around robot to ignore handling bar, etc
                    && (length_a>0.05 && length_b>0.05)  //filter out penalty mark
                )
                {
                    float thetaDeg = get_line2line_angle(ps0.x, ps0.y, ps1.x, ps1.y, ps2.x, ps2.y, ps3.x, ps3.y) * 180.0/ M_PI;
                    float deltaDeg = 90.0 - fabs(thetaDeg);
                    // std::cout << "deltaDeg=" << deltaDeg << std::endl;
                    if(fabs(deltaDeg)<20.0)
                    // if(1)
                    { //Allow max 20 degrees misalignment from ideal 90 degrees intersection
                        //TODO: some shit happening here with check of 90 angle on frame 737 in 2019_10_15_21h37m34s logs
                        //std::cout << "thetaDeg=" << thetaDeg <<", fabs(deltaDeg) = " << fabs(deltaDeg) << std::endl;
                        
                        //Good corner detected
                        
                        //Draw it
                        cv::Point2f Ufa(((float) la[0]), ((float) la[1]));
                        cv::Point2f Vfa(((float) la[2]), ((float) la[3]));
                        cv::Point2f Ufb(((float) lb[0]), ((float) lb[1]));
                        cv::Point2f Vfb(((float) lb[2]), ((float) lb[3]));
                        cv::Point2f C(((float) i_x), ((float) i_y));

                        cv::circle(im, C, 5, Scalar(0,0,255), -1);
                        cv::line(im, Ufa, Vfa, Scalar(0,255,255), 3);
                        cv::line(im, Ufb, Vfb, Scalar(0,255,255), 3);

                        CornerCandiateInSelf cornerCandidate;
                        cornerCandidate.pa0 = ps0;
                        cornerCandidate.pa1 = ps1;
                        cornerCandidate.pb0 = ps2;
                        cornerCandidate.pb1 = ps3;            
                        cornerCandidate.corner = pi;
                        cornerCandidate.score = my_norm(cornerCandidate.pa0, cornerCandidate.pa1) + my_norm(cornerCandidate.pb0, cornerCandidate.pb1);
                        cornerCandidate.valid = true;
                        cornerCandidates.push_back(cornerCandidate);
                    }
               }
            }

        }
    }
    
    
    //Merge close corners by keeping only one corner in a cluster with largest line lengths. Not optimal but works
    bool changes_done = true;
    while(changes_done) {
        changes_done = false;
        for( size_t i = 0; i < cornerCandidates.size(); i++ ) {
            for( size_t j = i+1; j < cornerCandidates.size(); j++ ) {
                cv::Point2f c1 =  cornerCandidates[i].corner; 
                cv::Point2f c2 =  cornerCandidates[j].corner;
                if((cornerCandidates[i].valid)&&(cornerCandidates[j].valid)) {
                    if(my_norm(c1,c2)<0.1) {
                        //Corners are too close, filter one of them
                        if(cornerCandidates[i].score>cornerCandidates[j].score) cornerCandidates[j].valid = false;
                        else cornerCandidates[i].valid = false;
                        changes_done = true;
                    }
                }
            }
        }
    }  

    // Populate valid_corners_out with only valid corners
    // TODO: check the length of all corner lines, and output two corners for a T-cross, and one corner for L-cross
    valid_corners_out.clear();
    for (const CornerCandiateInSelf& c : cornerCandidates) {
        if (c.valid) {
            // For each crossing check each of four sub-candidates
            cv::Point2f in_pa[4];
            cv::Point2f in_pb[4];
            cv::Point2f in_corner = c.corner;
            in_pa[0] = c.pa0; in_pb[0] = c.pb0;
            in_pa[1] = c.pa0; in_pb[1] = c.pb1;
            in_pa[2] = c.pa1; in_pb[2] = c.pb0;
            in_pa[3] = c.pa1; in_pb[3] = c.pb1;

            // Iterate for all subcandidates and pick the best one with longest segnents
            double best_subcandidate_score = 0;
            size_t best_subcandidate_index = 0;
            for(size_t i=0; i<4; i++) {
                
                cv::Point2f out_a;
                cv::Point2f out_b;
                cv::Point2f out_corner;
                cv::Point2f a = in_pa[i] - in_corner;
                cv::Point2f b = in_pb[i] - in_corner;
                double subcandidate_score = sqrt(pow(a.x, 2) + pow(a.y, 2)) + sqrt(pow(b.x, 2) + pow(b.y, 2));
                if(subcandidate_score > best_subcandidate_score) {
                    best_subcandidate_score = subcandidate_score;
                    best_subcandidate_index = i;
                }
            }

            // Check the direction of best candidate and return
            cv::Point2f best_a = in_pa[best_subcandidate_index] - in_corner;
            cv::Point2f best_b = in_pb[best_subcandidate_index] - in_corner;            
            double cross_product_z_component = best_a.x * best_b.y - best_b.x * best_a.y;

            cv::Point2f out_pa;
            cv::Point2f out_pb;
            cv::Point2f out_corner;
            // Rotation from a to b should be counter-clockwise. Check it as a sign of cross product
            if(cross_product_z_component<=0) {
                //We need to swap
                out_pa = in_pb[best_subcandidate_index];
                out_pb = in_pa[best_subcandidate_index];
                out_corner = in_corner;
            } else {
                out_pa = in_pa[best_subcandidate_index];
                out_pb = in_pb[best_subcandidate_index];
                out_corner = in_corner;  
            }            

            CornerResultInSelf cr;
            cr.pa = out_pa;
            cr.pb = out_pb;
            cr.corner = out_corner;
            valid_corners_out.push_back(cr);
            std::cout << "c.corner=" << c.corner << std::endl;
            
        }
    }  

    // Populate valid_corners_out with only valid corners, rotating by current_head_yaw_
    // valid_corners_out.clear();
    // double cos_yaw = cos(current_head_yaw_);
    // double sin_yaw = sin(current_head_yaw_);
    // for (const auto& c : cornerCandidates) {
    //     if (c.valid) {
    //         CornerCandiateInSelf rotated = c;
    //         auto rotate = [&](const cv::Point2f& pt) -> cv::Point2f {
    //             return cv::Point2f(
    //                 cos_yaw * pt.x - sin_yaw * pt.y,
    //                 sin_yaw * pt.x + cos_yaw * pt.y
    //             );
    //         };
    //         rotated.pa0 = rotate(c.pa0);
    //         rotated.pa1 = rotate(c.pa1);
    //         rotated.pb0 = rotate(c.pb0);
    //         rotated.pb1 = rotate(c.pb1);
    //         rotated.corner = rotate(c.corner);
    //         valid_corners_out.push_back(rotated);
    //     }
    // }      

    // Populate valid_lines output array
    valid_lines_out.clear();
        
    // Drawing simple RadarImg for debug
    // drawSimpleRadarImg();    
    Mat radarImg = Mat(800, 800, CV_8UC3);
    cv::Point2f center = cv::Point2f(radarImg.cols/2, radarImg.rows/2);
    float pix_in_m = 100;
    radarImg.setTo(0);   
    cv::line(radarImg, cv::Point2f(0, radarImg.rows/2), cv::Point2f(radarImg.cols, radarImg.rows/2), cv::Scalar(120,120,120), 1);
    cv::line(radarImg, cv::Point2f(radarImg.cols/2, 0), cv::Point2f(radarImg.cols/2, radarImg.rows), cv::Scalar(120,120,120), 1);
    cv::circle(radarImg, center, 1.0 * pix_in_m, cv::Scalar(120,120,120), 1);
    cv::circle(radarImg, center, 2.0 * pix_in_m, cv::Scalar(120,120,120), 1);
    cv::circle(radarImg, center, 3.0 * pix_in_m, cv::Scalar(120,120,120), 1);
    for( size_t i = 0; i < cornerCandidates.size(); i++ ) {
        if(cornerCandidates[i].valid) {
            cv::Point2f pa0i = cv::Point2f(-cornerCandidates[i].pa0.y, -cornerCandidates[i].pa0.x)*pix_in_m + center;
            cv::Point2f pa1i = cv::Point2f(-cornerCandidates[i].pa1.y, -cornerCandidates[i].pa1.x)*pix_in_m + center;
            cv::Point2f pb0i = cv::Point2f(-cornerCandidates[i].pb0.y, -cornerCandidates[i].pb0.x)*pix_in_m + center;
            cv::Point2f pb1i = cv::Point2f(-cornerCandidates[i].pb1.y, -cornerCandidates[i].pb1.x)*pix_in_m + center;
            cv::line( radarImg, pa0i, pa1i, Scalar(255,255,255), 2);
            cv::line( radarImg, pb0i, pb1i, Scalar(255,255,255), 2);
        }
    }

    cv::imshow("RadarImg", radarImg);

    // Return the result frame
    // return iresx;
    // return nmsx;
    // cv::circle( im, Point(im.cols/2, im.rows/2), 5,  Scalar(0,0,255), 1);
    

    return im;    
} 


void WhiteLinesZedDetector::setLineSizePixPlaneApproximationCoeff(double xa, double xb, double xc, double xd, 
                                                                  double ya, double yb, double yc, double yd) {
  planex_a = xa;
  planex_b = xb;
  planex_c = xc;
  planex_d = xd;
  planey_a = ya;
  planey_b = yb;
  planey_c = yc;
  planey_d = yd;  
}


double WhiteLinesZedDetector::my_norm(cv::Point2f a, cv::Point2f b) {
  return sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) );
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored in the floats i_x and i_y.
char WhiteLinesZedDetector::get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y, 
    float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    float ds,dt;
    ds = (-s2_x * s1_y + s1_x * s2_y);
    dt = (-s2_x * s1_y + s1_x * s2_y);
    
    if((ds!=0)&&(dt!=0)) {

	    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / ds;
	    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / dt;

	    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	    {
		// Collision detected
		if (i_x != NULL)
		    *i_x = p0_x + (t * s1_x);
		if (i_y != NULL)
		    *i_y = p0_y + (t * s1_y);
		return 1;
	    }
    }

    return 0; // No collision
}

int WhiteLinesZedDetector::getRegionSum(Mat isum, int x, int y, int w, int h) {

      int tl= isum.at<int>(y,x);
      int tr= isum.at<int>(y,x+w);
      int bl= isum.at<int>(y+h,x);
      int br= isum.at<int>(y+h,x+w);
      return br-bl-tr+tl;
}

void WhiteLinesZedDetector::non_maxima_suppression(const cv::Mat& image, cv::Mat& mask, int sizex, int sizey, int threshold) {
    // find pixels that are equal to the local neighborhood not maximum
    // This will keep 'plateaus' (multiple adjacent max pixels)
    // TODO: make the kernel of structuring element pick appropriate size of a line width (not constant alongside image)
    Mat kernel = getStructuringElement(cv::MORPH_RECT, Size(sizex,sizey));
    //cv::dilate(image, mask, cv::Mat());
    cv::dilate(image, mask, kernel);
    for(int y=0; y < image.rows; y++) {
      for(int x=0; x < image.cols; x++) {
        if((image.at<uchar>(y,x) < mask.at<uchar>(y,x)) || (image.at<uchar>(y,x)<threshold) ) mask.at<uchar>(y,x) = 0;
        else mask.at<uchar>(y,x) = 255;
      }
    }
}

void WhiteLinesZedDetector::plateuThinningX(const cv::Mat& image, cv::Mat& mask) {    
    //x traverse
    for(int y=0; y < image.rows; y++) {
        size_t i = 1;
        while (i < image.cols-1) {
            if (image.at<uchar>(y,i) > image.at<uchar>(y,i-1) && image.at<uchar>(y,i) > image.at<uchar>(y,i+1)) {
                // One pix max
                mask.at<uchar>(y,i) = 255;
                i++;
            }
            else if (image.at<uchar>(y,i) > image.at<uchar>(y,i-1) && image.at<uchar>(y,i) == image.at<uchar>(y,i+1)) {
                // Start of plateau
                size_t start = i;
                while (i < image.cols - 1 && image.at<uchar>(y,i) == image.at<uchar>(y,i+1)) {
                    i++;
                }
                size_t end = i;
                size_t center = (start + end) / 2;
                mask.at<uchar>(y,center) = 255;
                i = end + 1;
            }
            else {
                i++;
            }
        }
    }    
}

void WhiteLinesZedDetector::plateuThinningY(const cv::Mat& image, cv::Mat& mask) {    
    // traverse
    for(int x=0; x < image.cols; x++) {
        size_t i = 1;
        while (i < image.rows-1) {
            if (image.at<uchar>(i,x) > image.at<uchar>(i-1,x) && image.at<uchar>(i,x) > image.at<uchar>(i+1,x)) {
                // One pix max
                mask.at<uchar>(i,x) = 255;
                i++;
            }
            else if (image.at<uchar>(i,x) > image.at<uchar>(i-1,x) && image.at<uchar>(i,x) == image.at<uchar>(i+1,x)) {
                // Start of plateau
                size_t start = i;
                while (i < image.rows - 1 && image.at<uchar>(i,x) == image.at<uchar>(i+1,x)) {
                    i++;
                }
                size_t end = i;
                size_t center = (start + end) / 2;
                mask.at<uchar>(center,x) = 255;
                i = end + 1;
            }
            else {
                i++;
            }
        }
    }    
}





float WhiteLinesZedDetector::get_line2line_angle(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) //in radians
{
	float a = x1 - x2;
	float b = y1 - y2;
	float c = x3 - x4;
	float d = y3 - y4;
	//
	float cos_angle , angle;
	float mag_v1 = sqrt(a*a + b*b);
	float mag_v2 = sqrt(c*c + d*d);
	if((mag_v1 * mag_v2)!=0) {
		cos_angle = (a*c + b*d) / (mag_v1 * mag_v2);
		if(fabs(cos_angle)>1) return 0;
		angle = acos(cos_angle);
		return angle;
	}
	return 0;
}


//Rewritten from https://stackoverflow.com/questions/45531074/how-to-merge-lines-after-houghlinesp
float WhiteLinesZedDetector::get_line_magnitude(float x1, float y1, float x2, float y2) {
  //Get line (aka vector) length'
  return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

float WhiteLinesZedDetector::get_point2line_distance(float px, float py, float x1, float y1, float x2, float y2) {
  //Get distance between point and line
  //http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba

  float LineMag = get_line_magnitude(x1, y1, x2, y2);
  if (LineMag < 0.00000001) return 9999; //line are too short for proper computations

  float u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)));
  float u = u1 / (LineMag * LineMag);

  if ((u < 0.00001) || (u > 1)) {
    // point does not fall within the line segment, take the shorter distance to an endpoint
    float ix = get_line_magnitude(px, py, x1, y1);
    float iy = get_line_magnitude(px, py, x2, y2);
    return std::min(ix,iy);
  } else {
    //Intersecting point is on the line, use the formula
    float ix = x1 + u * (x2 - x1);
    float iy = y1 + u * (y2 - y1);
    return get_line_magnitude(px, py, ix, iy);
  }
}

float WhiteLinesZedDetector::get_segnent2segment_distance(float xa1, float ya1, float xa2, float ya2, float xb1, float yb1, float xb2, float yb2) {
  //Get all possible distances between each dot of two lines and second line and return the shortest
  float dist1 = get_point2line_distance(xa1,ya1, xb1, yb1, xb2, yb2);
  float dist2 = get_point2line_distance(xa2,ya2, xb1, yb1, xb2, yb2);
  float dist3 = get_point2line_distance(xb1,yb1, xa1, ya1, xa2, ya2);
  float dist4 = get_point2line_distance(xb2,yb2, xa1, ya1, xa2, ya2);

  return std::min(std::min(dist1, dist2), std::min(dist3, dist4));
}

void WhiteLinesZedDetector::merge_two_segments(float xa1, float ya1, float xa2, float ya2, 
                        float xb1, float yb1, float xb2, float yb2, 
                        float *xr1, float *yr1, float *xr2, float *yr2) 
{
  //Dumb merging of two segments by removing two points closest to centroid
  float centroid_x = (xa1+xa2+xb1+xb2)/4.0;
  float centroid_y = (ya1+ya2+yb1+yb2)/4.0;
  float dist[4];
  dist[0] = get_line_magnitude(xa1,ya1, centroid_x, centroid_y);
  dist[1] = get_line_magnitude(xa2,ya2, centroid_x, centroid_y);
  dist[2] = get_line_magnitude(xb1,yb1, centroid_x, centroid_y);
  dist[3] = get_line_magnitude(xb2,yb2, centroid_x, centroid_y);

  float max_dist1 = 0;
  int max_dist1_index = 0;
  for(int i=0;i<4;i++) {
    if(dist[i]>max_dist1) {
      max_dist1 = dist[i];
      max_dist1_index = i;      
    }
  }
  //std::cout << "max_dist1_index=" << max_dist1_index << std::endl;
  if(max_dist1_index==0) { *xr1 = xa1; *yr1 = ya1; }
  if(max_dist1_index==1) { *xr1 = xa2; *yr1 = ya2; }
  if(max_dist1_index==2) { *xr1 = xb1; *yr1 = yb1; }
  if(max_dist1_index==3) { *xr1 = xb2; *yr1 = yb2; }

  dist[max_dist1_index] = 0; //to skip this point in next iteration  
  float max_dist2 = 0;
  int max_dist2_index = 0;
  for(int i=0;i<4;i++) {
    if(dist[i]>max_dist2) {
      max_dist2 = dist[i];
      max_dist2_index = i;      
    }
  }  
  if(max_dist2_index==0) { *xr2 = xa1; *yr2 = ya1; }
  if(max_dist2_index==1) { *xr2 = xa2; *yr2 = ya2; }
  if(max_dist2_index==2) { *xr2 = xb1; *yr2 = yb1; }
  if(max_dist2_index==3) { *xr2 = xb2; *yr2 = yb2; }

  //std::cout << "Merging " << cv::Point2f(xa1,ya1) << "-" << cv::Point2f(xa2,ya2) << " and " << cv::Point2f(xb1,yb1) << "-" << cv::Point2f(xb2,yb2) << " to " << cv::Point2f(*xr1,*yr1) << "-" << cv::Point2f(*xr2,*yr2) << std::endl;
}

void WhiteLinesZedDetector::drawSimpleRadarImg(void) {



}