# APCE  matlab
function [APCE,Fmax] = apce(response)
    sum = 0;
    Fmax= max(max(response));
    Fmin = min(min(response));
    for i = 1: size(response,1)
        for j = 1 :size(response,2)
            sum = sum +( response(i,j) - Fmin)^2;
              
        end
    end
    
    Fmean =  sum / (size(response,1) * size(response,2));  
    
  APCE = (Fmax - Fmin)^2 / Fmean;
end

# APCE C++ 
cv::Point2i pi;
cv::Point2i minPoint;
double pv;
double minValue;
cv::minMaxLoc(res,&minValue,&pv,&minPoint,&pi);
peak_value = (float) pv;

double ave_res = 0;
double value_diff = 0;
double sum_diff = 0;
for(int i=0; i<res.cols; i++)
{
    for(int j=0; j<res.rows; j++)
    {   
        value_diff = res.at<float>(i,j) - minValue;
        sum_diff += std::pow(value_diff,2); 
    }
}

ave_res = sum_diff/(res.cols*res.rows);
double mmdiff=0;
mmdiff = pv-minValue;
mmdiff = std::pow(mmdiff,2);
double apce=mmdiff/ave_res;
apceValue = apce;

# PSR  C++
float Tracker::ComputePSR(const Mat &correlation_mat)
{//Compute Peak-to-Sidelobe Ratio

    double max_val = 0;
    Point max_loc;
    Mat PSR_mask = Mat::ones(correlation_mat.rows,correlation_mat.cols, CV_8U);
    Scalar mean,stddev;

    minMaxLoc(correlation_mat,NULL,&max_val,NULL,&max_loc);     //Get location of max arg

    //Define PSR mask
    int win_size = floor(this->PSR_mask/2);
    Rect mini_roi = Rect(std::max(max_loc.x - win_size,0), std::max(max_loc.y - win_size,0), this->PSR_mask, this->PSR_mask);

    //Handle image boundaries
    if ( (mini_roi.x+mini_roi.width) > PSR_mask.cols )
    {
        mini_roi.width = PSR_mask.cols - mini_roi.x;
    }
    if ( (mini_roi.y+mini_roi.height) > PSR_mask.rows )
    {
        mini_roi.height = PSR_mask.rows - mini_roi.y;
    }

    Mat temp = PSR_mask(mini_roi);
    temp *= 0;
    meanStdDev(correlation_mat,mean,stddev,PSR_mask);   //Compute matrix mean and std

    return (max_val - mean.val[0]) / stddev.val[0];     //Compute PSR
}