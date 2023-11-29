#include "camera.h"
int CameraNode::getAndSetCameraHeight()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取高度信息
    // get IInteger variable
    MVCC_INTVALUE stHeight = {0};
    nRet = MV_CC_GetIntValue(handle, "Height", &stHeight);
    if (MV_OK == nRet)
    {
        if (camera_height > stHeight.nMax)
        {
            camera_height = stHeight.nMax;
        }
        else if (camera_height < stHeight.nMin)
        {
            camera_height = stHeight.nMin;
        }
        printf("camera_height current value:%d, max value:%d, min value:%d, increment value:%d\n", stHeight.nCurValue, stHeight.nMax, stHeight.nMin, stHeight.nInc);
    }
    else
    {
        printf("get height failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置高度
    // set IInteger variable
    if (camera_height != stHeight.nCurValue)
    {
        nRet = MV_CC_SetIntValue(handle, "Height", camera_height);
        if (MV_OK == nRet)
        {
            printf("set height %d\n", camera_height);
        }
        else
        {
            printf("set height failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_height", camera_height));
    return state;
}
int CameraNode::getAndSetCameraWidth()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取宽度信息
    // get IInteger variable
    MVCC_INTVALUE stWidth = {0};
    nRet = MV_CC_GetIntValue(handle, "Width", &stWidth);
    if (MV_OK == nRet)
    {
        if (camera_width > stWidth.nMax)
        {
            camera_width = stWidth.nMax;
        }
        else if (camera_width < stWidth.nMin)
        {
            camera_width = stWidth.nMin;
        }
        printf("camera_width current value:%d, max value:%d, min value:%d, increment value:%d\n", stWidth.nCurValue, stWidth.nMax, stWidth.nMin, stWidth.nInc);
    }
    else
    {
        printf("get camera_width failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置宽度
    // set IInteger variable
    if (camera_width != stWidth.nCurValue)
    {
        nRet = MV_CC_SetIntValue(handle, "Width", camera_width);
        if (MV_OK == nRet)
        {
            printf("set camera_width %d\n", camera_width);
        }
        else
        {
            printf("set camera_width failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_width", camera_width));
    return state;
}
int CameraNode::getAndSetCameraFramerate()
{
    this->set_parameter(rclcpp::Parameter("camera_framerate", camera_framerate));
    return MV_OK;
}
int CameraNode::getAndSetCameraRoiOffsetX()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取X偏移信息
    // get IInteger variable
    MVCC_INTVALUE stOffsetX = {0};
    nRet = MV_CC_GetIntValue(handle, "OffsetX", &stOffsetX);
    if (MV_OK == nRet)
    {
        if (camera_roi_offset_x > stOffsetX.nMax)
        {
            camera_roi_offset_x = stOffsetX.nMax;
        }
        else if (camera_roi_offset_x < stOffsetX.nMin)
        {
            camera_roi_offset_x = stOffsetX.nMin;
        }
        printf("camera_OffsetX current value:%d, max value:%d, min value:%d, increment value:%d\n", stOffsetX.nCurValue, stOffsetX.nMax, stOffsetX.nMin, stOffsetX.nInc);
    }
    else
    {
        printf("get camera_OffsetX failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置X偏移
    // set IInteger variable
    if (camera_roi_offset_x != stOffsetX.nCurValue)
    {
        nRet = MV_CC_SetIntValue(handle, "OffsetX", camera_roi_offset_x);
        if (MV_OK == nRet)
        {
            printf("set camera_OffsetX %d\n", camera_roi_offset_x);
        }
        else
        {
            printf("set camera_OffsetX failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_roi_offset_x", camera_roi_offset_x));
    return state;
}
int CameraNode::getAndSetCameraRoiOffsetY()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取Y偏移信息
    // get IInteger variable
    MVCC_INTVALUE stOffsetY = {0};
    nRet = MV_CC_GetIntValue(handle, "OffsetY", &stOffsetY);
    if (MV_OK == nRet)
    {
        if (camera_roi_offset_y > stOffsetY.nMax)
        {
            camera_roi_offset_y = stOffsetY.nMax;
        }
        else if (camera_roi_offset_y < stOffsetY.nMin)
        {
            camera_roi_offset_y = stOffsetY.nMin;
        }
        printf("camera_OffsetY current value:%d, max value:%d, min value:%d, increment value:%d\n", stOffsetY.nCurValue, stOffsetY.nMax, stOffsetY.nMin, stOffsetY.nInc);
    }
    else
    {
        printf("get camera_OffsetY failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置Y偏移
    // set IInteger variable
    if (camera_roi_offset_y != stOffsetY.nCurValue)
    {
        nRet = MV_CC_SetIntValue(handle, "OffsetY", camera_roi_offset_y);
        if (MV_OK == nRet)
        {
            printf("set camera_OffsetY %d\n", camera_roi_offset_y);
        }
        else
        {
            printf("set camera_OffsetY failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_roi_offset_y", camera_roi_offset_y));
    return state;
}
int CameraNode::getAndSetCameraAutoExp()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取自动曝光信息
    MVCC_ENUMVALUE stExposureAuto = {0};
    nRet = MV_CC_GetEnumValue(handle, "ExposureAuto", &stExposureAuto);
    if (MV_OK == nRet)
    {
        if (camera_auto_exp != 0 && camera_auto_exp != 1 && camera_auto_exp != 2)
        {
            camera_auto_exp = stExposureAuto.nCurValue;
        }
        printf("auto exposure current value:%d\n", stExposureAuto.nCurValue);
    }
    else
    {
        printf("get auto exposure failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置自动曝光
    if (camera_auto_exp != stExposureAuto.nCurValue)
    {
        nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", camera_auto_exp);
        if (MV_OK == nRet)
        {
            printf("set auto exposure %d\n", camera_auto_exp);
        }
        else
        {
            printf("set auto exposure failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_auto_exp", camera_auto_exp));
    return state;
}
int CameraNode::getAndSetCameraAutoGain()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取自动增益信息
    MVCC_ENUMVALUE stGainAuto = {0};
    nRet = MV_CC_GetEnumValue(handle, "GainAuto", &stGainAuto);
    if (MV_OK == nRet)
    {
        if (camera_auto_gain != 0 && camera_auto_gain != 1 && camera_auto_gain != 2)
        {
            camera_auto_gain = stGainAuto.nCurValue;
        }
        printf("auto gain current value:%d\n", stGainAuto.nCurValue);
    }
    else
    {
        printf("get auto gain failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置自动增益
    if (camera_auto_gain != stGainAuto.nCurValue)
    {
        nRet = MV_CC_SetEnumValue(handle, "GainAuto", camera_auto_gain);
        if (MV_OK == nRet)
        {
            printf("set auto gain %d\n", camera_auto_gain);
        }
        else
        {
            printf("set auto gain failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_auto_gain", camera_auto_gain));
    return state;
}
int CameraNode::getAndSetCameraAutoWhitebalance()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取自动白平衡信息
    MVCC_ENUMVALUE stBalanceWhiteAuto = {0};
    nRet = MV_CC_GetEnumValue(handle, "BalanceWhiteAuto", &stBalanceWhiteAuto);
    if (MV_OK == nRet)
    {
        if (camera_auto_whitebalance != 0 && camera_auto_whitebalance != 1 && camera_auto_whitebalance != 2)
        {
            camera_auto_whitebalance = stBalanceWhiteAuto.nCurValue;
        }
        printf("auto whitebalance current value:%d\n", stBalanceWhiteAuto.nCurValue);
    }
    else
    {
        printf("get auto whitebalance failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置自动白平衡
    if (camera_auto_whitebalance != stBalanceWhiteAuto.nCurValue)
    {
        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", camera_auto_whitebalance);
        if (MV_OK == nRet)
        {
            printf("set auto whitebalance %d\n", camera_auto_whitebalance);
        }
        else
        {
            printf("set auto whitebalance failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_auto_whitebalance", camera_auto_whitebalance));
    return state;
}
int CameraNode::getAndSetCameraPixelFormat()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取图像格式信息
    MVCC_ENUMVALUE stPixelFormat = {0};
    nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &stPixelFormat);
    if (MV_OK == nRet)
    {
        printf("PixelFormat current value:%x\n", stPixelFormat.nCurValue);
    }
    else
    {
        printf("get PixelFormat failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置图像格式
    if (pixel_format != stPixelFormat.nCurValue)
    {
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", pixel_format);
        if (MV_OK == nRet)
        {
            printf("set PixelFormat %x\n", pixel_format);
        }
        else
        {
            printf("set PixelFormat failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    return state;
}
int CameraNode::getAndSetCameraExp()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取曝光信息
    // get IFloat variable
    MVCC_FLOATVALUE stExposureTime = {0};
    nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
    if (MV_OK == nRet)
    {
        if (camera_exp > stExposureTime.fMax)
        {
            camera_exp = stExposureTime.fMax;
        }
        else if (camera_exp < stExposureTime.fMin)
        {
            camera_exp = stExposureTime.fMin;
        }
        printf("exposure time current value:%f, max value:%f, min value:%f\n", stExposureTime.fCurValue, stExposureTime.fMax, stExposureTime.fMin);
    }
    else
    {
        printf("get exposure time failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置曝光
    // set IFloat variable
    if (abs(camera_exp - stExposureTime.fCurValue) > 0.00001)
    {
        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", camera_exp);
        if (MV_OK == nRet)
        {
            printf("set exposure time %f\n", camera_exp);
        }
        else
        {
            printf("set exposure time failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_exp", camera_exp));
    return state;
}
int CameraNode::getAndSetCameraGain()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取增益信息
    // get IFloat variable
    MVCC_FLOATVALUE stGain = {0};
    nRet = MV_CC_GetFloatValue(handle, "Gain", &stGain);
    if (MV_OK == nRet)
    {
        if (camera_gain > stGain.fMax)
        {
            camera_gain = stGain.fMax;
        }
        else if (camera_gain < stGain.fMin)
        {
            camera_gain = stGain.fMin;
        }
        printf("camera_gain current value:%f, max value:%f, min value:%f\n", stGain.fCurValue, stGain.fMax, stGain.fMin);
    }
    else
    {
        printf("get camera_gain failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置增益
    // set IFloat variable
    if (abs(camera_gain - stGain.fCurValue) > 0.01)
    {
        nRet = MV_CC_SetFloatValue(handle, "Gain", camera_gain);
        if (MV_OK == nRet)
        {
            printf("set camera_gain %f\n", camera_gain);
        }
        else
        {
            printf("set camera_gain failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_gain", camera_gain));
    return state;
}
int CameraNode::getAndSetCameraAutoMinexp()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取自动最小曝光信息
    // get IFloat variable
    MVCC_INTVALUE stAutoExposureTimeLowerLimit = {0};
    nRet = MV_CC_GetIntValue(handle, "AutoExposureTimeLowerLimit", &stAutoExposureTimeLowerLimit);
    if (MV_OK == nRet)
    {
        if (camera_auto_minexp > stAutoExposureTimeLowerLimit.nMax)
        {
            camera_auto_minexp = stAutoExposureTimeLowerLimit.nMax;
        }
        else if (camera_auto_minexp < stAutoExposureTimeLowerLimit.nMin)
        {
            camera_auto_minexp = stAutoExposureTimeLowerLimit.nMin;
        }
        printf("camera_auto_minexp current value:%d, max value:%d, min value:%d\n", stAutoExposureTimeLowerLimit.nCurValue, stAutoExposureTimeLowerLimit.nMax, stAutoExposureTimeLowerLimit.nMin);
    }
    else
    {
        printf("get camera_auto_minexp failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置自动最小曝光
    // set IFloat variable
    if (camera_auto_minexp != stAutoExposureTimeLowerLimit.nCurValue)
    {
        nRet = MV_CC_SetIntValue(handle, "AutoExposureTimeLowerLimit", camera_auto_minexp);
        if (MV_OK == nRet)
        {
            printf("set camera_auto_minexp %d\n", camera_auto_minexp);
        }
        else
        {
            printf("set camera_auto_minexp failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_auto_minexp", camera_auto_minexp));
    return state;
}
int CameraNode::getAndSetCameraAutoMaxexp()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取自动最大曝光信息
    // get IFloat variable
    MVCC_INTVALUE stAutoExposureTimeUpperLimit = {0};
    nRet = MV_CC_GetIntValue(handle, "AutoExposureTimeUpperLimit", &stAutoExposureTimeUpperLimit);
    if (MV_OK == nRet)
    {
        if (camera_auto_maxexp > stAutoExposureTimeUpperLimit.nMax)
        {
            camera_auto_maxexp = stAutoExposureTimeUpperLimit.nMax;
        }
        else if (camera_auto_maxexp < stAutoExposureTimeUpperLimit.nMin)
        {
            camera_auto_maxexp = stAutoExposureTimeUpperLimit.nMin;
        }
        printf("camera_auto_maxexp current value:%d, max value:%d, min value:%d\n", stAutoExposureTimeUpperLimit.nCurValue, stAutoExposureTimeUpperLimit.nMax, stAutoExposureTimeUpperLimit.nMin);
    }
    else
    {
        printf("get camera_auto_maxexp failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置自动最大曝光
    // set IFloat variable
    if (camera_auto_maxexp != stAutoExposureTimeUpperLimit.nCurValue)
    {
        nRet = MV_CC_SetIntValue(handle, "AutoExposureTimeUpperLimit", camera_auto_maxexp);
        if (MV_OK == nRet)
        {
            printf("set camera_auto_maxexp %d\n", camera_auto_maxexp);
        }
        else
        {
            printf("set camera_auto_maxexp failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_auto_maxexp", camera_auto_maxexp));
    return state;
}
int CameraNode::getAndSetCameraAutoMaxgain()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取自动最大增益信息
    // get IFloat variable
    MVCC_FLOATVALUE stAutoGainUpperLimit = {0};
    nRet = MV_CC_GetFloatValue(handle, "AutoGainUpperLimit", &stAutoGainUpperLimit);
    if (MV_OK == nRet)
    {
        if (camera_auto_maxgain > stAutoGainUpperLimit.fMax)
        {
            camera_auto_maxgain = stAutoGainUpperLimit.fMax;
        }
        else if (camera_auto_maxgain < stAutoGainUpperLimit.fMin)
        {
            camera_auto_maxgain = stAutoGainUpperLimit.fMin;
        }
        printf("camera_auto_maxgain current value:%f, max value:%f, min value:%f\n", stAutoGainUpperLimit.fCurValue, stAutoGainUpperLimit.fMax, stAutoGainUpperLimit.fMin);
    }
    else
    {
        printf("get camera_auto_maxgain failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置自动最大增益
    // set IFloat variable
    if (abs(camera_auto_maxgain - stAutoGainUpperLimit.fCurValue) > 0.01)
    {
        nRet = MV_CC_SetFloatValue(handle, "AutoGainUpperLimit", camera_auto_maxgain);
        if (MV_OK == nRet)
        {
            printf("set camera_auto_maxgain %f\n", camera_auto_maxgain);
        }
        else
        {
            printf("set camera_auto_maxgain failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_auto_maxgain", camera_auto_maxgain));
    return state;
}
int CameraNode::getAndSetCameraAutoMingain()
{
    int nRet = MV_OK;
    int state = MV_OK;
    // 获取自动最小增益信息
    // get IFloat variable
    MVCC_FLOATVALUE stAutoGainLowerLimit = {0};
    nRet = MV_CC_GetFloatValue(handle, "AutoGainLowerLimit", &stAutoGainLowerLimit);
    if (MV_OK == nRet)
    {
        if (camera_auto_mingain > stAutoGainLowerLimit.fMax)
        {
            camera_auto_mingain = stAutoGainLowerLimit.fMax;
        }
        else if (camera_auto_mingain < stAutoGainLowerLimit.fMin)
        {
            camera_auto_mingain = stAutoGainLowerLimit.fMin;
        }
        printf("camera_auto_mingain current value:%f, max value:%f, min value:%f\n", stAutoGainLowerLimit.fCurValue, stAutoGainLowerLimit.fMax, stAutoGainLowerLimit.fMin);
    }
    else
    {
        printf("get camera_auto_mingain failed! nRet [%x]\n", nRet);
        state = -1;
    }
    // 设置自动最小增益
    // set IFloat variable
    if (abs(camera_auto_mingain - stAutoGainLowerLimit.fCurValue) > 0.01)
    {
        nRet = MV_CC_SetFloatValue(handle, "AutoGainLowerLimit", camera_auto_mingain);
        if (MV_OK == nRet)
        {
            printf("set camera_auto_mingain %f\n", camera_auto_mingain);
        }
        else
        {
            printf("set camera_auto_mingain failed! nRet [%x]\n", nRet);
            state = -1;
        }
    }
    this->set_parameter(rclcpp::Parameter("camera_auto_mingain", camera_auto_mingain));
    return state;
}
// 从相机内部获取参数，如果与要求不一致，则进行修改
int CameraNode::getAndSetCameraParam(CameraParams which_param)
{
    int nRet = MV_OK;
    switch (which_param)
    {
    case CAMERA_HEIGHT:
        nRet = getAndSetCameraHeight();
        break;
    case CAMERA_WIDTH:
        nRet = getAndSetCameraWidth();
        break;
    case CAMERA_FRAMERATE:
        nRet = getAndSetCameraFramerate();
        break;
    case CAMERA_ROI_OFFSET_X:
        nRet = getAndSetCameraRoiOffsetX();
        break;
    case CAMERA_ROI_OFFSET_Y:
        nRet = getAndSetCameraRoiOffsetY();
        break;
    case CAMERA_AUTO_EXP:
        nRet = getAndSetCameraAutoExp();
        break;
    case CAMERA_AUTO_GAIN:
        nRet = getAndSetCameraAutoGain();
        break;
    case CAMERA_AUTO_WHITEBALANCE:
        nRet = getAndSetCameraAutoWhitebalance();
        break;
    case CAMERA_AUTO_MAXEXP:
        nRet = getAndSetCameraAutoMaxexp();
        break;
    case CAMERA_AUTO_MINEXP:
        nRet = getAndSetCameraAutoMinexp();
        break;
    case CAMERA_AUTO_MAXGAIN:
        nRet = getAndSetCameraAutoMaxgain();
        break;
    case CAMERA_AUTO_MINGAIN:
        nRet = getAndSetCameraAutoMingain();
        break;
    case CAMERA_EXP:
        nRet = getAndSetCameraExp();
        break;
    case CAMERA_GAIN:
        nRet = getAndSetCameraGain();
        break;
    case ALL_PARAMS:
        nRet = getAndSetCameraHeight();
        nRet = getAndSetCameraWidth();
        nRet = getAndSetCameraFramerate();
        nRet = getAndSetCameraRoiOffsetX();
        nRet = getAndSetCameraRoiOffsetY();
        nRet = getAndSetCameraAutoExp();
        nRet = getAndSetCameraAutoGain();
        nRet = getAndSetCameraAutoWhitebalance();
        nRet = getAndSetCameraAutoMaxexp();
        nRet = getAndSetCameraAutoMinexp();
        nRet = getAndSetCameraAutoMaxgain();
        nRet = getAndSetCameraAutoMingain();
        nRet = getAndSetCameraExp();
        nRet = getAndSetCameraGain();
        nRet = getAndSetCameraPixelFormat();
        break;
    case NONE:
        break;
    default:
        nRet = getAndSetCameraHeight();
        nRet = getAndSetCameraWidth();
        nRet = getAndSetCameraFramerate();
        nRet = getAndSetCameraRoiOffsetX();
        nRet = getAndSetCameraRoiOffsetY();
        nRet = getAndSetCameraAutoExp();
        nRet = getAndSetCameraAutoGain();
        nRet = getAndSetCameraAutoWhitebalance();
        nRet = getAndSetCameraAutoMaxexp();
        nRet = getAndSetCameraAutoMinexp();
        nRet = getAndSetCameraAutoMaxgain();
        nRet = getAndSetCameraAutoMingain();
        nRet = getAndSetCameraExp();
        nRet = getAndSetCameraGain();
        nRet = getAndSetCameraPixelFormat();
        break;
    }
    return nRet;
}
void CameraNode::param_event_callback(const std::shared_ptr<camera_interfaces::srv::ParamEvent::Request> request, std::shared_ptr<camera_interfaces::srv::ParamEvent::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received a request on topic \"%s\"", "param_event");
    switch (request->param_name)
    {
    case CAMERA_HEIGHT:
        camera_height = request->value;
        param_change = CAMERA_HEIGHT;
        break;
    case CAMERA_WIDTH:
        camera_width = request->value;
        param_change = CAMERA_WIDTH;
        break;
    case CAMERA_FRAMERATE:
        camera_framerate = request->value;
        param_change = CAMERA_FRAMERATE;
        break;
    case CAMERA_EXP:
        camera_exp = request->value;
        param_change = CAMERA_EXP;
        break;
    case CAMERA_GAIN:
        camera_gain = request->value;
        param_change = CAMERA_GAIN;
        break;
    case CAMERA_ROI_OFFSET_X:
        camera_roi_offset_x = request->value;
        param_change = CAMERA_ROI_OFFSET_X;
        break;
    case CAMERA_ROI_OFFSET_Y:
        camera_roi_offset_y = request->value;
        param_change = CAMERA_ROI_OFFSET_Y;
        break;
    case CAMERA_AUTO_EXP:
        camera_auto_exp = request->value;
        param_change = CAMERA_AUTO_EXP;
        break;
    case CAMERA_AUTO_GAIN:
        camera_auto_gain = request->value;
        param_change = CAMERA_AUTO_GAIN;
        break;
    case CAMERA_AUTO_WHITEBALANCE:
        camera_auto_whitebalance = request->value;
        param_change = CAMERA_AUTO_WHITEBALANCE;
        break;
    case CAMERA_AUTO_MAXEXP:
        camera_auto_maxexp = request->value;
        param_change = CAMERA_AUTO_MAXEXP;
        break;
    case CAMERA_AUTO_MINEXP:
        camera_auto_minexp = request->value;
        param_change = CAMERA_AUTO_MINEXP;
        break;
    case CAMERA_AUTO_MAXGAIN:
        camera_auto_maxgain = request->value;
        param_change = CAMERA_AUTO_MAXGAIN;
        break;
    case CAMERA_AUTO_MINGAIN:
        camera_auto_mingain = request->value;
        param_change = CAMERA_AUTO_MINGAIN;
        break;
    case NONE:
        break;
    default:
        response->success = false;
        response->status_message = "failed";
        break;
    }
    response->success = true;
    response->status_message = "success";
    response->camera_auto_exp = camera_auto_exp;
    response->camera_auto_gain = camera_auto_gain;
    response->camera_auto_maxexp = camera_auto_maxexp;
    response->camera_auto_maxgain = camera_auto_maxgain;
    response->camera_auto_minexp = camera_auto_minexp;
    response->camera_auto_mingain = camera_auto_mingain;
    response->camera_auto_whitebalance = camera_auto_whitebalance;
    response->camera_exp = camera_exp;
    response->camera_framerate = camera_framerate;
    response->camera_gain = camera_gain;
    response->camera_height = camera_height;
    response->camera_roi_offset_x = camera_roi_offset_x;
    response->camera_roi_offset_y = camera_roi_offset_y;
    response->camera_width = camera_width;
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", (long int)response->success);
}