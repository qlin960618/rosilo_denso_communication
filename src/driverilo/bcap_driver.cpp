/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_denso_communcation.
#
#    rosilo_denso_communcation is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_denso_communcation is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_denso_communcation.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include "bcap_driver.h"

#include <map>
#include <string>
#include <sstream>
#include <vector>

#include <sys/socket.h>

#define EMPTY_CHARP std::string("").c_str()

namespace driverilo
{

BCAPDriver::BCAPDriver(std::string server_ip_address, const int server_port_number, long socket_timeout)
{
    server_ip_address_    = server_ip_address;
    server_port_number_   = server_port_number;
    socket_timeout_       = socket_timeout;

    //Set joint positions structure used in bcap communication
    VariantInit(&set_joint_positions_variant_);
    VariantInit(&set_end_effector_pose_variant_);
    VariantInit(&unwanted_variant_result_);
    VariantInit(&empty_variant_);
}

BCAPDriver::~BCAPDriver()
{
    VariantClear(&set_joint_positions_variant_);
    VariantClear(&set_end_effector_pose_variant_);
    VariantClear(&unwanted_variant_result_);
    VariantClear(&empty_variant_);
}

HRESULT BCAPDriver::_set_socket_timeout(long micro_seconds)
{
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = micro_seconds;
    //ROS_WARN("Setting send timeout...");
    if (setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0)
    {
        //ROS_WARN("Failed to set send timeout");
        return E_FAIL;
    }
    //ROS_WARN("Setting receive timeout...");
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        //ROS_WARN("Failed to set recv timeout");
        return E_FAIL;
    }
    return S_OK;
}


//Error check function
bool BCAPDriver::_error_check(const HRESULT& result)
{
    if(result<0)
    {
        last_error_ = result;
        if(last_error_== E_UNEXPECTED )
            last_error_info_ = std::string("E_UNEXPECTED");
        if(last_error_== E_TIMEOUT )
            last_error_info_ = std::string("E_TIMEOUT");
        if(last_error_== E_FAIL )
            last_error_info_ = std::string("E_FAIL");
        if(last_error_== E_NOTIMPL )
            last_error_info_ = std::string("E_NOTIMPL");
        if(last_error_== E_ALREADY_REGISTER )
            last_error_info_ = std::string("E_ALREADY_REGISTER");
        if(last_error_== E_HANDLE )
            last_error_info_ = std::string("E_HANDLE");
        return false;
    }
    return true;
}

//Methods

bool BCAPDriver::open()
{
    std::ostringstream s;
    s << server_port_number_;
    std::string port_number_string(s.str());
    std::string command_string(std::string("udp:")+server_ip_address_+std::string(":")+port_number_string);
    return _error_check( bCap_Open_Client(command_string.c_str() , 1000 ,3, &socket_ ) );
}

bool BCAPDriver::close()
{
    return _error_check( bCap_Close_Client(&socket_) );
}

bool BCAPDriver::service_start()
{
    return _error_check( bCap_ServiceStart(socket_, nullptr) );
}

bool BCAPDriver::service_stop()
{
    return  _error_check ( bCap_ServiceStop(socket_) );
}

bool BCAPDriver::controller_connect()
{
    BSTR controller_name     = SysAllocString(L"");
    BSTR controller_provider = SysAllocString(L"caoProv.DENSO.VRC");
    BSTR controller_machine  = SysAllocString(L"localhost");
    BSTR controller_options  = SysAllocString(L"");

    bool return_value = _error_check(bCap_ControllerConnect(socket_, controller_name,controller_provider,controller_machine,controller_options, &controller_handle_));

    SysFreeString(controller_name);
    SysFreeString(controller_provider);
    SysFreeString(controller_machine);
    SysFreeString(controller_options);

    return return_value;
}

bool BCAPDriver::controller_disconnect()
{
    return _error_check(bCap_ControllerDisconnect(socket_, &controller_handle_));
}

bool BCAPDriver::get_robot()
{
    BSTR robot_name  = SysAllocString(L"");
    BSTR options     = SysAllocString(L"");

    bool return_value = _error_check(bCap_ControllerGetRobot(socket_, controller_handle_, robot_name, options, &robot_handle_));

    SysFreeString(robot_name);
    SysFreeString(options);

    return return_value;
}

bool BCAPDriver::release_robot()
{
    return _error_check (bCap_RobotRelease(socket_, &robot_handle_));
}


bool BCAPDriver::get_controller_variable_handle(const std::wstring& variable_name, uint32_t& handle)
{
    BSTR variable_name_bstr  = SysAllocString(variable_name.c_str());
    BSTR options             = SysAllocString(L"");

    bool return_value = _error_check (bCap_ControllerGetVariable(socket_, controller_handle_, variable_name_bstr, options, &handle));

    SysFreeString(variable_name_bstr);
    SysFreeString(options);

    return return_value;
}


bool BCAPDriver::get_controller_variable_int(const uint32_t& variable_handle, int& value_int)
{
    VARIANT value;
    VariantInit(&value);

    bool return_value = _error_check ( bCap_VariableGetValue(socket_, variable_handle, &value) );

    value_int = value.lVal;

    VariantClear(&value);

    return return_value;
}


inline bool BCAPDriver::_robot_execute(const std::wstring& command, const VARIANT& option, VARIANT& result)
{
    BSTR command_bstr = SysAllocString(command.c_str());

    bool return_value =_error_check(bCap_RobotExecute(socket_, robot_handle_, command_bstr, option, &result));

    SysFreeString(command_bstr);
    return return_value;
}

void BCAPDriver::initialize_controller_variable_handles()
{
    uint handle;

    get_controller_variable_handle(std::wstring(L"@ERROR_CODE"),handle);
    std::map<std::string, u_int>::value_type ERROR_CODE("ERROR_CODE",handle);
    controller_variable_handles_.insert(ERROR_CODE);

    get_controller_variable_handle(std::wstring(L"@ERROR_DESCRIPTION"),handle);
    std::map<std::string, u_int>::value_type ERROR_DESCRIPTION("ERROR_DESCRIPTION",handle);
    controller_variable_handles_.insert(ERROR_DESCRIPTION);

    get_controller_variable_handle(std::wstring(L"@MODE"),handle);
    std::map<std::string, u_int>::value_type MODE("MODE",handle);
    controller_variable_handles_.insert(MODE);

    get_controller_variable_handle(std::wstring(L"@EMERGENCY_STOP"),handle);
    std::map<std::string, u_int>::value_type EMERGENCY_STOP("EMERGENCY_STOP",handle);
    controller_variable_handles_.insert(EMERGENCY_STOP);
}

/********************************************************************************
         ********** COMMON FUNCTIONS THAT USE ROBOT GETCONTROLLERVARIABLEVALUE **********
         ********************************************************************************/

bool BCAPDriver::get_mode(int& mode)
{
    return _error_check( get_controller_variable_int(controller_variable_handles_["@MODE"],mode) );
}

bool BCAPDriver::get_emergency_stop(int& status)
{
    return _error_check( get_controller_variable_int(controller_variable_handles_["@EMERGENCY_STOP"],status) );
}

bool BCAPDriver::get_controller_status(int& status)
{
    return _error_check( get_controller_variable_int(controller_variable_handles_["@ERROR_CODE"],status) );
}

/*bool GetControllerErrorDescription(void* error_description)
        {
            return GetControllerVariableValue(controller_variable_handles_["@ERROR_DESCRIPTION"],error_description);
        }*/



/*************************************************************
         ********** COMMON FUNCTIONS THAT USE ROBOT EXECUTE **********
         *************************************************************/

bool BCAPDriver::clear_error(VARIANT& result)
{
    return _robot_execute(std::wstring(L"ClearError"),empty_variant_,result);
}

bool BCAPDriver::take_arm(VARIANT& result)
{
    return _robot_execute(std::wstring(L"TakeArm"),empty_variant_,result);
}

bool BCAPDriver::give_arm(VARIANT& result)
{
    return _robot_execute(std::wstring(L"GiveArm"),empty_variant_,result);
}

bool BCAPDriver::set_motor_state(bool state, VARIANT& result)
{
    VARIANT param;
    VariantInit(&param);
    u_int32_t* param_data;
    param.vt = VT_I4 | VT_ARRAY;
    param.parray = SafeArrayCreateVector(VT_I4,0,2);

    bool result_value;

    if(state)
    {
        SafeArrayAccessData(param.parray,(void**)&param_data);
        param_data[0] = 1;
        param_data[1] = 0;
        SafeArrayUnaccessData(param.parray);
        result_value = _robot_execute(std::wstring(L"Motor"),param,result);
    }
    else
    {
        SafeArrayAccessData(param.parray,(void**)&param_data);
        param_data[0] = 0;
        param_data[1] = 1;
        SafeArrayUnaccessData(param.parray);
        result_value = _robot_execute(std::wstring(L"Motor"),param,result);
    }

    VariantClear(&param);
    return result_value;
}

bool BCAPDriver::get_joint_positions(std::vector<double>& get_joint_positions_vector)
{
    VARIANT result;
    VariantInit(&result);

    bool return_value = _robot_execute(std::wstring(L"CurJnt"),empty_variant_, result);

    double* data_pointer;

    SafeArrayAccessData(result.parray,(void**)&data_pointer);
    get_joint_positions_vector.assign(data_pointer,data_pointer+8);
    SafeArrayUnaccessData(result.parray);

    VariantClear(&result);

    return return_value;
}

bool BCAPDriver::get_end_effector_pose_homogenous_transformation(std::vector<double>& get_end_effector_pose_homogeouns_transformation_vector)
{
    VARIANT result;
    VariantInit(&result);

    bool return_value = _robot_execute(std::wstring(L"CurTrn"),empty_variant_, result);

    double* data_pointer;

    SafeArrayAccessData(result.parray,(void**)&data_pointer);
    get_end_effector_pose_homogeouns_transformation_vector.assign(data_pointer,data_pointer+10);
    SafeArrayUnaccessData(result.parray);

    VariantClear(&result);

    return return_value;
}

bool BCAPDriver::set_speed(const float& speed, const float& acceleration, const float& deacceleration)
{
    float *param_data;
    VARIANT param;
    param.vt = VT_R4 | VT_ARRAY;
    param.parray = SafeArrayCreateVector(VT_R4,0,3);
    SafeArrayAccessData(param.parray,(void**)&param_data);
    param_data[0] = speed;
    param_data[1] = acceleration;
    param_data[2] = deacceleration;
    SafeArrayUnaccessData(param.parray);

    bool return_value = _error_check( _robot_execute(std::wstring(L"ExtSpeed"),param,unwanted_variant_result_));

    VariantClear(&param);

    return return_value;
}

bool BCAPDriver::set_and_get_joint_positions(const std::vector<double>& set_joint_positions_vector, std::vector<double>& get_joint_positions_vector)
{
    VARIANT request;
    VariantInit(&request);
    request.vt = VT_R8 | VT_ARRAY;
    request.parray = SafeArrayCreateVector(VT_R8,0,8);
    double* param_data;

    SafeArrayAccessData(request.parray,(void**)&param_data);
    param_data[0] = set_joint_positions_vector[0];
    param_data[1] = set_joint_positions_vector[1];
    param_data[2] = set_joint_positions_vector[2];
    param_data[3] = set_joint_positions_vector[3];
    param_data[4] = set_joint_positions_vector[4];
    param_data[5] = set_joint_positions_vector[5];
    param_data[6] = 0;
    param_data[7] = 0;
    SafeArrayUnaccessData(request.parray);

    VARIANT variant_result;
    VariantInit(&variant_result);

    bool return_value = _robot_execute(std::wstring(L"slvMove"),request,variant_result);

    //If the return value is false, we don't try to access the data.
    if(return_value)
    {
        double* data_pointer;
        SafeArrayAccessData(variant_result.parray,(void**)&data_pointer);
        get_joint_positions_vector.assign(data_pointer,data_pointer+8);
        SafeArrayUnaccessData(variant_result.parray);
    }

    VariantClear(&request);
    VariantClear(&variant_result);

    return return_value;
}

bool BCAPDriver::set_joint_positions(const std::vector<double>& set_joint_positions_vector, VARIANT& variant_result)
{
    VARIANT request;
    VariantInit(&request);
    request.vt = VT_R8 | VT_ARRAY;
    request.parray = SafeArrayCreateVector(VT_R8,0,8);
    double* param_data;

    SafeArrayAccessData(request.parray,(void**)&param_data);
    param_data[0] = set_joint_positions_vector[0];
    param_data[1] = set_joint_positions_vector[1];
    param_data[2] = set_joint_positions_vector[2];
    param_data[3] = set_joint_positions_vector[3];
    param_data[4] = set_joint_positions_vector[4];
    param_data[5] = set_joint_positions_vector[5];
    param_data[6] = 0;
    param_data[7] = 0;
    SafeArrayUnaccessData(request.parray);

    bool return_value = _robot_execute(std::wstring(L"slvMove"),request,variant_result);

    VariantClear(&request);

    return return_value;
}

bool BCAPDriver::set_end_effector_pose_homogenous_transformation(const std::vector<double>& set_end_effector_pose_vector, VARIANT& variant_result)
{
    VARIANT request;
    VariantInit(&request);
    request.vt = VT_R8 | VT_ARRAY;
    request.parray = SafeArrayCreateVector(VT_R8,0,10);
    double* param_data;

    SafeArrayAccessData(request.parray,(void**)&param_data);
    param_data[0] = set_end_effector_pose_vector[0];
    param_data[1] = set_end_effector_pose_vector[1];
    param_data[2] = set_end_effector_pose_vector[2];
    param_data[3] = set_end_effector_pose_vector[3];
    param_data[4] = set_end_effector_pose_vector[4];
    param_data[5] = set_end_effector_pose_vector[5];
    param_data[6] = set_end_effector_pose_vector[6];
    param_data[7] = set_end_effector_pose_vector[7];
    param_data[8] = set_end_effector_pose_vector[8];
    param_data[9] = set_end_effector_pose_vector[9];
    SafeArrayUnaccessData(request.parray);

    bool return_value = _robot_execute(std::wstring(L"slvMove"),request,variant_result);

    VariantClear(&request);

    return return_value;
}

bool BCAPDriver::set_slave_mode(const int32_t& value, VARIANT& result)
{
    VARIANT argument;
    VariantInit(&argument);
    argument.vt   = VT_I4;
    argument.lVal = value;

    bool return_value = _robot_execute(std::wstring(L"slvChangeMode"), argument, result);

    VariantClear(&argument);

    return return_value;
}


std::string BCAPDriver::get_last_error_info() const
{
    return last_error_info_;
}

HRESULT BCAPDriver::_get_last_error() const
{
    return last_error_;
}

}


