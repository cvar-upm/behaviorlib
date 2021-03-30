/*!*******************************************************************************************
 * \brief     Keyboard activation with behaviors implementation file.
 * \authors   Javier Melero Deza
 * \copyright Copyright (c) 2021 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef KEYBOARD_BEHAVIOR_ACTIVATION_MAIN_H
#define KEYBOARD_BEHAVIOR_ACTIVATION_MAIN_H

#include <string>
#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <cctype>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <behavior_execution_manager_msgs/ActivateBehavior.h>
#include <behavior_execution_manager_msgs/DeactivateBehavior.h>
#include <behavior_execution_manager_msgs/CheckSituation.h>
#include <curses.h>
#include <thread>
#include <locale.h>
#include "yaml-cpp/yaml.h"

//Inputs
#define ASCII_KEY_UP 65
#define ASCII_KEY_DOWN 66
#define ASCII_KEY_RIGHT 67
#define ASCII_KEY_LEFT 68

//Loop rate
#define FREQ_INTERFACE 50.0

ros::ServiceClient manage_behavior_srv;
ros::ServiceClient check_situation_srv;

void printoutControls();

const int LEFT = 1;
const int RIGHT = 2;
const int UP = 3;
const int DOWN = 4;
//Publishers

//Topics
std::string drone_id_namespace;
std::string config_file;
std::vector <std::string> mappings;
std::vector <std::string> names;
std::vector <std::string> behaviors;
std::vector <std::string> packages;
std::vector <std::string> key_printed;
std::vector <int> priorities; 
std::vector <std::string> parameters;

int pointer_c;



#endif
