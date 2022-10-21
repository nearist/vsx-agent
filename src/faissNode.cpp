/*
* Copyright (c) 2015-2018 in2H2 inc.
* System developed for in2H2 inc. by Intermotion Technology, Inc.
*
* Full system RTL, C sources and board design files available at https://github.com/nearist
*
* in2H2 inc. Team Members:
* - Chris McCormick - Algorithm Research and Design
* - Matt McCormick - Board Production, System Q/A
*
* Intermotion Technology Inc. Team Members:
* - Mick Fandrich - Project Lead
* - Dr. Ludovico Minati - Board Architecture and Design, FPGA Technology Advisor
* - Vardan Movsisyan - RTL Team Lead
* - Khachatur Gyozalyan - RTL Design
* - Tigran Papazyan - RTL Design
* - Taron Harutyunyan - RTL Design
* - Hayk Ghaltaghchyan - System Software
*
* Tecno77 S.r.l. Team Members:
* - Stefano Aldrigo, Board Layout Design
*
* We dedicate this project to the memory of Bruce McCormick, an AI pioneer
* and advocate, a good friend and father.
*
* These materials are provided free of charge: you can redistribute them and/or modify
* them under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* These materials are distributed in the hope that they will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
#include "faissNode.h"

namespace IFlex {
    faissNode::faissNode(const NodeConfig &config) : INode(config){
       //set supported distance modes

       //set supported query modes

       //Default mode configuration
       m_distance_mode = IFlex::DistanceMode::L2;
       m_query_mode = IFlex::QueryMode::KNN_A;

       m_read_count = 0;
       m_threshold_upper = 0;
       m_threshold_lower =0;

       m_read_queue = std::queue<iFlexNodeReader::packet_t>();

       m_running = false;

       m_pending_count =0;

       m_transaction_id = 0;

       //THINK ABOUT THE USE OF m_capacity
        return;
    }

    //PRIVATE FUNCTIONS WHEN I NEED THEM

    void faissNode::open(){
        return;
    }

    void faissNode::close(){
        return;
    }

    void faissNode::reset(){
        return;
    }

    void faissNode::setDistanceMode(DistanceMode mode){
        m_distance_mode = mode;
        return;
    }

    DistanceMode faissNode::getDistanceMode(){
        return m_distance_mode;
    }

    void faissNode::setQueryMode(QueryMode mode){
        m_query_mode = mode;
        return;
    }

    faissNode::QueryMode getQueryMode(){
        return m_query_mode;
    }

    void faissNode::setReadCount(uint16_t count){
        return;
    } 

    uint16_t faissNode::getReadCount(){
        return DSV_Count;
    }

    void faissNode::setThreshold(uint32_t threshold){
        return;
    }

    void faissNode::setThreshold(uint32_t threshold_lower, uint32_t threshold_higher){
        return;
    }

    uint32_t faissNode::getThreshold(){
        return Threshold_Lo;
    }

    uint32_t faissNode::getThreshold_Lo(){
        return Threshold_Lo;
    }

    uint32_t faissNode::getThreshold_Hi(){
        return Threshold_Hi;
    }

    void faissNode::dsLoad(uint64_t offset, const vector8_list_t &vectors){
        return;
    }

    void faissNode::dsLoadFromFile(const std::string &fileName, const std::string &datasetName, uint64_t offset, uint64_t count){
        return;
    }

    void faissNode::dsLoadRandom(uint64_t offset, uint64_t vector_count, uint64_t comp_count){
        return;
    }

    void faissNode::query(const vector8_list_t &vectors, vector_result32_list_t &results){
        return;
    }

    void faissNode::resetTimer(){
        m_duration = 0;
        return;
    }

    uint64_t faissNode::getTimerValue(){
        return m_duration;
    }
} //namespace faiss