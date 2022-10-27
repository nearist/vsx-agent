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
        if(std::find(m_supported_query_modes.begin(), m_supported_distance_modes.end(),mode) == m_supported_distance_modes.end()){
            throw DistanceModeNotSupportedException();
        }
        m_distance_mode = mode;
        return;
    }

    DistanceMode faissNode::getDistanceMode(){
        return m_distance_mode;
    }

    void faissNode::setQueryMode(QueryMode mode){
        if(std::find(m_supported_query_modes.begin(), m_supported_query_modes.end(),mode) == m_supported_query_modes.end()){
            throw QueryModeNotSupportedException();
        }
        m_query_mode = mode;
        return;
    }

    faissNode::QueryMode faissNode::getQueryMode(){
        return m_query_mode;
    }

    void faissNode::setReadCount(uint16_t count){
        m_read_count = count;
        return;
    } 

    uint16_t faissNode::getReadCount(){
        return m_read_count;
        //return DSV_Count;
    }

    void faissNode::setThreshold(uint32_t threshold){
        m_threshold_lower = threshold;
        return;
    }

    void faissNode::setThreshold(uint32_t threshold_lower, uint32_t threshold_upper){
        m_threshold_lower = threshold_lower;
        m_threshold_upper = threshold_upper;
        return;
    }

    uint32_t faissNode::getThreshold(){
        return m_threshold_lower;
    }

    uint32_t faissNode::getThreshold_Lo(){
        return m_threshold_lower;
    }

    uint32_t faissNode::getThreshold_Up(){
        return m_threshold_upper;
    }

    void faissNode::dsLoad(uint64_t offset, const vector8_list_t &vectors){
        return;
    }

    void faissNode::dsLoadFromFile(const std::string &fileName, const std::string &datasetName, uint64_t offset, uint64_t count){
        return;
    }

    void faissNode::dsLoadRandom(uint64_t offset, uint64_t vector_count, uint64_t comp_count){
        //reset the faiss instance (clear the memory)
        
        //load in vars
        m_offset = offset; // is this needed? is this the memory offset for the fpga's virtual memory space?

        uint64_t fv_count = vector_count;

        float *ds = new float[vector_count*comp_count];
        //TODO: calculate distribution between devices (if running multi GPU)
        //Additional feature if needed

        //create the random dataset
        
        fillVectorList(ds, vector_count, comp_count)

        //instantiate a faiss instance

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

    void faissNode::fillVectorList(float *DSVs, uint64_t vector_count, uint64_t comp_count){
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis;

        for(int i=0; i < vector_count; i++){
            for(int j = 0; j < comp_count; j++){
                DSVs[comp_count*i+j] = dis(gen);
            }
        }
        return;
    }

    void faissNode::fillVectorList(float *DSVs, uint64_t vector_count, uint64_t comp_count, float lower, float upper) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution <> dis(lower,upper);
        
        for(int i=0; i < vector_count; i++){
            for(int j=0; j<comp_count; j++){
                DSVs[i*comp_count + j] = dis(gen);
            }
        }
        return;
    }

} //namespace faiss