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
    faissNode::faissNode(const NodeConfig &config) : INode(config) {
       //set supported distance modes

       //set supported query modes

       //Default mode configuration
       m_distance_mode = IFlex::DistanceMode::L2;
       m_query_mode = IFlex::QueryMode::KNN_A;

       m_read_count = 0;
       m_threshold_upper = 0;
       m_threshold_lower =0;
        //this shouldn't be needed. only if an additional buffer layer beyond asio sockets is needed
       //m_read_queue = std::queue<iFlexNodeReader::packet_t>();

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
        if(std::find(m_supported_distance_modes.begin(), m_supported_distance_modes.end(),mode) == m_supported_distance_modes.end()){
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

    IFlex::QueryMode faissNode::getQueryMode(){
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
        m_index.reset();
        m_offset = offset;

        HighFive::File file(fileName, HighFive::File::ReadOnly);
        auto ds = file.getDataSet(datasetName);

        auto space = ds.getMemSpace().getDimensions();

        uint64_t vec_cnt = count;
        //uint64_t vec_cnt = space[0];
        uint64_t comp_cnt = space[1];

        //TODO add a catch if memspace isn't large enough
        //pull the data out of the hdf5 file into an object
        vector8_list_t data;

        ds.select({offset,0},{vec_cnt, comp_cnt},{}).read(data);

        //convert vector 8 to float array
        std::vector<float> float_data;

        for(uint64_t vec=0; vec<vec_cnt; vec++){
            for(uint64_t comp=0; comp<comp_cnt; comp++){
                float_data.emplace_back(data[vec][comp]);
            }
        }

        faiss::IndexFlatL2 m_index(comp_cnt);

        m_index.add(vec_cnt, float_data.data());


        return;
    }

    void faissNode::dsLoadRandom(uint64_t offset, uint64_t vector_count, uint64_t comp_count){
        //reset the faiss instance (clear the memory)
        m_index.reset();//reset the index
        //load in vars
        m_offset = offset; // is this needed? is this the memory offset for the fpga's virtual memory space?

        float *ds = new float[vector_count*comp_count];
        //TODO: calculate distribution between devices (if running multi GPU)
        //Additional feature if needed

        //fill in the random dataset
        fillVectorList(ds, vector_count, comp_count);

        //instantiate a faiss instance
        faiss::IndexFlatL2 m_index(comp_count);
        //load the dataset into the dataset object
        m_index.add(vector_count, ds);

        return;
    }

    void faissNode::query(const vector8_list_t &vectors, vector_result32_list_t &results){
        //mpa the difference between the old approach and what faiss wants
        //# queries, query dataset, neighbor#, result dist, result idx
        float dists[results.size()*m_read_count];
        faiss::Index::idx_t labels[results.size()*m_read_count];
        int i=0;
        for(auto query : results){
            for(auto k_result : query){
                dists[i] = (float) k_result.distance;
                labels[i] = (faiss::Index::idx_t) k_result.ds_id;
                i++;
            }
        }
        std::vector<float> float_vectors;
        for(vector8_t vec : vectors){
            for(uint8_t comp : vec){
                float_vectors.emplace_back(comp);
            }
        }
        float *queries = float_vectors.data();
        m_index.search(vectors.size(), queries, m_read_count,dists, labels);

        //i need to recast type back to uint32 and into the vectors
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

        for(uint64_t i=0; i < vector_count; i++){
            for(uint64_t j = 0; j < comp_count; j++){
                DSVs[comp_count*i+j] = dis(gen);
            }
        }
        return;
    }

    void faissNode::fillVectorList(float *DSVs, uint64_t vector_count, uint64_t comp_count, float lower, float upper) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution <> dis(lower,upper);
        
        for(uint64_t i=0; i < vector_count; i++){
            for(uint64_t j=0; j<comp_count; j++){
                DSVs[i*comp_count + j] = dis(gen);
            }
        }
        return;
    }
/*
    float* faissNode::vector8_2_float(vector8_list_t vec){
        //conversion of elements from uint8 to float
        //conversion of pointer type
        return;
    }

    vector8_list_t faissNode::float_2_vector8(float* array){
        //conversion of elements from float to uint8
        //conversion of pointer type
        return;
    }

    faiss::Index::idx_t* faissNode::vector32_2_idx(vector32_list_t vec){
        //conversion of elements from uint32 to faiss idx
        //conversion of pointer
        return;
    }

    vector32_list_t faissNode::idx_2_vector32(faiss::Index::idx_t* array){
        //conversion of elements from faiss idx to uint32
        return;
    }

    float* faissNode::vector32_2_float(vector32_list_t vec){
        //conversion of elements from uint32 to float
        //conversion of pointer type
        return;
    }

    vector32_list_t faissNode::float_2_vector32(float * array){
        //conversion of elements from float to uint32
        //conversion of pointer type
        return;
    }
*/


} //namespace faiss