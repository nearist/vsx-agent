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
#ifndef TRUNK_SW_INC_FAISSNODE_H_
#define TRUNK_SW_INC_FAISSNODE_H_

#include <queue>
#include <vector>
#include <map>
#include <numeric>
#include <algorithm>
#include <cmath>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <boost/algorithm/string/replace.hpp>

#define H5_USE_BOOST
#include <highfive/H5DataSet.hpp>
#include <highfive/H5File.hpp>

#include "INode.h"
#include "NodeConfig.h"
#include "LinuxWrapper.h"
#include "TimeMeter.h"

namespace IFlex {
class faissNode : public INode {
    std::string TAG = "faissNode"; 

    //CONSTANTS IF NEEDED

    protected:
        //TODO check data type for all of these declared vars 
        int32_t Threshold_Lo;
        int32_t Threshold_Hi;
        int16_t DSV_Count; //num of data set vectors
        int16_t DSV_Len; //num of components per data set vector
        int8_t  Comp_Len; //size of the components
        QueryMode Query_mode; //Inode Query mode
        DistanceMode Distance_mode; //Inode Distance mode

        std::vector<IFlex::DistanceMode> m_supported_distance_modes;
        std::vector<IFlex::QueryMode> m_supported_query_modes;
        
        uint16_t read_count; //k of knn

        //TODO: IDENT WHAT THIS IS NEEDED FOR
        //Multithreading vars
        int32_t m_df_read;
        int32_t m_df_write;

        uint16_t m_transaction_id;

        uint64_t m_pending_count;

        bool m_running;
        //TODO create packet structure for faiss buffers
        std::queue<packet_t> m_read_queue;

        //request and result map is not needed as it is a FPGA array problem        
        //possibly add something like this for multi GPU
        //line 190-195 of iFlexNode.h

        std::thread m_th_reader; //read thread
        std::thread m_th_parser; //write thread

        //TODO understand what memory spaces we are protecting with each of these
        std::mutex m_mtx_rq;
        std::mutex m_mtx_status;
        std::mutex m_mtx_write;

        //TODO WTF are these
        std::condition_variable m_cv_rq_not_empty;
        std::condition_variable m_cv_status_ready;
    protected:
        //This is where all internal private calls 
        //necessary should be declared

    public:
        explicit faissNode(const NodeConfig &config);

        void open() override;

        void close() override;

        void reset() override;

        //StartCalculation?

        //StopCalculation?

        //ResetCalculation?

        void setDistanceMode(DistanceMode mode) override;

        DistanceMode getDistanceMode();

        void setQueryMode(QueryMode mode) override;

        QueryMode getQueryMode();

        void setReadCount(uint16_t count) override; 

        uint16_t getReadCount();

        void setThreshold(uint32_t threshold) override;

        void setThreshold(uint32_t threshold_lower, uint32_t threshold_higher) override;

        uint32_t getThreshold();

        uint32_t getThreshold_Lo();

        uint32_t getThreshold_Up();

        void dsLoad(uint64_t offset, const vector8_list_t &vectors) override;

        void dsLoadFromFile(const std::string &fileName, const std::string &datasetName, uint64_t offset, uint64_t count) override;

        void dsLoadRandom(uint64_t offset, uint64_t vector_count, uint64_t comp_count) override;

        void query(const vector8_list_t &vectors, vector_result32_list_t &results) override;

        void resetTimer() override;

        uint64_t getTimerValue() override;

        //getSubNodeCount maybe a way to pass through GPU device count?
};   
} // namespace faiss
#endif //TRUNK_SW_INC_FAISSNODE_H_