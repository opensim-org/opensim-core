#ifndef OPENSIM_DATA_QUEUE_H_
#define OPENSIM_DATA_QUEUE_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  DataQueue.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include <queue>
#include <condition_variable>
#include <SimTKcommon.h>
#include <OpenSim/Common/osimCommonDLL.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This base class defines the interface for a DataQueue. A data structure
 * to maintain a queue of data to be passed between computations that are 
 * potentially different in processing speeds, decoupling the producers 
 * (e.g. File or live stream) from consumers. 
 *
 * Synchronization mechanism will be implmented later to allow handling of 
 * multiple threads or significant differences in speeds.
 *
 * @author Ayman Habib
 */
/** Template class to contain Queue Entries, typically timestamped */
template <class U> 
class DataQueueEntry_ {
public:
    DataQueueEntry_(double timeStamp, const SimTK::RowVectorView_<U>& data)
            : _timeStamp(timeStamp), _data(data){};
    DataQueueEntry_(const DataQueueEntry_& other)       = default;
    DataQueueEntry_(DataQueueEntry_&&)                  = default;
    DataQueueEntry_& operator=(const DataQueueEntry_&)  = default;
    virtual ~DataQueueEntry_(){};

    double getTimeStamp() const { return _timeStamp; };
    SimTK::RowVectorView_<U> getData() const { return _data; };

private:
    double _timeStamp;
    SimTK::RowVectorView_<U> _data;
};
/**
 * DataQueue is a wrapper around the std::queue customized to handle data 
 * processing and synchronization, and limiting the interface to only the 
 * subset of operations needed for this use case. 
 * Synchronization is experimental as of now. Client is responsible for 
 * making sure order is preserved.
 * timestamp is required to pass in data so that clients can enforce order,
 * however timestamp is not used/order-enforced internally.
 */
// @TODO Test support of multiple consumers. 
template<class T> class DataQueue_ {
//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    virtual ~DataQueue_() {}
    
    DataQueue_()                                = default;
    // using compiler generated methods here is problematic due to mutex 
    DataQueue_(const DataQueue_& other){ 
        m_data_queue = other.m_data_queue;
    };
    DataQueue_(DataQueue_&& other){ 
        m_data_queue = other.m_data_queue;
    };
    DataQueue_& operator=(const DataQueue_& other) { 
        m_data_queue = other.m_data_queue;
        return (*this);
    };

    //--------------------------------------------------------------------------
    // DataQueue Interface
    //--------------------------------------------------------------------------
    // push data and associated timestamp to the end of the queue
    void push_back(const double time, const SimTK::RowVectorView_<T>& data) { 
        SimTK::RowVector_<T>* deepCopy = new SimTK::RowVector_<T>(data);
        std::unique_lock<std::mutex> mlock(m_mutex);
        m_data_queue.push(DataQueueEntry_<T>(time, *deepCopy));
        mlock.unlock();     // unlock before notificiation to minimize mutex con
        m_cond.notify_one(); 
    }
    // pop the front of the queue and return data and associated timestamp
    void pop_front(double& time, SimTK::RowVector_<T>& data) { 
        std::unique_lock<std::mutex> mlock(m_mutex);
        while (m_data_queue.empty()) { m_cond.wait(mlock); }
        DataQueueEntry_<SimTK::Rotation> frontEntry = m_data_queue.front();
        m_data_queue.pop();
        mlock.unlock(); 
        time = frontEntry.getTimeStamp();
        data = frontEntry.getData();
    }
    // check if the queue is empty
    bool isEmpty() { 
        bool status = false;
        std::unique_lock<std::mutex> mlock(m_mutex);
        status = m_data_queue.empty();
        mlock.unlock(); 
        return status;
    }
private:
    // As of now we use std::queue but other data structures could be used as well
    std::queue<DataQueueEntry_<T>> m_data_queue;
    std::mutex m_mutex;
    std::condition_variable m_cond;

    //=============================================================================
};  // END of class templatized DataQueue_<T>
//=============================================================================
}

#endif // OPENSIM_DATA_QUEUE_H_
