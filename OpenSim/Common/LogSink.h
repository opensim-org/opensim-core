#ifndef OPENSIM_LOGSINK_H_
#define OPENSIM_LOGSINK_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  LogSink.h                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
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

#include "osimCommonDLL.h"
#include <iostream>
#include <spdlog/sinks/base_sink.h>
#include <mutex>

// This file is not included in osimCommon.h. Only include
// this file when deriving from LogSink.

namespace OpenSim {

/// Derive from this class to implement your own way of reporting logged
/// messages.
class OSIMCOMMON_API LogSink : public spdlog::sinks::base_sink<std::mutex> {
public:
    virtual ~LogSink() = default;
protected:
    /// This function is invoked whenever a message is logged at the desired
    /// Log::Level.
    virtual void sinkImpl(const std::string& msg) = 0;
    virtual void flushImpl() {}
private:
    void sink_it_(const spdlog::details::log_msg& msg) override final {
        sinkImpl(std::string(msg.payload.begin(), msg.payload.end()));
    }
    void flush_() override final { flushImpl(); }
};

/// This sink stores all messages in a string. This is useful for testing the
/// content of logs.
class OSIMCOMMON_API StringLogSink : public LogSink {
public:
    /// Clear the contents of the string.
    void clear() {
        m_messages.clear();
    }
    /// Obtain the string.
    const std::string& getString() const {
        return m_messages;
    }
protected:
    void sinkImpl(const std::string& msg) override {
        m_messages += msg + "\n";
    }
private:
    std::string m_messages;
};

} // namespace OpenSim

#endif // OPENSIM_LOGSINK_H_
