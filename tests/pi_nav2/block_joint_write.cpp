// Delay actual Fast DDS writes in the joint publisher during the stationary trial.
#include <chrono>
#include <cstdio>
#include <dlfcn.h>
#include <thread>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>

namespace eprosima::fastdds::dds {
bool DataWriter::write(void* data)
{
    using Write = bool (*)(DataWriter*, void*);
    static auto write_sample = reinterpret_cast<Write>(
        dlsym(RTLD_NEXT, "_ZN8eprosima7fastdds3dds10DataWriter5writeEPv"));
    if (get_topic()->get_name() == "rt/joint_states") {
        static unsigned samples;
        ++samples;
        if (samples == 60 || samples == 135) {
            const unsigned delay_ms = samples == 60 ? 450 : 1200;
            std::fprintf(stderr, "Test joint write delay %u ms\n", delay_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    return write_sample(this, data);
}
}
