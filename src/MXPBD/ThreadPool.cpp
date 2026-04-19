#include "MXPBD/ThreadPool.h"

namespace MXPBD
{
    void GetPCore(std::uint32_t& count, std::uint64_t& mask)
    {
        count = 0;
        mask = 0;
        DWORD len = 0;
        GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &len);
        std::vector<BYTE> buffer(len);
        if (GetLogicalProcessorInformationEx(RelationProcessorCore, reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buffer.data()), &len))
        {
            BYTE* ptr = buffer.data();
            BYTE maxEfficiency = 0;
            while (ptr < buffer.data() + len)
            {
                auto info = reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(ptr);
                if (info->Relationship == RelationProcessorCore)
                {
                    if (info->Processor.EfficiencyClass > maxEfficiency)
                    {
                        maxEfficiency = info->Processor.EfficiencyClass;
                    }
                }
                ptr += info->Size;
            }
            ptr = buffer.data();
            while (ptr < buffer.data() + len)
            {
                auto info = reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(ptr);
                if (info->Relationship == RelationProcessorCore)
                {
                    if (info->Processor.EfficiencyClass == maxEfficiency)
                    {
                        mask |= info->Processor.GroupMask[0].Mask;
                    }
                }
                ptr += info->Size;
            }
        }
        count = std::popcount(mask);
    }
}